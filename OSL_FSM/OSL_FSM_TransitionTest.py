'''
##################################### OPEN #####################################
This script is used for testing the state transitions of the knee or ankle for the Open Source Leg (OSL) in a predictable manner

Last Update: 23 June 2021
Updates:
    - Created
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Imports for Standard Python
from time import sleep, time
import datetime
import os, sys
import math
import numpy as np
import yaml

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_Calibration_Package as pac
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Homing as home
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Storage as stor
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_DeviceOpenClose as opcl
from OSL_Modules.OSL_4Bar import OSL_4BarFunctions_Mapping as four
from OSL_Modules.OSL_Torque import OSL_TorqueFunctions_StiffnessDamping as tor
from OSL_Modules.OSL_Battery import OSL_BatteryFunctions_UVLO as bat

#################################### SETUP #####################################

# Create Class Object for Actuator Commands
FX = fx.FlexSEA()

# Open Device ID and Start Streaming
devId = opcl.devOpen(FX)

# Determine How Calibration Data will be Provided
calChoice = int(input('Use existing calibration data or run calibration sequence (1 for Existing, 2 for New Data): '))

try:

    if calChoice == 1:

        # Load Calibration Data Previously Stored in Ankle_Cal.yaml
        calData = stor.calLoad(devId)
        print('Data Loaded Successfully')

    else:

        # Use as Catch All to Fall Back on Rerunning Calibration as a Precaution
        if calChoice != 2:

            print('Invalid option chosen, running calibration sequence as back up...')

        # Create Class Object for Calibration Data, Run Calibration Sequence
        calData = pac.CalDataSingle()

        if devId == osl.devKnee:

            calData = pac.kneeCal(devId, FX, calData)

        elif devId == osl.devAnk:

            calData = pac.ankleCalMot(devId, FX, calData, cal=1)

        else:

            raise RuntimeError('Invalid device ID. Check device ID and compare with OSL_Constants.py stored values.')

        print('Calibration successful...')

except Exception as error:

    # Print Out Error that Occurred
    print('Error Occurred:')
    print(error)

    # Gracefully Exit Script by Closing Stream and Device ID
    opcl.devClose(devId,FX)
    raise

# States (Unspecified, Early Stance, Late Stance, Early Swing, Late Swing)
STATES = (0, 1, 2, 3, 4)

# Joint Stiffness (Nm/deg/kg), Joint Damping (Nms/deg/kg), Joint Equilibrium (deg)
if devId == osl.devKnee:

    stiffJoint = (0, 0.15, 0.20, 0.02, 0.02)
    dampJoint = (1, 0, 0, 0.0012, 0.0012)
    equilJoint = (0, 2, 2, 80, 2)

else:

    stiffJoint = (0.03, 0.06, 0.08, 0.05, 0.05)
    dampJoint = (0.001, 0.0003, 0.00015, 0.0003, 0.0003)
    equilJoint = (0, -5, 5, 0, 0)

# Weight of User (kg)
usrWeight = 72

# Actuator Impedance Gains Initialization
GAINS = {'kp': 40, 'ki': 400, 'K': 50, 'B': 650, 'FF': 128}

# Read Initial Angle
actData = FX.read_device(devId)
motAng = actData.mot_ang

# Set Impedance Gains
FX.set_gains(devId, GAINS['kp'], GAINS['ki'], 0, GAINS['K'], GAINS['B'], GAINS['FF'])

# Initialize Motor Position at Initial Angle
FX.send_motor_command(devId, fxe.FX_IMPEDANCE, motAng)

# Initialize in Unspecified state
stateCur = STATES[0]

############################# STATE USER SWITCHING #############################

try:

    while True:

        # Read Current Motor Information and Grab Motor Encoder
        actData = FX.read_device(devId)
        motAng = actData.mot_ang

        # Calculate Desired Motor Encoder, Current Ankle Joint Angle, Transmission Ratio
        if devId == osl.devKnee:

            motDes = calData.angExtMot - calData.bpdMot*equilJoint[stateCur]
            jointAng = (motAng - calData.angExtMot)/calData.bpdMot
            TR = calData.bpdMot/osl.deg2count

        else:

            motDes = four.anklePosMappingJoint(equilJoint[stateCur], calData)
            jointAng = four.anklePosMappingMot(motAng, calData)
            TR = four.ankleTRMappingMot(motAng, calData)

        # Calculate Desired Motor Stiffness
        desK = tor.motStiffness(usrWeight, stiffJoint[stateCur], TR)

        # Calculate Desired Motor Damping
        desB = tor.motDamping(usrWeight, dampJoint[stateCur], TR)

        # Update Stiffness/Damping Gains
        GAINS['K'], GAINS['B'] = desK, desB

        # Set Impedance Gains
        FX.set_gains(devId, GAINS['kp'], GAINS['ki'], 0, GAINS['K'], GAINS['B'], GAINS['FF'])

        # Send Motor Command
        FX.send_motor_command(devId, fxe.FX_IMPEDANCE, motDes)

        sleep(osl.dtCenti)
        # Read Current Motor Information and Grab Motor Encoder
        actData = FX.read_device(devId)
        motAng = actData.mot_ang

        # Print Information to User
        print('Previous State Joint Angle:', jointAng, '\nCurrent State Motor Encoder:', motAng, '\nCommanded State:', stateCur, '\nCommanded Motor Encoder:', motDes, '\nStiffness, Damping:', desK, desB)
        print('_______________________________________________________________')

        usrInp = input('Choose next state (1, 2, 3, 4) or ''stop'' to end script: ')

        if usrInp in 'stop':

            break

        else:

            stateCur = STATES[int(usrInp)]

except Exception as error:

    # Print Error
    print(error)

finally:

    # Gracefully Exit Script by Closing Stream and Device ID
    opcl.devClose(devId, FX)
