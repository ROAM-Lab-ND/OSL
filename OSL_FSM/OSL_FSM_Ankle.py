'''
##################################### OPEN #####################################
This script is the original finite state machine designed for walking on the Open Source Leg (OSL) ankle-only configuration.

Last Update: 21 June 2021
Updates:
    - General Bug Fixes From 06/14/21-06/18/21
    - Improved Comments and Documentation
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

# Imports for Force/EMG Sensors
from signal import signal, SIGINT
import spidev

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
from OSL_Modules.OSL_Force import OSL_ForceFunctions_Full as pres
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
        calData = pac.ankleCalMot(devId, FX, calData, cal=1)

        print('Calibration successful...')

except Exception as error:

    # Print Out Error that Occurred
    print('Error Occurred:')
    print(error)

    # Gracefully Exit Script by Closing Stream and Device ID
    opcl.devClose(devId,FX)
    raise

# Create Class Object for SPI Functions Related to Force/EMG Sensors
sleep(5*osl.dtCenti)
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz=1000000

# Ankle States (Unspecified, Early Stance, Late Stance, Swing)
STATES = (0, 1, 2, 3)

# Ankle Stiffness (Nm/deg/kg)
stiffAnk = (0.03, 0.06, 0.08, 0.05)

# Ankle Damping (Nms/deg/kg)
dampAnk = (0.001, 0.0003, 0.00015, 0.0003)

# Ankle Equilibrium Positions Relative to Vertical Orientation (DF (-), PF (+))
equilAnk = (0, -5, 5, 0)

# Weight of User (kg)
usrWeight = 72

# Actuator Impedance Gains Initialization
GAINSAnk = {'kp': 40, 'ki': 400, 'K': 50, 'B': 650, 'FF': 128}

# Read Initial Angle
actDataAnk = FX.read_device(devId)
angInitAnk = actDataAnk.mot_ang

# Set Impedance Gains
FX.set_gains(devId, GAINSAnk['kp'], GAINSAnk['ki'], 0, GAINSAnk['K'], GAINSAnk['B'], GAINSAnk['FF'])

# Initialize Motor Position at Initial Angle
FX.send_motor_command(devId, fxe.FX_IMPEDANCE, angInitAnk)

# Initialize in Unspecified state
stateCur = STATES[0]

# Force Sensor On/Off Thresholds
threshForceOn = 15
threshForceOff = 8

# Dorsiflexion Threshold for Late Stance Trigger
threshDF = -10

# Initializing Force Sensor Readings for Current/Previous Time Steps
pad0Val = pad0ValPrev1 = pad0ValPrev2 = pad1Val = pad1ValPrev1 = pad1ValPrev2  = 0
pad0ValMax = pad1ValMax = 0

# Set First Pass Flag for Entry into Early Stance
firstPass = True

# Battery Checking Threshold (sec)
minBatCheck = 60
batTime = datetime.datetime.now()

# Early Stance Duration Threshold (msec)
minEarlyStance = 500

############################# FINITE STATE MACHINE #############################

try:

    while True:

        # If Checking Threshold Has Passed, Check Battery Voltage
        if (datetime.datetime.now() - batTime).seconds > minBatCheck:

            bat.voltCheck(devId, FX)
            batTime = datetime.datetime.now()

        # Set Force Sensor Historical Data
        pad0ValPrev2 = pad0ValPrev1
        pad0ValPrev1 = pad0Val

        pad1ValPrev2 = pad1ValPrev1
        pad1ValPrev1 = pad1Val

        # Read Current Force Sensor Data
        pad0Val = pres.readadc(osl.pChan0, spi) # toe
        pad1Val = pres.readadc(osl.pChan1, spi) # heel
        print('Pressure Pad Value: %d %d' % (pad0Val, pad1Val))

        # Read Current Motor Information and Grab Motor Encoder
        actDataAnk = FX.read_device(devId)
        motAnk = actDataAnk.mot_ang

        # Calculate Current Ankle Joint Angle and Transmission Ratio
        jointAnk = four.anklePosMappingMot(motAnk, calData)
        TR = four.ankleTRMappingMot(motAnk, calData)

        if stateCur == 0:

            # Print State
            stateName = 'Unspecified'
            print(stateName)

            # Calculate Desired Ankle Damping
            desBAnk = tor.motDamping(usrWeight, dampAnk[stateCur], TR)

            # If Force Sensors Surpass Threshold, Transition to State 1
            if pres.forceOn(pad0Val, pad0ValPrev1, pad0ValPrev2, threshForceOn) or pres.forceOn(pad1Val, pad1ValPrev1, pad1ValPrev2, threshForceOn):

                stateCur = STATES[1]
                print('Transitioning to Early Stance')

        elif stateCur == 1:

            # Print State
            stateName = 'Early Stance'
            print(stateName)

            # Hardcoded Desired Ankle Damping (Will Be Changed)
            desBAnk = 1000

            # If First Entry into State 1, Grab Instance of Entry
            if firstPass:

                firstPass = False
                firstPassTime = datetime.datetime.now()
                print('First Pass:', firstPassTime)

            # If Time in State 1 Below Threshold, Print Time Spent So Far
            if (datetime.datetime.now() - firstPassTime).seconds == 0 and (datetime.datetime.now() - firstPassTime).microseconds/1000 < minEarlyStance:

                print((datetime.datetime.now() - firstPassTime).microseconds/1000)

                if pad0Val > pad0ValMax:

                        pad0ValMax = pad0Val

                if pad1Val > pad1ValMax:

                        pad1ValMax = pad1Val

            # If Time in State 1 Surpasses Threshold, Check Ankle Angle
            else:

                # If Ankle Angle Surpasses Threshold, Transition to State 2
                if jointAnk < threshDF:

                    stateCur = STATES[2]
                    print('Transitioning to Late Stance')

                else:

                    print('Has not passed dorsiflexion threshold')

            # If Force Sensors Below Threshold, Transition to State 3
            if pres.forceOff(pad0Val, pad0ValPrev1, threshForceOff) and pres.forceOff(pad1Val, pad1ValPrev1, threshForceOff):

                stateCur = STATES[3]
                print('Transitioning to Swing!')

        elif stateCur == 2:

            # Print State
            stateName = 'Late Stance'
            print(stateName)

            # Hardcoded Desired Ankle Damping (Will Be Changed)
            desBAnk = 2000

            # If Force Sensors Below Threshold, Transition to State 3
            if pres.forceOff(pad0Val, pad0ValPrev1, threshForceOff) and pres.forceOff(pad1Val, pad1ValPrev1, threshForceOff):

                stateCur = STATES[3]
                print('Transitioning to Swing!')

        elif stateCur == 3:

            # Print State
            stateName = 'Swing'
            print(stateName)

            # Hardcoded Desired Ankle Damping (Will Be Changed)
            desBAnk = 1000

            # If Force Sensors Surpass Threshold, Transition to State 1
            if pres.forceOn(pad0Val, pad0ValPrev1, pad0ValPrev2, threshForceOn) or pres.forceOn(pad1Val, pad1ValPrev1, pad1ValPrev2, threshForceOn):

                stateCur = STATES[1]
                firstPass = True
                print('Transitioning to Early Stance')

        # Calculate Desired Motor Stiffness
        desKAnk = tor.motStiffness(usrWeight, stiffAnk[stateCur], TR)

        # Calculate Desired Motor Encoder Value
        motDesAnk = four.anklePosMappingJoint(equilAnk[stateCur], calData)

        # Update Stiffness/Damping Gains
        GAINSAnk['K'], GAINSAnk['B'] = desKAnk, desBAnk

        # Set Impedance Gains
        FX.set_gains(devId, GAINSAnk['kp'], GAINSAnk['ki'], 0, GAINSAnk['K'], GAINSAnk['B'], GAINSAnk['FF'])

        # Send Motor Command
        FX.send_motor_command(devId, fxe.FX_IMPEDANCE, motDesAnk)

        # Delay
        sleep(osl.dtCenti)

except Exception as error:

    # Print Error
    print(error)

finally:

    # Gracefully Exit Script by Closing Stream and Device ID
    opcl.devClose(devId, FX)
