'''
##################################### OPEN #####################################
This script is the original finite state machine designed for walking on the Open Source Leg (OSL) ankle-only configuration.

Last Update: 11 June 2021
Updates:
    - Created
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Standard Python Modules
from time import sleep, time
import datetime
import os, sys
import math
import numpy as np
import yaml

from signal import signal, SIGINT
import spidev

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_Calibration_Package as pac
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Homing as home
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Storage as stor
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_DeviceOpenClose as opcl
from OSL_Modules.OSL_4Bar import OSL_4BarFunctions_Mapping as four
from OSL_Modules.OSL_Force import OSL_ForceFunctions_Full as pres
from OSL_Modules.OSL_Torque import OSL_TorqueFunctions_StiffnessDamping as tor

#################################### SETUP #####################################

# Connect to actuator and open data stream
FX = fx.FlexSEA()

devId = opcl.devOpen(FX)

calChoice = int(input('Use existing calibration data or run calibration sequence (1 for Existing, 2 for New Data): '))

try:

    if calChoice == 1:

        calData = stor.calLoad(devId)
        print('Data Loaded Successfully')

    else:

        if calChoice != 2:

            print('Invalid option chosen, running calibration sequence as back up...')


        calData = pac.CalDataSingle()
        calData = pac.ankleCalMot(devId,FX,calData,cal=1)

        print('Calibration successful...')

except Exception as error:

    print('Error Occurred:')
    print(error)
    opcl.devClose(devId,FX)
    raise

sleep(0.05)
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz=1000000

# Setting up States
# States are (Unspecified, Early_Stance, Late_Stance, Early Swing, Late Swing)
STATES = (0, 1, 2, 3)
stiffAnk = (0.03, 0.06, 0.08, 0.05) # Nm/deg/kg
dampAnk = (0.001, 0.0003, 0.00015, 0.0003) # Nms/deg/kg
equilAnk = (0, -5, 5, 0) # ankle degrees from vertical, 0, -5, 10, 0 (positive plantarflexion)

usrWeight = 72 # [kgs]

GAINSAnk = {'kp': 40, 'ki': 400, 'K': 50, 'B': 650, 'FF': 128}

# Read initial angle
actDataAnk = FX.read_device(devId)
angInitAnk = actDataAnk.mot_ang

# Set gains (in order: kp, ki, kd, K, B & ff)
FX.set_gains(devId, GAINSAnk['kp'], GAINSAnk['ki'], 0, GAINSAnk['K'], GAINSAnk['B'], GAINSAnk['FF'])

# Setpoint = initial angle
FX.send_motor_command(devId, fxe.FX_IMPEDANCE, angInitAnk)

# Select transition rate and positions
#num_time_steps = int(500)
#sleep(0.4)

stateCur = STATES[0]

# Force Sensor On/Off Thresholds
threshForceOn = 15
threshForceOff = 8

# Dorsiflexion Threshold for Late Stance Trigger
threshDF = -10

# Initializing Force Sensor Readings for Previous Time Steps
pad0Val = pad0ValPrev1 = pad0ValPrev2 = pad1Val = pad1ValPrev1 = pad1ValPrev2  = 0
pad0ValMax = pad1ValMax = 0

firstPass = True

minEarlyStance = 500 # milliseconds

try:

    while True:

        pad0ValPrev2 = pad0ValPrev1
        pad0ValPrev1 = pad0Val

        pad1ValPrev2 = pad1ValPrev1
        pad1ValPrev1 = pad1Val

        pad0Val = pres.readadc(osl.pChan0,spi) # toe
        pad1Val = pres.readadc(osl.pChan1,spi) # heel
        print('Pressure Pad Value: %d %d' %(pad0Val, pad1Val))

        actDataAnk = FX.read_device(devId)

        motAnk = actDataAnk.mot_ang
        jointAnk = four.anklePosMappingMot(motAnk,calData)
        TR = four.ankleTRMappingMot(motAnk,calData)

        if stateCur == 0:

            stateName = 'Unspecified'
            print(stateName)

            desBAnk = tor.motDamping(usrWeight, dampAnk[stateCur], TR)

            if pres.forceOn(pad0Val, pad0ValPrev1, pad0ValPrev2, threshForceOn) or pres.forceOn(pad1Val, pad1ValPrev1, pad1ValPrev2, threshForceOn):

                stateCur = STATES[1]
                print('Transitioning to Early Stance')

        elif stateCur == 1:

            stateName = 'Early Stance'
            print(stateName)

            desBAnk = 1000

            if firstPass:

                firstPass = False
                firstPassTime = datetime.datetime.now()
                print('First Pass:', firstPassTime)

            if (datetime.datetime.now() - firstPassTime).seconds == 0 and (datetime.datetime.now() - firstPassTime).microseconds/1000 < minEarlyStance:

                print((datetime.datetime.now() - firstPassTime).microseconds/1000)

                if pad0Val > pad0ValMax:

                        pad0ValMax = pad0Val

                if pad1Val > pad1ValMax:

                        pad1ValMax = pad1Val

            else:
                #if pad_value_1 < 0.7*max_pressure_value_1 and pad_value_0 > 0.5*max_pressure_value_0: # add an additional nested if statement related to the angle position
                if jointAnk < threshDF:

                    stateCur = STATES[2]
                    print('Transitioning to Late Stance')

                else:

                    print('Has not passed dorsiflexion threshold')

            if pres.forceOff(pad0Val, pad0ValPrev1, threshForceOff) and pres.forceOff(pad1Val, pad1ValPrev1, threshForceOff):

                stateCur = STATES[3]
                print('Transitioning to Swing!')

        elif stateCur == 2:

            stateName = 'Late Stance'
            print(stateName)

            desBAnk = 2000

            if pres.forceOff(pad0Val, pad0ValPrev1, threshForceOff) and pres.forceOff(pad1Val, pad1ValPrev1, threshForceOff):

                stateCur = STATES[3]
                print('Transitioning to Swing!')

        elif stateCur == 3:

            stateName = 'Swing'
            print(stateName)

            desBAnk = 1000

            if pres.forceOn(pad0Val, pad0ValPrev1, pad0ValPrev2, threshForceOn) or pres.forceOn(pad1Val, pad1ValPrev1, pad1ValPrev2, threshForceOn):

                stateCur = STATES[1]
                firstPass = True
                print('Transitioning to Early Stance')

        desKAnk = tor.motStiffness(usrWeight, stiffAnk[stateCur], TR)

        motDesAnk = four.anklePosMappingJoint(equilAnk[stateCur], calData)

        # Control gain constants
        GAINSAnk['K'], GAINSAnk['B'] = desKAnk, desBAnk

        # Set gains (in order: kp, ki, kd, K, B & ff)
        FX.set_gains(devId, GAINSAnk['kp'], GAINSAnk['ki'], 0, GAINSAnk['K'], GAINSAnk['B'], GAINSAnk['FF'])

        FX.send_motor_command(devId, fxe.FX_IMPEDANCE, motDesAnk)

        sleep(0.1)

except Exception as error:

    print(error)

finally:

    opcl.devClose(devId,FX)
