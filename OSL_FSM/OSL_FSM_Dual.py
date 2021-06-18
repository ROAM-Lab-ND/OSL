'''
##################################### OPEN #####################################
This script is the original finite state machine designed for walking on the Open Source Leg (OSL) knee-ankle configuration.

Last Update: 16 June 2021
Updates:
    - Updated to match syntax of OSL_FSM_Ankle.py
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

from OSL_Modules.OSL_Calibration_Dual import OSL_Constants as osl
from OSL_Modules.OSL_Calibration_Dual import OSL_Calibration_Package as pac
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Homing as home
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Storage as stor
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_DeviceOpenClose as opcl
from OSL_Modules.OSL_4Bar_Dual import OSL_4BarFunctions_Mapping as four
from OSL_Modules.OSL_Force_Dual import OSL_ForceFunctions_Full as pres
from OSL_Modules.OSL_Torque_Dual import OSL_TorqueFunctions_StiffnessDamping as tor

#################################### SETUP #####################################

# Connect to actuator and open data stream
FX = fx.FlexSEA()

devId = opcl.devOpen(FX)

calChoice = int(input('Use existing calibration data or run calibration sequence (1 for Existing, 2 for New Data): '))

try:

    if calChoice == 1:

        calData = stor.calLoad()
        print('Data Loaded Successfully')

    else:

        if calChoice != 2:

            print('Invalid option chosen, running calibration sequence as back up...')


        calData = pac.CalDataDual()
        calData = pac.dualCalMot(devId,FX,calData)

        print('Calibration successful...')

except Exception as error:

    print('Error Occurred:')
    print(error)
    opcl.devClose(devId,FX)
    raise

TR = [calData.bpdMot[0]/osl.deg2count, 0]

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz=1000000

# Setting up States
# States are (Unspecified, Early_Stance, Late_Stance, Early Swing, Late Swing)
STATES = (0, 1, 2, 3, 4)

stiffKnee = (0, 0.15, 0.20, 0.02, 0.02)
stiffAnk = (0.03, 0.06, 0.08, 0.05, 0.05) # Nm/deg/kg

dampKnee = (0, 0, 0, 0.0012, 0.0012)
dampAnk = (0.001, 0.0003, 0.00015, 0.0003, 0.0003) # Nms/deg/kg
equilKnee = (0, 1, 1, 90, 0)
equilAnk = (0, -5, 5, 0, 0) # ankle degrees from vertical, 0, -5, 10, 0 (positive plantarflexion)

usrWeight = 72 # [kgs]

GAINSKnee = {'kp': 40, 'ki': 400, 'K': 50, 'B': 650, 'FF': 128}
GAINSAnk = {'kp': 40, 'ki': 400, 'K': 50, 'B': 650, 'FF': 128}

# Read initial angle
actDataKnee = FX.read_device(devId[0])
actDataAnk = FX.read_device(devId[1])
motKnee = actDataKnee.mot_ang
motAnk = actDataAnk.mot_ang


# Set gains (in order: kp, ki, kd, K, B & ff)
FX.set_gains(devId[0], GAINSKnee['kp'], GAINSKnee['ki'], 0, GAINSKnee['K'], GAINSKnee['B'], GAINSKnee['FF'])
FX.set_gains(devId[1], GAINSAnk['kp'], GAINSAnk['ki'], 0, GAINSAnk['K'], GAINSAnk['B'], GAINSAnk['FF'])

# Setpoint = initial angle
FX.send_motor_command(devId[0], fxe.FX_IMPEDANCE, motKnee)
FX.send_motor_command(devId[1], fxe.FX_IMPEDANCE, motAnk)

# Select transition rate and positions
#num_time_steps = int(500)
#sleep(0.4)

stateCur = STATES[0]

# Force Sensor On/Off Thresholds
threshForceOn = 15
threshForceOff = 8

# Dorsiflexion Threshold (Degrees) for Late Stance Trigger
threshDF = -10

# Angular Velocity Threshold (Rad/Sec) for Late Swing Trigger
threshRot = 0.3

# Initializing Force Sensor Readings for Previous Time Steps
pad0Val = pad0ValPrev1 = pad0ValPrev2 = pad1Val = pad1ValPrev1 = pad1ValPrev2  = 0
pad0ValMax = pad1ValMax = 0

firstPass = True

minEarlyStance = 500 # milliseconds

try:

    while True:

        motKneePrev = motKnee

        pad0ValPrev2 = pad0ValPrev1
        pad0ValPrev1 = pad0Val

        pad1ValPrev2 = pad1ValPrev1
        pad1ValPrev1 = pad1Val

        pad0Val = pres.readadc(osl.pChan0) # toe
        pad1Val = pres.readadc(osl.pChan1) # heel
        print('Pressure Pad Value: %d %d' %(pad0Val, pad1Val))

        actDataKnee = FX.read_device(devId[0])
        actDataAnk = FX.read_device(devId[1])

        motKnee = actDataKnee.mot_ang
        jointKnee = (motKnee - calData.angExtMot[0])/calData.bpdMot[0]

        motAnk = actDataAnk.mot_ang
        jointAnK = four.anklePosMappingMot(motAnk,calData)
        TR[1] = four.ankleTRMappingMot(motAnk,calData)

        gyroKnee = (actDataKnee.gyroz*osl.deg2rad)/osl.gyroConv - calData.gyro
        diffMotKnee = ((motKnee - motKneePrev)*osl.deg2rad)/calData.bpdMot[0]

        rotKnee = gyroKnee - diffMotKnee/osl.dtCenti

        if stateCur == 0:

            stateName = 'Unspecified'
            print(stateName)

            desBKnee = tor.motDamping(usrWeight, dampKnee[stateCur], TR[0])
            desBAnk = tor.motDamping(usrWeight, dampAnk[stateCur], TR[1])

            if pres.forceOn(pad0Val, pad0ValPrev1, pad0ValPrev2, threshForceOn) or pres.forceOn(pad1Val, pad1ValPrev1, pad1ValPrev2, threshForceOn):

                stateCur = STATES[1]
                print('Transitioning to Early Stance')

        elif stateCur == 1:

            stateName = 'Early Stance'
            print(stateName)

            desBKnee = 1000
            desBAnk = 1000

            if firstPass:

                firstPass = False
                firstPassTime = datetime.datetime.now()
                print('First Pass:', firstPassTime)

            if (datetime.datetime.now() - firstPassTime).seconds == 0 and (datetime.datetime.now() - firstPassTime).microseconds/1000 < minEarlyStance:

                print((datetime.datetime.now() - time_first_pass).microseconds/1000)
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

            desBKnee = 2000
            desBAnk = 2000

            if pres.forceOff(pad0Val, pad0ValPrev1, threshForceOff) and pres.forceOff(pad1Val, pad1ValPrev1, threshForceOff):

                stateCur = STATES[3]
                print('Transitioning to Early Swing')

        elif stateCur == 3:

            stateName = 'Early Swing'
            print(stateName)

            desBKnee = 1000
            desBAnk = 1000

            if pres.forceOn(pad0Val, pad0ValPrev1, pad0ValPrev2, threshForceOn) or pres.forceOn(pad1Val, pad1ValPrev1, pad1ValPrev2, threshForceOn):

                stateCur = STATES[1]
                firstPass = True
                print('Transitioning to Early Stance')

            if rotKnee > threshRot:

                stateCur = STATES[4]
                print('Transitioning to Late Swing')

        elif stateCur == 4:

            stateName = 'Late Swing'
            print(stateName)

            desBKnee = 1000
            desBAnk = 1000

            if pres.forceOn(pad0Val, pad0ValPrev1, pad0ValPrev2, threshForceOn) or pres.forceOn(pad1Val, pad1ValPrev1, pad1ValPrev2, threshForceOn):

                stateCur = STATES[1]
                firstPass = True
                print('Transitioning to Early Stance')


        desKKnee = tor.motStiffness(usrWeight, stiffKnee[stateCur], TR[0])
        desKAnk = tor.motStiffness(usrWeight, stiffAnk[stateCur], TR[1])

        motDesKnee = calData.bpdMot[0]*equilKnee[stateCur] + calData.angExtMot[0]
        motDesAnk = four.anklePosMappingJoint(equilAnk[stateCur], calData)

        # Control gain constants
        GAINSAnk['K'], GAINSAnk['B'] = desKAnk, desBAnk
        GAINSKnee['K'], GAINSKnee['B'] = desKKnee, desBKnee

        # Set gains (in order: kp, ki, kd, K, B & ff)
        FX.set_gains(devId[0], GAINSKnee['kp'], GAINSKnee['ki'], 0, GAINSKnee['K'], GAINSKnee['B'], GAINSKnee['FF'])
        FX.set_gains(devId[1], GAINSAnk['kp'], GAINSAnk['ki'], 0, GAINSAnk['K'], GAINSAnk['B'], GAINSAnk['FF'])

        FX.send_motor_command(devId[0], fxe.FX_IMPEDANCE, motDesKnee)
        FX.send_motor_command(devId[1], fxe.FX_IMPEDANCE, motDesAnk)

        sleep(osl.dtCenti)

except Exception as error:

    print(error)

finally:

    opcl.devClose(devId,FX)
