'''
##################################### OPEN #####################################
This package holds the functions called by OSL_Calibration_Package.py to calibrate the Angles in the Dephy actuator

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
    - Updated main function to use OSL_CalibrationFunctions_DeviceOpenClose for opening and closing devices
    - Updated delays to use parameters from OSL_Constants
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

# Imports for Standard Python
from time import sleep, time, strftime
import os, sys
import math
import numpy as np
import scipy as sp
import yaml

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_Calibration_Package as pac
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_DeviceOpenClose as opcl

############################# FUNCTION DEFINITIONS #############################

def angleZero(devId, FX, volt):

    '''
    Function for determining encoder values at hard stops for knee or ankle of Open Source Leg (OSL).
    Inputs:
        devId - Device ID of actuator to encoder values at hard stop for
        FX - Class object with flexSEA Dephy functions for reading actuator data
        volt - Voltage value to run motor at
    Outputs:
        motFinal - Motor encoder value at hard stop
        jointFinal - Joint encoder value at hard stop
    '''

    # Alert User Actuator is About to be Run
    print('Running Actuator...')
    sleep(2*osl.dtDeci)

    # Send Motor Command
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, volt)

    # Read Current Motor Information and Grab Encoder
    actData = FX.read_device(devId)
    motCur = actData.mot_ang
    jointCur = actData.ank_ang

    # Print Encoder to Screen
    print(motCur, jointCur)

    # Delay
    sleep(osl.dtDeci)

    # Initialize Boolean for Calibration Sequence
    run = True

    while run:

        # Set Encoder Historical Data
        motPrev = motCur
        jointPrev = jointCur

        # Read Current Motor Information and Grab Encoder
        actData = FX.read_device(devId)
        motCur = actData.mot_ang
        jointCur = actData.ank_ang

        # Calculate Difference Between Previous and Current Encoder
        angDiffM = motCur - motPrev
        angDiffJ = jointCur - jointPrev

        print(motCur, angDiffM, jointCur, angDiffJ)

        # Delay
        sleep(osl.dtCenti)

        # If Motor Rotation is Below Threshold, System is at Hard Stop
        if abs(angDiffM) <= osl.deg2count/2:

            # Send Motor Command to Stop
            FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

            # Read Current Motor Information and Grab Encoder
            actData = FX.read_device(devId)
            motFinal = actData.mot_ang
            jointFinal = actData.ank_ang

            # Set Run Flag to False
            run = False
            print('Completed Min/Max')

    # Return Encoder Hard Stop Values
    return motFinal, jointFinal

def angleCal(devId, FX, volt=750):

    '''
    Function for calibrating angle-related data for knee or ankle of Open Source Leg (OSL).
    Inputs:
        devId - Device ID of actuator to encoder values at hard stop for
        FX - Class object with flexSEA Dephy functions for reading actuator data
        volt - Voltage value to run motor at
    Outputs:
        motFinal - Motor encoder value at hard stop
        jointFinal - Joint encoder value at hard stop
    '''

    # Set Motor Rotation Value Based On Device ID
    if devId == osl.devAnk:

        romJoint = 30
        romMot = 124

    else:

        romJoint = 120
        romMot = romJoint

    # Alert User Actuator is About to be Run
    print('Running Actuator to Min and Max...')
    sleep(2*osl.dtDeci)

    # Calibrate Encoder Values at Extension Hard Stop (PF For Ankle)
    angExtM, angExtJ = angleZero(devId, FX, abs(volt))
    sleep(osl.dtMilli)

    # Calibrate Encoder Values at Flexion Hard Stop (DF For Ankle)
    angFlexM, angFlexJ = angleZero(devId, FX, -abs(volt))
    sleep(osl.dtMilli)

    # Calculate Bits Per Degree Ratio
    bpdM = (angExtM - angFlexM)/romMot
    bpdJ = (angExtJ - angFlexJ)/romJoint

    # If No Joint Encoder Attached, Set Bits Per Degree Ratio to -2
    if bpdJ == 0:

        bpdJ = -2

    # Print Information to User
    print('Bit to Degree Motor Ratio: ', bpdM)
    print('Bit to Degree Joint Ratio: ', bpdJ)

    print('Confirmation Test...')
    sleep(2*osl.dtDeci)

    # Initialize Boolean for Full Sweep
    run = True

    print('%-15s %-7s %-7s %-15s %-7s %-7s' % ('Current Motor: ', 'Degrees', 'Radians', 'Current Joint: ', 'Degrees', 'Radians'))
    sleep(2*osl.dtDeci)

    # Send Motor Command (2x Passed In Voltage)
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, volt*2)

    # Read Current Motor Information and Grab Encoder
    actData = FX.read_device(devId)
    motVal = actData.mot_ang
    jointVal = actData.ank_ang

    # Calculate Current Encoder Angles (Degs,Rads)
    motCur = (angExtM - motVal)/bpdM
    motCurRad = np.multiply(motCur, osl.deg2rad)

    jointCur = (angExtJ - jointVal)/bpdJ
    jointCurRad = np.multiply(jointCur, osl.deg2rad)

    # Print Information to User
    print('%-15s %-7f %-7f %-15s %-7f %-7f' % ('Current Motor: ', motCur, motCurRad, 'Current Ankle:', jointCur, jointCurRad))

    sleep(3*osl.dtCenti)

    while run:

        # Set Encoder Historical Data
        motPrev = motVal
        jointPrev = jointVal

        # Read Current Motor Information and Grab Encoder
        actData = FX.read_device(devId)
        motVal = actData.mot_ang
        jointVal = actData.ank_ang

        # Calculate Current Encoder Angles (Degs,Rads)
        motCur = (angExtM - motVal)/bpdM
        motCurRad = np.multiply(motCur, osl.deg2rad)

        jointCur = (angExtJ - jointVal)/bpdJ
        jointCurRad = np.multiply(jointCur, osl.deg2rad)

        # Calculate Difference Between Previous and Current Encoder
        angDiffM = motVal - motPrev

        # Print Information to User
        print('%-15s %-7f %-7f %-15s %-7f %-7f' % ('Current Motor:', motCur, motCurRad, 'Current Ankle:', jointCur, jointCurRad))

        # Delay
        sleep(osl.dtCenti)

        # If Motor Rotation is Below Threshold, System is at Hard Stop
        if abs(angDiffM) <= osl.deg2count/2:

            # Send Motor Command to Stop
            FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

            # Set Run Flag to False
            run = False
            print('Completed Test Run')

    print('Angle Calibration Complete.')
    sleep(osl.dtDeci)

    # Return Calibration Data
    return angExtM, angFlexM, bpdM, angExtJ, angFlexJ, bpdJ

############################# MAIN FUN DEFINITIONS #############################

def main():

    '''
    For standalone calling of the angle calibration functions
    '''

    # Create Class Object for Actuator Commands
    FX = fx.FlexSEA()

    # Open Device ID and Start Streaming
    devId = opcl.devOpen(FX)

    calData = pac.CalDataSingle()

    try:

        angM1, angM2, bpdM, angJ1, angJ2, bpdJ = angleCal(devId, FX)

        calData.angExtMot = angM1
        calData.angFlexMot = angM2
        calData.bpdMot = bpdM

        calData.angExtJoint = angJ1
        calData.angFlexJoint = angJ2
        calData.bpdJoint = bpdJ

        calData.angVertJoint = calData.angExtJoint - 15*calData.bpdJoint

    except Exception as error:

        print('Error Occurred')
        print(error)

    finally:

        opcl.devClose(devId, FX)

if __name__ == '__main__':

    main()
