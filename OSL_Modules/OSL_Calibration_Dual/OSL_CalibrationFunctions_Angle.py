'''
##################################### OPEN #####################################
This package holds the functions called by OSL_Calibration_Package.py to calibrate the Angles in the Dephy actuator

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

# Imports for Standard Python
from time import sleep, time, strftime
import os, sys
import math
import numpy as np

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration_Dual import OSL_Constants as osl
from OSL_Modules.OSL_Calibration_Dual import OSL_Calibration_Package as pac
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_DeviceOpenClose as opcl

############################# FUNCTION DEFINITIONS #############################

def angleZero(devId, FX, volt):

    '''
    Function for determining encoder values at hard stops for knee or ankle of Open Source Leg (OSL).
    Inputs:
        devId - Device IDs of actuators to encoder values at hard stop for
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
    FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, volt)
    FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, volt)

    # Read Current Motor Information and Grab Encoder
    actDataKnee = FX.read_device(devId[0])
    actDataAnk = FX.read_device(devId[1])

    motCurKnee = actDataKnee.mot_ang
    jointCurKnee = actDataKnee.ank_ang
    motCurAnk = actDataAnk.mot_ang
    jointCurAnk = actDataAnk.ank_ang

    # Print Information to User
    print(motCurKnee, jointCurKnee, motCurAnk, jointCurAnk)

    # Delay
    sleep(2*osl.dtDeci)

    # Initialize Booleans for Calibration Sequence
    runKnee = True
    runAnk = True
    run = True

    while run:

        # Set Encoder Historical Data
        motPrevKnee = motCurKnee
        jointPrevKnee = jointCurKnee
        motPrevAnk = motCurAnk
        jointPrevAnk = jointCurAnk

        # Read Current Motor Information and Grab Encoder
        actDataKnee = FX.read_device(devId[0])
        actDataAnk = FX.read_device(devId[1])

        motCurKnee = actDataKnee.mot_ang
        jointCurKnee = actDataKnee.ank_ang
        motCurAnk = actDataAnk.mot_ang
        jointCurAnk = actDataAnk.ank_ang

        # Calculate Difference Between Previous and Current Encoder
        angDiffMKnee = motCurKnee - motPrevKnee
        angDiffJKnee = jointCurKnee - jointPrevKnee
        angDiffMAnk = motCurAnk - motPrevAnk
        angDiffJAnk = jointCurAnk - jointPrevAnk

        print(motCurKnee, angDiffMKnee, jointCurKnee, angDiffJKnee, motCurAnk, angDiffMAnk, jointCurAnk, angDiffJAnk)

        sleep(osl.dtCenti)

        # If Motor Rotation is Below Threshold, Knee is at Hard Stop
        if (abs(angDiffMKnee) <= osl.deg2count/2) and (runKnee):

            # Send Motor Command to Stop
            FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, 0)

            # Read Current Motor Information and Grab Encoder
            actDataKnee = FX.read_device(devId[0])
            motFinalKnee = actDataKnee.mot_ang
            jointFinalKnee = actDataKnee.ank_ang

            # Set (Knee) Run Flag to False
            runKnee = False

        # If Motor Rotation is Below Threshold, Ankle is at Hard Stop
        if (abs(angDiffMAnk) <= osl.deg2count/2) and (runAnk):

            # Send Motor Command to Stop
            FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, 0)

            # Read Current Motor Information and Grab Encoder
            actDataAnk = FX.read_device(devId[1])
            motFinalAnk = actDataAnk.mot_ang
            jointFinalAnk = actDataAnk.ank_ang

            # Set (Ankle) Run Flag to False
            runAnk = False

        # If Knee and Ankle Run Flags Not Set, All Hard Stops Reached
        if (not runKnee) and (not runAnk):

            # Set Fun Flag to False
            run = False

            # Store Final Encoder Values
            motFinal = [motFinalKnee, motFinalAnk]
            jointFinal = [jointFinalKnee, jointFinalAnk]

            print('Completed Sweep')

    # Return Calibrated Data
    return motFinal, jointFinal

def angleCal(devId, FX, volt=750):

    '''
    Function for calibrating angle-related data for knee and ankle of Open Source Leg (OSL).
    Inputs:
        devId - Device ID of actuator to encoder values at hard stop for
        FX - Class object with flexSEA Dephy functions for reading actuator data
        volt - Voltage value to run motor at
    Outputs:
        motFinal - Motor encoder value at hard stop
        jointFinal - Joint encoder value at hard stop
    '''

    # Initialize Range of Motion
    romMot = [120, 124]
    romJoint = [120, 30]

    # Alert User Actuators Are About to Run
    print('Running Actuator to Min and Max...')
    sleep(2*osl.dtDeci)

    # Calibrate Encoder Values at Extension Hard Stop (PF For Ankle)
    angExtM, angExtJ = angleZero(devId, FX, abs(volt))
    sleep(osl.dtMilli)

    # Calibrate Encoder Values at Flexion Hard Stop (DF For Ankle)
    angFlexM, angFlexJ = angleZero(devId, FX, -abs(volt))
    sleep(osl.dtMilli)

    # Calculate Bits Per Degree Ratio
    bpdM = ((np.asarray(angExtM) - np.asarray(angFlexM))/np.asarray(romMot)).tolist()

    bpdJ = ((np.asarray(angExtJ) - np.asarray(angFlexJ))/np.asarray(romJoint)).tolist()

    # If No Joint Encoder Attached, Set Bits Per Degree Ratio to -2
    if bpdJ[0] == 0:

        bpdJ[0] = -2

    if bpdJ[1] == 0:

        bpdJ[1] = -2

    # Print Information to User
    print('Bit to Degree Motor Ratio: ', bpdM)
    print('Bit to Degree Joint Ratio: ', bpdJ)

    print('Confirmation Test...')
    sleep(2*osl.dtCenti)

    # Initialize Boolean for Full Sweep
    runKnee = True
    runAnk = True
    run = True

    # Send Motor Command (2x Passed In Voltage)
    FX.send_motor_command(devId[0],fxe.FX_VOLTAGE,abs(volt)*2)
    FX.send_motor_command(devId[1],fxe.FX_VOLTAGE,abs(volt)*2)

    # Read Current Motor Information and Grab Encoder
    actDataKnee = FX.read_device(devId[0])
    actDataAnk  = FX.read_device(devId[1])

    motValKnee = actDataKnee.mot_ang
    jointValKnee = actDataKnee.ank_ang
    motValAnk = actDataAnk.mot_ang
    jointValAnk = actDataAnk.ank_ang

    motVal = [motValKnee,motValAnk]
    jointVal = [jointValKnee,jointValAnk]

    # Calculate Current Encoder Angles (Degs,Rads)
    motCur = (np.asarray(angExtM) - np.asarray(motVal))/np.asarray(bpdM)
    motCurRad = np.multiply(motCur, osl.deg2rad)

    jointCur = (np.asarray(angExtJ) - np.asarray(jointVal))/np.asarray(bpdJ)
    jointCurRad = np.multiply(jointCur, osl.deg2rad)

    # Print calibrated angle in degrees and radians to screen
    print('%-15s %-7f %-7f %-15s %-7f %-7f' % ('Knee Mot/Joint',motCur[0],jointCur[0],'Ankle Mot/Joint',motCur[1],jointCur[1]))

    # Set motor to twice the calibration voltage for quicker run time
    FX.send_motor_command(devId[0],fxe.FX_VOLTAGE,abs(volt)*2)
    FX.send_motor_command(devId[1],fxe.FX_VOLTAGE,abs(volt)*2)

    sleep(3*osl.dtCenti)

    while run:

        # Set Encoder Historical Data
        motPrev = motVal
        jointPrev = jointVal

        # Read Current Motor Information and Grab Encoder
        actDataKnee = FX.read_device(devId[0])
        actDataAnk = FX.read_device(devId[1])

        motValKnee = actDataKnee.mot_ang
        jointValKnee = actDataKnee.ank_ang
        motValAnk = actDataAnk.mot_ang
        jointValAnk = actDataAnk.ank_ang

        motVal = [motValKnee,motValAnk]
        jointVal = [jointValKnee,jointValAnk]

        # Calculate Current Encoder Angles (Degs,Rads)
        motCur = (np.asarray(angExtM) - np.asarray(motVal))/np.asarray(bpdM)
        motCurRad = np.multiply(motCur, osl.deg2rad)

        jointCur = (np.asarray(angExtJ) - np.asarray(jointVal))/np.asarray(bpdJ)
        jointCurRad = np.multiply(jointCur, osl.deg2rad)

        # Calculate Difference Between Previous and Current Encoder
        angDiffM = np.asarray(motVal) - np.asarray(motPrev)

        # Print Information to User
        print('%-15s %-7f %-7f %-15s %-7f %-7f' % ('Knee Mot/Joint',motCur[0],jointCur[0],'Ankle Mot/Joint',motCur[1],jointCur[1]))

        sleep(osl.dtCenti)

        # If Motor Rotation is Below Threshold, Knee is at Hard Stop
        if (abs(angDiffM[0]) <= osl.deg2count/2) and (runKnee):

            # Send Motor Command to Stop
            FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, 0)

            # Set (Knee) Run Flag to False
            runKnee = False

        # If Motor Rotation is Below Threshold, Ankle is at Hard Stop
        if (abs(angDiffM[1]) <= osl.deg2count/2) and (runAnk):

            # Send Motor Command to Stop
            FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, 0)

            # Set (Ankle) Run Flag to False
            runAnk = False

        # If Knee and Ankle Run Flags Not Set, All Hard Stops Reached
        if (not runKnee) and (not runAnk):

            run = False

    print('Angle Calibration Complete.')

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

    calData = pac.CalDataDual(1)

    try:

        angM1, angM2, bpdM, angJ1, angJ2, bpdJ = angleCal(devId, FX)

        calData.angExtMot = [int(angM1[0]), int(angM1[1])]
        calData.angFlexMot = [int(angM2[0]), int(angM2[1])]

        calData.bpdMot = [float(bpdM[0]), float(bpdM[1])]

        calData.angExtJoint = [int(angJ1[0]), int(angJ1[1])]
        calData.angFlexJoint = [int(angJ2[0]), int(angJ2[1])]

        calData.bpdJoint = [float(bpdJ[0]), float(bpdJ[1])]

        calData.angVertJointAnk = calData.angExtJoint[1] - 15*calData.bpdJoint[1]
        calData.angVertMotAnk = calData.angExtMot[1] - 67*calData.bpdMot[1]

    except Exception as error:

        print('Error occurred')
        print(error)

    finally:

        opcl.devClose(devId, FX)

if __name__ == '__main__':

    main()
