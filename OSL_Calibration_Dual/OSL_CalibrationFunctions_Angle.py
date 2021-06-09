'''
##################################### OPEN #####################################
This package holds the functions called by OSL_Calibration_Package.py to calibrate the Angles in the Dephy actuator

Last Update: 8 June 2021
Updates:
    - Updated main() to use updated class object structure
    - Updated main() to use finally tag for proper syntax of closing actuator stream
    - Updated imports
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

from time import sleep, time, strftime
import os, sys
import math
import numpy as np

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe
from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_Calibration_Package as pac

################################# CALIBRATION ##################################
def angleZero(devId,FX,volt):

    '''
    This function is called by angleCal to determine the encoder values at the hardstops for the knee and ankle actuators
    '''
    print('Running Actuator...')
    sleep(1)

    # Start running motor at inputted voltage
    FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, volt)
    FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, volt)

    # Grab Current Encoder Value
    actDataKnee = FX.read_device(devId[0])
    actDataAnk = FX.read_device(devId[1])

    motCurKnee = actDataKnee.mot_ang
    jointCurKnee = actDataKnee.ank_ang
    motCurAnk = actDataAnk.mot_ang
    jointCurAnk = actDataAnk.ank_ang
    sleep(0.3)

    # Print Encoder Value to Screen
    print(motCurKnee,jointCurKnee,motCurAnk,jointCurAnk)

    # Boolean for tracking when to cut the motor
    runKnee = True
    runAnk = True
    run = True

    while run:

        # Set Tracking Angle to Previously Measured Angle
        motPrevKnee = motCurKnee
        jointPrevKnee = jointCurKnee
        motPrevAnk = motCurAnk
        jointPrevAnk = jointCurAnk

        # Grab Current Encoder Value
        actDataKnee = FX.read_device(devId[0])
        actDataAnk = FX.read_device(devId[1])

        motCurKnee = actDataKnee.mot_ang
        jointCurKnee = actDataKnee.ank_ang
        motCurAnk = actDataAnk.mot_ang
        jointCurAnk = actDataAnk.ank_ang

        # Calculate Difference between Tracking Angle and Current Angle
        angDiffMKnee = motCurKnee - motPrevKnee
        angDiffJKnee = jointCurKnee - jointPrevKnee
        angDiffMAnk = motCurAnk - motPrevAnk
        angDiffJAnk = jointCurAnk - jointPrevAnk

        print(motCurKnee,angDiffMKnee,jointCurKnee,angDiffJKnee,motCurAnk,angDiffMAnk,jointCurAnkangDiffJAnk)

        sleep(0.1)

        # If calculated difference is smaller than half a degree movement, the
        # limit has been reached
        if (abs(angDiffMKnee) <= deg2count/2) and (runKnee):

            # Set motor voltage to zero to stop the actuator
            FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, 0)

            # Grab final encoder value reading
            actDataKnee = FX.read_device(devId[0])
            motFinalKnee = actDataKnee.mot_ang
            jointFinalKnee = actDataKnee.ank_ang

            # Set boolean to exit loop
            runKnee = False

        if (abs(angDiffMAnk) <= deg2count/2) and (runAnk):

            # Set motor voltage to zero to stop the actuator
            FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, 0)

            # Grab final encoder value reading
            actDataAnk = FX.read_device(devId[1])
            motFinalAnk = actDataAnk.mot_ang
            jointFinalAnk = actDataAnk.ank_ang

            # Set boolean to exit loop
            runAnk = False

        if (not runKnee) and (not runAnk)
            run = False
            motFinal = [motFinalKnee,motFinalAnk]
            jointFinal = [jointFinalKnee,jointFinalAnk]
            print('Completed Sweep')

    # Return Calibration Value for Encoders for future use
    return motFinal,jointFinal

def angleCal(devId,FX,volt=750):

    '''
    This function is used to determine the calibration values of the joint angle for the Knee actuator
    '''

    '''
    This function is used to determine the calibration values of the joint angle for the Knee or Ankle actuator
    '''

    romMot = [120,124]
    romJoint = [120,30]

    print('Running Actuator to Min and Max...')
    sleep(0.5)

    # Call to angleZero function for determining encoder value at full extension
    # Full extension is considered when the knee is straight to hard stop
    # Full extension is considered when the ankle is in plantar flexion
    angExtM,angExtJ = angleZero(devId,FX,abs(volt))
    sleep(osl.dt)

    # Call to angleZero function for determining encoder value at full flexion
    # Full flexion is considered when then knee is bent to hard stop
    # Full flexion is considered when the ankle is in dorsiflexion
    angFlexM,angFlexJ = angleZero(devId,FX,-abs(volt))
    sleep(osl.dt)

    # Calculate the ticks per degree conversion value for the motor and joint
    bpdM = ((np.asarray(angExtM) - np.asarray(angFlexM))/np.asarray(romMot)).tolist()
    bpdJ = ((np.asarray(angExtJ) - np.asarray(angFlexJ))/np.asarray(romJoint)).tolist()

    # Print ticks per degree ratio to screen
    print('Bit to Degree Motor Ratio: ', bpdM)
    print('Bit to Degree Joint Ratio: ', bpdJ)
    sleep(0.5)
    print('Confirmation Test...')
    sleep(0.5)

    # Set boolean for tracking when to cut the motor
    runKnee = True
    runAnk = True
    run = True
    sleep(0.5)

    # Set motor to twice the calibration voltage for quicker run time
    FX.send_motor_command(devId[0],fxe.FX_VOLTAGE,abs(volt)*2)
    FX.send_motor_command(devId[1],fxe.FX_VOLTAGE,abs(volt)*2)

    # Grab encoder angle value (need uncalibrated value angVal for later)
    actDataKnee = FX.read_device(devId[0])
    actDataAnk  = FX.read_device(devId[1])

    motValKnee = actDataKnee.mot_ang
    jointValKnee = actDataKnee.ank_ang
    motValAnk = actDataAnk.mot_ang
    jointValAnk = actDataAnk.ank_ang

    motVal = [motValKnee,motValAnk]
    jointVal = [jointValKnee,jointValAnk]

    # Calculate the calibrated value of the current angle, convert to degrees
    # Convert calibrated value of the current angle to radians
    motCur = (np.asarray(angExtM) - np.asarray(motVal))/np.asarray(bpdM)
    motCurRad = np.multiply(motCur,osl.deg2rad)
    if 0 not in bpdJ:
        jointCur = (np.asarray(angExtJ) - np.asarray(jointVal))/np.asarray(bpdJ)
        jointCurRad = np.multiply(jointCur,osl.deg2rad)

    sleep(0.3)

    # Print calibrated angle in degrees and radians to screen
    if 0 not in bpdJ:
        print('%-15s %-7f %-7f %-15s %-7f %-7f' % ('Knee Mot/Joint',motCur[0],jointCur[0],'Ankle Mot/Joint',motCur[1],jointCur[1])
    else:
        print('%-15s %-7f %-15s %-7f' % ('Knee Mot:',motCur[0],'Ankle Mot:',motCur[1]))

    # Set motor to twice the calibration voltage for quicker run time
    FX.send_motor_command(devId[0],fxe.FX_VOLTAGE,abs(volt)*2)
    FX.send_motor_command(devId[1],fxe.FX_VOLTAGE,abs(volt)*2)

    while run:

        # Set Tracking Angle to Previously Measured Angle
        motPrev = motVal
        jointPrev = jointVal

        # Grab Current Encoder Value
        actDataKnee = FX.read_device(devId[0])
        actDataAnk = FX.read_device(devId[1])

        motValKnee = actDataKnee.mot_ang
        jointValKnee = actDataKnee.ank_ang
        motValAnk = actDataAnk.mot_ang
        jointValAnk = actDataAnk.ank_ang

        motVal = [motValKnee,motValAnk]
        jointVal = [jointValKnee,jointValAnk]

        # Calculate the calibrated value of the current angle, convert to degrees
        # Convert calibrated value of the current angle to radians
        motCur = (np.asarray(angExtM) - np.asarray(motVal))/np.asarray(bpdM)
        motCurRad = np.multiply(motCur,osl.deg2rad)
        if 0 not in bpdJ:
            jointCur = (np.asarray(angExtJ) - np.asarray(jointVal))/np.asarray(bpdJ)
            jointCurRad = np.multiply(jointCur,osl.deg2rad)

        # Calculate Difference between Tracking Angle and Current Angle
        angDiffM = np.asarray(motVal) - np.asarray(motPrev)

        print(motCurKnee,angDiffMKnee,jointCurKnee,angDiffJKnee,motCurAnk,angDiffMAnk,jointCurAnkangDiffJAnk)

        # Calculate calibrated encoder angle in degrees and radians
        motCur = (angExtM - motVal)/bpdM
        motCurRad = np.multiply(motCur,osl.deg2rad)
        if bpdJ != 0:
            jointCur = (angExtJ - jointVal)/bpdJ
            jointCurRad = np.multiply(jointCur,osl.deg2rad)

        # Calculate uncalibrated difference between tracking and current angle
        angDiffM = motVal - motPrev

        # Print calibrated angle in degrees and radians to screen
        if 0 not in bpdJ:
            print('%-15s %-7f %-7f %-15s %-7f %-7f' % ('Knee Mot/Joint',motCur[0],jointCur[0],'Ankle Mot/Joint',motCur[1],jointCur[1])
        else:
            print('%-15s %-7f %-15s %-7f' % ('Knee Mot:',motCur[0],'Ankle Mot:',motCur[1]))

        sleep(0.05)

        # If calculated difference is smaller than half a degree movement, the
        # limit has been reached
        if (abs(angDiffM[0]) <= deg2count/2) and (runKnee):

            # Set motor voltage to zero to stop the actuator
            FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, 0)

            # Set boolean to exit loop
            runKnee = False

        if (abs(angDiffM[1]) <= deg2count/2) and (runAnk):

            # Set motor voltage to zero to stop the actuator
            FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, 0)

            # Set boolean to exit loop
            runAnk = False

        if (not runKnee) and (not runAnk)
            run = False

    print('Angle Calibration Complete.')
    sleep(0.5)

    return angExtM,angFlexM,bpdM,angExtJ,angFlexJ,bpdJ

def main():

    '''
    For standalone calling of the angle calibration functions
    '''

    #import numpy as np
    thisdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.append(thisdir)

    # Find directory
    scriptPath = os.path.dirname(os.path.abspath(__file__))
    fpath = scriptPath + '/Ports_Dual.yaml'
    ports, baudRate = fxu.load_ports_from_file(fpath)

    # Standard setup that is not crucial for understanding the script
    print(ports,'\n',baudRate)
    port1 = str(ports[0])
    port2 = str(ports[1])
    baudRate=int(baudRate)
    debugLvl=6

    # Connect to actuator and open data stream
    FX = fx.FlexSEA()

    devId = FX.open(port1,baudRate,debugLvl)
    devId = [devId,FX.open(port2,baudRate,debugLvl)]
    FX.start_streaming(devId[0],freq=100,log_en=False)
    sleep(0.05)
    FX.start_streaming(devId[1],freq=100,log_en=False)
    sleep(0.05)

    calData = pac.CalDataDual(1)

    try:

        angM1,angM2,bpdM,angJ1,angJ2,bpdJ = angleCal(devId,FX)
        calData.angExtMot = [int(angM1[0]),int(angM1[1])]
        calData.angFlexMot = [int(angM2[0]),int(angM2[1])]

        calData.bpdMot = [float(bpdM[0]),float(bpdM[1])]

        calData.angExtJoint = [int(angJ1[0]),int(angJ1[1])]
        calData.angFlexJoint = [int(angJ2[0]),int(angJ2[1])]

        calData.bpdJoint = [float(bpdJ[0]),float(bpdJ[1])]

        calData.angVertJointAnk = calData.angExtJoint[1] - 15*calData.bpdJoint[1]
        calData.angVertMotAnk = calData.angExtMot[1] - 67*calData.bpdMot[1]

    except KeyboardInterrupt:

        print('User Interruption Occurred')

        # Disable the controller, send 0 PWM
        sleep(0.05)
        FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, 0)
        sleep(0.05)
        FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, 0)

    except Exception as error:

        print(error)

        # Disable the controller, send 0 PWM
        sleep(0.05)
        FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, 0)
        sleep(0.05)
        FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, 0)

    finally:

        FX.stop_streaming(devId[0])
        sleep(0.05)
        FX.stop_streaming(devId[1])
        sleep(0.1)
        FX.close(devId[0])
        sleep(0.05)
        FX.close(devId[1])
        print("Graceful Exit Complete")

if __name__ == '__main__':

    main()
