'''
##################################### OPEN #####################################
This package holds the functions called by OSL_Calibration_Package.py to calibrate the Angles in the Dephy actuator

Last Update: 16 June 2021
Updates:
    - Updated bpdJ calculation to set value to -2 if no external encoder is attached to avoid issues with division by zero.
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

from time import sleep, time, strftime
import os, sys
import math
import numpy as np
import scipy as sp
import yaml

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe
from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_Calibration_Package as pac

################################# CALIBRATION ##################################
def angleZero(devId,FX,volt):

    '''
    This function is called by angleCal to determine the Encoder Values at Limit Points of the Knee (0 Degrees and 120 Degrees) or Ankle (0 Degrees and 30 Degrees)
    '''

    print('Running Actuator...')
    sleep(1)

    # Start running motor at inputted voltage
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, volt)

    # Grab Current Encoder Value
    actData = FX.read_device(devId)
    motCur = actData.mot_ang
    jointCur = actData.ank_ang
    sleep(0.3)

    # Print Encoder Value to Screen
    print(motCur,jointCur)

    # Boolean for tracking when to cut the motor
    run = True

    while run:

        sleep(0.1)

        # Set Tracking Angle to Previously Measured Angle
        motPrev = motCur
        jointPrev = jointCur

        # Grab Current Angle Value and Print to Screen
        actData = FX.read_device(devId)
        motCur = actData.mot_ang
        jointCur = actData.ank_ang

        # Calculate Difference between Tracking Angle and Current Angle
        angDiffM = motCur - motPrev
        angDiffJ = jointCur - jointPrev

        print(motCur,angDiffM,jointCur,angDiffJ)

        # If calculated difference is smaller than half a degree movement, the
        # limit has been reached
        if abs(angDiffM) <= osl.deg2count/2:

            # Print difference and ticks per degree to screen
            print('angDiff: ', angDiffM)
            print('degToCount/2: ', osl.deg2count/2)

            # Set motor voltage to zero to stop the actuator
            FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

            # Grab final encoder value reading
            actData = FX.read_device(devId)
            motFinal = actData.mot_ang
            jointFinal = actData.ank_ang

            # Set boolean to exit loop
            run = False
            print('Completed Min/Max')

            # Return Calibration Value for Encoders for future use
            return motFinal,jointFinal

def angleCal(devId,FX,romJoint,volt=750):

    '''
    This function is used to determine the calibration values of the joint angle for the Knee or Ankle actuator
    '''

    if romJoint==30:
        romMot=124
    else:
        romMot=romJoint

    print('Running Actuator to Min and Max...')
    sleep(1)

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
    bpdM = (angExtM - angFlexM)/romMot
    bpdJ = (angExtJ - angFlexJ)/romJoint

    if bpdJ == 0:

        bpdJ = -2

    # Print ticks per degree ratio to screen
    print('Bit to Degree Motor Ratio: ', bpdM)
    print('Bit to Degree Joint Ratio: ', bpdJ)
    sleep(0.5)
    print('Confirmation Test...')
    sleep(0.5)

    # Set boolean for tracking when to cut the motor
    run = True
    print('%-15s %-7s %-7s %-15s %-7s %-7s' % ('Current Motor: ','Degrees','Radians','Current Joint: ','Degrees','Radians'))
    sleep(0.5)

    # Set motor to twice the calibration voltage for quicker run time
    FX.send_motor_command(devId,fxe.FX_VOLTAGE,volt*2)

    # Grab encoder angle value (need uncalibrated value angVal for later)
    actData = FX.read_device(devId)
    motVal = actData.mot_ang
    jointVal = actData.ank_ang

    # Calculate the calibrated value of the current angle, convert to degrees
    # Convert calibrated value of the current angle to radians
    motCur = (angExtM - motVal)/bpdM
    motCurRad = np.multiply(motCur,osl.deg2rad)

    jointCur = (angExtJ - jointVal)/bpdJ
    jointCurRad = np.multiply(jointCur,osl.deg2rad)

    sleep(0.3)

    # Print calibrated angle in degrees and radians to screen
    print('%-15s %-7f %-7f %-15s %-7f %-7f' % ('Current Motor: ', motCur, motCurRad, 'Current Ankle:', jointCur, jointCurRad))

    while run:

        # Set tracking angle to previous uncalibrated value
        motPrev = motVal
        jointPrev = jointVal

        # Set motor to twice the calibration voltage for quicker run time
        FX.send_motor_command(devId,fxe.FX_VOLTAGE,volt*2)

        # Grab encoder angle value
        actData = FX.read_device(devId)
        motVal = actData.mot_ang
        jointVal = actData.ank_ang

        # Calculate calibrated encoder angle in degrees and radians
        motCur = (angExtM - motVal)/bpdM
        motCurRad = np.multiply(motCur,osl.deg2rad)

        jointCur = (angExtJ - jointVal)/bpdJ
        jointCurRad = np.multiply(jointCur,osl.deg2rad)

        # Calculate uncalibrated difference between tracking and current angle
        angDiffM = motVal - motPrev

        # Print current calibrated angle value in degrees and radians to screen
        print('%-15s %-7f %-7f %-15s %-7f %-7f' % ('Current Motor:',motCur,motCurRad,'Current Ankle:',jointCur,jointCurRad))
        sleep(0.05)

        # If calculated difference is smaller than half a degree movement, the
        # limit has been reached
        if abs(angDiffM) <= osl.deg2count/2:

            # Set motor voltage to zero to stop the motor
            FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

            # Set boolean to exit the loop
            run = False
            print('Completed Test Run')

    print('Angle Calibration Complete.')
    sleep(0.5)

    # Return encoder value at each limit point and measured conversion value
    return angExtM,angFlexM,bpdM,angExtJ,angFlexJ,bpdJ


def main(dev):

    '''
    For standalone calling of the angle calibration functions
    '''

    try:
        if dev == 0:
            rangeOfMotion = 120
        elif dev == 1:
            rangeOfMotion = 30
        else:
            raise Exception('Invalid joint chosen')

    except Exception as error:

        print('Error occurred')
        print(error)
        raise

    #import numpy as np
    thisdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.append(thisdir)

    # Find directory
    scriptPath = os.path.dirname(os.path.abspath(__file__))
    fpath = scriptPath + '/Ports_Single.yaml'
    ports, baudRate = fxu.load_ports_from_file(fpath)

    # Standard setup that is not crucial for understanding the script
    print(ports,'\n',baudRate)
    port = str(ports[0])
    baudRate=int(baudRate)
    debugLvl=6

    # Connect to actuator and open data stream
    FX = fx.FlexSEA()

    devId = FX.open(port,baudRate,debugLvl)
    FX.start_streaming(devId,freq=100,log_en=False)
    sleep(0.1)

    calData = pac.CalDataSingle(1,dev)

    try:

        angM1,angM2,bpdM,angJ1,angJ2,bpdJ = angleCal(devId,FX,rangeOfMotion)
        calData.angExtMot = angM1
        calData.angFlexMot = angM2
        calData.bpdMot = bpdM

        if rangeOfMotion == 30:
            calData.angExtJoint = angJ1
            calData.angFlexJoint = angJ2
            calData.bpdJoint = bpdJ
            calData.angVertJoint = calData.angExtJoint + 20*calData.bpdJoint

    except Exception as error:

        print('Error Occurred')
        print(error)

    finally:

        # Disable the controller, send 0 PWM
        sleep(0.05)
        FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)
        sleep(0.1)

        FX.stop_streaming(devId)
        sleep(0.2)
        FX.close(devId)
        sleep(0.1)
        print("Graceful Exit Complete")

if __name__ == '__main__':

    dev = int(input('Which joint is this for? (0 for Knee, 1 for Ankle): '))
    main(dev)
