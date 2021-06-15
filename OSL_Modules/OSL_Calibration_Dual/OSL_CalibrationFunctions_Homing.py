'''
##################################### OPEN #####################################
This package holds the functions called for homing the knee and ankle actuators of the Open Source Leg (OSL) to vertical orientation.

Last Update: 8 June 2021
Updates:
    - Created dualHomeJoint for homing ankle joint with external encoder data
    - Updated dualHomeMot to correctly base ankDiffM off of vertical orientation value
    - Updated main() to use finally tag for proper syntax of closing actuator streams
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

from time import sleep, time, strftime
import math
import numpy as np
import yaml

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe
from OSL_Modules.OSL_Calibration import OSL_Constants as osl

################################# CALIBRATION ##################################

def kneeHome(devId,FX,volt=750):

    '''
    This function is used to return the knee actuator angle to the zero degree mark in a safe manner.

    Note that the zero degree mark is located at a mechanical hard stop for the knee.  Therefore, the voltage commanded to the motor should not change signs. The voltage command is kept relatively low to prevent extending beyond the mechanical hard stop and snapping back slightly short of zero degrees.
    '''

    print('Running Actuator...')
    sleep(0.3)

    # Start running motor at inputted voltage
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, volt)

    # Grab Current Encoder Value
    actData = FX.read_device(devId)
    angCur = actData.mot_ang

    # Boolean for tracking when to cut the motor
    run = True

    while run:

        sleep(0.1)

        # Set Tracking Angle to Previously Measured Angle
        angPrev = angCur

        # Grab Current Angle Value and Print to Screen
        actData = FX.read_device(devId)
        angCur = actData.mot_ang

        # Calculate Difference between Tracking Angle and Current Angle
        angDiff = angCur - angPrev

        # If calculated difference is smaller than half a degree movement, the
        # limit has been reached

        if abs(angDiff) <= osl.deg2count/2:

            # Set motor voltage to zero to stop the actuator
            FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

            # Grab final encoder value reading
            actData = FX.read_device(devId)
            angFinal = actData.mot_ang

            # Set boolean to exit loop
            run = False
            print('Homing Complete')

def ankleHomeJoint(devId,FX,angVertJ,volt=400,valReturn=1):

    '''
    This function is used to return the ankle actuator angle to the vertical degree mark in a safe manner.  Returns vertical angle motor value (ticks).

    NOTE: THIS SHOULD NOT BE CALLED BEFORE COMPLETING THE ANGLE CALIRATION FOR THE ANKLE. THE CALIBRATION FUNCTIONS CAN BE FOUND IN OSL_CalibrationFunctions_Angle.py
    '''

    print('Running Actuator...')
    sleep(0.3)

    # Grab Current Encoder Value
    actData = FX.read_device(devId)
    angCur = actData.ank_ang

    if angCur < angVertJ:

        volt = abs(volt)
    else:

        volt = -abs(volt)


    # Start running motor at inputted voltage
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, volt)

    while run:

        sleep(0.1)

        # Set motor to twice the calibration voltage for quicker run time
        FX.send_motor_command(devId,fxe.FX_VOLTAGE,volt)

        # Grab encoder angle value
        actData = FX.read_device(devId)
        motVal = actData.mot_ang
        jointVal = actData.ank_ang

        # Calculate uncalibrated difference between tracking and current angle
        jointDiff = jointVal - angVertJ

        # Print current calibrated angle value in degrees and radians to screen
        print('%-15s %-7f %-15s %-7f %-7f' % ('Current Motor:',motVal,'Current Ankle:',jointVal,jointDiff))

        # If calculated difference is smaller than half a degree movement, the
        # limit has been reached
        if abs(jointDiff) <= 2*osl.deg2count:

            # Set motor voltage to zero to stop the motor
            FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

            # Grab encoder angle value (need uncalibrated value angVal for later)
            actData = FX.read_device(devId)
            angVertM = actData.mot_ang

            # Set boolean to exit the loop
            run = False
            print('Vertical Homing Complete')

    if valReturn:
        return angVertM

def ankleHomeMot(devId,FX,angVertM,volt=400,valReturn=1):

    '''
    This function is used to return the ankle actuator angle to the vertical degree mark in a safe manner. Returns vertical angle joint value (ticks).

    NOTE: THIS SHOULD NOT BE CALLED BEFORE COMPLETING THE ANGLE CALIRATION FOR THE ANKLE. THE CALIBRATION FUNCTIONS CAN BE FOUND IN OSL_CalibrationFunctions_Angle.py
    '''

    print('Running Actuator...')
    sleep(0.3)

    # Grab Current Encoder Value
    actData = FX.read_device(devId)
    angCur = actData.mot_ang

    if angCur < angVertM:

        volt = abs(volt)

    else:

        volt = -abs(volt)


    # Start running motor at inputted voltage
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, volt)

    sleep(0.05)

    while run:

        # Set motor to twice the calibration voltage for quicker run time
        FX.send_motor_command(devId,fxe.FX_VOLTAGE,volt)

        # Grab encoder angle value
        actData = FX.read_device(devId)
        motVal = actData.mot_ang

        # Calculate uncalibrated difference between tracking and current angle
        motDiff = motVal - angVertM

        # Print current calibrated angle value in degrees and radians to screen
        print('%-15s %-7f %-7f' % ('Current Motor:',motVal,motDiff))

        sleep(0.05)

        # If calculated difference is smaller than half a degree movement, the
        # limit has been reached
        if abs(motDiff) <= 2*osl.deg2count:

            # Set motor voltage to zero to stop the motor
            FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

            # Grab encoder angle value (need uncalibrated value angVal for later)
            actData = FX.read_device(devId)
            angVertJ = actData.ank_ang

            # Set boolean to exit the loop
            run = False
            print('Vertical Homing Complete')

    if valReturn:
        return angVertJ

def dualHomeMot(devId,FX,angVertM,valReturn=1,volt=400):

    print('Running Actuators...')
    sleep(0.3)

    # Start running motor at inputted voltage for knee
    FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, abs(volt))

    # Grab Current Encoder Value for Ankle
    actDataAnk = FX.read_device(devId[1])
    angCurAnk = actDataAnk.mot_ang

    if angCurAnk < angVertM:

        volt = abs(volt)

    else:

        volt = -abs(volt)

    # Start running motor at inputted voltage for knee
    FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, abs(volt))

    # Grab Current Encoder Value
    actDataKnee = FX.read_device(devId[0])
    actDataAnk = FX.read_device(devId[1])

    angCurKnee = actDataKnee.mot_ang
    angCurAnk = actDataAnk.mot_ang

    # Boolean for tracking when to cut the motor
    runKnee = True
    runAnk = True
    run = True

    sleep(0.05)

    while run:

        # Set Tracking Angle to Previously Measured Angle
        angPrevKnee = angCurKnee
        angPrevAnk = angCurAnk

        # Grab Current Encoder Value
        actDataKnee = FX.read_device(devId[0])
        actDataAnk = FX.read_device(devId[1])

        angCurKnee = actDataKnee.mot_ang
        angCurAnk = actDataAnk.mot_ang

        # Calculate Difference between Tracking Angle and Current Angle
        angDiffKnee = angCurKnee - angPrevKnee
        angDiffAnk = angCurAnk - angVertM

        sleep(0.05)

        # If calculated difference is smaller than half a degree movement, the
        # limit has been reached
        if (abs(angDiffKnee) <= osl.deg2count/2) and (runKnee):

            # Set motor voltage to zero to stop the actuator
            FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, 0)

            # Grab final encoder value reading
            actDataKnee = FX.read_device(devId[0])
            angFinalKnee = actDataKnee.mot_ang

            # Set boolean to exit loop
            runKnee = False

        if (abs(angDiffAnk) <= 2*osl.deg2count) and (runAnk):

            # Set motor voltage to zero to stop the actuator
            FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, 0)

            # Grab final encoder value reading
            actDataAnk = FX.read_device(devId[1])
            angFinalMAnk = actDataAnk.mot_ang
            angFinalJAnk = actDataAnk.ank_ang

            # Set boolean to exit loop
            runAnk = False

        if (not runKnee) and (not runAnk):

            run = False

    if valReturn:
        return angFinalJAnk

def dualHomeJoint(devId,FX,angVertJ,valReturn=1,volt=400):

    print('Running Actuators...')
    sleep(0.3)

    # Start running motor at inputted voltage for knee
    FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, abs(volt))

    # Grab Current Encoder Value for Ankle
    actDataAnk = FX.read_device(devId[1])
    angCurAnk = actDataAnk.ank_ang

    if angCurAnk < angVertJ:

        volt = abs(volt)

    else:

        volt = -abs(volt)

    # Start running motor at inputted voltage for knee
    FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, abs(volt))

    # Grab Current Encoder Value
    actDataKnee = FX.read_device(devId[0])
    actDataAnk = FX.read_device(devId[1])

    angCurKnee = actDataKnee.mot_ang
    angCurAnk = actDataAnk.ank_ang

    # Boolean for tracking when to cut the motor
    runKnee = True
    runAnk = True
    run = True

    sleep(0.05)

    while run:

        # Set Tracking Angle to Previously Measured Angle
        angPrevKnee = angCurKnee
        angPrevAnk = angCurAnk

        # Grab Current Encoder Value
        actDataKnee = FX.read_device(devId[0])
        actDataAnk = FX.read_device(devId[1])

        angCurKnee = actDataKnee.mot_ang
        angCurAnk = actDataAnk.ank_ang

        # Calculate Difference between Tracking Angle and Current Angle
        angDiffKnee = angCurKnee - angPrevKnee
        angDiffAnk = angCurAnk - angVertJ

        sleep(0.05)

        # If calculated difference is smaller than half a degree movement, the
        # limit has been reached
        if (abs(angDiffKnee) <= osl.deg2count/2) and (runKnee):

            # Set motor voltage to zero to stop the actuator
            FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, 0)

            # Grab final encoder value reading
            actDataKnee = FX.read_device(devId[0])
            angFinalKnee = actDataKnee.mot_ang

            # Set boolean to exit loop
            runKnee = False

        if (abs(angDiffAnk) <= 2*osl.deg2count) and (runAnk):

            # Set motor voltage to zero to stop the actuator
            FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, 0)

            # Grab final encoder value reading
            actDataAnk = FX.read_device(devId[1])
            angFinalMAnk = actDataAnk.mot_ang
            angFinalJAnk = actDataAnk.ank_ang

            # Set boolean to exit loop
            runAnk = False

        if (not runKnee) and (not runAnk):

            run = False

    if valReturn:
        return angFinalMAnk

def main(dev):

    '''
    For standalone calling of the Homing functions
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
    baudRate=int(baudRate)
    debugLvl=6

    # Connect to actuator and open data stream
    FX = fx.FlexSEA()

    devId1 = FX.open(port1,baudRate,debugLvl)
    devId2 = FX.open(port2,baudRate,debugLvl)

    if devId1 == 19048:
        devId = [devId1,devId2]
    else:
        devId = [devId2,devId1]

    FX.start_streaming(devId[0],freq=100,log_en=False)
    FX.start_streaming(devId[1],freq=100,log_en=False)
    sleep(0.1)

    try:

        if dev == 0:

            pFile = open('Knee_Cal.yaml')
            calData = yaml.load(pFile, Loader=yaml.Loader)
            pFile.close()
            kneeHome(devId[0],FX)

        elif dev == 1:

            pFile = open('Ankle_Cal.yaml')
            calData = yaml.load(pFile, Loader=yaml.Loader)
            pFile.close()
            ankleHomeMot(devId[1],FX,calData.angVertMotAnk,valReturn=0)

        elif dev == 2:

            pFile = open('Dual_Cal.yaml')
            calData = yaml.load(pfile, Loader=yaml.Loader)
            pFile.close()
            dualHomeMot(devId,FX,calData.angVertMotAnk,valReturn=0)

        else:

            raise Exception('Invalid choice for joint chosen')

    except Exception as error:
        
        print('Error Occurred')
        print(error)

        sleep(0.05)
        # Disable the controller, send 0 PWM
        FX.send_motor_command(devId[0], fxe.FX_VOLTAGE, 0)
        sleep(0.05)
        FX.send_motor_command(devId[1], fxe.FX_VOLTAGE, 0)
        sleep(0.05)

    finally:

        FX.stop_streaming(devId[0])
        sleep(0.05)
        FX.stop_streaming(devId[1])
        sleep(0.05)
        FX.close(devId[0])
        sleep(0.05)
        FX.close(devId[1])
        sleep(0.05)
        print("Graceful Exit Complete")

if __name__ == '__main__':

    dev = int(input('Which joint do you want to home? (0 for Knee, 1 for Ankle, 2 for Both): '))
    main(dev)