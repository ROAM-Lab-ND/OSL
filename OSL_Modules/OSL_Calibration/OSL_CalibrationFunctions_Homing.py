'''
##################################### OPEN #####################################
This package holds the functions called for homing the knee and ankle actuators of the Open Source Leg (OSL) to vertical orientation.

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
    - Updated delays to use parameters from OSL_Constants
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

# Imports for Standard Python
from time import sleep, time, strftime
import math
import numpy as np
import yaml

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_DeviceOpenClose as opcl

################################# CALIBRATION ##################################

def kneeHome(devId, FX, volt = 400):

    '''
    Function for returning knee joint of Open Source Leg (OSL) to vertical orientation safely
    Inputs:
        devId - Device ID of actuator
        FX - Class object with flexSEA Dephy functions for reading actuator data
        volt - Voltage value to run motor at
    Outputs:
        None
    '''

    # Alert User Actuator is About to be Run
    print('Running Actuator...')
    sleep(osl.dtDeci)

    # Send Motor Command
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, volt)

    # Read Current Motor Information and Grab Encoder
    actData = FX.read_device(devId)
    angCur = actData.mot_ang

    # Initialize Boolean for Calibration Sequence
    run = True

    while run:

        # Set Encoder Historical Data
        angPrev = angCur

        # Read Current Motor Information and Grab Encoder
        actData = FX.read_device(devId)
        angCur = actData.mot_ang

        # Calculate Difference Between Previous and Current Encoder
        angDiff = angCur - angPrev

        # Delay
        sleep(osl.dtCenti)

        # If Motor Rotation is Below Threshold, System is at Hard Stop
        if abs(angDiff) <= osl.deg2count/2:

            # Send Motor Command to Stop
            FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

            # Set Run Flag to False
            run = False
            print('Homing Complete')

def ankleHomeJoint(devId, FX, angVertJ, volt=400, valReturn=1):

    '''
    Function for returning ankle joint of Open Source Leg (OSL) to vertical orientation safely. Outputs vertical orientation motor encoder value if flagged. This form uses the external motor encoder
    Inputs:
        devId - Device ID of actuator
        FX - Class object with flexSEA Dephy functions for reading actuator data
        angVertJ - External encoder value at vertical orientation
        volt - Voltage value to run motor at
        valReturn - Flag whether to return vertical orientation motor encoder
    Outputs:
        angVertM - Motor encoder value at vertical orientation
    '''

    # Alert User Actuator is About to be Run
    print('Running Actuator...')
    sleep(osl.dtDeci)

    # Read Current Motor Information and Grab Encoder
    actData = FX.read_device(devId)
    angCur = actData.ank_ang

    # Set Appropriate Voltage Based On Current Angle
    if angCur < angVertJ:

        volt = abs(volt)

    else:

        volt = -abs(volt)

    # Initialize Boolean for Calibration Sequence
    run = True

    # Send Motor Command
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, volt)

    # Delay
    sleep(osl.dtCenti)

    while run:

        # Read Current Motor Information and Grab Encoder
        actData = FX.read_device(devId)
        motVal = actData.mot_ang
        jointVal = actData.ank_ang

        # Calculate Difference Between Vertical and Current Encoder
        jointDiff = jointVal - angVertJ

        # Print Information to User
        print('%-15s %-7f %-15s %-7f %-7f' % ('Current Motor:', motVal, 'Current Ankle:', jointVal, jointDiff))

        # Delay
        sleep(osl.dtCenti)

        # If Joint Angle is at Vertical Orientation, Stop
        if abs(jointDiff) <= 2*osl.deg2count:

            # Send Motor Command to Stop
            FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

            # Read Current Motor Information and Grab Encoder
            actData = FX.read_device(devId)
            angVertM = actData.mot_ang

            # Set Run Flag to False
            run = False
            print('Vertical Homing Complete')

    # If Value Flag Set, Return Vertical Orientation Motor Encoder
    if valReturn:

        return angVertM

def ankleHomeMot(devId,FX,angVertM,volt=400,valReturn=1):

    '''
    Function for returning ankle joint of Open Source Leg (OSL) to vertical orientation safely. Outputs vertical orientation joint encoder value if flagged. This form uses the internal motor encoder
    Inputs:
        devId - Device ID of actuator to encoder values at hard stop for
        FX - Class object with flexSEA Dephy functions for reading actuator data
        angVertM - Internal encoder value at vertical orientation
        volt - Voltage value to run motor at
        valReturn - Flag whether to return vertical orientation motor encoder
    Outputs:
        angVertJ - Joint encoder value at vertical orientation
    '''

    # Alert User Actuator is About to be Run
    print('Running Actuator...')
    sleep(osl.dtDeci)

    # Read Current Motor Information and Grab Encoder
    actData = FX.read_device(devId)
    angCur = actData.mot_ang

    # Set Appropriate Voltage Based On Current Angle
    if angCur < angVertM:

        volt = abs(volt)

    else:

        volt = -abs(volt)

    # Initialize Boolean for Calibration Sequence
    run = True

    # Send Motor Command
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, volt)

    # Delay
    sleep(osl.dtCenti)

    while run:

        # Read Current Motor Information and Grab Encoder
        actData = FX.read_device(devId)
        motVal = actData.mot_ang
        jointVal = actData.ank_ang

        # Calculate Difference Between Vertical and Current Encoder
        motDiff = motVal - angVertM

        # Print Information to User
        print('%-15s %-7f %-15s %-7f %-7f' % ('Current Motor:', motVal, 'Current Ankle:', jointVal, motDiff))

        # Delay
        sleep(osl.dtCenti)

        # If Motor Angle is at Vertical Orientation, Stop
        if abs(motDiff) <= 2*osl.deg2count:

            # Send Motor Command to Stop
            FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

            # Read Current Motor Information and Grab Encoder
            actData = FX.read_device(devId)
            angVertJ = actData.ank_ang

            # Set Run Flag to False
            run = False
            print('Vertical Homing Complete')

    # If Value Flag Set, Return Vertical Orientation Motor Encoder
    if valReturn:

        return angVertJ

############################# MAIN FUN DEFINITIONS #############################

def main():

    '''
    For standalone calling of the IMU calibration functions
    '''

    # Create Class Object for Actuator Commands
    FX = fx.FlexSEA()

    # Open Device ID and Start Streaming
    devId = opcl.devOpen(FX)

    try:

        if devId == osl.devKnee:

            pFile = open('Knee_Cal.yaml')
            calData = yaml.load(pFile, Loader=yaml.Loader)
            pFile.close()

            kneeHome(devId,FX)

        elif devId == osl.devAnk:

            pFile = open('Ankle_Cal.yaml')
            calData = yaml.load(pFile, Loader=yaml.Loader)
            pFile.close()

            ankleHome(devId, FX, calData.angVertJointAnk)

        else:

            raise RuntimeError('Invalid device ID. Check device ID and compare with OSL_Constants.py stored values.')

    except Exception as error:

        print('Error Occurred')
        print(error)

    finally:

        opcl.devClose(devId, FX)

if __name__ == '__main__':

    main()
