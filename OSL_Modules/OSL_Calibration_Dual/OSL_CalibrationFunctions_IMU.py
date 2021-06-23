'''
##################################### OPEN #####################################
This package holds the functions called by OSL_Calibration_Package.py to calibrate the IMU sensor in the Dephy actuator for dual actuator situations

NOTE: The functions gyroCal and accelCal are identical to the single actuator functions of the same name. The gyroscope and accelerometer data currently only pertains to use with the knee actuator.  If future work dictates that IMU data is needed for the ankle actuator as well, this will result in a restructure then.

Last Update: 23 June 2021
Updates:
    - Changed calibration processes to be completed on raw bits
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

def gyroCal(devId, FX):

    '''
    Function for calibrating z-axis gyroscope data for knee or ankle of Open Source Leg (OSL).
    Inputs:
        devId - Device ID of actuator to encoder values at hard stop for
        FX - Class object with flexSEA Dephy functions for reading actuator data
    Outputs:
        gyroBias - Calibrated bias of z-axis gyroscope sensor
    '''

    # Alert User to Not Move Actuator
    print('Please keep actuator still')
    print('%-8s %-15s %-15s' % ('Count', 'Gyro Z Value', 'Running Average'))
    sleep(3*osl.dtDeci)

    # Initialize Counter and Average
    count = 0
    gyroBias = 0

    while count < (3*osl.sec10)/osl.dtMilli:

        # Update Counter
        count += 1

        # Read Current Motor Information and Grab Gyroscope
        actData = FX.read_device(devId)
        gyroZ = actData.gyroz

        # Calculate Running Average Bias
        gyroBias = (gyroBias*(count - 1) + gyroZ)/count

        # Print Information to User
        print('%-8i %-15f %-15f' % (count, gyroZ, gyroBias))

        # Delay
        sleep(osl.dtMilli)

    print('Running Confirmation Test (Converted to Rad/Sec)')
    print('%-8s %-15s %-15s' % ('Count', 'Gyro Z Value', 'Average'))
    sleep(5*osl.dtDeci)

    # Initialize Counter and Average
    count = 0
    calAvg = 0

    while count < (osl.sec10)/osl.dtMilli:

        # Update Counter
        count += 1

        # Read Current Motor Information and Grab Gyroscope
        actData = FX.read_device(devId)
        gyroZ = actData.gyroz

        # Calculate Bias Corrected Gyroscope and Average
        calCur = ((gyroZ - gyroBias)*osl.deg2rad)/osl.gyroConv
        calAvg = (calAvg*(count - 1) + calCur)/count

        # Print Information to User
        print('%-8i %-15f %-15f' % (count, calCur, calAvg))

        # Delay
        sleep(osl.dtMilli)

    print('Gyro Calibration Complete')

    # Return Gyroscope Calibration
    return gyroBias

def accelCal(devId, FX):

    '''
    Function for calibrating x-axis, y-axis accelerometer data for knee of Open Source Leg (OSL).
    Inputs:
        devId - Device ID of actuator to encoder values at hard stop for
        FX - Class object with flexSEA Dephy functions for reading actuator data
    Outputs:
        xBias - Calibrated bias of x-axis accelerometer sensor
        yBias - Calibrated bias of y-axis acceleromater sensor
    '''

    # Alert User to Not Move Actuator
    print('Please keep actuator still')
    print('%-5s %-10s %-10s %-10s %-10s' % ('Count', 'AccelX', 'X Bias', 'AccelY', 'Y Bias'))
    sleep(3*osl.dtDeci)

    # Initialize Counter and Averages
    count = 0
    xBias = 0
    yBias = 0

    while count < (3*osl.sec10)/osl.dtMilli:

        # Update Counter
        count += 1

        # Read Current Motor Information and Grab x-Axis, y-Axis Accelerometer
        actData = FX.read_device(devId)
        xAccel = actData.accelx
        yAccel = actData.accely

        # Calculate Running Average Bias
        xBias = (xBias*(count - 1) + xAccel)/count
        yBias = (yBias*(count - 1) + yAccel)/count

        # Print Information to User
        print('%-5i %-10f %-10f %-10f %-10f' % (count, xAccel, xBias, yAccel, yBias))

        # Delay
        sleep(osl.dtMilli)

    print('Running Confirmation Test (Converted to Gs)')
    print('%-5s %-10s %-10s' % ('Count', 'AccelX', 'AccelY'))
    sleep(5*osl.dtDeci)

    # Initialize Counter
    count = 0

    while count < (osl.sec10)/osl.dtMilli:

        # Update Counter
        count += 1

        # Read Current Motor Information and Grab x-Axis, y-Axis Accelerometer
        actData = FX.read_device(devId)
        xAccel = actData.accelx
        yAccel = actData.accely

        # Calculate Bias Corrected x-Axis, y-Axis Accelerometer
        xCal = (xAccel - xBias)/osl.accelConv
        yCal = (yAccel - yBias)/osl.accelConv

        # Print Information to User
        print('%-5i %-10f %-10f' % (count, xCal, yCal))

        # Delay
        sleep(osl.dtMilli)

    print('Accel Calibration Complete')

    # Return Acceleromater Calibration
    return xBias, yBias

############################# MAIN FUN DEFINITIONS #############################

def main():

    '''
    For standalone calling of the IMU calibration functions
    '''

    # Create Class Object for Actuator Commands
    FX = fx.FlexSEA()

    # Open Device ID and Start Streaming
    devId = opcl.devOpen(FX)

    calData = pac.CalDataSingle()

    try:

        calData.gyro = gyroCal(devId[0], FX)

        xBias,yBias = accelCal(devId[0], FX)

        calData.xAccel = xBias
        calData.yAccel = yBias

    except Exception as error:

        print('Error Occurred')
        print(error)

    finally:

        opcl.devClose(devId, FX)

if __name__ == '__main__':

    main()
