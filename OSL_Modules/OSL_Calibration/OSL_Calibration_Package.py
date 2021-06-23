'''
##################################### OPEN #####################################
This package is a wrapper for completing calibration of the Dephy actuators of the Open Source Leg (OSL).

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
    - Updated main function to use OSL_CalibrationFunctions_DeviceOpenClose for opening and closing devices
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_IMU as imu
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Angle as ang
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Homing as home
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Storage as stor
from OSL_Modules.OSL_4Bar_Dual import OSL_4BarFunctions_Mapping as four
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_DeviceOpenClose as opcl

############################# CALIBRATION CLASSES ##############################

class CalDataSingle:

    '''
    Class used for storing calibration data of a single actuator
    Structure:
        cal - Calibration Being Run (0: IMU only, 1: Angle only, 2+: Both)
        gyro - Z-Axis Gyroscope Bias
        xAccel - X-Axis Accelerometer Bias
        yAccel - Y-Axis Accelerometer Bias
        angExtMot - Motor Encoder Tick Value at Full Extension
        angFlexMot - Motor Encoder Tick Value at Full Flexion
        bpdMot - Ticks Per Degree Conversion Unit for Motor
        angExtJoint - Joint Encoder Tick Value at Full Extension
        angFlexJoint - Joint Encoder Tick Value at Full Flexion
        angVertJoint - Joint Encoder Tick Value at Vertical Orientation
        bpdJoint - Ticks Per Degree Conversion Unit for Joint
    '''

    # Init Method for Knee or Ankle Module
    def __init__(self, cal=2, gyro=0, xAccel=0, yAccel=0, angExtM=0, angFlexM=0, bpdM=0, angExtJ=0, angFlexJ=0, bpdJ=0):

        # If only worried about IMU, load IMU
        if cal == 0:

            self.gyro = gyro
            self.xAccel = xAccel
            self.yAccel = yAccel

        # If only worried about Angle, load Angle
        elif cal == 1:

            self.angExtMot = angExtM
            self.angFlexMot = angFlexM
            self.bpdMot = bpdM

            self.angExtJoint = angExtJ
            self.angFlexJoint = angFlexJ
            self.angVertJoint = self.angExtJoint - 15*bpdJ
            self.angVertMot = self.angExtMot - 15*bpdM
            self.bpdJoint = bpdJ

        # If no specification, load All
        else:

            self.gyro = gyro
            self.xAccel = xAccel
            self.yAccel = yAccel
            self.angExtMot = angExtM
            self.angFlexMot = angFlexM
            self.bpdMot = bpdM

            self.angExtJoint = angExtJ
            self.angFlexJoint = angFlexJ
            self.angVertJoint = self.angExtJoint - 15*bpdJ
            self.angVertMot = self.angExtMot - 15*bpdM
            self.bpdJoint = bpdJ

############################# FUNCTION DEFINITIONS #############################

def kneeCal(devId, FX, calData, cal = 2):

    '''
    Function for calibrating the knee module and storing the calibration data in a class object
    Inputs:
        devId - Device ID of actuator to calibrate
        FX - Class object with flexSEA Dephy functions
        calData - Class object to store calibration data in
        cal - Calibration variable to determine what data to calibrate (0 for IMU Only, 1 for Angle Only, 2+ for All Data)
    Outputs:
        calData - Class object to store calibration data in
    '''

    # If Angle Only is NOT Specified, Run IMU Calibration
    if cal != 1:

        # Calibration for Z-Axis Gyroscope
        calData.gyro = imu.gyroCal(devId, FX)

        # Calibration for X-Axis, Y-Axis Accelerometers
        calData.xAccel,calData.yAccel = imu.accelCal(devId, FX)

    # If IMU Only is NOT Specified, Run Angle Calibration
    if cal != 0:

        # Calibration for Motor, Joint Hard Stops and Bits Per Degree Ratios
        calData.angExtMot, calData.angFlexMot, calData.bpdMot, calData.angExtJoint, calData.angFlexJoint, calData.bpdJoint = ang.angleCal(devId, FX)

    # Let User Specify if Calibration Data Should Be Saved
    storeCheck = input('Store calibration data in .yaml file as well? [y/n]: ')

    # If Save Flag Set, Store Calibration Data
    if storeCheck in 'yes':

        # Store Calibration Date for Future Use
        stor.calDump(calData, devId)
        print('Data stored in Knee_Cal.yaml...')

    else:

        print('Data not stored in Knee_Cal.yaml...')

    # Return Calibration Data
    return calData

def ankleCalJoint(devId, FX, calData, cal=2):

    '''
    Function for calibrating the ankle module and storing the calibration data in a class object. This form uses the external ankle encoder
    Inputs:
        devId - Device ID of actuator to calibrate
        FX - Class object with flexSEA Dephy functions
        calData - Class object to store calibration data in
        cal - Calibration variable to determine what data to calibrate (0 for IMU Only, 1 for Angle Only, 2+ for All Data)
    Outputs:
        calData - Class object to store calibration data in
    '''

    # Checkpoint for Vertical Orientation of Ankle for IMU Calibration
    vertCheck = input('Stand Ankle Module Up, and Hit Enter To Continue (important if calibrating IMU data)...')

    # If IMU Only is NOT Specified, Run Angle Calibration
    if cal != 0:

        # Calibration for Motor, Joint Hard Stops and Bits Per Degree Ratios
        calData.angExtMot, calData.angFlexMot, calData.bpdMot, calData.angExtJoint, calData.angFlexJoint, calData.bpdJoint = ang.angleCal(devId, FX)

        # Calculate Vertical Orientation Joint Encoder
        calData.angVertJoint = calData.angExtJoint + 15*calData.bpdJoint

        # Home Ankle to Vertical Orientation, Calibrate Vertical Motor Encoder
        calData.angVertMot = home.ankleHomeJoint(devId, FX, calData.angVertJoint)

    # If Angle Only is NOT Specified, Run IMU Calibration
    if cal != 1:

        # Calibration for Z-Axis Gyroscope
        calData.gyro = imu.gyroCal(devId, FX)

        # Calibration for X-Axis, Y-Axis Accelerometers
        calData.xAccel, calData.yAccel = imu.accelCalVert(devId, FX)

    # Let User Specify if Calibration Data Should Be Saved
    storeCheck = input('Store calibration data in .yaml file as well? [y/n]: ')

    # If Save Flag Set, Store Calibration Data
    if storeCheck in 'yes':

        # Store Calibration Date for Future Use
        stor.calDump(calData, devId)
        print('Data stored in Ankle_Cal.yaml...')

    else:

        print('Data not stored in Ankle_Cal.yaml...')

    # Return Calibration Data
    return calData

def ankleCalMot(devId, FX, calData, cal=2):

    '''
    Function for calibrating the ankle module and storing the calibration data in a class object. This form uses the internal motor encoder
    Inputs:
        devId - Device ID of actuator to calibrate
        FX - Class object with flexSEA Dephy functions
        calData - Class object to store calibration data in
        cal - Calibration variable to determine what data to calibrate (0 for IMU Only, 1 for Angle Only, 2+ for All Data)
    Outputs:
        calData - Class object to store calibration data in
    '''

    # Checkpoint for Vertical Orientation of Ankle for IMU Calibration
    vertCheck = input('Stand Ankle Module Up, and Hit Enter To Continue...')

    # If IMU Only is NOT Specified, Run Angle Calibration
    if cal != 0:

        # Calibration for Motor, Joint Hard Stops and Bits Per Degree Ratios
        calData.angExtMot, calData.angFlexMot, calData.bpdMot, calData.angExtJoint, calData.angFlexJoint, calData.bpdJoint = ang.angleCal(devId, FX)

        # Calculate Vertical Orientation Motor Encoder
        calData.angVertMot = calData.angExtMot - 67*calData.bpdMot

        # Home Ankle to Vertical Orientation, Calibrate Vertical Joint Encoder
        calData.angVertJoint = home.ankleHomeMot(devId, FX, calData.angVertMot)

    # If Angle Only is NOT Specified, Run IMU Calibration
    if cal != 1:

        # Calibration for Z-Axis Gyroscope
        calData.gyro = imu.gyroCal(devId, FX)

        # Calibration for X-Axis, Y-Axis Accelerometers
        calData.xAccel, calData.yAccel = imu.accelCalVert(devId, FX)

    # Let User Specify if Calibration Data Should Be Saved
    storeCheck = input('Store calibration data in .yaml file as well? [y/n]: ')

    # If Save Flag Set, Store Calibration Data
    if storeCheck in 'yes':

        # Store Calibration Date for Future Use
        stor.calDump(calData, devId)
        print('Data stored in Ankle_Cal.yaml...')

    else:

        print('Data not stored in Ankle_Cal.yaml...')

    # Return Calibration Data
    return calData

############################# MAIN FUN DEFINITIONS #############################

def main(cal):

    '''
    For standalone calling of the angle calibration functions
    '''

    # Create Class Object for Actuator Commands
    FX = fx.FlexSEA()

    # Open Device ID and Start Streaming
    devId = opcl.devOpen(FX)

    # Initialize Calibration Data Class Object
    calData = CalDataSingle(cal)

    try:

        # Choose Appropriate Calibration Process Based On Device ID
        if devId == osl.devKnee:

            kneeCal(devId, FX, calData, cal)

        elif devId == osl.devAnk:

            ankleCal(devId, FX, calData, cal)

        else:

            raise RuntimeError('Invalid device ID. Check device ID and compare with OSL_Constants.py stored values.')

    except Exception as error:

        # Print Error
        print('Error Occurred')
        print(error)

    finally:

        # Gracefully Exit Script by Closing Stream and Device ID
        opcl.devClose(devId, FX)

if __name__ == '__main__':

    cal = int(input('What data needs calibrating? (0 for IMU, 1 for Angle, 2 for Both): '))
    main(cal)
