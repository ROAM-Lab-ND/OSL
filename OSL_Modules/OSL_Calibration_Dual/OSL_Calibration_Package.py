'''
##################################### OPEN #####################################
This package is a wrapper for completing calibration of the Dephy actuators of the Open Source Leg (OSL) for knee-ankle dual activity.

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration_Dual import OSL_Constants as osl
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_IMU as imu
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Angle as ang
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Homing as home
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Storage as stor
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_DeviceOpenClose as opcl

############################# CALIBRATION CLASSES ##############################

class CalDataDual:

    '''
    Class used for storing calibration data of dual actuators
    Structure:
        cal - Calibration Being Run (0: IMU only, 1: Angle only, 2+: Both)
        gyro - Z-Axis Gyroscope Bias
        xAccel - X-Axis Accelerometer Bias
        yAccel - Y-Axis Accelerometer Bias
        angExtMot - Motor Encoder Tick Value at Full Extension
        angFlexMot - Motor Encoder Tick Value at Full Flexion
        angExtJoint- Joint Encoder Tick Value at Full Extension
        angFlexJoint - Joint Encoder Tick Value at Full Flexion
        angVertMot - Motor Encoder Tick Value at Vertical Ankle Orientation
        angVertJoint - Joint Encoder Tick Value at Vertical Ankle Orientation
        bpdMot - Ticks Per Degree Conversion Unit for Motor
        bpdJoint - Ticks Per Degree Conversion Unit for Joint
    '''

    def __init__(self, cal=2, gyro=0, xAccel=0, yAccel=0, angExtMK=0, angFlexMK=0, angExtMA=0, angFlexMA=0, angExtJM=0, angFlexJM=0, angExtJA=0, angFlexJA=0, bpdMK=0, bpdMA=0, bpdJK=0, bpdJA=0):

        # If only worried about IMU, load IMU
        if cal == 0:

            self.gyro = gyro

            self.xAccel = xAccel
            self.yAccel = yAccel

        # If only worried about Angle, load Angle
        elif cal == 1:

            self.angExtMot = [angExtMK, angExtMA]
            self.angFlexMot = [angFlexMK, angFlexMA]

            self.angExtJoint = [angExtJM, angExtJA]
            self.angFlexJoint = [angFlexJM, angFlexJA]

            self.angVertJoint = self.angExtJoint[1] + 15*bpdJA
            self.angVertMot = self.angExtMot[1] + 67*bpdMA

            self.bpdMot = [bpdMK, bpdMA]
            self.bpdJoint = [bpdJK, bpdJA]


        # If no specification, load All
        else:

            self.gyro = gyro

            self.xAccel = xAccel
            self.yAccel = yAccel

            self.angExtMot = [angExtMK, angExtMA]
            self.angFlexMot = [angFlexMK, angFlexMA]

            self.angExtJoint = [angExtJM, angExtJA]
            self.angFlexJoint = [angFlexJM, angFlexJA]

            self.angVertJoint = self.angExtJoint[1] - 15*bpdJA
            self.angVertMot = self.angExtMot[1] - 67*bpdMA

            self.bpdMot = [bpdMK, bpdMA]
            self.bpdJoint = [bpdJK, bpdJA]

############################# FUNCTION DEFINITIONS #############################

def dualCalMot(devId, FX, calData, cal=2):

    '''
    Function for calibrating the knee and ankle module of the Open Source Leg (OSL) and storing the calibration data in a class object. This form uses the internal ankle encoder.
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
        calData.gyro = imu.gyroCal(devId[0], FX)

        # Calibration for X-Axis, Y-Axis Calibration
        calData.xAccel,calData.yAccel = imu.accelCal(devId[0], FX)

    # If IMU Only is NOT Specified, Run Angle Calibration
    if cal != 0:

        # Calibration for Motor, Joint Hard Stops and Bits Per Degree Ratios
        angExtM, angFlexM, bpdM, angExtJ, angFlexJ, bpdJ = ang.angleCal(devId, FX)

        calData.angExtMot = [angExtM[0], angExtM[1]]
        calData.angFlexMot = [angFlexM[0], angFlexM[1]]
        calData.bpdMot = [bpdM[0], bpdM[1]]
        calData.angExtJoint = [angExtJ[0], angExtJ[1]]
        calData.angFlexJoint = [angFlexJ[0], angFlexJ[0]]
        calData.bpdJoint = [bpdJ[0], bpdJ[1]]

        # Calculate Vertical Orientation Motor Encoder
        calData.angVertMot = calData.angExtMot[1] - 67*calData.bpdMot[1]

        # Home Ankle to Vertical Orientation, Calibrate Vertical Joint Encoder
        calData.angVertJoint = home.ankleHomeMot(devId[1], FX, calData.angVertMot)

    # Let User Specify if Calibration Data Should Be Saved
    storeCheck = input('Store calibration data in Dual_Cal.yaml file as well? [y/n]: ')

    # If Save Flag Set, Store Calibration Data
    if storeCheck in 'yes':

        # Store Calibration Date for Future Use
        stor.calDump(calData)
        print('Data stored in Dual_Cal.yaml...')

    else:

        print('Data not stored in Dual_Cal.yaml...')

    # Return Calibration Data
    return calData

def dualCalJoint(devId, FX, calData, cal=2):

    '''
    Function for calibrating the knee and ankle module of the Open Source Leg (OSL) and storing the calibration data in a class object. This form uses the external ankle encoder.
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
        calData.gyro = imu.gyroCal(devId[0], FX)

        # Calibration for X-Axis, Y-Axis Calibration
        calData.xAccel,calData.yAccel = imu.accelCal(devId[0], FX)

    # If IMU Only is NOT Specified, Run Angle Calibration
    if cal != 0:

        # Calibration for Motor, Joint Hard Stops and Bits Per Degree Ratios
        angExtM, angFlexM, bpdM, angExtJ, angFlexJ, bpdJ = ang.angleCal(devId, FX)

        calData.angExtMot = [angExtM[0], angExtM[1]]
        calData.angFlexMot = [angFlexM[0], angFlexM[1]]
        calData.bpdMot = [bpdM[0], bpdM[1]]
        calData.angExtJoint = [angExtJ[0], angExtJ[1]]
        calData.angFlexJoint = [angFlexJ[0], angFlexJ[0]]
        calData.bpdJoint = [bpdJ[0], bpdJ[1]]

        # Calculate Vertical Orientation Joint Encoder
        calData.angVertJoint = calData.angExtJoint[1] - 15*calData.bpdJoint[1]

        # Home Ankle to Vertical Orientation, Calibrate Vertical Motor Encoder
        calData.angVertMot = home.ankleHomeJoint(devId[1], FX, calData.angVertJoint)

    # Let User Specify if Calibration Data Should Be Saved
    storeCheck = input('Store calibration data in Dual_Cal.yaml file as well? [y/n]: ')

    # If Save Flag Set, Store Calibration Data
    if storeCheck in 'yes':

        # Store Calibration Date for Future Use
        stor.calDump(calData)
        print('Data stored in Dual_Cal.yaml...')

    else:

        print('Data not stored in Dual_Cal.yaml...')

    # Return Calibration Data
    return calData

############################# MAIN FUN DEFINITIONS #############################

def main():

    '''
    For standalone calling of the angle calibration functions
    '''

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
    port = str(ports[0])
    baudRate=int(baudRate)
    debugLvl=6

    # Grab Class Object for Dephy Actuator Functions
    FX = fx.FlexSEA()

    # Open device and begin streaming data
    devId = FX.open(port,baudRate,debugLvl)
    FX.start_streaming(devId,freq=100,log_en=False)
    sleep(0.1)

    calData = CalDataDual(cal)

    try:
        dualCalMot(devId,FX,calData,cal)
    except Exception as error:
        print('Error Occurred')
        print(error)

        # Disable the controller, send 0 PWM
        sleep(0.05)
        FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)
        sleep(0.1)

        FX.stop_streaming(devId)
        sleep(0.2)
        FX.close(devId)
        sleep(0.1)
        print("Graceful Exit Complete")
    '''

if __name__ == '__main__':

    print('Standalone use of this module is not yet supported.')
    '''
    cal = int(input('What data needs calibrating? (0 for IMU, 1 for Angle, 2 for Both): '))
    main(cal)
    '''
