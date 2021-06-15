'''
##################################### OPEN #####################################
This package is a wrapper for completing calibration of the Dephy actuators of the Open Source Leg (OSL).

Last Update: 8 June 2021
Updates:
    - Updated import file path location for OSL_Calibration
    - Updated import file path location for OSL_4Bar
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_IMU as imu
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Angle as ang
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Homing as home
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Storage as stor
from OSL_Modules.OSL_4Bar_Dual import OSL_4BarFunctions_Mapping as four

############################# CALIBRATION CLASSES ##############################

'''
First Class Can Be Called For Either Knee or Ankle Single Actuator
Second Class Can Be Called For Dual Actuator Systems
'''

class CalDataSingle:

    '''
    Class used for storing calibration data of a single actuator
    Structure:
        cal - Calibration Being Run (0: IMU only, 1: Angle only, 2+: Both)
        dev - Joint Being Calibrated (0: Knee, 1: Ankle, 2+: Unspecified)
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
    def __init__(self,cal=2,dev=2,gyro=0,xAccel=0,yAccel=0,angExtM=0,angFlexM=0,bpdM=0,angExtJ=0,angFlexJ=0,bpdJ=0):

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

            # If working with device other than knee, load Joint Angle
            if dev != 0:

                self.angExtJoint = angExtJ
                self.angFlexJoint = angFlexJ
                self.angVertJoint = self.angExtJoint+20*bpdJ
                self.angVertMot = self.angExtMot+20*self.bpdMot
                self.bpdJoint = bpdJ

        # If no specification, load All
        else:
            self.gyro = gyro
            self.xAccel = xAccel
            self.yAccel = yAccel
            self.angExtMot = angExtM
            self.angFlexMot = angFlexM
            self.bpdMot = bpdM

            # If working with device other than knee, load Joint Angle
            if dev != 0:
                self.angExtJoint = angExtJ
                self.angFlexJoint = angFlexJ
                self.angVertJoint = self.angExtJoint+20*bpdJ
                self.angVertMot = self.angExtMot+20*self.bpdMot
                self.bpdJoint = bpdJ

def kneeCal(devId,FX,calData,cal=2):

    if cal != 1:

        # Z-Axis Gyroscope Calibration
        calData.gyro = imu.gyroCal(devId,FX)

        # X-Axis, Y-Axis Calibration
        calData.xAccel,calData.yAccel = imu.accelCal(devId,FX)

    if cal != 0:

        # Motor Hardstops and Bits Per Degree Calculation
        calData.angExtMot,calData.angFlexMot,calData.bpdMot = ang.angleCal(devId,FX,romJoint=120)

    storeCheck = input('Store calibration data in .yaml file as well? [y/n]: ')

    if storeCheck in 'yes':

        # Store calData in .yaml file for future pulls
        stor.calDump(calData,0)
        print('Data stored in Knee_Cal.yaml...')

    else:

        print('Data not stored in Knee_Cal.yaml...')

    return calData

def ankleCalJoint(devId,FX,calData,cal=2):

    # Check that ankle is upright
    vertCheck = input('Stand Ankle Module Up, and Hit Enter To Continue...')

    if cal != 0:

        # Motor/Joint Hardstops and Bits Per Degree Calculations
        calData.angExtMot,calData.angFlexMot,calData.bpdMot,calData.angExtJoint,calData.angFlexJoint,calData.bpdJoint = ang.angleCal(devId,FX,romJoint=30)

        # Joint Vertical Orientation Calculation
        calData.angVertJoint = calData.angExtJoint + 15*calData.bpdJoint

        # Motor Vertical Orientation Calculation
        calData.angVertMot = home.ankleHomeJoint(devId,FX,calData.angVertJoint)

    if cal != 1:

        # Z-Axis Gyroscope Calculation
        calData.gyro = imu.gyroCal(devId,FX)

        # X-Axis, Y-Axis Calibration
        calData.xAccel,calData.yAccel = imu.ankleVertAccelCal(devId,FX)

    storeCheck = input('Store calibration data in .yaml file as well? [y/n]: ')

    if storeCheck in 'yes':

        # Store calData in .yaml file for future pulls
        stor.calDump(calData,1)
        print('Data stored in Ankle_Cal.yaml...')

    else:

        print('Data not stored in Ankle_Cal.yaml...')

    return calData

def ankleCalMot(devId,FX,calData,cal=2):

    # Check that ankle is upright
    vertCheck = input('Stand Ankle Module Up, and Hit Enter To Continue...')

    if cal != 0:

        # Motor/Joint Hardstops and Bits Per Degree Calculations
        calData.angExtMot,calData.angFlexMot,calData.bpdMot,calData.angExtJoint,calData.angFlexJoint,calData.bpdJoint = ang.angleCal(devId,FX,romJoint=30)

        # Joint Vertical Orientation Calculation
        calData.angVertMot = calData.angExtMot - 67*calData.bpdMot

        print(calData.angExtMot,calData.angVertMot,calData.angFlexMot)

        # Motor Vertical Orientation Calculation
        calData.angVertJoint = home.ankleHomeMot(devId,FX,calData.angVertMot)

    if cal != 1:

        # Z-Axis Gyroscope Calculation
        calData.gyro = imu.gyroCal(devId,FX)

        # X-Axis, Y-Axis Calibration
        calData.xAccel,calData.yAccel = imu.ankleVertAccelCal(devId,FX)

    storeCheck = input('Store calibration data in .yaml file as well? [y/n]: ')

    if storeCheck in 'yes':

        # Store calData in .yaml file for future pulls
        stor.calDump(calData,1)
        print('Data stored in Ankle_Cal.yaml...')

    else:

        print('Data not stored in Ankle_Cal.yaml...')

    return calData

def main(dev,cal):

    '''
    For standalone calling of the angle calibration functions
    '''

    try:
        if (dev != 0) or (dev != 1):
            raise Exception('Invalid joint chosen')
    except:
        print('Error occurred')
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

    # Grab Class Object for Dephy Actuator Functions
    FX = fx.FlexSEA()

    # Open device and begin streaming data
    devId = FX.open(port,baudRate,debugLvl)
    FX.start_streaming(devId,freq=100,log_en=False)
    sleep(0.1)

    calData = CalDataSingle(cal,dev)

    try:
        
        if dev == 0:
            kneeCal(devId,FX,calData,cal)
        elif dev == 1:
            ankleCal(devId,FX,calData,cal)

    except:

        print('Error Occurred')

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

    dev = int(input('Which joint do you want to calibrate? (0 for Knee, 1 for Ankle): '))
    cal = int(input('What data needs calibrating? (0 for IMU, 1 for Angle, 2 for Both): '))
    main(dev,cal)
