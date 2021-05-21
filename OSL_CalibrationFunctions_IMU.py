'''
##################################### OPEN #####################################
This package holds the functions called by OSL_Calibration_Package.py to calibrate the IMU sensor in the Dephy actuator

Last Update: 20 May 2021
Updates: Added support for standalone use
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

from time import sleep, time, strftime
import math
import numpy as np
import OSL_Constants

# Actuator Modules (Most Start with fx)
from flexsea import fxEnums as fxe

################################# CALIBRATION ##################################
def gyroCal(devId,FX,N=30000,Nvis=15000,run=1):
    '''
    This function is used to determine the bias of the z-Axis Gyroscope of the Knee or Ankle actuator
    '''

    print('Pulling in Gyro Reading for Zero Calibration...')
    print('Please keep actuator still (wait for confirmation)')
    sleep(0.2)
    print('%-8s %-15s %-15s' % ('Count','Gyro Z Value','Running Average'))
    sleep(0.5)

    # Initialize Counter and Average
    count = 0
    gyro_avg = 0

    while count<N:

        # Increment Counter
        count=count+1

        # Grab z-Axis Gyroscope Reading, convert to rad/sec
        actData = FX.read_device(devId)
        gyro_z = np.multiply(np.true_divide(actData.gyroz,gyroConv),deg2rad)

        # Calculate running average of z-Axis Gyroscope Reading
        gyro_avg = (gyro_avg*(count-1)+gyro_z)/count

        # Print Current Value and Average Value to Screen
        print('%-8i %-15f %-15f' % (count,gyro_z,gyro_avg))
        sleep(dt)

    print('Zero-Point Calibration Complete.  Running Visual Confirmation Test')
    sleep(0.5)
    print('%-8s %-15s %-15s' % ('Count','Gyro Z Value','Average'))
    sleep(1)

    # Initialize Counter and Post-Calibration Average
    count = 0
    cal_avg = 0

    while count<Nvis:

        # Increment Counter
        count=count+1

        # Grab z-Axis Gyroscope Reading, convert to rad/sec
        actData = FX.read_device(devId)
        gyro_z = np.multiply(np.true_divide(actData.gyroz,gyroConv),deg2rad)

        # Calculate calibrated z-Axis Gyroscope value and running average
        cal_z = gyro_z - gyro_avg
        cal_avg = (cal_avg*(count-1)+cal_z)/count

        # Print current calibrated z-Axis Gyroscope Value and Running Average
        # to Screen
        print('%-8i %-15f %-15f' % (count,cal_z,cal_avg))
        sleep(dt)

    print('Gyro Calibration Run ',run,' Complete')
    # Return Calibration Bias for future use
    return float(gyro_avg)

def accelCal(devId,FX,N=30000,Nvis=15000,run=1):

    '''
    This function is used to determine the bias of the x-Axis and y-Axis Accelerometer of the Knee or Ankle actuator
    '''

    print('Pulling in Accelerometer Reading for Zero Calibration...')
    sleep(0.5)
    print('%-5s %-10s %-10s %-10s %-10s' % ('Count','AccelX','Avg_X','AccelY','Avg_Y'))
    sleep(1)

    # Initialize Counter and Average for x-Axis and y-Axis
    count = 0
    x_avg = 0
    y_avg = 0

    while count<N:

        # Increment Counter
        count=count+1

        # Grab x-Axis and y-Axis Accelerometer Values, convert to gravity
        actData = FX.read_device(devId)
        xAccel = np.true_divide(actData.accelx,accelConv)
        yAccel = np.true_divide(actData.accely,accelConv)

        # Calculate running average of x-Axis and y-Axis Accelerometer Values
        x_avg = (x_avg*(count-1)+xAccel)/count
        y_avg = (y_avg*(count-1)+yAccel)/count

        # Print Current Readings and Running Average Readings to Screen
        print('%-5i %-10f %-10f %-10f %-10f' % (count,xAccel,x_avg,yAccel,y_avg))
        sleep(dt)

    print('Accelerometer Calibration Complete.  Running Visual Confirmation Test')
    sleep(0.5)
    print('%-5s %-10s %-10s' % ('Count','AccelX','AccelY'))
    sleep(1)

    # Initialize Counter
    count = 0

    while count<Nvis:

        # Increment Counter
        count=count+1

        # Grab x-Axis and y-Axis Accelerometer Values, convert to gravity
        actData = FX.read_device(devId)
        xAccel = np.true_divide(actData.accelx,accelConv)
        yAccel = np.true_divide(actData.accely,accelConv)

        # Calculate calibrated Accelerometer values
        x_cal = xAccel - x_avg
        y_cal = yAccel - y_avg

        # Print calibrated Accelerometer values to screen
        print('%-5i %-10f %-10f' % (count,x_cal,y_cal))
        sleep(dt)

    print('Accel Calibration Run ',run,' Complete')
    # Return Calibration Biases for future use
    return float(x_avg),float(y_avg)

def ankleVertAccelCal(devId,FX,xZero,yZero,Nvis=15000):

    '''
    This function is called to calibrate the X-Axis and Y-Axis Accelerometers of the ankle module such that vertical joint ankle is the 'Zeroed' point.
    NOTE 1: THIS SHOULD NOT BE CALLED BEFORE THE ANKLE JOINT IS VERTICALLY ORIENTED AND THE ANKLE MODULE IS UPRIGHT. A vertical homing function can be found in OSL_CalibrationFunctions_Homing.py
    NOTE 2: 'Zeroed' refers to the vertical orientation of the ankle (Y-Axis reads 1 [vertical] while X-Axis reads 0 [horizontal])
    '''

    print('%-10s %-15s %-15s' % ('Count','Accel Off X','Accel Off Y'))

    count = 0
    xVert = 0
    yVert = 0

    while count < Nvis:

        count = count + 1

        actData = FX.read_device(devId)
        xBias = np.true_divide(actData.accelx,accelConv) - xZero
        yBias = 1 - (np.true_divide(actData.accely,accelConv) - yZero)

        xVert = (xVert*(count - 1) + xBias)/count
        yVert = (yVert*(count - 1) + yBias)/count

        print('%-10i %-15f %-15f' % (count,xVert,yVert))

        sleep(dt)

    print('Ankle vertical calibration complete...')

    return float(xVert),float(yVert)

def main():

    '''
    For standalone calling of the IMU calibration functions
    '''

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

    calData = CalDataSingle()

    try:

        calData.gyro = gyroCal(devId,FX)
        xBias,yBias = accelCal(devId,FX)
        calData.xAccel = xBias
        calData.yAccel = yBias

    except KeyboardInterrupt:

        print('User Interruption Occurred')

        # Disable the controller, send 0 PWM
        FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)
        sleep(0.1)

        FX.stop_streaming(devId)
        sleep(0.2)
        FX.close(devId)
        sleep(0.1)
        print("Graceful Exit Complete")

    except:

        print('Error Occurred')

        # Disable the controller, send 0 PWM
        FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)
        sleep(0.1)

        FX.stop_streaming(devId)
        sleep(0.2)
        FX.close(devId)
        sleep(0.1)
        print("Graceful Exit Complete")

if __name__ == '__main__':
    main()
