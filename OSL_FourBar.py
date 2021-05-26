'''
##################################### OPEN #####################################
This package houses the functions for calculating the position and transmission ratio mapping of the ankle for the Open Source Leg (OSL).

Last Update: 20 May 2021
Updates: Created
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

from OSL_Calibration import OSL_Constants as osl
from OSL_Calibration import OSL_CalibrationFunctions_Storage as stor
from OSL_Calibration import OSL_CalibrationFunctions_Homing as home

############################# FUNCTION DEFINITION ##############################

def anklePosMapping(devId,jointDeltaDeg,calData):

    '''
    This function is called to obtain the value of the motor encoder for a desired ankle angle.
    Inputs:
        devId - ID of the actuator grabbed from FX.open() command
        jointDeltaDeg - Number of degrees the desired angle ankle is from vertical
        calData - Class structure that holds all of the actuator calibration data
    Outputs:
        motDesTick - Tick value to pass to FX.send_motor_command() for desired ankle angle
    '''

    # Polyfit coefficients
    polyCoeff = [-1.200028470379417e-6,3.916871256831076e-4,-0.047626777467870,2.592217413695131,-52.816226201897430,0]

    # Calculate vertical orientation for joint in terms of degrees
    jointVertDeg = (calData.angVertJoint-calData.angExtJoint)/calData.bpdJoint+54.8

    # Calculate vertical orientation for motor in terms of degrees
    motVertDeg = np.polyval(polyCoeff,jointVertDeg)

    # Add on desired offset to obtain desired joint position in degrees
    jointDesDeg = jointDeltaDeg + jointVertDeg
    #jointDesTick = (jointDesDeg - 54.8)*calData.bpdJoint + calData.angExtJoint

    # Calculate desired motor position in degrees
    motDesDeg = np.polyval(polyCoeff,jointDesDeg)

    # Calculate motor desired offset in degrees and ticks
    motDeltaDeg = (motDesDeg - motVertDeg)
    motDeltaTick = motDeltaDeg*calData.bpdMot

    # Calculate desired motor position in ticks
    motDesTick = motDeltaTick + calData.angVertMot

    # Return desired motor position in ticks
    return motDesTick

def ankleTRMapping(devId,jointAng,calData):

    '''
    This function is called to obtain the value of the transmission ratio for the current ankle angle.
    Inputs:
        devId - ID of the actuator grabbed from FX.open() command
        jointAng - Current joint angle in ticks
        calData - Class structure that holds all of the actuator calibration data
    Outputs:
        desTR - Calculated transmission ratio to use in stiffness/damping conversions based on current joint position
    '''

    # Polyfit coefficients
    # Polyfit was conducted with the ankle angle already shifted to [-20,10] range
    polyCoeff = [2.601481109902289e-6, 5.974073629756083e-5, 2.341789879637116e-04,-0.002269319912352, 0.067834298168660, 0.420479741974045, 42.270467940947240]

    # Calculate joint angle in degrees from vertical
    jointAngDeg = (jointAng-calData.angExtJoint)/calData.bpdJoint

    # Calculate desired transmission ratio based on current joint angle
    desTR = np.polyval(polyCoeff,jointAngDeg)

    # Return desired transmission ratio
    return desTR

def main(map):

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

    try:
        calData = stor.calLoad(1)

        if map == 0:

            print('Homing to vertical orientation...')
            home.ankleHome(devId,FX,calData.angVertJoint,volt=-750,valReturn=0)
            angOff = float(input('Set offset from vertical (in degrees): '))

            motDesTick = anklePosMapping(devId,angOff,calData)
            motDesDeg = (motDesTick - calData.angExtJoint)/calData.bpdJoint -10.5
            print('Determined Desired Motor Tick: {} \nDetermined Desired Motor Angle: {}'.format(motDesTick,motDesDeg))

        elif map == 1:

            curAng = float(input('Set angle from vertical to calculate transmission ratio from (in degrees): '))

            desTR = ankleTRMapping(devId,curAng,calData)
            print('Determined Transmission Ratio: {}'.format(desTR))

        else:
            raise Exception('Invalid mapping option chosen')

    except:

        print('Failed to complete test...')

        sleep(0.05)
        # Disable the controller, send 0 PWM
        FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)
        sleep(0.1)

        FX.stop_streaming(devId)
        sleep(0.2)
        FX.close(devId)
        sleep(0.1)
        print("Graceful Exit Complete")

if __name__ == '__main__':

    map = int(input('Which mapping are you testing? (0 for Position, 1 for Transmission Ratio): '))
    main(map)
