'''
##################################### OPEN #####################################
This package houses the functions for calculating the position and transmission ratio mapping of the ankle for the Open Source Leg (OSL).

NOTE: THIS MODULE IS DESIGNED WITH RESPECT TO AN ANKLE JOINT SPACE RANGING FROM -15 DEGREES (DF) TO 15 DEGREES (PF). VERTICAL ORIENTATION OF THE ANKLE CORRESPONDS TO A VALUE OF 0 DEGREES.

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Imports for Standard Python
from time import sleep, time, strftime
import numpy as np

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Storage as stor
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_Homing as home

############################# FUNCTION DEFINITION ##############################

def anklePosMappingJoint(jointDesDeg,calData):

    '''
    This function is called to obtain the value of the motor encoder for a desired ankle angle.
    Inputs:
        jointDesDeg - Number of degrees the desired ankle angle is from vertical
        calData - Class structure that holds all of the actuator calibration data
    Outputs:
        motDesTick - Tick value to pass to FX.send_motor_command() for desired ankle angle
    '''

    # Polyfit Coefficients (Joint to Motor)
    polyCoef = [6.904006316218462e-6, 2.855672135787040e-5, 9.847287004188480e-4, 0.008459960933132, 3.432060542616289, 1.088851620694406e-15]

    # Calculate Desired Motor Position (Degrees)
    motDesDeg = np.polyval(polyCoef,jointDesDeg)

    # Calculate Desired Motor Position (Ticks)
    motDesTick = motDesDeg*calData.bpdMot + calData.angVertMot

    # Return Desired Motor Position (Ticks)
    return motDesTick

def anklePosMappingMot(motAng,calData):

    '''
    This function is called to obtain the value of the joint angle for a given motor tick reading.
    Inputs:
        motAng - Current motor angle in ticks
        calData - Class structure that holds all of the actuator calibration data
    Outputs:
        jointAng - Current joint angle in degrees
    '''

    # Polyfit Coefficients (Motor to Joint)
    polyCoef = [1.139469359425673e-11,2.961158110823952e-8,-1.192721072846565e-5,-2.636391981738515e-4,0.293325991108028,0.008567682193295]

    # Calculate Desired Motor Position (Degrees)
    motAngDeg = (motAng - calData.angVertMot)/calData.bpdMot

    # Calculate Desired Joint Position (Degrees)
    jointAng = np.polyval(polyCoef,motAngDeg)

    # Return Desired Joint Position (Degrees)
    return jointAng

def ankleTRMappingJoint(jointAng,calData):

    '''
    This function is called to obtain the value of the transmission ratio for the current ankle angle.
    Inputs:
        jointAng - Current joint angle in ticks
        calData - Class structure that holds all of the actuator calibration data
    Outputs:
        desTR - Calculated transmission ratio to use in stiffness/damping conversions based on current joint position
    '''

    # Polyfit Coefficients (Joint to TR)
    polyCoef = [2.601481109902332e-6, 9.858841845509533e-6, -3.218758554395808e-4, -8.191030841061497e-4, 0.088589729972813, 0.312298762934481, 41.953174794753980]

    # Calculate Joint Angle from Vertical Orientation (Degrees)
    jointAngDeg = (jointAng - calData.angVertJoint)/calData.bpdJoint

    # Calculate Transmission Ratio Corresponding to Current Joint Angle
    desTR = np.polyval(polyCoef,jointAngDeg)

    # Return Transmission Ratio
    return desTR

def ankleTRMappingMot(motAng,calData):

    '''
    This function is called to obtain the value of the transmission ratio for the current motor angle.
    Inputs:
        motAng - Current joint angle in ticks
        calData - Class structure that holds all of the actuator calibration data
    Outputs:
        desTR - Calculated transmission ratio to use in stiffness/damping conversions based on current joint position
    '''

    # Polyfit Coefficients (Motor to TR)
    polyCoef = [1.906223726201300e-10, -3.700386409901678e-9, 2.957465762186836e-7, 7.791368079535717e-6, 0.005526776998495, 0.075173504601046, 42.152801658472660]

    # Calculate Motor Angle from Vertical Orientation (Degrees)
    motAngDeg = (motAng - calData.angVertMot)/calData.bpdMot

    # Calculate Transmission Ratio Corresponding to Current Joint Angle
    desTR = np.polyval(polyCoef,motAngDeg)

    # Return Transmission Ratio
    return desTR

def main(map):

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
    baudRate=int(baudRate)
    debugLvl=6

    # Connect to actuator and open data stream
    FX = fx.FlexSEA()

    for port in ports:
        devId = FX.open(str(port),baudRate,debugLvl)
        sleep(0.1)
        if devId != 14449:
            FX.close(devId)
    try:

        FX.start_streaming(devId,freq=100,log_en=False)

    except Exception as error:

        print(error)
        sleep(0.05)
        FX.close(devId)

    sleep(0.1)

    try:
        calData = stor.calLoad()

        if map == 0:

            print('Homing to vertical orientation...')
            home.ankleHomeJoint(devId,FX,calData.angVertJoint,volt=400,valReturn=0)
            angOff = float(input('Set offset from vertical (in degrees): '))

            motDesTick = anklePosMappingJoint(devId,angOff,calData)
            motDesDeg = (motDesTick - calData.angExtJoint)/calData.bpdJoint[1]

            print('Determined Desired Motor Tick: {} \nDetermined Desired Motor Angle: {}'.format(motDesTick,motDesDeg))

        elif map == 1:

            curAng = float(input('Set angle from vertical to calculate transmission ratio from (in degrees): '))

            desTR = ankleTRMappingMot(devId,curAng,calData)
            print('Determined Transmission Ratio: {}'.format(desTR))

        else:
            raise Exception('Invalid mapping option chosen')

    except Exception as error:

        print('Failed to complete test...')
        print(error)

        sleep(0.05)
        # Disable the controller, send 0 PWM
        FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)

    finally:

        sleep(0.1)
        FX.stop_streaming(devId)
        sleep(0.2)
        FX.close(devId)
        sleep(0.1)
        print("Graceful Exit Complete")
    '''

if __name__ == '__main__':

    print('Standalone execution of this script is not currently supported.')
    '''
    map = int(input('Which mapping are you testing? (0 for Position, 1 for Transmission Ratio): '))
    main(map)
    '''
