'''
##################################### OPEN #####################################
This package houses the functions for calculating the position and transmission ratio mapping of the ankle for the Open Source Leg (OSL).

Last Update: 2 June 2021
Updates: Updated anklePosMapping to bypass use of ankle encoder data due to concerns with data reading from external encoder. Updated anklePosMapping with new polyfit coefficients for range of motion [-15 (dorsiflexion), 15 (plantar flexion)]. Updated ankleTRMapping to ankleTRMappingAnk with new polyfit coefficients for range of motion [-15 (dorsiflexion), 15 (plantar flexion)]. Created ankleTRMappingMot with new polyfit coefficients for range of motion [-15 (dorsiflexion), 15 (plantar flexion)].
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

def anklePosMapping(devId,jointDesDeg,calData):

    '''
    This function is called to obtain the value of the motor encoder for a desired ankle angle.
    Inputs:
        devId - ID of the actuator grabbed from FX.open() command
        jointDesDeg - Number of degrees the desired ankle angle is from vertical
        calData - Class structure that holds all of the actuator calibration data
    Outputs:
        motDesTick - Tick value to pass to FX.send_motor_command() for desired ankle angle
    '''

    # Polyfit coefficients
    polyCoef = [6.90400631621864e-06, -0.000144043436547594, 0.002139595852316, -0.010657469264743, 3.42861224487564, -3.00193673363941e-14]

    # Calculate desired motor position in degrees
    motDesDeg = np.polyval(polyCoef,jointDesDeg)

    # Calculate motor desired offset in degrees and ticks
    motDesTick = motDesDeg*calData.bpdMot + calData.angVertMot

    # Return desired motor position in ticks
    return motDesTick

def ankleTRMappingJoint(devId,jointAng,calData):

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
    polyCoeff = [2.601481109902300e-06, -6.818559145155934e-05, 4.072085146360515e-04, 0.001579421711307, 0.064660231016916,-0.492062230425264, 42.517478966840436]

    # Calculate joint angle in degrees from vertical
    jointAngDeg = (jointAng - calData.angVertJoint)/calData.bpdJoint

    # Calculate desired transmission ratio based on current joint angle
    desTR = np.polyval(polyCoeff,jointAngDeg)

    # Return desired transmission ratio
    return desTR

def ankleTRMappingMot(devId,motAng,calData):

    '''
    This function is called to obtain the value of the transmission ratio for the current motor angle.
    Inputs:
        devId - ID of the actuator grabbed from FX.open() command
        motAng - Current joint angle in ticks
        calData - Class structure that holds all of the actuator calibration data
    Outputs:
        desTR - Calculated transmission ratio to use in stiffness/damping conversions based on current joint position
    '''

    # Polyfit coefficients
    # Polyfit was conducted with the ankle angle already shifted to [-20,10] range
    polyCoeff = [1.906223726201309e-10, -2.323035971161657e-08, 1.445394668235981e-06, -4.217995699735862e-05, 0.006072376026348, -0.115880735295439, 42.477096278915720]

    # Calculate motor angle in degrees from vertical
    motAngDeg = (motAng - calData.angVertMot)/calData.bpdMot

    # Calculate desired transmission ratio based on current motor angle
    desTR = np.polyval(polyCoeff,motAngDeg)

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
            motDesDeg = (motDesTick - calData.angExtJoint)/calData.bpdJoint

            print('Determined Desired Motor Tick: {} \nDetermined Desired Motor Angle: {}'.format(motDesTick,motDesDeg))

        elif map == 1:

            curAng = float(input('Set angle from vertical to calculate transmission ratio from (in degrees): '))

            desTR = ankleTRMappingMot(devId,curAng,calData)
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
