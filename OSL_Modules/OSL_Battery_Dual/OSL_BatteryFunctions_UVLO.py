'''
##################################### OPEN #####################################
This package holds the functions for checking the charge of the batteries used for the Open Source Leg (OSL) actuators and avoid use at dangerous voltage levels

Last Update: 18 June 2021
Updates:
    - Created
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

from time import sleep, time, strftime

from flexsea import flexsea as fx

# Actuator Modules (Most Start with fx)
from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_DeviceOpenClose as opcl

############################# FUNCTION DEFINITIONS #############################

def voltCheck(devId,FX):

    '''
    Function for checking the voltage being supplied to the motor from the batteries.
    Inputs:
        devId - Device ID of actuators to check voltage level for
        FX - Class object with flexSEA Dephy functions for reading actuator data
    Outputs:
        None
    '''

    actDataKnee = FX.read_device(devId[0])
    actDataAnk = FX.read_device(devId[1])

    voltBatKnee = actDataKnee.batt_volt
    voltBatAnk = actDataAnk.batt_volt

    if voltBatKnee < osl.voltThresh:

        voltWarn(devId[0],voltBatKnee)

    if voltBatAnk < osl.voltThresh:

        voltWarn(devId[1],voltBatAnk)

    if voltBatKnee < osl.voltThresh - 1000:

        voltShutOff(devId[0])

    if voltBatAnk < osl.voltThres - 1000:

        voltShutOff(devId[1])

def voltWarn(devId, volt):

    '''
    Function for warning the user that the battery voltage is approaching dangerous levels.

    Inputs:
        devId - Device ID of actuator to check voltage level for
        volt - voltage value read from device
    Outputs:
        None
    '''

    if devId == osl.devKnee:

        devName = 'KNEE'

    else:

        devName = 'ANKLE'

    print('WARNING: UVLO THRESHOLD TRIGGERED FOR ', devName, 'ACTUATOR')
    print('UVLO THRESHOLD: ', osl.voltThresh, '  ', devName, 'VOLTAGE: ', volt)

def voltShutOff(devId):

    '''
    Function for warning the user that the voltage shut off will be occurring in five seconds to allow time to react and prepare
    Inputs:
        devId - Device ID of actuator to check voltage level for
    Outputs:
        None
    '''

    if devId == osl.devKnee:

        devName = 'KNEE'

    else:

        devName = 'ANKLE'

    print('WARNING: VOLTAGE TOO LOW FOR ', devName, 'ACTUATOR')

    count = 5

    while count:

        print('SHUTTING DOWN ACTUATORS IN: ', count, 'SEC')
        count -= 1

        sleep(osl.dtSec)

    raise RuntimeError('UVLO Triggered')

def main(devId,FX):

    try:

        while True:

            voltCheck(devId,FX)
            sleep(0.1)

    except Exception as error:

        print(error)

    finally:

        opcl.devClose(devId,FX)

if __name__ == '__main__':

    FX = fx.FlexSEA()

    devId = opcl.devOpen(FX)

    main(devId,FX)
