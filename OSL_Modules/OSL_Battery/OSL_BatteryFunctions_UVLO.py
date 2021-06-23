'''
##################################### OPEN #####################################
This package holds the functions for checking the charge of the batteries used for the Open Source Leg (OSL) actuators and avoid use at dangerous voltage levels

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

# Imports for Standard Python
from time import sleep, time, strftime

# Imports for FlexSEA
from flexsea import flexsea as fx

# Imports for OSL
from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_DeviceOpenClose as opcl

############################# FUNCTION DEFINITIONS #############################

def voltCheck(devId, FX):

    '''
    Function for checking the voltage being supplied to the motor from the batteries.
    Inputs:
        devId - Device ID of actuator to check voltage level for
        FX - Class object with flexSEA Dephy functions for reading actuator data
    Outputs:
        None
    '''

    # Read Current Motor Information and Grab Battery Voltage
    actData = FX.read_device(devId)
    voltBat = actData.batt_volt

    # If Voltage Below Warning Threshold, Send Warning
    if voltBat < osl.threshBat:

        voltWarn(devId, voltBat)

    # If Voltage Well Below Warning Threshold, Initiate Shutdown Sequence
    if voltBat < osl.threshBat - 500:

        voltShutOff(devId)

def voltWarn(devId, volt):

    '''
    Function for warning the user that the battery voltage is approaching dangerous levels.

    Inputs:
        devId - Device ID of actuator to check voltage level for
        volt - voltage value read from device
    Outputs:
        None
    '''

    # Device Classifier
    if devId == osl.devKnee:

        devName = 'KNEE'

    else:

        devName = 'ANKLE'

    # Print Warning That Battery Voltage Is Running Low
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

    # Voltage Classifier
    if devId == osl.devKnee:

        devName = 'KNEE'

    else:

        devName = 'ANKLE'

    # Print Warning
    print('WARNING: VOLTAGE TOO LOW FOR ', devName, 'ACTUATOR')

    count = 5

    # Initiate Five Second Countdown Before Raising Exception
    while count:

        print('SHUTTING DOWN ACTUATOR IN: ', count, 'SEC')
        count -= 1

        sleep(osl.dtSec)

    # Raise Exception
    raise RuntimeError('UVLO Triggered')

############################# MAIN FUN DEFINITIONS #############################

def main(devId, FX):

    try:

        while True:

            # Check Voltage Seen By Device
            voltCheck(devId, FX)
            sleep(0.1)

    except Exception as error:

        # Print Error
        print(error)

    finally:

        # Gracefully Exit Script by Closing Stream and Device ID
        opcl.devClose(devId, FX)

if __name__ == '__main__':

    FX = fx.FlexSEA()

    devId = opcl.devOpen(FX)

    main(devId, FX)
