'''
##################################### OPEN #####################################
This package holds the functions for opening and closing the actuators for knee-ankle configuration

Last Update: 4 June 2021
Updates:
    - Created
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

from time import sleep, time, strftime
import os, sys
import math

# Actuator Modules (Most Start with fx)
from flexsea import fxUtils as fxu

from OSL_Modules.OSL_Calibration_Dual import OSL_Constants as osl

############################# FUNCTION DEFINITIONS #############################

def devOpen(FX):

    '''
    Function for opening streams to Dephy actuators and obtaining device id.
    Inputs:
        FX - Class object with flexSEA Dephy functions for opening and streaming
    Outputs:
        devId - List of device ids for future use with FX functions
    '''

    # Find directory
    scriptPath = os.path.dirname(os.path.abspath(__file__))
    fpath = scriptPath + '/Ports_Dual.yaml'

    ports, baudRate = fxu.load_ports_from_file(fpath)

    # Print ports and baudrate values loaded, convert to proper data types
    print(ports,'\n',baudRate)
    port1 = str(ports[0])
    port2 = str(ports[1])
    baudRate=int(baudRate)
    debugLvl=6

    # Open devices to grab device ids and set debug level
    devId1 = FX.open(port1,baudRate,debugLvl)
    devId2 = FX.open(port2,baudRate,debugLvl)

    # Create list of device ids where KNEE device is first in the list
    if devId1 == osl.devKnee:
        devId = [devId1,devId2]
    else:
        devId = [devId2,devId1]

    # Start streaming data to/from opened devices
    FX.start_streaming(devId[0],freq=100,log_en=False)
    FX.start_streaming(devId[1],freq=100,log_en=False)
    sleep(0.1)

    return devId

def devClose(devId):

    '''
    Function for closing streams to Dephy actuators
    Inputs:
        devId - List of device ids currently open and streaming
    Outputs:
        None
    '''

    for id in devId:

        # Disable the controller, send 0 PWM
        FX.send_motor_command(id, fxe.FX_VOLTAGE, 0)
        sleep(0.05)

        # Stop streaming to device
        FX.stop_streaming(id)
        sleep(0.05)

        # Close device fully
        FX.close(id)
        sleep(0.05)

        print('Graceful Exit Completed')

def main():

    '''
    Most likely no reason to have main() function for this module
    '''

if __name__ == '__main__':

    print('Standalone execution of this module is not yet supported.')
