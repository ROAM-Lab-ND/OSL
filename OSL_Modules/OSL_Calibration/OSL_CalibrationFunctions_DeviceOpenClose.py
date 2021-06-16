'''
##################################### OPEN #####################################
This package holds the functions for opening and closing the actuators for single actuator configuration configuration

Last Update: 16 June 2021
Updates:
    - Removed unnecessary imports, updated comments
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

from time import sleep, time, strftime
import os, sys
import math

# Actuator Modules (Most Start with fx)
from flexsea import fxUtils as fxu

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
    fpath = scriptPath + '/Ports_Single.yaml'

    # Load ports and baudrate from file
    ports, baudRate = fxu.load_ports_from_file(fpath)

    # Print ports and baudrate values loaded, convert to proper data types
    print(ports,'\n',baudRate)
    port = str(ports[0])
    baudRate=int(baudRate)
    debugLvl=6

    # Open device to grab device id and set debug level
    devId = FX.open(port,baudRate,debugLvl)

    # Start streaming data to/from opened device
    FX.start_streaming(devId,freq=100,log_en=False)
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
