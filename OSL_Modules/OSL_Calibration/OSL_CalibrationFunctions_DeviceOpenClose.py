'''
##################################### OPEN #####################################
This package holds the functions for opening and closing the actuators for single actuator configuration configuration

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
    - Updated delays to use parameters from OSL_Constants
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

# Imports for Standard Python
from time import sleep, time, strftime
import os, sys
import math

# Imports for FlexSEA
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

from OSL_Modules.OSL_Calibration import OSL_Constants as osl

############################# FUNCTION DEFINITIONS #############################

def devOpen(FX):

    '''
    Function for opening streams to Dephy actuators and obtaining device id.
    Inputs:
        FX - Class object with flexSEA Dephy functions for opening and streaming
    Outputs:
        devId - List of device ids for future use with FX functions
    '''

    # Set Directory Where Desired File Exists
    scriptPath = os.path.dirname(os.path.abspath(__file__))
    fpath = scriptPath + '/Ports_Single.yaml'

    # Load Ports and Baudrate
    ports, baudRate = fxu.load_ports_from_file(fpath)

    # Convert Ports and Baudrate to Correct Data Type
    port = str(ports[0])
    baudRate=int(baudRate)
    debugLvl=6

    # Open Device ID
    devId = FX.open(port, baudRate, debugLvl)

    # Start Streaming Device ID
    FX.start_streaming(devId, freq=100, log_en=False)
    sleep(osl.dtCenti)

    # Return Device ID
    return devId

def devClose(devId, FX):

    '''
    Function for closing streams to Dephy actuators
    Inputs:
        devId - List of device ids currently open and streaming
        FX - Class object with flexSEA Dephy functions for opening and streaming
    Outputs:
        None
    '''

    # Send Motor Command to Stop
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)
    sleep(5*osl.dtCenti)

    # Close Stream To Device ID
    FX.stop_streaming(devId)
    sleep(5*osl.dtCenti)

    # Close Device ID
    FX.close(devId)
    sleep(5*osl.dtCenti)

    print('Graceful Exit Completed')

############################# MAIN FUN DEFINITIONS #############################

def main():

    '''
    Most likely no reason to have main() function for this module
    '''

if __name__ == '__main__':

    print('Standalone execution of this module is not yet supported.')
