'''
##################################### OPEN #####################################
This package holds the functions for dumping and loading calibration data to yaml files for future use with the Open Source Leg (OSL)

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

# Imports for Standard Python
from time import sleep, time, strftime
import math
import numpy as np
import yaml

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration import OSL_Constants as osl

############################# FUNCTION DEFINITIONS #############################

def calDump(calData, dev):

    '''
    Function for dumping calibration data to the proper yaml file for future use
    Inputs:
        calData - Class structure storing the calibration data to save
        dev - Calibration data joint (0 for Knee, 1 for Ankle)
    Outputs:
        None
    '''

    # Open Appropriate File Based On Device ID
    if dev == osl.devKnee:

        pFile = open('Knee_Cal.yaml','w')

    elif dev == osl.devAnk:

        pFile = open('Ankle_Cal.yaml','w')

    else:

        raise RuntimeError('Invalid joint option for data dump')

    # Dump Calibration Data to File and Close
    pDoc = yaml.dump(calData, pFile, Dumper = yaml.Dumper)
    pFile.close()

def calLoad(dev):

    '''
    Function for loading calibration data from the proper yaml file
    Inputs:
        dev - Calibration data joint (0 for Knee, 1 for Ankle)
    Outputs:
        calData - Class structure storing the calibration data
    '''

    # Open Appropriate File Based on Device ID
    if dev == osl.devKnee:

        pFile = open('Knee_Cal.yaml')

    elif dev == osl.devAnk:

        pFile = open('Ankle_Cal.yaml')

    else:

        raise RuntimeError('Invalid joint option for data load')

    # Load Data From File and Close
    calData = yaml.load(pFile, Loader = yaml.Loader)
    pFile.close()

    # Return Calibration Data
    return calData

############################# MAIN FUN DEFINITIONS #############################

def main(dev):

    '''
    try:

        calData = calLoad(dev)

    except:

        print('Failed to complete load test...')
    '''

if __name__ == '__main__':

    print('Standalone execution of this module is not currently available.')
    '''
    dev = int(input('Which joint is calibrated? (0 for Knee, 1 for Ankle): '))
    main(dev)
    '''
