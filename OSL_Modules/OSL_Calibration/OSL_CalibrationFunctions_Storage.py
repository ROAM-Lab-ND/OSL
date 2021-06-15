'''
##################################### OPEN #####################################
This package holds the functions for dumping and loading calibration data to yaml files for future use with the Open Source Leg (OSL)

Last Update: 8 June 2021
Updates:
    - Updated import file path location for OSL_Calibration
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

from time import sleep, time, strftime
import math
import numpy as np
import yaml

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe
from OSL_Modules.OSL_Calibration import OSL_Constants as osl

############################# FUNCTION DEFINITIONS #############################

def calDump(calData,dev,cal=2):

    '''
    Function for dumping calibration data to the proper yaml file for future use
    Inputs:
        calData - Class structure storing the calibration data to save
        dev - Calibration data joint (0 for Knee, 1 for Ankle)
    '''

    if dev == 0:
        pFile = open('Knee_Cal.yaml','w')
    elif dev == 1:
        pFile = open('Ankle_Cal.yaml','w')
    else:
        raise Exception('Invalid joint option for data dump')

    pDoc = yaml.dump(calData,pFile,Dumper=yaml.Dumper)
    pFile.close()

def calLoad(dev):

    '''
    Function for loading calibration data from the proper yaml file
    Inputs:
        dev - Calibration data joint (0 for Knee, 1 for Ankle)
    Outputs:
        calData - Class structure storing the calibration data
    '''

    if dev == 0:

        pFile = open('Knee_Cal.yaml')

    elif dev == 1:

        pFile = open('Ankle_Cal.yaml')

    else:

        raise Exception('Invalid joint option for data load')

    calData = yaml.load(calData,pFile,Loader=yaml.Loader)
    pFile.close()

    return calData

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
