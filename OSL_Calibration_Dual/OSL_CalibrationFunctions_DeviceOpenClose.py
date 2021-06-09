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
import math
import numpy as np
import yaml

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe

############################# FUNCTION DEFINITIONS #############################

def dualOpen(calData,cal=2):

    '''
    Function for dumping calibration data to the proper yaml file for future use
    Inputs:
        calData - Class structure storing the calibration data to save
        cal - Calibration data objects to save
    '''

    pFile = open('Dual_Cal.yaml','w')

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

    pFile = open('Dual_Cal.yaml')

    calData = yaml.load(calData,pFile,Loader=yaml.Loader)
    pFile.close()

    return calData

def main():

    try:
        calData = calLoad(dev)

    except:
        print('Failed to complete load test...')

if __name__ == '__main__':

    print('NOTE: Only loading of saved calibration data can be explicitly tested from this script. To test dumping, please run a calibration.')
    main()
