'''
##################################### OPEN #####################################
This package holds the functions for dumping and loading calibration data to yaml files for future use with the Open Source Leg (OSL)

Last Update: 9 June 2021
Updates:
    - Updated imports
#################################### CLOSE #####################################
'''

#################################### IMPORTS ###################################

import math
import yaml

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe

############################# FUNCTION DEFINITIONS #############################

def calDump(calData):

    '''
    Function for dumping calibration data to the proper yaml file for future use
    Inputs:
        calData - Class structure storing the calibration data to save
    '''

    pFile = open('Dual_Cal.yaml','w')

    pDoc = yaml.dump(calData,pFile,Dumper=yaml.Dumper)
    pFile.close()

def calLoad():

    '''
    Function for loading calibration data from the proper yaml file
    Inputs:
        None
    Outputs:
        calData - Class structure storing the calibration data
    '''

    pFile = open('Dual_Cal.yaml')

    calData = yaml.load(calData,pFile,Loader=yaml.Loader)
    pFile.close()

    return calData

def main():

    '''
    try:
        calData = calLoad(dev)
        print(vars(calData))

    except:
        print('Failed to complete load test...')
    '''

if __name__ == '__main__':

    print('Standalone calling of this module is currently unavailable.')
    # main()
