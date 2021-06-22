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
import math
import yaml

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxEnums as fxe

############################# FUNCTION DEFINITIONS #############################

def calDump(calData):

    '''
    Function for dumping calibration data to the proper yaml file for future use
    Inputs:
        calData - Class structure storing the calibration data to save
    '''

    # Open Appropriate File
    pFile = open('Dual_Cal.yaml','w')

    # Dump Calibration Data to File and Close
    pDoc = yaml.dump(calData, pFile, Dumper = yaml.Dumper)
    pFile.close()

def calLoad():

    '''
    Function for loading calibration data from the proper yaml file
    Inputs:
        None
    Outputs:
        calData - Class structure storing the calibration data
    '''

    # Open Appropriate File
    pFile = open('Dual_Cal.yaml')

    # Load Calibration Data from File and Close
    calData = yaml.load(pFile, Loader = yaml.Loader)
    pFile.close()

    return calData

############################# MAIN FUN DEFINITIONS #############################

def main():

    '''
    try:
        calData = calLoad(dev)
        print(vars(calData))

    except:
        print('Failed to complete load test...')
    '''

if __name__ == '__main__':

    print('Standalone execution of this module is currently unavailable.')
    # main()
