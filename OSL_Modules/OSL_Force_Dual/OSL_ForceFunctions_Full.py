'''
##################################### OPEN #####################################
This package houses the functions for calculating the position and transmission ratio mapping of the ankle for the Open Source Leg (OSL).

Last Update: 8 June 2021
Updates:
    - Changed updates syntax to match other scripts for easier parsing
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

from OSL_Modules.OSL_Calibration_Dual import OSL_Constants as osl
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Storage as stor
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Homing as home

############################# FUNCTION DEFINITION ##############################

def readadc(adcChan):

    '''
    This function is used to return the force sensor data for whichever channel is passed in.

    NOTE: Some of the documentation may be slightly misleading. Please refer to Ryan Posh for more information

    INPUT:
        adcChan - Channel associated with force sensor of interest

    OUTPUT:
        data - Reading output from force sensor of interest
    '''

    # read SPI data from the MCP3008, 8 channels in total
    if adcChan > 7 or adcNum < 0:
        return -1
    spiDat = spi.xfer2([1, 8 + adcChan << 4, 0])
    data = ((spiDat[1] & 3) << 8) + spiDat[2]
    return data

def forceOn(padValCur, padValPrev1, padValPrev2, thresh):

    '''
    This function is used to determine whether a force sensor of interest is considered 'activated' with intention. The determining factor involves readings surpassing a threshold for at least three consecutive time steps

    NOTE: Some of the documentation may be slightly misleading. Please refer to Ryan Posh for more information

    INPUT:
        padValCur - Current force sensor data reading
        padValPrev1 - Force sensor data reading one time step in the past
        padValPrev2 - Force sensor data reading two time steps in the past
        thresh - Threshold for determining if intenttional activation has occurred

    OUTPUT:
        intent - Boolean determining whether intentional activation has occurred
    '''

    # Place sensor readings in list for checking with all() function
    padVal = [padValCur,padValPrev1,padValPrev2]

    # If all sensor readings are above threshold, intent = True
    intent = all( i > thresh for i in padVal)

    return intent

def pressure_off_transition(padValCur, padValPrev, thresh):

    '''
    This function is used to determine whether a force sensor of interest is considered 'deactivated' with intention. The determining factor involves readings below a threshold for at least two consecutive time steps

    NOTE: Some of the documentation may be slightly misleading. Please refer to Ryan Posh for more information

    INPUT:
        padValCur - Current force sensor data reading
        padValPrev - Force sensor data reading one time step in the past
        thresh - Threshold for determining if intenttional deactivation has occurred

    OUTPUT:
        intent - Boolean determining whether intentional deactivation has occurred
    '''

    # Place sensor readings in list for checking with all() function
    padVal = [padValCur,padValPrev]

    # If all sensor readings are below threshold, intent = True
    intent = all( i <= thresh for i in padVal)

    return intent
