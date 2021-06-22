'''
##################################### OPEN #####################################
This package houses the functions for calculating the position and transmission ratio mapping of the ankle for the Open Source Leg (OSL).

NOTE: These functions are identical to the single actuator functions of the same name.

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration_Dual import OSL_Constants as osl
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Storage as stor
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Homing as home

############################# FUNCTION DEFINITIONS #############################

def readadc(adcChan, spi):

    '''
    Function for evaluating force sensor data

    NOTE: Some of the documentation may be slightly misleading. Please refer to Ryan Posh for more information

    INPUT:
        adcChan - Channel associated with force sensor of interest
    OUTPUT:
        data - Reading output from force sensor of interest
    '''

    # Read SPI Data
    if adcChan > 7 or adcChan < 0:
        return -1

    # Correct SPI Data to Appropriate Data Format
    spiDat = spi.xfer2([1, 8 + adcChan << 4, 0])
    data = ((spiDat[1] & 3) << 8) + spiDat[2]

    # Return Force Sensor Data
    return data

def forceOn(padValCur, padValPrev1, padValPrev2, thresh):

    '''
    Function to determine whether a force sensor of interest is considered 'activated' with intention. The determining factor involves readings surpassing a threshold for at least three consecutive time steps

    NOTE: Some of the documentation may be slightly misleading. Please refer to Ryan Posh for more information

    INPUT:
        padValCur - Current force sensor data reading
        padValPrev1 - Force sensor data reading one time step in the past
        padValPrev2 - Force sensor data reading two time steps in the past
        thresh - Threshold for determining if intenttional activation has occurred
    OUTPUT:
        intent - Boolean determining whether intentional activation has occurred
    '''

    # Combine Force Sensor History
    padVal = [padValCur,padValPrev1,padValPrev2]

    # Set Intent Flag Based On Force Sensor History
    intent = all( i > thresh for i in padVal)

    # Return Intent Flag
    return intent

def forceOff(padValCur, padValPrev, thresh):

    '''
    Function to determine whether a force sensor of interest is considered 'deactivated' with intention. The determining factor involves readings below a threshold for at least two consecutive time steps.

    NOTE: Some of the documentation may be slightly misleading. Please refer to Ryan Posh for more information

    INPUT:
        padValCur - Current force sensor data reading
        padValPrev - Force sensor data reading one time step in the past
        thresh - Threshold for determining if intenttional deactivation has occurred
    OUTPUT:
        intent - Boolean determining whether intentional deactivation has occurred
    '''

    # Combine Force Sensor History
    padVal = [padValCur,padValPrev]

    # Set Intent Flag Based On Force Sensor History
    intent = all( i > thresh for i in padVal)

    # Return Intent Flag
    return intent
