'''
##################################### OPEN #####################################
This script allows for testing the calibration of the ankle or knee of the Open Source Leg (OSL) without having to move the calibration yaml file locations

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
    - Updated main function to use OSL_CalibrationFunctions_DeviceOpenClose for opening and closing devices
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Imports for Standard Python
from time import sleep, time, strftime
import os, sys
import math
import numpy as np

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_Calibration_Package as pac
from OSL_Modules.OSL_Calibration import OSL_CalibrationFunctions_DeviceOpenClose as opcl

#################################### SETUP #####################################

# Create Class Object for Actuator Commands
FX = fx.FlexSEA()

# Open Device ID and Start Streaming
devId = opcl.devOpen(FX)

# Initialize Calibration Data Class Object
calData = pac.CalDataSingle()

# Let User Define Which Calibration Sequence to Run
cal = int(input('What do you want to calibrate? (0 for IMU, 1 for Angle, 2 for Both): '))

############################# CALIBRATION SEQUENCE #############################

try:

    # Choose Appropriate Calibration Process Based On Device ID
    if devId == osl.devKnee:

        pac.kneeCal(devId, FX, calData, cal)

    elif devId == osl.devAnk:

        pac.ankleCalMot(devId, FX, calData, cal)

    else:

        raise RuntimeError('Invalid device ID. Check device ID and compare with OSL_Constants.py stored values.')

except Exception as error:

    # Print Error
    print(error)

finally:

    # Gracefully Exit Script by Closing Stream and Device ID
    opcl.devClose(devId, FX)

print('Script Complete')
