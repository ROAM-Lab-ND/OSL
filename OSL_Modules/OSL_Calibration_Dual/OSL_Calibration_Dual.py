'''
##################################### OPEN #####################################
This script allows for testing the calibration of the ankle or knee of the Open Source Leg (OSL) without having to move the calibration yaml file locations

Last Update: 21 June 2021
Updates:
    - Improved Comments and Documentation
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

# Imports for Standard Python
from time import sleep, time, strftime
import os, sys
import math
import numpy as np
import scipy as sp
import yaml

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration_Dual import OSL_Constants as osl
from OSL_Modules.OSL_Calibration_Dual import OSL_Calibration_Package as pac
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_DeviceOpenClose as opcl

#################################### SETUP #####################################

# Create Class Object for Actuator Commands
FX = fx.FlexSEA()

# Open Device ID and Start Streaming
devId = opcl.devOpen(FX)

# Create Class Object for Calibration Data
calData = pac.CalDataDual()

# Let User Choose Calibration
cal = int(input('What do you want to calibrate? (0 for IMU, 1 for Angle, 2 for Both): '))

try:

    # Run Calibration Sequence
    pac.dualCalMot(devId, FX, calData, cal)

except Exception as error:

    # Print Error
    print(error)

finally:

    # Gracefully Exit Script by Closing Stream and Device ID
    opcl.devClose(devId, FX)

print('Script Complete')
