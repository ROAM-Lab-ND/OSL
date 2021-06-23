'''
##################################### OPEN #####################################
This script houses the parameters and conversion factors that will be used with the Open Source Leg (OSL).

The parameters stored in this script pertain to various factors of the control scheme, ranging from general conversion units to sensor specifics.

Last Update: 21 June 2021
Updates:
    - Added Time Steps and Time Lengths Parameters
    - Added Battery Voltage Parameters
#################################### CLOSE #####################################
'''

############################### PACKAGE IMPORTS ################################

import numpy as np

############################ CONSTANTS / PARAMETERS ############################

# Degree/Radians Conversion
deg2rad = np.pi/180
rad2deg = 180/np.pi

# Dephy Actuator Data Conversions (Pulled from Wiki)
deg2count = 2**14/360
gyroConv = 32.8
accelConv = 8192

# Time Steps (sec)
dtMilli = 0.001
dtCenti = 0.01
dtDeci = 0.1
dtSec = 1.0

# Time Lengths (sec)
sec10 = 10
sec100 = 100

# Dephy Actuator Stiffness and Damping Parameters
stiffK = ((0.096*deg2count)/np.sqrt(3))/(1000*2**9)
dampB = ((0.096*deg2count)/np.sqrt(3))/(1000*1000)

# Force Sensor Channel Values
pChan0 = 0
pChan1 = 1

# Device IDs for Knee and Ankle actuators
devKnee = 17763
devAnk = 14449

# Battery Voltage Threshold (mV) and Checking Rate (min)
batCheckRate = 1
threshBat = 28000
