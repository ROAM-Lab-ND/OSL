'''
##################################### OPEN #####################################
This script houses the parameters and conversion factors that will be used with the Open Source Leg (OSL).

The parameters stored in this script pertain to various factors of the control scheme, ranging from general conversion units to sensor specifics.

Last Update: 18 May 2021
Updates: Created
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

# Time Step
dt = 0.001

# Dephy Actuator Stiffness and Damping Parameters
stiffK = ((0.096*deg2count)/np.sqrt(3))/(1000*2**9)
dampB = ((0.096*deg2count)/np.sqrt(3))/(1000*1000)
