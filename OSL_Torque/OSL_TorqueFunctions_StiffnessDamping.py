'''
##################################### OPEN #####################################
This script allows for testing the calibration of the ankle or knee of the Open Source Leg (OSL) without having to move the calibration yaml file locations

Last Update: 26 May 2021
Updates: Created
#################################### CLOSE #####################################
'''
############################### PACKAGE IMPORTS ################################
# Standard Python Modules
from time import sleep, time, strftime
import os, sys
import math
import numpy as np
import scipy as sp
import yaml

# Actuator Modules (Most Start with fx)
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

from OSL_Calibration import OSL_Constants as osl
from OSL_Calibration import OSL_Calibration_Package as pac

############################# FUNCTION DEFINITION ##############################

def motStiffness(userWeight, stiffNormDes, jointTR):
        """
        This function is used to determine the appropriate stiffness parameter K to pass to the impedance control for a Dephy actuator.

        NOTE: This current version is written for Dephy firmware version 5.0

        INPUT:
            userWeight - weight of the user in kilograms
            stiffNormDes - desired normalized stiffness for the ankle
            jointTR - transmission ratio for the current position of the joint

        OUTPUT:
            desK - desired stiffness parameter K to pass to impedance controller
        """
        # high des stiffness = 0.12 Nm/deg/kg
        # stnd des stiffness = 0.06 Nm/deg/kg
        # low des stiffness  = 0.02 Nm/deg/kg
        # Sup et al. use {0.0678, 0.1214, 0.0214, 0.0428}
        # OSL folks use {0.07, 0.07, 0.05, 0.05}

        # Calculate desired unnormalized joint stiffness
        stiffDes = userWeight*stiffNormDes

        # Calculate desired unnormalized motor stiffness
        stiffMotDes = stiffDes/(jointTR**2)

        # Calculate desired stiffness parameter K
        desK = int(stiffMotDes/osl.stiffK)

        return(desK)

def motDamping(userWeight, dampNormDes, jointTR):
        """
        This function is used to determine the appropriate damping parameter B to pass to the impedance control for a Dephy actuator.

        NOTE: This current version is written for Dephy firmware version 5.0

        INPUT:
            userWeight - weight of the user in kilograms
            dampNormDes - desired normalized damping for the ankle
            jointTR - transmission ratio for the current position of the joint

        OUTPUT:
            desB - desired damping parameter B to pass to impedance controller
        """
        # Rouse et al. use {0.00035, 0.00017, 0.00035, 0.00017}
        # OSL folks use {0, 0, 0, 0}

        # Calculate desired unnormalized joint damping
        dampDes = userWeight*dampNormDes

        # Calculate desired unnormalized motor damping
        dampMotDes = dampDes/(jointTR**2)

        # Calculate desired damping parameter B
        desB = int(dampMotDes/osl.dampB)

        return(desired_Zb)
