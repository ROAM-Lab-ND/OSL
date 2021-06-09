'''
##################################### OPEN #####################################
This script allows for testing the calibration of the ankle or knee of the Open Source Leg (OSL) without having to move the calibration yaml file locations

Last Update: 8 June 2021
Updates:
    - Updated loading structure to load both actuators correctly without requiring user assistance
    - Updated calibration to point to motor encoder method
    - Updated try statement use finally tag for proper syntax of closing actuator stream
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

from OSL_Modules.OSL_Calibration import OSL_Constants as osl
from OSL_Modules.OSL_Calibration import OSL_Calibration_Package as pac

#################################### SETUP #####################################
# Find directory
scriptPath = os.path.dirname(os.path.abspath(__file__))
fpath = scriptPath + '/Ports_Single.yaml'
ports, baudRate = fxu.load_ports_from_file(fpath)

# Standard setup that is not crucial for understanding the script
print(ports,'\n',baudRate)
port1 = str(ports[0])
port2 = str(ports[1])
baudRate=int(baudRate)
debugLvl=6

# Connect to actuator and open data stream
FX = fx.FlexSEA()

devId1 = FX.open(port1,baudRate,debugLvl)
devId2 = FX.open(port2,baudRate,debugLvl)

if devId1 == 19048:
    devId = [devId1,devId2]
else:
    devId = [devId2,devId1]

FX.start_streaming(devId[0],freq=100,log_en=False)
FX.start_streaming(devId[1],freq=100,log_en=False)
sleep(0.1)

calData = osl.CalDataDual()

cal = int(input('What do you want to calibrate? (0 for IMU, 1 for Angle, 2 for Both): '))

try:

    dualCalMot(devId,FX,calData,cal)

except Exception as error:
    print(error)

    # Disable the controller, send 0 PWM
    sleep(0.05)
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)
    sleep(0.1)

finally:

    FX.stop_streaming(devId)
    sleep(0.2)
    FX.close(devId)
    sleep(0.1)
    print("Graceful Exit Complete")

print('Script Complete')
