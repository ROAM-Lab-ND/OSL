'''
##################################### OPEN #####################################
This script allows for testing the calibration of the ankle or knee of the Open Source Leg (OSL) without having to move the calibration yaml file locations

Last Update: 26 May 2021
Updates: Bug fixes with respect to imports
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

#################################### SETUP #####################################
# Find directory
scriptPath = os.path.dirname(os.path.abspath(__file__))
fpath = scriptPath + '/Ports_Single.yaml'
#print('Path: ',fpath)
ports, baudRate = fxu.load_ports_from_file(fpath)

# Standard setup that is not crucial for understanding the script
print(ports,'\n',baudRate)
port = str(ports[0])
baudRate=int(baudRate)
debugLvl=6

# Connect to actuator and open data stream
FX = fx.FlexSEA()

devId = FX.open(port,baudRate,debugLvl)
FX.start_streaming(devId,freq=100,log_en=False)
sleep(0.1)

dev = int(input('Which device do you want to calibrate? (0 for Knee, 1 for Ankle): '))
cal = int(input('What do you want to calibrate? (0 for IMU, 1 for Angle, 2 for Both): '))

calData = pac.CalDataSingle(cal,dev)

try:

    if dev == 0:
        pac.kneeCal(devId,FX,calData,cal)
    elif dev == 1:
        pac.ankleCal(devId,FX,calData,cal)
    else:
        raise Exception('Invalid device chosen...')

    # Disable the controller, send 0 PWM
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)
    sleep(0.1)

    FX.stop_streaming(devId)
    sleep(0.2)
    FX.close(devId)
    sleep(0.1)

except KeyboardInterrupt:

        print('User Interruption Occurred')

        # Disable the controller, send 0 PWM
        FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)
        sleep(0.1)

        FX.stop_streaming(devId)
        sleep(0.2)
        FX.close(devId)
        sleep(0.1)
        print("Graceful Exit Complete")

except Exception as e:
    print(e)

    # Disable the controller, send 0 PWM
    FX.send_motor_command(devId, fxe.FX_VOLTAGE, 0)
    sleep(0.1)

    FX.stop_streaming(devId)
    sleep(0.2)
    FX.close(devId)
    sleep(0.1)
    print("Graceful Exit Complete")

print('Script Complete')

