""" This Script is front end / GUI to allow control of omni droid using:
- Displacement / Pose control: input via GUI -DONE
- Closed loop control as above -DONE
- Manual drive using game controller """

import logging
import signal
import sys
import time
import pygame
from PyJoystick import pyJoystick

import numpy as np

from DroidControl import droidControl 

# Setup logging
def initLogger():
    # Log file location
    logfile = 'debug.log'

    # Define your own logger name
    global logger
    logger = logging.getLogger("droidPilotLog")
    # Set default logging level to DEBUG
    logger.setLevel(logging.DEBUG)

    # create console handler
    print_format = logging.Formatter('[%(levelname)s] (%(threadName)-9s) %(message)s')
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG)
    console_handler.setFormatter(print_format)

    # create log file handler
    # and define a custom log format, set its log level to DEBUG
    log_format = logging.Formatter('[%(asctime)s] %(levelname)-8s %(name)-12s %(message)s')
    file_handler = logging.FileHandler(logfile)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(log_format)

    #Add handlers to the logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
# Ctrl - C signal handler    
def signal_handler(signal, frame):
    global shutdown_flag
    shutdown_flag = True
    dc.close()
    print('Closing gracefully. Bye Bye!')
    sys.exit()
# Initialisation of class instances, variables
def initPilot():
    # Initialise shutdown
    shutdown_flag = False
    signal.signal(signal.SIGINT, signal_handler) 

    global dc
    dc = droidControl()
    dc.Kprop = 1.5 # Proportional gain
    dc.Kint = 40 # Integral gain
    dc.Kder = 0.001 # Derivative gain
    dc.runCommand = 0.0
    dc.LinearSpeed = 1.0
    dc.AngularSpeed = 3.1415

    global f710
    f710 = pyJoystick()
    



if __name__ == '__main__':
    initLogger()
    initPilot()

    while f710.stop == 0:
#         t0 = time.time()
        f710.getJoystickInput()
        print([f710.dirX, -f710.dirY, f710.rotL, f710.rotR])
#         print(time.time() - t0)
        dc.processCommands(f710.dirX, f710.dirY, f710.rotL, f710.rotR)
        dc.driveDroid()
        time.sleep(0.05)
        
    dc.saveOutput(dc.saveState,filename='telemetry.csv')
    dc.runCommand = 0
    dc.writeSerial()
    dc.close()
    time.sleep(2)
    sys.exit()
