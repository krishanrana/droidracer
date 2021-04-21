""" This Script is front end / GUI to allow control of omni droid using:
- Displacement / Pose control: input via GUI
- Closed loop control as above
- Manual drive using game controller """

import copy
import logging
import os
import signal
import struct
import sys
import time
from threading import Thread

import matplotlib.pyplot as plt
import numpy as np
import PySimpleGUI as psg
import serial

from DroidControl import droidControl 



# Setup logging
def initLogger():
    # Log file location
    logfile = 'debug.log'

    # Define your own logger name
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
# Method to test drive droid using static input
def manualSetInput():

    t0 = time.time()
            
    #Use GUI loop for online changes
    # Get user input 
    dc.LinearSpeed = 0.2 # m.s^-1
    dc.AngularSpeed = 0.1 * np.pi # degrees.s^-1

    Heading = 3*np.pi/5 # radians in robot frame
    Distance = 1 # metres
    Rotation = np.pi/4 # radian in world frame

    dc.Kprop = 1.5 # Proportional gain
    dc.Kint = 40 # Integral gain
    dc.Kder = 0.001 # Derivative gain
    dc.runCommand = 0.0

    dc.testVectorDrive(Distance, Heading)
    time.sleep(2)
    dc.testRotationDrive(Rotation)
    print('All done')
# Method to drive droid using GUI pose / direction control
def guiInput():
    psg.theme('Reddit')  

    llayout = [[sg.Text('Drive robot to:'), sg.Text(size=(18,1), key='-OUTPUT-')],
            [sg.Text('Heading:'), sg.Slider(range=(-180,180),default_value=0,
            key='-HEAD-', size=(20,15), orientation='horizontal',font=('Helvetica', 12))],
            [sg.Text('Distance:'), sg.Slider(range=(0,2000), default_value=0,
            key='-DIST-', size=(20,15), orientation='horizontal',font=('Helvetica', 12))],
            [sg.Text('Rotation:'), sg.Slider(range=(-180,180),default_value=0,
            key='-ROT-', size=(20,15),orientation='horizontal',font=('Helvetica', 12))],
            [sg.Text('Speed:'), sg.Slider(range=(0,10),default_value=0,
            key='-SPD-',size=(20,15),orientation='horizontal',font=('Helvetica', 12))],
            [sg.Button('Run'), sg.Button('Exit')]]
    

    window = psg.Window('GUI muthafucka!', layout)

    while True:  # Event Loop
        event, values = window.read()
        print(event, values)
        if event == psg.WIN_CLOSED or event == 'Exit':
            break
        if event == 'Run':
            # change the "output" element to be the value of "input" element
            window['-OUTPUT-'].update('Running')
    

    window.close()



if __name__ == '__main__':
    initLogger()
    initPilot()
    manualSetInput()
    dc.close




