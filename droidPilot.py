""" This Script is front end / GUI to allow control of omni droid using:
- Displacement / Pose control: input via GUI -DONE
- Closed loop control as above -DONE
- Manual drive using game controller """

import logging
import signal
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import PySimpleGUI as psg

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

    dc.testVectorDrive(Distance, Heading)
    time.sleep(2)
    dc.testRotationDrive(Rotation)
    print('All done')
# Method to drive droid using GUI pose / direction control
def guiInput():
    psg.theme('Reddit')  

    layout = [[psg.Text('Robot status:'), psg.Text(size=(18,1), key='-STATUS-')],
            [psg.Text('Heading:'), psg.Slider(range=(180,-180),default_value=0,
            key='-HEAD-', size=(20,15), orientation='horizontal',font=('Helvetica', 12))],
            [psg.Text('Distance:'), psg.Slider(range=(0,2000), default_value=0,
            key='-DIST-', size=(20,15), orientation='horizontal',font=('Helvetica', 12))],
            [psg.Text('Rotation:'), psg.Slider(range=(180,-180),default_value=0,
            key='-ROT-', size=(20,15),orientation='horizontal',font=('Helvetica', 12))],
            [psg.Text('Linear Speed:'), psg.Slider(range=(1,10),default_value=0,
            key='-L_SPD-',size=(20,15),orientation='horizontal',font=('Helvetica', 12))],
            [psg.Text('Angular Speed:'), psg.Slider(range=(1,10),default_value=0,
            key='-A_SPD-',size=(20,15),orientation='horizontal',font=('Helvetica', 12))],
            [psg.Checkbox('Use Odometry Filter', default=False, key='-FILTER-'),
            psg.Slider(range=(0,100),default_value=90,key='-DATA_A-', size=(20,15),
            orientation='horizontal',font=('Helvetica', 12))],
            [psg.Checkbox('Relative Motion', default=True, key='-MOTION-'),psg.Button('Run'), psg.Button('Exit')]]
    
    

    window = psg.Window('GUI muthafucka!', layout)

    while True:  # Event Loop
        event, values = window.read()
        print(event, values)
        if event == psg.WIN_CLOSED or event == 'Exit':
            break
        if event == 'Run':
            # change the "output" element to be the value of "input" element   
            window['-STATUS-'].update('Running')
            # Clear previous droid states - for relative motion only
            if values['-MOTION-'] == True:
                dc.xEst = np.array([0,0,0])
                dc.vEst = np.array([0,0,0])
            if values['-FILTER-'] == True:
                dc.compFilterEncoderValue = values.get('-DATA_A-')/100
                odoType = 'imuFusion'
            else:
                odoType = 'encoder' 
            # move robot to command
            deg2rad = np.pi/180
            Heading = (values.get('-HEAD-') + 90) *deg2rad
            Distance = values.get('-DIST-')/1000
            Rotation = ((values.get('-ROT-') + 360) % 360) *deg2rad
            dc.AngularSpeed = values.get('-A_SPD-')/10 * dc.MaxAngularVelocity
            dc.LinearSpeed = values.get('-L_SPD-')/10 * dc.MaxLinearVelocity
            # transform heading / distance to X,Y (X to right, Y forwards, heading angle CCW from Y, Rotation 0-360)
            target = np.array([Distance * np.cos(Heading),Distance * np.sin(Heading), Rotation])
            logger.debug('Target x: {0:0.3f}, y: {1:0.3f}, theta: {2:0.3f},'.format(target[0],target[1],target[2]))
            dc.navController(target, odoType) # 'imuFusion','none'
            window['-STATUS-'].update('Movement complete')
            
    

    window.close()



if __name__ == '__main__':
    initLogger()
    initPilot()
    guiInput()
#     manualSetInput()
    dc.close
