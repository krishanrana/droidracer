import cv2
import time
import numpy as np
import logging
logging.basicConfig(level=logging.DEBUG,
                      format='[%(levelname)s] (%(threadName)-9s) %(message)s',)

import piBotClient
from constants import *

SPEED = 1
OMEGA = 2

SAVE_VIDEO = False
noPressCounter = 0

if __name__=='__main__':

    if SAVE_VIDEO:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        size = (int(DEFAULT_CAM_W*CAM_SCALING),int(DEFAULT_CAM_H*CAM_SCALING))
        out = cv2.VideoWriter('output.avi',fourcc, 5.0, size)

    # Connect to the server
    droid = piBotClient.PiBotClient()
    droid.StartCameraStream()

    # Wait until a first frame is available
    while (not droid.frame_available):
        time.sleep(0.01)
    key = cv2.waitKey(10)

    while key != ord('q'):

        # Grab the frame
        if droid.frame_available:
            im = droid.frame
            droid.frame_available = False

            if SAVE_VIDEO:
                out.write(im)

            cv2.imshow("Droid Vision", im)
        

        # Get the next key press
        key = cv2.waitKey(10)

        '''
        #####################################################################################
            DRIVING KEYS
        #####################################################################################
        '''
        if droid.state == 0:
            if key == ord('w'): # Forward
                logging.debug("Going forward")
                droid.setSpeed(SPEED, 1.571, 0)
                noPressCounter = 0

            elif key == ord('s'): # Back
                logging.debug("Going back")
                droid.setSpeed(SPEED, 4.712, 0)
                noPressCounter = 0

            elif key == ord('a'): # Left
                logging.debug("Going left")
                droid.setSpeed(SPEED, 3.142, 0)
                noPressCounter = 0

            elif key == ord('d'): # Right
                logging.debug("Going right")
                droid.setSpeed(SPEED, 0, 0)
                noPressCounter = 0
            
            elif key == ord('z'): # Spin left
                logging.debug("Spinning left")
                droid.setSpeed(0, 0, OMEGA)
                noPressCounter = 0

            elif key == ord('c'): # Spin right
                logging.debug("Spinning right")
                droid.setSpeed(0, 0, -OMEGA)
                noPressCounter = 0

            elif key == ord('x'): # Stop
                logging.debug("Stop")
                droid.setSpeed(0, 0, 0)
                noPressCounter = 0
                
            else:
                noPressCounter += 1
                print('Counter: ', noPressCounter)
                if noPressCounter >= 10:
                    droid.setSpeed(0, 0, 0)
                    noPressCounter = 0

        '''
        #####################################################################################
            SETTING KEYS
        #####################################################################################
        '''
        if key == ord('m'): # Manual mode
            logging.debug("Manual mode initialised")
            droid.setMode('manual')

        elif key == ord('k'): # Auto mode
            logging.debug("Autonomous mode initialised")
            droid.setMode('auto')

    droid.StopCamStream()

    if SAVE_VIDEO:
        out.release() # Finish the movie file off

    logging.info("Press any key to exit.")
    cv2.waitKey(0)

