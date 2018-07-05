import cv2
import time
import numpy as np
import pygame
import logging
logging.basicConfig(level=logging.INFO,
                      format='[%(levelname)s] (%(threadName)-9s) %(message)s',)

import piBotClient
from constants import *

SPEED = 1.0
OMEGA = 1.0

SAVE_VIDEO = True
noPressCounter = 0

if __name__=='__main__':

    pygame.init()
    screen = pygame.display.set_mode((100,50))

    keys = pygame.key.get_pressed()

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

    while (not keys[pygame.K_q]): # Quit key 'q'

        # Grab the frame
        if droid.frame_available:
            im = droid.frame
            droid.frame_available = False

            if SAVE_VIDEO:
                out.write(im)

            cv2.imshow("Droid Vision", im)
            cv2.waitKey(5)
        
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:                
                keys = pygame.key.get_pressed()
                '''
                #####################################################################################
                    DRIVING KEYS
                #####################################################################################
                '''        

                if droid.state == 0:

                    if (keys[pygame.K_w] and keys[pygame.K_d]): # Forward right
                        droid.setSpeed(SPEED, 1.571, -OMEGA)

                    elif (keys[pygame.K_w] and keys[pygame.K_a]): # Forward left
                        droid.setSpeed(SPEED, 1.571, OMEGA)

                    elif (keys[pygame.K_s] and keys[pygame.K_d]): # Back right
                        droid.setSpeed(-SPEED, 1.571, OMEGA)

                    elif (keys[pygame.K_s] and keys[pygame.K_a]): # Back Left
                        droid.setSpeed(-SPEED, 1.571, -OMEGA)

                    elif (keys[pygame.K_w]): # Forward
                        logging.debug("Going forward")
                        droid.setSpeed(SPEED, 1.571, 0)

                    elif (keys[pygame.K_s]): # Back
                        logging.debug("Going back")
                        droid.setSpeed(SPEED, 4.712, 0)   
                                    
                    elif (keys[pygame.K_d]): # Spin CW
                        logging.debug("Spinning clockwise")
                        droid.setSpeed(0, 0, -OMEGA)

                    elif (keys[pygame.K_a]): # Spin CCW
                        logging.debug("Spinning counterclockwise")
                        droid.setSpeed(0, 0, OMEGA)

                    elif (keys[pygame.K_LEFT]): # Strafe left
                        logging.debug("Strafing left")
                        droid.setSpeed(SPEED, 3.141, 0)

                    elif (keys[pygame.K_RIGHT]): # Strafe right
                        logging.debug("Strafing right")
                        droid.setSpeed(SPEED, 0, 0)

                    else:
                        logging.debug("Stop")
                        droid.setSpeed(0, 0, 0)
                '''
                #####################################################################################
                    SETTING KEYS
                #####################################################################################
                '''
                if (keys[pygame.K_m]): # Manual mode
                    logging.info("Manual mode initialised")
                    droid.setMode('manual')

                elif (keys[pygame.K_k]): # Auto mode
                    logging.info("Autonomous mode initialised")
                    droid.setMode('auto')
                
                elif (keys[pygame.K_y]): # Take photo
                    logging.info("Taking photo")
                    cv2.imwrite('test_videos/image.jpg', im)

    droid.StopCamStream()

    if SAVE_VIDEO:
        out.release() # Finish the movie file off

    pygame.quit()