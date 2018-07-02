import cv2
import time
import numpy as np
import logging
logging.basicConfig(level=logging.DEBUG,
                      format='[%(levelname)s] (%(threadName)-9s) %(message)s',)

import piBotClient

SPEED = 1

if __name__=='__main__':

    # Connect to the server
    droid = piBotClient.PiBotClient()
    droid.StartCameraStream()

    # Wait until a first frame is available
    while (not droid.frame_available):
        time.sleep(0.01)
    key = cv2.waitKey(100)

    while key != ord('q'):
        im = droid.frame

        cv2.imshow("Droid Vision", im)
        key = cv2.waitKey(100)

        if key == ord('w'): # Forward
            logging.debug("Going forward")
            droid.setSpeed(SPEED, np.pi/2, 0)

        elif key == ord('s'): # Back
            logging.debug("Going back")
            droid.setSpeed(SPEED, 3*np.pi/2, 0)

        elif key == ord('a'): # Left
            logging.debug("Going left")
            droid.setSpeed(SPEED, np.pi, 0)

        elif key == ord('d'): # Right
            logging.debug("Going right")
            droid.setSpeed(SPEED, 0, 0)

        else:
            droid.setSpeed(0, 0, 0)

        logging.debug("Key pressed: %s", key)
