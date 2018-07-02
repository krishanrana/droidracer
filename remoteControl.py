import cv2
import time
import numpy as np
import logging
logging.basicConfig(level=logging.DEBUG,
                      format='[%(levelname)s] (%(threadName)-9s) %(message)s',)

import piBotClient


if __name__=='__main__':

    # Connect to the server
    droid = piBotClient.PiBotClient()
    droid.StartCameraStream()

    # Wait until a frame is available
    while (not droid.frame_available):
        time.sleep(0.01)
    
    im = droid.frame

    cv2.imshow("Droid Vision")
    key = cv2.waitKey(10)

    if key == ord('w'): # Forward
        logging.debug("Going forward")
        droid.setSpeed(2, np.pi, 0)

    elif key == ord('s'): # Back
        logging.debug("Going back")
        droid.setSpeed(-2, np.pi, 0)

    elif key == ord('a'): # Left
        logging.debug("Going left")
        droid.setSpeed(0, np.pi, 0.5)

    elif key == ord('d'): # Right
        logging.debug("Going right")
        droid.setSpeed(0, np.pi, -0.5)

    else:
        droid.setSpeed(0, 0, 0)

