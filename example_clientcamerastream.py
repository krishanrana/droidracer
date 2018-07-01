'''
Example to show usage of the camera stream
ON THE SERVER. The client is still able to access the feed
'''
import time
import cv2

import piBotClient
#from constants import *


DEFAULT_HOST = '172.19.6.225'

PORT_CMDS_A = 5000
PORT_CMDS_B = 5001
PORT_CAMERA = 5002

DEFAULT_CAM_W = 640
DEFAULT_CAM_H = 480

if __name__=="__main__":
    droid = piBotClient.PiBotClient(host=DEFAULT_HOST)
    droid.StartCameraStream()
    while True:
        while not droid.frame_available:   # Wait till a frame is in the buffer
            time.sleep(0.01)
        droid.frame_available = False   # Reset the flag so you aren't processing the same image over and over
        im = droid.frame
        print("Received image of size: ")
        print(im.shape)
        cv2.imshow('Test window', im)
        cv2.waitKey(10)
        # Do want you want with im
