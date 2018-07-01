'''
Example to show usage of the camera stream
ON THE SERVER. The client is still able to access the feed
'''
import time
import cv2

import piBotClient
from constants import *

if __name__=="__main__":
    droid = piBotClient.PiBotClient(host=DEFAULT_HOST)
    
    print('Setting to auto')
    droid.setMode('auto')
    time.sleep(2)

    print('Setting to manual')
    droid.setMode('manual')
