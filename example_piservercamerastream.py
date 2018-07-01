'''
Example to show usage of the camera stream
ON THE SERVER. The client is still able to access the feed
'''
import time

import piBotServer


if __name__=="__main__":
    droid = piBotServer.PiBotServer()
    droid.StartCameraStream()           # Make sure you start the camera stream!
    droid.StartServers()                # Only if you are connecting a client
    while True:
        while not droid.cam.frame_available:   # Wait till a frame is in the buffer
            time.sleep(0.01)
        droid.cam.frame_available = False   # Reset the flag so you aren't processing the same image over and over
        im = droid.cam.read()
        print("Received image of size: ")
        print(im.shape)
        # Do want you want with im
