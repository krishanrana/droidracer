import time

import piBotServer

if __name__=="__main__":
    droid = piBotServer.PiBotServer()
    droid.StartCameraStream()           # Make sure you start the camera stream!
    while True:
        while not droid.cam.frame_available:   # Wait till a frame read to the buffer
            time.sleep(0.01)
        droid.cam.frame_available = False   # Reset the flag so you aren't processing the same image over and over
        im = droid.cam.read()
        print("The size of the image received is: ")
        print(im.shape)
        # Do want you want with im
