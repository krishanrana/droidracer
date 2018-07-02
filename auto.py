import numpy as np
import time
import cv2

import piBotServer
import visionFunctions
from constants import *

def navigation(Heading, leftOffset, rightOffset, obstacle, obDist):
    
    raceSpeed = 1
    Komega = 2
    Kh = 1
    #Kt = 1
    
    # Determine angular velocity based on camera direction 
    theta = (1.5708 - Heading)
    omega =  theta * Komega 
    
    speed = raceSpeed
        
    # Determine vehicle heading vector in radians from x right = 0
    #trackOffset = leftOffset - rightOffset
    
    # Create a vehicle direction vector
    #vector = (theta * Kh) + (trackOffset * Kt)
    
    vector = (theta * Kh)
    
    return speed, vector, omega


def begin(droid):

    vision = visionFunctions.droidVision()

    while(not shutdown_sig):
        # Vision
        frame = droid.cam.read()
        avHeading, avLeftOffset, avRightOffset, obstacle, avObDist = vision.processFrame(frame)
        # Copy the debug frame to the droid class
        droid.frame_edited = vision.frame_edited
        
        # Nav
        speed, vector, omega = navigation(avHeading, avLeftOffset, avRightOffset, obstacle, avObDist)

        # Control
        droid.setSpeed(speed, vector, omega)



def stop(droid):
    global shutdown_sig
    shutdown_sig = True


def signalHandler(signal, frame):
    stop()


# JUST FOR TESTING
if __name__ == "__main__":
    import signal
    shutdown_sig = False
    signal.signal(signal.SIGINT, signalHandler)

    droid = piBotServer.PiBotServer()
    droid.StartCameraStream()           # Make sure you start the camera stream!
    droid.StartServers()                # Only if you are connecting a client
    droid.debug = True

    begin(droid) # The main loop

    time.sleep(0.1)
    droid.setSpeed(0,0,0)
    droid.close()