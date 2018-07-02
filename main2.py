import numpy as np
import time
import piBotServer
import visionFunctions
import signal

droid = piBotServer.PiBotServer()
droid.StartCameraStream()           # Make sure you start the camera stream!
droid.StartServers()                # Only if you are connecting a client
droid.debug = True


def tohex(val):
    out = hex(((abs(val) ^ 0xffff) + 1) & 0xffff)
    return out


########Testing how to send the 3 sets of data to the arduino
def navigation(Heading, leftOffset, rightOffset, obstacle, obDist):
    
    raceSpeed = 1
    Komega = 2
    Kh = 1
    #Kt = 1
    
    # Determine angular velocity based on camera direction 
    
    theta = (1.5708 - Heading)
    omega =  theta * Komega 
    
    # Determine speed based on state (ready = 0, race = 1, obstacles = 1 - K/distance to obstacle)
    # If checking systems
    if droid.state == 0:
        speed = 0
    # Ready to race
    elif droid.state == 1:
        speed = raceSpeed
        
    else:
        speed = 0
        
    # Determine vehicle heading vector in radians from x right = 0
   # trackOffset = leftOffset - rightOffset
    
    # Create a vehicle direction vector
    #vector = (theta * Kh) + (trackOffset * Kt)
    
    vector = (theta * Kh)
    
    return speed, vector, omega


def main():

    vision = visionFunctions.droidVision()
    droid.state = 0
    print('Entering while...')

    while not droid.cam.frame_available:   # Wait till a frame is in the buffer
        time.sleep(0.01)

    while(not shutdown_sig):
        # Vision
        frame = droid.cam.read()
        avHeading, avLeftOffset, avRightOffset, obstacle, avObDist = vision.processFrame(frame)
        droid.debug_frame = vision.debugFrame
        
        # Nav
        speed, vector, omega = navigation(avHeading, avLeftOffset, avRightOffset, obstacle, avObDist)

        # Control
        print(speed)
        print(vector)
        print(omega)
        droid.setSpeed(speed, vector, omega)


def signalHandler(signal, frame):
    global shutdown_sig
    shutdown_sig = True
    droid.close()


if __name__ == "__main__":
    shutdown_sig = False
    signal.signal(signal.SIGINT, signalHandler)
    main()
