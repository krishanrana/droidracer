import numpy as np
import time
import piBotServer
import visionFunctions
import signal

import serial

ser = serial.Serial('/dev/ttyUSB4', 9600)
s = [0]


droid = piBotServer.PiBotServer()
droid.StartCameraStream()           # Make sure you start the camera stream!
droid.StartServers()                # Only if you are connecting a client









def tohex(val):
    out = hex(((abs(val) ^ 0xffff) + 1) & 0xffff)
    return out







########Testing how to send the 3 sets of data to the arduino
def navigation(state, Heading, leftOffset, rightOffset, obstacle, obDist):
    
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




def piToArduino(speed, vector, omega):
    speed = str(speed)
    vector = str(vector)
    omega = str(omega)
    data_out = speed + "," + vector + "," + omega + "\n"

    ser.write(data_out.encode("ascii"))

def main():


    vision = visionFunctions.droidVision()
    
    print('Entering while...')



    while not droid.cam.frame_available:   # Wait till a frame is in the buffer
        time.sleep(0.01)
    droid.cam.frame_available = False 

    while(1):
        
       
        frame = droid.cam.read()
            
        avHeading, avLeftOffset, avRightOffset, obstacle, avObDist = vision.processFrame(frame)
        
        speed , vector, omega = navigation(avHeading, avLeftOffset, avRightOffset, obstacle, avObDist)
        
        piToArduino(speed, vector, omega)


def signalHandler(signal, frame):
    ser.close()
    pass


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signalHandler)
    
    main()


#TODO: USE CTR C INTERRUPT TO CLOSE SERIAL PORT ser.close()
