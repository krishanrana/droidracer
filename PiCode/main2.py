import cv2
import numpy as np
import time
from visionFunctions import droidVision

from collections import deque
import serial

ser = serial.Serial('/dev/ttyUSB4', 9600)
s = [0]

def tohex(val):
    out = hex(((abs(val) ^ 0xffff) + 1) & 0xffff)
    return out

state = 0

########Testing how to send the 3 sets of data to the arduino
def navigation(state, Heading, leftOffset, rightOffset, obstacle, obDist, obHeading, obEdge ):
    # Determine angular velocity based on camera direction 
    
    Komega = 5
    theta = (1.5708 - Heading)
    omega =  theta * Komega 
    
    # Determine speed based on state (ready = 0, race = 1, obstacles = 1 - K/distance to obstacle)
    # If checking systems or ready to race
    if state == 0 || 1:
        speed = 0
    # If racing
    elif state == 2:
        if obstacle:
           speed = raceSpeed * 1/obDist 
        else
           speed = raceSpeed
    # If lost
    elif state == 3: 
        speed = 0.2 * raceSpeed
    # If race finished
    elif state == 4: 
        speed = 0
        
    else:
        speed = 0
        
    # Determine vehicle heading vector in radians from x right = 0
    trackOffset = leftOffset - rightOffset
    vector = (theta * Kh) + (trackOffset * Kt)
    
    
    
    
    return omega




def piToArduino(speed, vector, omega)
    speed = str(speed)
    vector = str(vector)
    omega = str(omega)
    data_out = speed + "," + vector + "," + omega + "\n"

    ser.write(data_out.encode("ascii"))

def main():
    histHeading = deque([0],10)
    histOffset = deque([0],10)
    failedFrame = 0
    vision = droidVision()
    cap = cv2.VideoCapture('DRC2017Short.mp4')
    print('Entering while...')



############################


    while(cap.isOpened()):

        ret, frame = cap.read()
        dataAvailable, newHeading, newOffset = vision.processFrame(frame)

        if dataAvailable:
            failedFrame = 0
            histHeading.append(newHeading)
            histOffset.append(newOffset)
        else:
            failedFrame += 1

        if failedFrame >= 10:
            print("WARNING, 10 failed frames")

        avHeading = np.nanmedian(histHeading)
        avOffset = np.nanmedian(histOffset)
        speed , vector, omega = navigation()
        piToArduino(speed, vector, omega)
        #print("new data")
        #print(dataAvailable, ' ', newHeading, ' ', newOffset)
        #print("moving average")
        #print(dataAvailable, ' ', avHeading, ' ', avOffset)

    cap.release()



if __name__ == "__main__":

    main()


#TODO: USE CTR C INTERRUPT TO CLOSE SERIAL PORT ser.close()
