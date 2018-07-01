import cv2
import numpy as np
import time
from visionFunctions import droidVision
from visionFunctions import goal
from visionFunctions import obstacle




from collections import deque

font = cv2.FONT_HERSHEY_SIMPLEX

def main():
    # Initialise FIFO queues
    histVPHeading = deque([0],7)
    histDroidOffset = deque([0],7)
    histObject = deque([0],3)
#    histObEdge = deque([0],5)
    histObDist = deque([0],5)
    # Initialise other variables 
    failedFrame = 0
    # Create object instances
    vision = droidVision()
    cap = cv2.VideoCapture('DRC2017Short.mp4')
    print('Entering while...')

    while(cap.isOpened()):
        # Process vision
        ret, frame = cap.read()
        frame = frame[450:frame.shape[0]-1,:,:]


        dataAvailable, vpX, vpY, vpHeading, droidOffset, obCentre, obWidth, obDistance, obHeading = vision.processFrame(frame)
              
        # Filter outputs
        if dataAvailable:
            failedFrame = 0
            histVPHeading.append(vpHeading)
            histDroidOffset.append(droidOffset)
            if obCentre:
                histObject.append(1)
                histObDist.append(obDistance)
            else:
                histObject.append(0)
            
        else:
            failedFrame += 1
        # If lines are not detected for 10 frames, remove history
        if failedFrame >= 10:
            histVPHeading = deque([0],10)
            histDroidOffset = deque([0],10)
            print("WARNING, 10 failed frames")
        # Calculate acceptable average of historical data
        avHeading = np.nanmedian(histVPHeading)
        avOffset = np.nanmedian(histDroidOffset)
        
        # Calculate path
        # To be function called if frame at framerate
        
        # Initialise state position, angle for frame
#        droidTheta = deque([0],2)
#        droidX = deque([0],2)
#        droidY = deque([0],2)
#        startTime = time.time()
#         #Between frames (not sure how to do this)
#
#        # Read current IMU feed
#        if IMUAvailable:
#            deltaTime = time.time() - startTime
#            startTime = time.time()
#            [IMUaccelX, IMUaccelY, IMUomega] = getIMU()
#
#
#            droidTheta.append()
#
##         Course correction - Angle
##         Face towards Vanishing point, reduce error to heading
#
#         #Tunable parameter - how to work out?
#        K_theta = 5
#        thetaError = 0 - avHeading
#        omega = K_theta * thetaError
#
#        # Calculate vector for travel
#        speed = 1
#        heading = avHeading


        # Display results
        
        centreY = 400 # Result of calibration

        width = frame.shape[1]
        centreX = int(width/2)
        cv2.putText(frame, "{:10.2f}".format(np.degrees(avHeading)), (20, 50), font, 0.8, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, "{:10.2f}".format(avOffset), (20, 100), font, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.line(frame,(centreX,centreY),(obCentre[0],obCentre[1]),(0,0,255),2)
        
        cv2.imshow('frame',frame)
        time.sleep(0.1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        print("new data")
        print(dataAvailable, ' ', np.degrees(vpHeading), ' ', droidOffset)
        print("moving average")
        print(dataAvailable, ' ', avHeading, ' ', avOffset)
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":

    main()

    
