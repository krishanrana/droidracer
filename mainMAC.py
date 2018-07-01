import cv2
import numpy as np
import time
from visionFunctions import droidVision


from collections import deque

font = cv2.FONT_HERSHEY_SIMPLEX

def main():
    # Initialise FIFO queues
    histVPHeading = deque([0],10)
    histLeftOffset = deque([0],10)
    histRightOffset = deque([0],10)
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

        frameOut, dataAvailable, vpHeading, droidOffset, obCentre, obWidth, obDistance, obHeading = vision.processFrame(frame)
              
        # Filter outputs
        if dataAvailable == 1:
            failedFrame = 0
            histVPHeading.append(vpHeading)
            histLeftOffset.append(droidOffset[0])
            histRightOffset.append(droidOffset[1])
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
            histLeftOffset = deque([0],10)
            histRightOffset = deque([0],10)
            print("WARNING, 10 failed frames")
        # Calculate acceptable average of historical data
        
        avHeading = np.nanmedian(histVPHeading)
        avOffset = (np.nanmedian(histLeftOffset) - np.nanmedian(histRightOffset))
        
        
        centreY = 400

        width = frame.shape[1]
        centreX = int(width/2)
        cv2.putText(frameOut, "{:10.2f}".format(np.degrees(avHeading)), (20, 50), font, 0.8, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frameOut, "{:10.2f}".format(avOffset), (20, 100), font, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frameOut, "{:10.2f}".format(obDistance), (20, 150), font, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.line(frameOut,(centreX,centreY),(obCentre[0],obCentre[1]),(0,0,255),2)
        cv2.line(frameOut,(centreX,centreY),(int(centreY*(np.tan(avHeading))),0),(255,0,255),2)
        
        cv2.imshow('frame',frameOut)
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

    
