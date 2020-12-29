from __future__ import division
import cv2
import numpy as np
import logging
import math
import time
from collections import deque

from constants import *


H = np.array([[ -1.07693537e+01,  -1.03211106e+00,   4.41554723e+03],
              [ -5.61158586e-02,   7.60254152e-01,  -7.36972948e+03],
              [ -3.59519827e-04,  -5.10817084e-02,   1.00000000e+00]])
            


class droidVision():

    def __init__(self):
        
        self.dataAvailable = 0
        self.failedFrame = 0
        self.histVPHeading = deque([0],3)
        self.histLeftOffset = deque([0],3)
        self.histRightOffset = deque([0],3)
        self.histObDist = deque([0],3)
        self.obMissing = 0
        self.obstacle = False

        self.frameNo = 0
        self.kernel = np.ones((5,5),np.uint8)

        self.vpX = 0
        self.vpY = 0
        self.leftOffset = 0
        self.rightOffset = 0

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        self.threshYellow = 160
        self.threshBlue = 110
        self.threshPurple = 160
        
        self.cannyLow = 30
        self.cannyHigh = 80
        
        self.thetaThresh = np.pi/180 *1.5
        self.rhoThresh = 80
        self.minLineLength = 100 
        self.maxLineGap = 30 
        self.centreY = DEFAULT_CAM_H # Result of calibration
        self.canny = np.empty((DEFAULT_CAM_H,DEFAULT_CAM_W,3))
        self.frame_edited = np.empty((DEFAULT_CAM_H,DEFAULT_CAM_W,3))

    # Main method
    def processFrame(self, frame):
        
        centreX = np.round(frame.shape[1]/2)
        clA,clB = self.raw2Frame(frame)
            
        purple, yellow, blue = self.thresholdFrame(clA, clB)
        self.yellow = yellow *255
        self.blue = blue *255
        self.purple = purple *255
        
        yM,yEdgeCrossing,yMeanPoint = self.detectLine(yellow)
        bM,bEdgeCrossing,bMeanPoint = self.detectLine(blue)
        self.vanishingPoint(yM,yEdgeCrossing,bM,bEdgeCrossing)
        self.robotHeading(yM,yMeanPoint,bM,bMeanPoint)
        
        goalHeading = np.nanmean(self.histVPHeading)
        trackLeftOffset = np.nanmean(self.histLeftOffset)
        trackRightOffset = np.nanmean(self.histRightOffset)
        obstacleDist = np.nanmean(self.histObDist)
        
        
        try:
            cv2.line(self.frame_edited,(int(bEdgeCrossing),int(self.centreY)),(int(self.vpX),int(self.vpY)),(0,255,0),2)
            cv2.line(self.frame_edited,(int(yEdgeCrossing),int(self.centreY)),(int(self.vpX),int(self.vpY)),(0,255,0),2)
            # Show direction to vanishing point
            cv2.line(self.frame_edited,(int(centreX),int(self.centreY)),(int(self.vpX),int(self.vpY)),(0,0,255),2)
        except:
            pass #print('cant show lines')  
        # Draw outputs
        try:
            cv2.line(self.frame_edited,(0,bottomEdge[1]),(width,bottomEdge[1]),(0,255,0),2)
            cv2.circle(self.frame_edited, center, 5, (0,0,255), -1)
        except:
            logging.debug('no object rect')
            
        
        return goalHeading, trackLeftOffset, trackRightOffset, self.obstacle, obstacleDist
           
            
    def raw2Frame(self,frame):
    
        if frame is not None:
            # Create frames for processing
#            frameLAB = cv2.cvtColor(frame,cv2.COLOR_BGR2LAB) 
            frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2LAB) 

            self.frame_edited = np.copy(frame)
#            l,a,b = cv2.split(frameLAB)
            l,a,b = cv2.split(frameHSV)

#            clA = self.clahe.apply(a) # histogram adjustment
#            clB = self.clahe.apply(b) # histogram adjustment
            
            return a,b
        else:
            self.failedFrame +=1
                
    def BGR2invariant(self,frame,alpha):
        #G - B - R BGR
        invariant = 0.5 + np.log(frame[:,:,1]) - alpha * np.log(frame[:,:,2]) - (1-alpha) * np.log(frame[:,:,0]);
        return invariant
        
        
    def thresholdFrame(self, clA, clB):
        thresh,purple = cv2.threshold(clA,self.threshPurple,1,cv2.THRESH_BINARY)
        retY,yellow = cv2.threshold(clB,self.threshYellow,1,cv2.THRESH_BINARY)
        retB,blue = cv2.threshold(clB,self.threshBlue,1,cv2.THRESH_BINARY)
        blue = cv2.bitwise_not(blue)-254 # invert blue line
        
#        ret2,th2 = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        return purple, yellow, blue
    
    def detectLine(self,channel):
            channel = cv2.Canny(channel, self.cannyLow, self.cannyHigh,5)
#            channel = cv2.morphologyEx(channel, cv2.MORPH_CLOSE, self.kernel)

            lines = np.squeeze(cv2.HoughLinesP(channel,cv2.HOUGH_PROBABILISTIC,self.thetaThresh,self.rhoThresh,self.minLineLength,self.maxLineGap))#detect lines
            self.canny = channel
            if lines.ndim >= 2:
                grad = (lines[:,0]-lines[:,2])/(lines[:,1]-lines[:,3]+0.0001)# find gradient of lines
                filt = rejectOutliers(grad, m=5)
                M = np.median(filt)
                #find intersection point with baseline centreY, using gradient and mean point
                meanPoint = np.sum((lines[:,0] + lines[:,2])/(2*lines.shape[0])),np.sum((lines[:,1] + lines[:,3])/(2*lines.shape[0]))
                EdgeCrossing = meanPoint[0] + M * (self.centreY - meanPoint[1])
                #crossingPoint = [pointX,pointY]
            
                for x1,y1,x2,y2 in lines:
                    cv2.line(self.frame_edited,(x1,y1),(x2,y2),(0,255,0),1)
            else:
#                print('VS209: HoughLines not found')
                M = None
                EdgeCrossing = None
                meanPoint = None
                
                    
            return M,EdgeCrossing,meanPoint
                    
 
    def vanishingPoint(self,yM,yEdgeCrossing,bM,bEdgeCrossing):
        # Conditional to create vP or virtual vP
        # Both lines visible
        if bM != None and yM != None:
            
            self.vpY = (yEdgeCrossing - bEdgeCrossing)/(bM - yM) + self.centreY
            self.vpX = bM * (self.vpY - self.centreY) + bEdgeCrossing
            self.dataAvailable = 1
            
            # Only Blue line visible, set VP to be far away and extend visible line
        elif bM != None and yM == None:
            self.vpY = -10000 - self.centreY
            self.vpX = bM * self.vpY + bEdgeCrossing
            self.dataAvailable = 1
            
            # Only Yellow line visible
        elif yM != None and bM == None:
            self.vpY = -10000 - self.centreY
            self.vpX = yM * self.vpY + yEdgeCrossing
            self.dataAvailable = 1
                      
         # No lines visible   
        else:
            self.dataAvailable = 0
  # Temporary only, use potential field system          
    def robotHeading(self,yM,yMeanPoint,bM,bMeanPoint):
        if self.dataAvailable:     
            # Using Homography to compute heading angle
            realCoords = robotFrame([self.vpX,self.vpY],H)
            Heading = math.atan2(-realCoords[1], -realCoords[0])
            # logging.debug("Heading: %.2f", heading_deg)
            self.histVPHeading.append(Heading)
            
            if bM != None:
                leftOffset = findTrackOffset(bMeanPoint, realCoords)
                self.histLeftOffset.append(leftOffset)

            if yM != None:
                rightOffset = findTrackOffset(yMeanPoint, realCoords)
                self.histRightOffset.append(rightOffset)
           
        else:
            self.failedFrame += 1
            # If lines are not detected for N frames, remove history
        if self.failedFrame >= 3:
            self.histVPHeading = deque([0],3)
            self.histLeftOffset = deque([0],3)
            self.histRightOffset = deque([0],3)
            # logging.debug('10 failed frames')
        
        if self.obstacle:
            self.obMissing = 0
            self.histObDist.append(obDistance)
            
        else:
            self.obMissing +=1
            
        if self.obMissing >= 3:
            self.histObDist = deque([0],3)
            # logging.debug('Lost the obstacle')
                

    
    def detectObjects(self,purple):    
            # process purple objects
            purple = cv2.morphologyEx(purple, cv2.MORPH_OPEN, self.kernel)
            __, contours, __ = cv2.findContours(purple,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
    
                # find centroid of largest blob
                blob = max(contours, key=lambda el: cv2.contourArea(el))
                M = cv2.moments(blob)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                # Find edges of obstacle
#                leftEdge = tuple(blob[blob[:,:,0].argmin()][0])
#                rightEdge = tuple(blob[blob[:,:,0].argmax()][0])
#                topEdge = tuple(blob[blob[:,:,1].argmin()][0])
                bottomEdge = tuple(blob[blob[:,:,1].argmax()][0])
                # Calculate distance to object
                                
                obDistance = objectDistance(DEFAULT_CAM_H, DEFAULT_CAM_TILT, DEFAULT_CAM_HEIGHT, bottomEdge)
               
                self.obstacle = True
            else:
                self.obstacle = False
                

            
def rejectOutliers(data, m = 10.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/(mdev if mdev else 1.)
    return data[s<m]

# Remove this function
def objectDistance(VertPix, tiltAngle, Height, bottomEdge):
    
    Y = VertPix/2.0 - bottomEdge[1]

    fieldAngle = 0.8517
    pxAngle = ((Y * fieldAngle)/(VertPix))
    obDist = Height * np.tan(tiltAngle + pxAngle - (3/2*np.pi))
    print('Object!:',obDist)
    return obDist
# Remove this function
def findTrackOffset(Point, realCoords):
    
    # Point on line
    x1 = realCoords[0]
    y1 = realCoords[1]
    
    # Vanishing point
    x2 = Point[0]
    y2 = Point[1]
    
    # Centre point of droid
    x0 = 0
    y0 = 0
      
    dx = x2 - x1
    dy = y2 - y1
    
    mag = np.sqrt(dx*dx + dy*dy)
    dx = dx/mag
    dy = dy/mag
    
    # translate the point and get the dot product
    Lambda = (dx * (x0 - x1)) + (dy * (y0 - y1))
    
    
    x4 = (dx * Lambda) + x1
    y4 = (dy * Lambda) + y1
    
    offset = np.sqrt((x4-x0)**2 + (y4-y0)**2)
    
    return offset

def robotFrame(cPoint,H):
    rPoint = np.dot(H, [cPoint[0],cPoint[1],1])
    rPoint = rPoint/rPoint[2]
    
    return rPoint



'''
Use this to put a test video through the vision 
processing algorithm to see how it performs.
Doesn't require the droid!
'''
if __name__=='__main__':

    def nothing(*arg):
        pass
    
    cv2.namedWindow("Processed")
    cv2.moveWindow('Processed',0,0)
    cv2.namedWindow("Canny")
    cv2.moveWindow('Canny',900,0)
    cv2.namedWindow("Yellow")
    cv2.moveWindow("Yellow",450,400)
    cv2.namedWindow("Blue")
    cv2.moveWindow("Blue",0,400)
    cv2.namedWindow("Purple")
    cv2.moveWindow("Purple",450,0)
    
    cv2.createTrackbar("Minimum Line Length", "Processed", 1,100, nothing)
    cv2.createTrackbar("Theta", "Processed", 1,100, nothing)
    cv2.createTrackbar("Rho", "Processed", 1,100, nothing)

    cv2.createTrackbar("CannyLow", "Canny", 1,300, nothing)
    cv2.createTrackbar("CannyHigh", "Canny", 1,300, nothing)
    
    cv2.createTrackbar("Threshold value", "Yellow", 0, 255, nothing)
    cv2.createTrackbar("Threshold value", "Blue", 0, 255, nothing)
    cv2.createTrackbar("Threshold value", "Purple", 0, 255, nothing)

    cv2.setTrackbarPos("Minimum Line Length", "Processed", 30)
    cv2.setTrackbarPos("Theta", "Processed", 15)
    cv2.setTrackbarPos("Rho", "Processed", 80)
    
    cv2.setTrackbarPos("CannyLow", "Canny", 30)
    cv2.setTrackbarPos("CannyHigh", "Canny", 80)
    
    cv2.setTrackbarPos("Threshold value", "Yellow", 160)
    cv2.setTrackbarPos("Threshold value", "Blue", 110)
    cv2.setTrackbarPos("Threshold value", "Purple", 160) 
    idx = 0
    
    cap = cv2.VideoCapture('test_videos/output2.avi')
    vis = droidVision()
    while(cap.isOpened()):
        t0 = time.time()
        ret, frame = cap.read()
        if frame is not None:
            # Rescale image to 416, 304
            frame = cv2.resize(frame,(416,304))
            
            idx = idx + 1
#            print('fps, ',1.0/(time.time() - t0))
            print('Frame',idx)
            vis.processFrame(frame)
            cv2.imshow("Processed", vis.frame_edited)
            cv2.imshow("Canny", vis.canny)
            cv2.imshow("Blue",vis.blue)
            cv2.imshow("Yellow",vis.yellow)
            cv2.imshow("Purple",vis.purple)
            cv2.waitKey(1)
            # Move forward 10 frames and pause
            if idx == 30:
                play = True
                idx = 0 
                k = cv2.waitKey(100)
                print("Press ESC to stop, 's' key to move frame")

                while play is True:
                    vis.thetaThresh = np.pi/18 * cv2.getTrackbarPos("Theta", "Processed")
                    vis.rhoThresh = cv2.getTrackbarPos("Rho", "Processed")    
                    vis.minLineLength = cv2.getTrackbarPos("Minimum Line Length", "Processed")
                    
                    vis.cannyLow = cv2.getTrackbarPos("CannyLow", "Canny")  
                    vis.cannyHigh = cv2.getTrackbarPos("CannyHigh", "Canny")  
                    
                    vis.threshYellow = cv2.getTrackbarPos("Threshold value", "Yellow")
                    vis.threshBlue = cv2.getTrackbarPos("Threshold value", "Blue")
                    vis.threshPurple = cv2.getTrackbarPos("Threshold value", "Purple")
                  
                    vis.processFrame(frame)
                    
                    print("Yellow", vis.threshYellow)
                    print("Blue", vis.threshBlue)
                    print("Purple", vis.threshPurple)
                    print("minimum seg length", vis.minLineLength)
                    print("Theta", vis.thetaThresh)
                    print("Rho", vis.rhoThresh)
                    print("cannyLow", vis.cannyLow)
                    print("cannyHigh", vis.cannyHigh)

                    cv2.imshow("Processed", vis.frame_edited)
                    cv2.imshow("Canny", vis.canny)

                    cv2.imshow("Blue",vis.blue)
                    cv2.imshow("Yellow",vis.yellow)
                    cv2.imshow("Purple",vis.purple)
                    
                    k = cv2.waitKey(100)
                    if k == 27:         # wait for ESC key to exit
                        cap.release()
                        cv2.destroyAllWindows()
                        break
                    elif k == ord('s'):         # wait for ESC key to exit
                        play = False
            
        else:        
            print ('releasing resources') 
            cap.release()
            cv2.destroyAllWindows()
            break