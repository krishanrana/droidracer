from __future__ import division
import cv2
import numpy as np
import logging
import math
import time
from collections import deque

from constants import *

FRAME_SCALE = 1
BLUE_THRESH = 110
YELLOW_THRESH = 160

H = np.array([[ -1.07693537e+01,  -1.03211106e+00,   4.41554723e+03],
              [ -5.61158586e-02,   7.60254152e-01,  -7.36972948e+03],
              [ -3.59519827e-04,  -5.10817084e-02,   1.00000000e+00]])
            


class droidVision():

    def __init__(self):
        
        self.dataAvailable = 0
        self.failedFrame = 0
        self.DEQUELENGTH = 3
        self.histVPHeading = deque([np.nan],self.DEQUELENGTH)
        self.histLeftOffset = deque([np.nan],self.DEQUELENGTH)
        self.histRightOffset = deque([np.nan],self.DEQUELENGTH)
        self.histObDist = deque([np.nan],self.DEQUELENGTH)
        self.obMissing = 0
        self.obstacle = False

        self.frameNo = 0
        self.kernel = np.ones((5,5),np.uint8)

        self.vpX = 0
        self.vpY = 0
        self.leftOffset = 0
        self.rightOffset = 0

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        self.thetaThresh = np.pi/180 *1.5
        self.rhoThresh = 80
        self.minLineLength = 100 * FRAME_SCALE
        self.maxLineGap = 30 * FRAME_SCALE
        self.centreY = DEFAULT_CAM_H # Result of calibration

        self.frame_edited = np.empty((DEFAULT_CAM_H,DEFAULT_CAM_W,3))

    # Main method
    def processFrame(self, frame):
        
        centreX = np.round(frame.shape[1]/2)
        clA,clB = self.raw2Frame(frame)
        # Threshold image: blue, purple, yellow    
        purple, yellow, blue = self.thresholdFrame(clA, clB)
        # Find yellow and blue lines; gradient, baseline crossing point, middle
        yM,yEdgeCrossing,yMeanPoint = self.detectLine(yellow)
        bM,bEdgeCrossing,bMeanPoint = self.detectLine(blue)
        obCentre, obBottom = self.detectObjects(purple)
        # Find vanishing point (goal)
        self.vanishingPoint(yM,yEdgeCrossing,bM,bEdgeCrossing)
        
        if self.failedFrame >= self.DEQUELENGTH:
            self.histVPHeading = deque([np.nan],self.DEQUELENGTH)
            self.histLeftOffset = deque([np.nan],self.DEQUELENGTH)
            self.histRightOffset = deque([np.nan],self.DEQUELENGTH)
            self.failedFrame = 0
        if self.obMissing >= self.DEQUELENGTH:
            self.histObDist = deque([np.nan],self.DEQUELENGTH)
            self.obMissing = 0
            # logging.debug('10 failed frames')
        self.goalHeading(yM,bM,bMeanPoint,yMeanPoint)
        self.obstacleHeading(obCentre, obBottom)
        
        goalHeading = np.nanmedian(self.histVPHeading)
        trackLeftOffset = np.nanmedian(self.histLeftOffset)
        trackRightOffset = np.nanmedian(self.histRightOffset)
        print('histOBDist',self.histObDist)
        obstacleDist = np.nanmedian(self.histObDist)
        
        
        if bM is not None:
            cv2.line(self.frame_edited,(int(bEdgeCrossing),int(self.centreY)),(int(self.vpX),int(self.vpY)),(0,255,0),2)
        if yM is not None:
            cv2.line(self.frame_edited,(int(yEdgeCrossing),int(self.centreY)),(int(self.vpX),int(self.vpY)),(0,255,0),2)
        if self.vpX is not None:# Show direction to vanishing point
            cv2.line(self.frame_edited,(int(centreX),int(self.centreY)),(int(self.vpX),int(self.vpY)),(0,0,255),2)
        else:
            print('cant show lines')  
        # Draw outputs
        try:
            cv2.line(self.frame_edited,(0,obBottom[1]),(DEFAULT_CAM_W,obBottom[1]),(0,255,0),2)
            cv2.circle(self.frame_edited, obCentre, 5, (0,0,255), -1)
        except:
            logging.debug('no object rect')
            
        
        return goalHeading, trackLeftOffset, trackRightOffset, self.obstacle, obstacleDist
           
            
    def raw2Frame(self,frame):
    
        if frame is not None:
            # Create frames for processing
            frameLAB = cv2.cvtColor(frame,cv2.COLOR_BGR2LAB) 
            self.frame_edited = np.copy(frame)
            l,a,b = cv2.split(frameLAB)
            clA = self.clahe.apply(a) # histogram adjustment
            clB = self.clahe.apply(b) # histogram adjustment
            return clA,clB
        else:
            self.failedFrame +=1
                

    def thresholdFrame(self, clA, clB):
        thresh,purple = cv2.threshold(clA,160,1,cv2.THRESH_BINARY)
        retY,yellow = cv2.threshold(clB,YELLOW_THRESH,1,cv2.THRESH_BINARY)
        retB,blue = cv2.threshold(clB,BLUE_THRESH,1,cv2.THRESH_BINARY)
        blue = cv2.bitwise_not(blue)-254 # invert blue line
        
        return purple, yellow, blue
    
    def detectLine(self,channel):
            channel = cv2.morphologyEx(channel, cv2.MORPH_OPEN, self.kernel)
            lines = np.squeeze(cv2.HoughLinesP(channel,1,self.thetaThresh,self.rhoThresh,self.minLineLength,self.maxLineGap))#detect lines
    
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
                print('VS130: HoughLines not found')
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
    def goalHeading(self,yM,bM,bMeanPoint,yMeanPoint):
        if self.dataAvailable:     
            # Using Homography to compute heading angle
            Vanishing_R = robotFrame([self.vpX,self.vpY],H)          
            Heading = math.atan2(-Vanishing_R[1], -Vanishing_R[0])
            # logging.debug("Heading: %.2f", heading_deg)
            self.histVPHeading.append(Heading)
            # THIS CODE NEEDS WORK
            if bM != None:
                BlueCentre_R = robotFrame([bMeanPoint[0],bMeanPoint[1]],H)
                leftOffset = findTrackOffset(BlueCentre_R, Vanishing_R)
                self.histLeftOffset.append(leftOffset)
            else: self.histLeftOffset.append(np.nan)

            if yM != None:
                YellowCentre_R = robotFrame([yMeanPoint[0],yMeanPoint[1]],H)
                rightOffset = findTrackOffset(YellowCentre_R, Vanishing_R)
                self.histRightOffset.append(rightOffset)
            else: self.histRightOffset.append(np.nan)
           
        else:
            self.failedFrame += 1
            # If lines are not detected for N frames, remove history
        
            
    def obstacleHeading(self,obCentre, obBottom):          
        if self.obstacle:
            self.obMissing = 0
            print('obBottom',obBottom)
            self.histObDist.append(obBottom[1])
            
        else:
            self.obMissing +=1
            self.histObDist.append(np.nan)
                

    
    def detectObjects(self,purple):    
            # process purple objects
            purple = cv2.morphologyEx(purple, cv2.MORPH_OPEN, self.kernel)
            __, contours, __ = cv2.findContours(purple,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
    
                # find centroid of largest blob
                blob = max(contours, key=lambda el: cv2.contourArea(el))
                M = cv2.moments(blob)
                centre = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                # Find edges of obstacle
#                leftEdge = tuple(blob[blob[:,:,0].argmin()][0])
#                rightEdge = tuple(blob[blob[:,:,0].argmax()][0])
#                topEdge = tuple(blob[blob[:,:,1].argmin()][0])
                bottomEdge = tuple(blob[blob[:,:,1].argmax()][0])
                
                # Calculate distance to object
                obCentre = robotFrame(centre,H)
                obBottom = robotFrame([centre[0],bottomEdge[1]],H)
                               
                self.obstacle = True
            else:
                self.obstacle = False
                obCentre = None
                obBottom = None
                
            return obCentre, obBottom
                

            
def rejectOutliers(data, m = 10.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/(mdev if mdev else 1.)
    return data[s<m]


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

    cap = cv2.VideoCapture('test_videos/output3.avi')
    vis = droidVision()


    while(cap.isOpened()):
        t0 = time.time()
        ret, frame = cap.read()
#        frame = cv2.resize(frame, (DEFAULT_CAM_W,DEFAULT_CAM_H))
        if frame is not None:
            vis.processFrame(frame)
            print('fps, ',1.0/(time.time() - t0))

            cv2.imshow("Vision Testing", vis.frame_edited)

            cv2.waitKey(5)
        else:        
            print ('releasing resources') 
            cap.release()
            cv2.destroyAllWindows()
            break