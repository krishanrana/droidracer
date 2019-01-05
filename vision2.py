import cv2
import numpy as np
import logging
import math
import time
from collections import deque

from constants import *

FRAME_SCALE = 3
BLUE_THRESH = 110
YELLOW_THRESH = 160

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
        
        
    def processFrame(self, frame):
        
        width = frame.shape[1]
        bM = None
        yM = None
        centreX = np.round(frame.shape[1]/2)
        clA,clB = self.raw2Frame(frame)
        
        
        # process yellow line
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, self.kernel)
        yline = np.squeeze(cv2.HoughLinesP(yellow,1,self.thetaThresh,self.rhoThresh,self.minLineLength,self.maxLineGap))#detect lines

        if yline.ndim >= 2:

            ygrad = (yline[:,0]-yline[:,2])/(yline[:,1]-yline[:,3]+0.001)# find gradient of lines
            yfilt = rejectOutliers(ygrad, m=5)
            yM = np.median(yfilt)
            #find intersection point with baseline centreY, using gradient and mean point
            ypointX,ypointY = np.sum((yline[:,0] + yline[:,2])/(2*yline.shape[0])),np.sum((yline[:,1] + yline[:,3])/(2*yline.shape[0]))
            yEdgeCrossing = ypointX + yM*(self.centreY-ypointY)
            for x1,y1,x2,y2 in yline: 
                cv2.line(self.frame,(x1,y1),(x2,y2),(0,0,255),1)
            

        # process blue line
        blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, self.kernel)
        bline = np.squeeze(cv2.HoughLinesP(blue,1,self.thetaThresh,self.rhoThresh,self.minLineLength,self.maxLineGap))#detect lines

        if bline.ndim >= 2:
            bgrad = (bline[:,0]-bline[:,2])/(bline[:,1]-bline[:,3]+0.0001)# find gradient of lines
            bfilt = rejectOutliers(bgrad, m=5)
            bM = np.median(bfilt)
            #find intersection point with baseline centreY, using gradient and mean point
            bpointX,bpointY = np.sum((bline[:,0] + bline[:,2])/(2*bline.shape[0])),np.sum((bline[:,1] + bline[:,3])/(2*bline.shape[0]))
            bEdgeCrossing = bpointX + bM * (self.centreY - bpointY)
            
            for x1,y1,x2,y2 in bline:
                cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),1)
        

        # Conditional to create vP or virtual vP
        # Both lines visible
        if bM != None and yM != None:
            
            self.vpY = (yEdgeCrossing - bEdgeCrossing)/(bM - yM) + self.centreY
            self.vpX = bM * (self.vpY - self.centreY) + bEdgeCrossing
            self.dataAvailable = 1
            # Blue line visible
        elif bM != None:
            self.vpY = -10000 - self.centreY
            self.vpX = bM * self.vpY + bEdgeCrossing
            
            # Yellow line visible
        elif yM != None:
            self.vpY = -10000 - self.centreY
            self.vpX = yM * self.vpY + yEdgeCrossing
            

            
         # No lines visible   
        else:
            self.dataAvailable = 0
            
        
        if self.dataAvailable:     
            # Using Homography to compute heading angle
            realCoords = robotFrame([self.vpX,self.vpY],H)
            Heading = math.atan2(-realCoords[1], -realCoords[0])
            heading_deg = Heading * 180/np.pi
            # logging.debug("Heading: %.2f", heading_deg)
            self.histVPHeading.append(Heading)
            
            if bM != None:
                leftOffset = findTrackOffset([bpointX,bpointY], realCoords)
                self.histLeftOffset.append(leftOffset)

            if yM != None:
                rightOffset = findTrackOffset([ypointX,ypointY], realCoords)
                self.histRightOffset.append(rightOffset)

            
            
        else:
            self.failedFrame += 1
            # If lines are not detected for 10 frames, remove history
        if self.failedFrame >= 3:
            self.histVPHeading = deque([0],3)
            self.histLeftOffset = deque([0],3)
            self.histRightOffset = deque([0],3)
            # logging.debug('10 failed frames')
        
        if obstacle:
            self.obMissing = 0
            self.histObDist.append(obDistance)
            
        else:
            self.obMissing +=1
            
        if self.obMissing >= 3:
            self.histObDist = deque([0],3)
            # logging.debug('Lost the obstacle')
                
            
        avHeading = np.nanmedian(self.histVPHeading)
        avLeftOffset = np.nanmedian(self.histLeftOffset)
        avRightOffset = np.nanmedian(self.histRightOffset)
        avObDist = np.nanmedian(self.histObDist)

        try:
            cv2.line(self.frame_edited,(int(bEdgeCrossing),int(self.centreY)),(int(self.vpX),int(self.vpY)),(0,255,0),2)
            cv2.line(self.frame_edited,(int(yEdgeCrossing),int(self.centreY)),(int(self.vpX),int(self.vpY)),(0,255,0),2)
            # Show direction to vanishing point
            cv2.line(self.frame_edited,(int(centreX),int(self.centreY)),(int(self.vpX),int(self.vpY)),(0,0,255),2)
        except:
            print('cant show lines')  
   
        return avHeading, avLeftOffset, avRightOffset, obstacle, avObDist

    def detectLine(self,channel):
            channel = cv2.morphologyEx(channel, cv2.MORPH_OPEN, self.kernel)
            lines = np.squeeze(cv2.HoughLinesP(channel,1,self.thetaThresh,self.rhoThresh,self.minLineLength,self.maxLineGap))#detect lines
    
            if lines.ndim >= 2:
                grad = (lines[:,0]-lines[:,2])/(lines[:,1]-lines[:,3]+0.0001)# find gradient of lines
                filt = rejectOutliers(grad, m=5)
                M = np.median(filt)
                #find intersection point with baseline centreY, using gradient and mean point
                pointX,pointY = np.sum((lines[:,0] + lines[:,2])/(2*lines.shape[0])),np.sum((lines[:,1] + lines[:,3])/(2*lines.shape[0]))
                EdgeCrossing = pointX + M * (self.centreY - pointY)
                
                for x1,y1,x2,y2 in lines:
                    cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),1)
                    
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
                leftEdge = tuple(blob[blob[:,:,0].argmin()][0])
                rightEdge = tuple(blob[blob[:,:,0].argmax()][0])
                topEdge = tuple(blob[blob[:,:,1].argmin()][0])
                bottomEdge = tuple(blob[blob[:,:,1].argmax()][0])
                # Calculate distance to object
                obDistance = objectDistance(DEFAULT_CAM_H, DEFAULT_CAM_TILT, DEFAULT_CAM_HEIGHT, bottomEdge)
    
                # Draw outputs
                try:
                    cv2.line(frame,(0,bottomEdge[1]),(width,bottomEdge[1]),(0,255,0),2)
                    cv2.circle(frame, center, 5, (0,0,255), -1)
                except:
                    logging.debug('no object rect')
                
                obstacle = True
            else:
                obstacle = False
                

            
def rejectOutliers(data, m = 10.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/(mdev if mdev else 1.)
    return data[s<m]


def objectDistance(VertPix, tiltAngle, Height, bottomEdge):
    
    Y = VertPix/2.0 - bottomEdge[1]

    fieldAngle = 0.8517
    pxAngle = ((Y * fieldAngle)/(VertPix))
    obDist = Height * np.tan(tiltAngle + pxAngle - (3/2*np.pi))
    print('Object!:',obDist)
    return obDist

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