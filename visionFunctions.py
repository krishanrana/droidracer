import cv2
import numpy as np

from collections import deque

class droidVision():

    def __init__(self):

        self.dataAvailable = 0
        self.failedFrame = 0
        self.histVPHeading = deque([0],10)
        self.histLeftOffset = deque([0],10)
        self.histRightOffset = deque([0],10)
        self.histObDist = deque([0],10)
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
        self.minLineLength = 100
        self.maxLineGap = 30
        self.centreY = 400 # Result of calibration


    def processFrame(self, frame):
        
        width = frame.shape[1]

        
        centreX = np.round(frame.shape[1]/2)
        processed = cv2.medianBlur(frame,5)
        processed = cv2.cvtColor(processed,cv2.COLOR_BGR2LAB) 
        l,a,b = cv2.split(processed)
        clA = self.clahe.apply(a) # histogram adjustment
        # Split into yellow and blue lines
        thresh,purple = cv2.threshold(clA,160,1,cv2.THRESH_BINARY)
        
        clB = self.clahe.apply(b) # histogram adjustment
        # Split into yellow and blue lines
        retY,yellow = cv2.threshold(clB,170,1,cv2.THRESH_BINARY)
        retB,blue = cv2.threshold(clB,100,1,cv2.THRESH_BINARY)
        blue = cv2.bitwise_not(blue)-254 # invert blue line
        try:
            # process yellow line
            yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, self.kernel)
            #yellow = cv2.Canny(yellow,0,1,apertureSize = 5)
            yline = np.squeeze(cv2.HoughLinesP(yellow,1,self.thetaThresh,self.rhoThresh,self.minLineLength,self.maxLineGap))#detect lines
            ygrad = (yline[:,0]-yline[:,2])/(yline[:,1]-yline[:,3]+0.001)# find gradient of lines
            yfilt = rejectOutliers(ygrad, m=10)
            ymag = np.sqrt((yline[:,1]-yline[:,3])**2+(yline[:,0]-yline[:,2])**2) # find magnitude of lines
            yM = np.sum((yfilt*ymag),axis=0)/np.sum(ymag,axis=0) # find weighted average gradient
            #find intersection point with baseline centreY, using gradient and mean point
            ypointX,ypointY = np.sum((yline[:,0] + yline[:,2])/(2*yline.shape[0])),np.sum((yline[:,1] + yline[:,3])/(2*yline.shape[0]))
            yZeroCrossing = ypointX + yM*(self.centreY-ypointY)
            #for x1,y1,x2,y2 in yline: 
            #cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),1)
        except:
            print('No yellow') 
            
        try:   
            # process blue line
            blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, self.kernel)
    #        blue = cv2.Canny(blue,0,1,apertureSize = 5)
            bline = np.squeeze(cv2.HoughLinesP(blue,1,self.thetaThresh,self.rhoThresh,self.minLineLength,self.maxLineGap))#detect lines
            bgrad = (bline[:,0]-bline[:,2])/(bline[:,1]-bline[:,3])# find gradient of lines
            bfilt = rejectOutliers(bgrad, m=10)

            bmag = np.sqrt((bline[:,1]-bline[:,3])**2+(bline[:,0]-bline[:,2])**2) # find magnitude of lines
            bM = np.sum((bfilt*bmag),axis=0)/np.sum(bmag,axis=0) # find weighted average gradient
            #find intersection point with baseline centreY, using gradient and mean point
            bpointX,bpointY = np.sum((bline[:,0] + bline[:,2])/(2*bline.shape[0])),np.sum((bline[:,1] + bline[:,3])/(2*bline.shape[0]))
            bZeroCrossing = bpointX + bM*(self.centreY-bpointY)
        
    #        for x1,y1,x2,y2 in bline:
    #            cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),1)
        except:
            print('No blue')
            
        try:
            # process purple objects
            purple = cv2.morphologyEx(purple, cv2.MORPH_OPEN, self.kernel)
            # blob detect,get centroids
            __, contours, __ = cv2.findContours(purple,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            # find centroid of largest blob
        
            blob = max(contours, key=lambda el: cv2.contourArea(el))
            M = cv2.moments(blob)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            # Find edges of obstacle
            cnt = blob

            leftEdge = tuple(cnt[cnt[:,:,0].argmin()][0])
            rightEdge = tuple(cnt[cnt[:,:,0].argmax()][0])
            topEdge = tuple(cnt[cnt[:,:,1].argmin()][0])
            bottomEdge = tuple(cnt[cnt[:,:,1].argmax()][0])
            # Calculate distance to object
            obDistance = (self.centreY - bottomEdge[1])*0.04

            # Draw outputs
            cv2.rectangle(frame,(0,bottomEdge[1]),(width,bottomEdge[1]),(0,0,255),2)
            cv2.circle(frame, center, 5, (0,0,255), -1)
            obstacle = 1
            
        except:
            print('No objects, drive fast!') 
            obstacle = 0
            

        
        try:
            leftOffset = (centreX-bZeroCrossing)
            rightOffset = (yZeroCrossing - centreX)
            centreOffset = rightOffset-leftOffset
            self.vpY = (yZeroCrossing - bZeroCrossing)/(bM - yM) + self.centreY
            self.vpX = bM * (self.vpY - self.centreY) + bZeroCrossing
            Heading = np.arctan((self.vpX - centreX)/(self.centreY - self.vpY))
            self.dataAvailable = 1

    
        except:
            print('all fucked right now')
            self.dataAvailable = 0
            
        if self.dataAvailable:   
            self.histVPHeading.append(Heading)
            self.histLeftOffset.append(leftOffset)
            self.histRightOffset.append(rightOffset)
        else:
            self.failedFrame += 1
            # If lines are not detected for 10 frames, remove history
        if self.failedFrame >= 10:
            self.histVPHeading = deque([0],10)
            self.histLeftOffset = deque([0],10)
            self.histRightOffset = deque([0],10)
            print('10 failed frames')
        
        if obstacle:
            self.obMissing = 0
            self.histObDist.append(obDistance)
            
        else:
            self.obMissing +=1
            
        if self.obMissing >= 10:
            self.histObDist = deque([0],10)
            print('Lost the obstacle')
                
            
        avHeading = np.nanmedian(self.histVPHeading)
        avLeftOffset = np.nanmedian(self.histLeftOffset)
        avRightOffset = np.nanmedian(self.histRightOffset)
        avObDist = np.nanmedian(self.histObDist)
            
        return avHeading, avLeftOffset, avRightOffset, obstacle, avObDist


            
def rejectOutliers(data, m = 10.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/(mdev if mdev else 1.)
    return data[s<m]




class goal():
    def __init__(self):
        self.vP = [0,0]
        self.offset = [0,0]
        # possible states ['NoLines' 0, 'OneLine' 1, 'TwoLines'2]
        self.linestate = 0
        
        pass
    
        
        
        
        
class obstacle():
    def __init__(self):

        self.number = 0
        self.centre = [0,0]
        self.distance = 0
        self.edges = [0,0]
        
        pass
        
        
        

class droidState():
    def __init__(self):

        # possible states ['start', 'ready', 'run', 'lost','finish']
        self.droidstate = 0
        # possible states = ['Correct', 'Incorrect', 'Unknown']
        #self.trackstate = 'Unknown'
        
        pass