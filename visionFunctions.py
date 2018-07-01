import cv2
import numpy as np



class droidVision():

    def __init__(self):
        pass


    def processFrame(self, frame):

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        thetaThresh = np.pi/180 *1.5
        rhoThresh = 80
        minLineLength = 100
        maxLineGap = 30
        kernel = np.ones((5,5),np.uint8)
        
        # Calibrate frame centre and add pixel/m map
        centreY = 400 # Result of calibration
        mPerPixel = 0.004
        
        # Initialise variables
        width = frame.shape[1]
        dataAvailable = 1; 
        vpHeading = 0
        droidOffset = 0
        obCentre = [0,0]
        obWidth = 0
        obDistance = 0
        obHeading = 0
        
        #State flags
        blueDetect = 0
        yellowDetect = 0
        obstacleDetect = 0
        
        # Work with frame
        centreX = np.round(width/2)
        processed = cv2.medianBlur(frame,5)
        lab = cv2.cvtColor(processed,cv2.COLOR_BGR2LAB) 
        l,a,b = cv2.split(lab)

        # histogram adjustment
        clA = clahe.apply(a) 
        clB = clahe.apply(b) 
        
        # Split into yellow and blue and green lines
        retY,yellow = cv2.threshold(clB,170,1,cv2.THRESH_BINARY)
        retB,blue = cv2.threshold(clB,100,1,cv2.THRESH_BINARY)
#        retG,green = cv2.threshold(clA,100,1,cv2.THRESH_BINARY)
        retP,purple = cv2.threshold(clA,160,1,cv2.THRESH_BINARY)
        
#        green = cv2.bitwise_not(green) - 254 # invert green line
        blue = cv2.bitwise_not(blue) - 254 # invert blue line
        try:
            # process yellow line
            yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
            yLine = np.squeeze(cv2.HoughLinesP(yellow,1,thetaThresh,rhoThresh,minLineLength,maxLineGap))
            yGrad = (yLine[:,0]-yLine[:,2])/(yLine[:,1]-yLine[:,3] + 0.001)
            yFilt = rejectOutliers(yGrad, m=10)
            
            yMag = np.sqrt((yLine[:,1]-yLine[:,3])**2+(yLine[:,0]-yLine[:,2])**2) 
            yM = np.sum((yFilt*yMag),axis=0)/np.sum(yMag,axis=0) 
            # find intersection point with baseline centreY, using gradient and mean point
            yX,yY = np.sum((yLine[:,0] + yLine[:,2])/(2*yLine.shape[0])),np.sum((yLine[:,1] + yLine[:,3])/(2*yLine.shape[0]))
            yZero = yX + yM*(centreY-yY)
            yellowDetect = 1
        except:
            print('No yellow') 
            yellowDetect = 0
            
            
        # Detect blue line    
        try:   
            # process blue line
            blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, kernel)
            bLine = np.squeeze(cv2.HoughLinesP(blue,1,thetaThresh,rhoThresh,minLineLength,maxLineGap))
            bGrad = (bLine[:,0]-bLine[:,2])/(bLine[:,1]-bLine[:,3] + 0.001)
            bFilt = rejectOutliers(bGrad,m=10)

            bMag = np.sqrt((bLine[:,1]-bLine[:,3])**2+(bLine[:,0]-bLine[:,2])**2) 
            bM = np.sum((bFilt*bMag),axis=0)/np.sum(bMag,axis=0) 
            
            #find intersection point with baseline centreY, using gradient and mean point
            bX,bY = np.sum((bLine[:,0] + bLine[:,2])/(2*bLine.shape[0])),np.sum((bLine[:,1] + bLine[:,3])/(2*bLine.shape[0]))
            bZero = bX + bM * (centreY-bY)
            blueDetect = 1

        except:
            print('No blue')
            blueDetect = 0
        # Detect finish line - Unfinished, need example
#        try:   
#            # process green line
#            green = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernel)
#            gline = np.squeeze(cv2.HoughLinesP(green,1,thetaThresh,rhoThresh,minLineLength,maxLineGap))#detect lines
#            ggrad = (gline[:,0]-gline[:,2])/(gline[:,1]-gline[:,3])# find gradient of lines
#            gmag = np.sqrt((gline[:,1]-gline[:,3])**2+(gline[:,0]-gline[:,2])**2) # find magnitude of lines
#            gM = np.sum((ggrad*gmag),axis=0)/np.sum(gmag,axis=0) # find weighted average gradient
#                      
#
#        except:
#            print('No green') 
            
        # Detect obstacles
        try:
            # process purple objects
            purple = cv2.morphologyEx(purple, cv2.MORPH_OPEN, kernel)
            __, contours, __ = cv2.findContours(purple,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            # find centroid of largest blob
            blob = max(contours, key=lambda el: cv2.contourArea(el))
            M = cv2.moments(blob)
            obCentre = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            # Find edges of obstacle
            blob = blob

            obLeft = tuple(blob[blob[:,:,0].argmin()][0])
            obRight = tuple(blob[blob[:,:,0].argmax()][0])
            #obTop = tuple(blob[blob[:,:,1].argmin()][0])
            obBottom = tuple(blob[blob[:,:,1].argmax()][0])
            
            # Calculate distance to object
            obDistance = (centreY - obBottom[1]) * mPerPixel
            obWidth = obRight - obLeft
            obHeading = np.arctan((obCentre[0] - centreX)/(centreY - obCentre[1]))
            print('object detected')
            obstacleDetect = 1

        except:
            print('No objects, drive fast!') 
            obstacleDetect = 0
            
            
        linestate = yellowDetect + blueDetect
        
        try:   
            if linestate == 0:
                dataAvailable = 0
                
            elif linestate == 1:
                dataAvailable = 1
                
                
            else:
                dataAvailable = 1
                leftOffset = (centreX-bZero)
                rightOffset = (yZero - centreX)
                droidOffset = [leftOffset, rightOffset]
                vpY = (yZero - bZero)/(bM - yM) + centreY
                vpX = bM * (vpY - centreY) + bZero
                vpHeading = np.arctan((vpX - centreX)/(centreY - vpY))
                  
        except:
            print('all fucked right now')
            dataAvailable = 0;
            
        return dataAvailable, vpX, vpY, vpHeading, droidOffset, obCentre, obWidth, obDistance, obHeading


            
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
        self.droidstate = 'start'
        # possible states = ['Correct', 'Incorrect', 'Unknown']
        self.trackstate = 'Unknown'
        
        pass