# Set up script
import cv2
import numpy as np
import time

frameNo = 0
estimateIDX = 0
kernel = np.ones((5,5),np.uint8)
oldDistance = np.zeros((1,10))
oldHeading = np.zeros((1,10))
oldCentreOffset = np.zeros((1,10))



cap = cv2.VideoCapture('/Volumes/dougBrain1/Droidracer/DroidVision/DRC2017Short.mp4')
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
thetaThresh = np.pi/180 *1.5
rhoThresh = 80
minLineLength = 100
maxLineGap = 30
centreY = 400 # Result of calibration

font = cv2.FONT_HERSHEY_SIMPLEX
e1 = cv2.getTickCount()
# Main loop for frame processing
while(cap.isOpened()):
    #Process entire image
    ret, frame = cap.read()
    try:
        width = frame.shape[1]
    except:
        break
    frameNo += 1
    estimateIDX += 1
    if estimateIDX > 9:
        estimateIDX = 9
        oldDistance[0,0:8] = oldDistance[0,1:9]

            
    frame = frame[450:frame.shape[0]-1,:,:]
#    frame = cv2.resize(frame,(0,0), fx=0.5, fy=0.5)
    centreX = np.round(frame.shape[1]/2)
    processed = cv2.GaussianBlur(frame,(15,15),0)
    processed = cv2.cvtColor(processed,cv2.COLOR_BGR2LAB) 
    l,a,b = cv2.split(processed)
    clA = clahe.apply(a) # histogram adjustment
    # Split into yellow and blue lines
    thresh,purple = cv2.threshold(clA,160,1,cv2.THRESH_BINARY)
    
    clB = clahe.apply(b) # histogram adjustment
    # Split into yellow and blue lines
    retY,yellow = cv2.threshold(clB,170,1,cv2.THRESH_BINARY)
    retB,blue = cv2.threshold(clB,100,1,cv2.THRESH_BINARY)
    blue = cv2.bitwise_not(blue)-254 # invert blue line
    try:
        # process yellow line
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
#        yellow = cv2.Canny(yellow,0,1,apertureSize = 5)
        yline = np.squeeze(cv2.HoughLinesP(yellow,1,thetaThresh,rhoThresh,minLineLength,maxLineGap))#detect lines
        ygrad = (yline[:,0]-yline[:,2])/(yline[:,1]-yline[:,3])# find gradient of lines
        ymag = np.sqrt((yline[:,1]-yline[:,3])**2+(yline[:,0]-yline[:,2])**2) # find magnitude of lines
        yM = np.sum((ygrad*ymag),axis=0)/np.sum(ymag,axis=0) # find weighted average gradient
        #find intersection point with baseline centreY, using gradient and mean point
        ypointX,ypointY = np.sum((yline[:,0] + yline[:,2])/(2*yline.shape[0])),np.sum((yline[:,1] + yline[:,3])/(2*yline.shape[0]))
        yZeroCrossing = ypointX + yM*(centreY-ypointY)
        for x1,y1,x2,y2 in yline: 
            cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),1)
    except:
        print('No yellow') 
        
    try:   
        # process blue line
        blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, kernel)
#        blue = cv2.Canny(blue,0,1,apertureSize = 5)
        bline = np.squeeze(cv2.HoughLinesP(blue,1,thetaThresh,rhoThresh,minLineLength,maxLineGap))#detect lines
        bgrad = (bline[:,0]-bline[:,2])/(bline[:,1]-bline[:,3])# find gradient of lines
        bmag = np.sqrt((bline[:,1]-bline[:,3])**2+(bline[:,0]-bline[:,2])**2) # find magnitude of lines
        bM = np.sum((bgrad*bmag),axis=0)/np.sum(bmag,axis=0) # find weighted average gradient
        #find intersection point with baseline centreY, using gradient and mean point
        bpointX,bpointY = np.sum((bline[:,0] + bline[:,2])/(2*bline.shape[0])),np.sum((bline[:,1] + bline[:,3])/(2*bline.shape[0]))
        bZeroCrossing = bpointX + bM*(centreY-bpointY)
    
        for x1,y1,x2,y2 in bline:
            cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),1)
    except:
        print('No blue')
        
    try:
        # process purple objects
        purple = cv2.morphologyEx(purple, cv2.MORPH_OPEN, kernel)
        # blob detect,get centroids
        im2, contours, hierarchy = cv2.findContours(purple,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        # find centroid of largest blob
    
        blob = max(contours, key=lambda el: cv2.contourArea(el))
        M = cv2.moments(blob)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
        # Find edges of obstacle
        cnt = contours[0]

        leftEdge = tuple(cnt[cnt[:,:,0].argmin()][0])
        rightEdge = tuple(cnt[cnt[:,:,0].argmax()][0])
        topEdge = tuple(cnt[cnt[:,:,1].argmin()][0])
        bottomEdge = tuple(cnt[cnt[:,:,1].argmax()][0])
        # Calculate distance to object
        obDistance = (centreY - bottomEdge[1])*0.04

        # Draw outputs
        cv2.rectangle(frame,(0,bottomEdge[1]),(width,bottomEdge[1]),(0,0,255),2)
        cv2.circle(frame, center, 5, (0,0,255), -1)
    except:
        print('No objects, drive fast!') 
        obDistance = oldDistance[0,estimateIDX-1]

    
    try:
        leftOffset = (centreX-bZeroCrossing)
        rightOffset = (yZeroCrossing - centreX)
        centreOffset = rightOffset-leftOffset
        vpY = (yZeroCrossing - bZeroCrossing)/(bM - yM) + centreY
        vpX = bM * (vpY - centreY) + bZeroCrossing
        Heading = np.arctan((vpX - centreX)/(centreY - vpY))
        

      
    except:
        print('all fucked right now')
        Heading = oldHeading[0,estimateIDX-1]
        centreOffset = oldCentreOffset[0,estimateIDX-1]

        
        
    # moving average filter states
    
    oldDistance[0,estimateIDX] = obDistance
    oldHeading[0,estimateIDX] = Heading
    oldCentreOffset[0,estimateIDX] = centreOffset
    
    
    
    try:
        cv2.putText(frame, "{:10.2f}".format(oldHeading), (20, 50), font, 0.8, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, "{:10.2f}".format(oldCentreOffset), (20, 100), font, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, "{:10.2f}".format(oldDistance[0,estimateIDX]), (20, 150), font, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.line(frame,(int(centreX),int(centreY)),(int(vpX),int(vpY)),(0,0,255),3)
        cv2.line(frame,(int(bZeroCrossing),int(centreY)),(int(vpX),int(vpY)),(0,255,0),1)
        cv2.line(frame,(int(yZeroCrossing),int(centreY)),(int(vpX),int(vpY)),(0,255,0),1)
        
    except:
        print('print problems')

        
    cv2.imshow('frame',frame)

    #time.sleep(0.1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
e2 = cv2.getTickCount()
t = (e2 - e1)/(cv2.getTickFrequency()*frameNo)
print( t )

cap.release()
cv2.destroyAllWindows()

