#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 19 17:32:10 2019

@author: dougpalmer
"""
from __future__ import division
import cv2
import numpy as np
import time
import vision



# Setup classes
cap = cv2.VideoCapture('test_videos/output3.avi')
vis = vision.droidVision()

class droidThresh():

    def __init__(self):

    
        self.icol = (0, 0, 0, 255, 255, 255)   # New start
 
        cv2.namedWindow('colorTest')
        # Lower range colour sliders.
        cv2.createTrackbar('lowHue', 'colorTest', icol[0], 255, nothing)
        cv2.createTrackbar('lowSat', 'colorTest', icol[1], 255, nothing)
        cv2.createTrackbar('lowVal', 'colorTest', icol[2], 255, nothing)
        # Higher range colour sliders.
        cv2.createTrackbar('highHue', 'colorTest', icol[3], 255, nothing)
        cv2.createTrackbar('highSat', 'colorTest', icol[4], 255, nothing)
        cv2.createTrackbar('highVal', 'colorTest', icol[5], 255, nothing)



    def setGUI(self):
        

    
        # Get HSV values from the GUI sliders.
        lowHue = cv2.getTrackbarPos('lowHue', 'colorTest')
        lowSat = cv2.getTrackbarPos('lowSat', 'colorTest')
        lowVal = cv2.getTrackbarPos('lowVal', 'colorTest')
        highHue = cv2.getTrackbarPos('highHue', 'colorTest')
        highSat = cv2.getTrackbarPos('highSat', 'colorTest')
        highVal = cv2.getTrackbarPos('highVal', 'colorTest')
         
        # HSV values to define a colour range we want to create a mask from.
        colorLow = np.array([lowHue,lowSat,lowVal])
        colorHigh = np.array([highHue,highSat,highVal])
        mask = cv2.inRange(frameHSV, colorLow, colorHigh)
        
        
    def thresholdImage(self,frame):
        
  
    
        # Show the first mask
        cv2.imshow('mask-plain', mask)
     
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     
        contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
        biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
         
        x,y,w,h = cv2.boundingRect(biggest_contour)
        cv2.rectangle(frameHSV,(x,y),(x+w,y+h),(0,255,0),2)
     
        
	
    # k = cv2.waitKey(5) & 0xFF
    # if k == 27:
    #     break
    # print('fps - ', 1/(time.time() - timeCheck))
    
# cv2.destroyAllWindows()
# vidCapture.release()


# Open frame
while(cap.isOpened()):
    t0 = time.time()
    ret, frame = cap.read()
    if frame is not None:
        # Rescale image to 416, 304
        frame = cv2.resize(frame,(416,304))
        vis.processFrame(frame)
        print('fps, ',1.0/(time.time() - t0))

        cv2.imshow("Vision Testing", vis.frame_edited)

        cv2.waitKey(1)
    else:        
        print ('releasing resources') 
        cap.release()
        cv2.destroyAllWindows()
        break