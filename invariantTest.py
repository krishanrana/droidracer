#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 30 22:43:20 2019

@author: dougpalmer
"""

from __future__ import division
import cv2
import numpy as np
from vision3 import droidVision
import time


def BGR2invariant(frame,alpha):
        #G - B - R BGR
        invariant = 0.5 + np.log(frame[:,:,1]) - alpha * np.log(frame[:,:,2]) - (1-alpha) * np.log(frame[:,:,0]);
        invariant = np.uint8(np.round(invariant * 255))
        return invariant



if __name__=='__main__':
    
    def nothing(*arg):
        pass
    
    
    
    L1 = 460e-9;
    L2 = 520e-9;
    L3 = 600e-9;
    alpha = (L1*(L3-L2))/(L2*(L3-L1));
    
    cap = cv2.VideoCapture('test_videos/output2.avi')
    vis = droidVision()
    
    cv2.namedWindow("BGR")
    cv2.moveWindow('BGR',0,0)
    
    cv2.namedWindow("INV")
    cv2.moveWindow('INV',500,0)
    
    cv2.createTrackbar("Alpha", "INV", 0, 100, nothing)

    cv2.setTrackbarPos("Alpha", "INV", int(alpha*100))
    
    
    
    
    while(cap.isOpened()):
        ret, frame = cap.read()
        if frame is not None:
            alpha = cv2.getTrackbarPos("Alpha", "INV")    
            alphaNorm = alpha/100
            invariant = BGR2invariant(frame,alphaNorm)
            cv2.imshow("BGR",frame)
            cv2.imshow("INV",invariant)
            k = cv2.waitKey(1)
            if k == 27:         # wait for ESC key to exit
                cap.release()
                cv2.destroyAllWindows()
                break
            
            
        else:        
            print ('releasing resources') 
            cap.release()
            cv2.destroyAllWindows()
            break
