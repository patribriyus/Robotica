# -*- coding: utf-8 -*-
"""
Created on Sat Mar 20 20:29:04 2021

@author: patri
"""

import cv2


def camInit():
    # Setup default values for SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
    
    # These are just examples, tune your own if needed
    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200
    
    # Filter by Area
    params.filterByArea = True
    params.minArea = 200
    params.maxArea = 10000
    
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
    
    # Filter by Color
    params.filterByColor = False
    # not directly color, but intensity on the channel input
    #params.blobColor = 0
    
    # Filter by Convexity
    #params.filterByConvexity = False
    params.filterByConvexity = True
    params.minConvexity = 0.87
    
    # Filter by Inertia
    #params.filterByInertia = False
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    
    
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
    	detector = cv2.SimpleBlobDetector(params)
    else :
    	detector = cv2.SimpleBlobDetector_create(params)
      
    return detector

