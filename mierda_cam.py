# -*- coding: utf-8 -*-
"""
Created on Sat Mar 20 21:08:58 2021

@author: patri
"""


import cv2
import picamera
from picamera.array import PiRGBArray
import numpy as np
import time

ESC = 27
KNN = 1
MOG2 = 2

cam = picamera.PiCamera()

cam.resolution = (320, 240)
cam.rotation = 180
#cam.resolution = (640, 480)
cam.framerate = 32
rawCapture = PiRGBArray(cam, size=(320, 240))
#rawCapture = PiRGBArray(cam, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)

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
    
# Elegimos el umbral de rojo en HSV
redMin1 = (170,100,20)
redMax1 = (179,255,255)
# Elegimos el segundo umbral de rojo en HSV
redMin2 = (0,100,20)
redMax2 = (8,255,255)


for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    img = img.array
    cv2.imshow('Captura', img)
    frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    k = cv2.waitKey(1) & 0xff
    if k == ESC:
        cam.close()
        break
        
    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # hacemos la mask y filtramos en la original
    mask1 = cv2.inRange(img_HSV, redMin1, redMax1)
    mask2 = cv2.inRange(img_HSV, redMin2, redMax2)
    mask_red = mask1 + mask2
    
    # detector finds "dark" blobs by default, so invert image for results with same detector
    keypoints_red = detector.detect(255-mask_red)
    
    # documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
    #"x":the x coordinate of each blob in the image.
    #"y":the y coordinate of each blob in the image.
    #"size":the diameter of the circle containing the blob.
    for kp in keypoints_red:
    	print (kp.pt[0], kp.pt[1], kp.size)

cv2.destroyAllWindows()

