# -*- coding: utf-8 -*-
#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;

# Read image
img = cv2.imread("prueba.jpg")
img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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

#keypoints on original image (will look for blobs in grayscale)
keypoints = detector.detect(img_HSV)
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show blobs
cv2.imshow("Keypoints on Gray Scale", im_with_keypoints)
cv2.waitKey(0)

#filter certain COLOR channels

# Elegimos el umbral de rojo en HSV
redMin1 = (170,100,20)
redMax1 = (179,255,255)
# Elegimos el segundo umbral de rojo en HSV
redMin2 = (0,100,20)
redMax2 = (8,255,255)

# hacemos la mask y filtramos en la original
mask1 = cv2.inRange(img_HSV, redMin1, redMax1)
mask2 = cv2.inRange(img_HSV, redMin2, redMax2)
mask_red = mask1 + mask2

# apply the mask
red = cv2.bitwise_and(img, img, mask = mask_red)
cv2.imshow("Red regions", np.hstack([img, red]))

# detector finds "dark" blobs by default, so invert image for results with same detector
keypoints_red = detector.detect(255-mask_red)

# documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
#"x":the x coordinate of each blob in the image.
#"y":the y coordinate of each blob in the image.
#"size":the diameter of the circle containing the blob.
for kp in keypoints_red:
	print (kp.pt[0], kp.pt[1], kp.size)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(img, keypoints_red, np.array([]),
	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show mask and blobs found
cv2.imshow("Keypoints on RED", im_with_keypoints)
cv2.waitKey(0)

