import cv2
import numpy as np

imgB = cv2.imread('blue.png')

hsvB = cv2.cvtColor(imgB, cv2.COLOR_BGR2HSV)

# Threshold of blue in HSV space
lower_blue = np.array([90, 50, 70])
upper_blue = np.array([128, 255, 255])

# preparing the mask to overlay
mask = cv2.inRange(hsvB, lower_blue, upper_blue)
    
# The black region in the mask has the value of 0,
# so when multiplied with original image removes all non-blue regions
result = cv2.bitwise_and(imgB, imgB, mask = mask)

cv2.imshow('frame', imgB)
cv2.waitKey(0)
cv2.imshow('mask', mask)
cv2.waitKey(0)
cv2.imshow('result', result)
cv2.waitKey(0)