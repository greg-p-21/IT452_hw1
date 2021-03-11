import cv2
import numpy as np

color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
              'white': [[180, 18, 255], [0, 0, 231]],
              'red1': [[180, 255, 255], [159, 50, 70]],
              'red2': [[9, 255, 255], [0, 50, 70]],
              'green': [[89, 255, 255], [36, 50, 70]],
              'blue': [[128, 255, 255], [90, 50, 70]],
              'yellow': [[35, 255, 255], [25, 50, 70]],
              'purple': [[158, 255, 255], [129, 50, 70]],
              'orange': [[24, 255, 255], [10, 50, 70]],
              'gray': [[180, 18, 230], [0, 0, 40]]}

imgB = cv2.imread('blue.png')
imgG = cv2.imread('green.png')

hsvB = cv2.cvtColor(imgB, cv2.COLOR_BGR2HSV)
hsvG = cv2.cvtColor(imgG, cv2.COLOR_BGR2HSV)

# Threshold of blue in HSV space
# lower_blue = np.array([90, 50, 70])
# upper_blue = np.array([128, 255, 255])

# # Threshold of green in HSV space
# lower_green = np.array()
# upper_green = np.array()

# preparing the mask to overlay
mask = cv2.inRange(hsvB, color_dict_HSV['blue'][1], color_dict_HSV['blue'][0])
maskG = cv2.inRange(hsvG, color_dict_HSV['green'][1], color_dict_HSV['green'][0])
    
# The black region in the mask has the value of 0,
# so when multiplied with original image removes all non-blue regions
result = cv2.bitwise_and(imgG, imgG, mask = maskG)

aug = imgG

aug[maskG != 0] = [0,255,0]

cv2.imshow('frame', imgG)
cv2.waitKey(0)
cv2.imshow('result', result)
cv2.waitKey(0)
cv2.imshow('augmented', aug)
cv2.waitKey(0)