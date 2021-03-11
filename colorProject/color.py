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

img = cv2.imread('purple.png')

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Threshold of blue in HSV space
# lower_blue = np.array([90, 50, 70])
# upper_blue = np.array([128, 255, 255])

# # Threshold of purple in HSV space
# lower_green = np.array()
# upper_green = np.array()

print(color_dict_HSV['purple'][1])
# preparing the mask to overlay
mask = cv2.inRange(hsv, np.array(color_dict_HSV['purple'][1]), np.array(color_dict_HSV['purple'][0]))
    
# The black region in the mask has the value of 0,
# so when multiplied with original image removes all non-blue regions
result = cv2.bitwise_and(img, img, mask = mask)

aug = img

aug[mask != 0] = [0,255,0]

cv2.imshow('frame', img)
cv2.waitKey(0)
cv2.imshow('result', result)
cv2.waitKey(0)
cv2.imshow('augmented', aug)
cv2.waitKey(0)