import cv2
import numpy as np

imgB = cv2.imread('blue.JPG')

hsvB = cv2.cvtColor(imgB, cv2.COLOR_BGR2HSV)

