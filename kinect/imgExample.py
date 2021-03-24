import rospy
from turtleAPI import robot
from Augment import augment
import cv2
import numpy as np
a = augment()

try:
  print("creating robot")
  r= robot()
  while not rospy.is_shutdown():
    depth=r.getDepth()
    grad = depth/5
    #switches rows and columns
    s = a.switch(grad) #col by row
    print("depth", s)
    
    img=r.getImage()
    print("img", img.T)
    cv2.imshow("Depth",grad)
    cv2.imshow("Image",img)
    cv2.waitKey(1)
except Exception as e:
  print(e)
  rospy.loginto("node now terminated")
