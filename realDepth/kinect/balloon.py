import rospy
from Augment import augment
from Drive import drive
import cv2
import time

try:
    a = augment()
    d = drive()
    while not rospy.is_shutdown():
        #grab visual from robot
        img = d.getImage()
        #augment image to view target balloon, returns binary mask
        aug = a.identify(img)
        
        #Splits mask into left and right sides
        augL = aug[0:len(aug)//2]
        augR = aug[len(aug)//2:len(aug)]       
        left = 0
        right = 0
        for i in range(320):
            if(255 in augL[i]):
                left += 1
            if(255 in augR[i]):
                right += 1

        #determine if target exists (impacts driving protocol)
        target = False
        if left >= 10 or right >= 10:
            target = True
        print("left", left, "right", right)
        
        #determine if robot will hit something (impacts driving protocol)
        
        
        
        #drive
        d.protocol(target, left, right)

except Exception as e:
    print(e)
    rospy.loginto("node now terminated")


