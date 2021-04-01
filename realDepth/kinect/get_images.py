import cv2
import rospy,time,tf

from turtleAPI import robot
from filter import Filter
from pid import PID

def getColor():
    # start robot and ask for color
    colors = ['red', 'blue', 'green', 'purple', 'yellow']
    color = raw_input("Choose your color: ")

    if color not in colors:
        print("Please choose from one of the following colors (case sensitive):\n")
        print(colors)
        color = raw_input("Choose your color: ")
    if color not in colors:
        exit(1)

    return color

try:
    # start robot and ask for color
    print("creating robot")
    color = getColor()
    r = robot()
    found = False

    while not rospy.is_shutdown():
        # get image and depth info
        img = r.getImage()
        depth=r.getDepth()
        # create filter object
        flter = Filter(img, color, depth)

        fname = color+'.png'
        # show image 
        cv2.imshow("Filtered Image", f_img)
        cv2.imwrite(fname, img)
        cv2.waitKey(0)


except Exception as e:
  print(e)
  rospy.loginto("node now terminated")