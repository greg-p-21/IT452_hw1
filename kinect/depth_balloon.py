import cv2
import rospy,time,tf

from turtleAPI import robot
from filter import Filter
from pid import PID

THRESHOLD = 30
MAX_DISTANCE = 0.17
ANGLE_SPEED = 0.2
LINEAR_SPEED = 0.3

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
    r = robot()
    found = False
    color = getColor()

    #create pid controller
    pid = PID(kp=.0006, ki=.00006, kd=.001)
    
    while not rospy.is_shutdown():
        # get image and depth info
        img = r.getImage()
        depth=r.getDepth()
        # create filter object
        flter = Filter(img, color, depth)

        # get filter info 
        f_img = flter.get_filtered
        loc_val, num_detected = flter.get_PID_value
        mean_distance = flter.mean_distance
        print("distance", mean_distance)

        # show image 
        cv2.imshow("Filtered Image", f_img)
        cv2.waitKey(1)

        # make sure there is enough of the color there
        if num_detected > THRESHOLD:
            pid_angSpeed = pid(loc_val)
            # print(pid.components)
            found = True
        else:
            found = False

        # check if arrived at balloon
        if mean_distance < MAX_DISTANCE:
            r.drive(angSpeed=0, linSpeed=0)
            print("arrived")
            rospy.signal_shutdown("made it to balloon")
        elif not found: # Balloon not in view
            r.drive(angSpeed=ANGLE_SPEED, linSpeed=0)
        else: # Correct Balloon in image 
            r.drive(angSpeed=pid_angSpeed, linSpeed=LINEAR_SPEED)
        time.sleep(0.1)

except Exception as e:
  print(e)
  rospy.loginto("node now terminated")
