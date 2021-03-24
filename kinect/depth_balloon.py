import cv2
import numpy as np
import rospy,time,tf
from turtleAPI import robot

from filter import Filter
from pid import PID

# def PID_img(curr_img, end_img, prev_img):
#     Kp = 0.0006
#     Ekp = Kp*curr_img

#     Ki = 0.0
#     Eki = 0
#     for i in range(0,len(prev_img)):
#         Eki += prev_img[i]
#     Eki = Ki*Eki

#     Kd = 0
#     Ekd = curr_img - prev_img[len(prev_img)-2]
#     Ekd = Kd*Ekd

#     return Ekp + Eki + Ekd

try:
    colors = ['red', 'blue', 'green', 'purple', 'yellow']
    print("creating robot")
    r = robot()
    color = raw_input("Choose your color: ")
    found = False
    # popped = False

    if color not in colors:
        print("Please choose from one of the following colors (case sensitive):\n")
        print(colors)
        color = raw_input("Choose your color: ")
    if color not in colors:
        exit(1)

    # prev_img = []
    pid = PID(kp=.0006, ki=.0001, kd=.0001)
    while not rospy.is_shutdown():
        img = r.getImage()
        depth=r.getDepth()
        flter = Filter(img, color, depth)

        f_img = flter.get_filtered()
        loc_val, num_detected = flter.get_PID_value()
        mean_distance = flter.mean_distance()
        print("distance", mean_distance)

        cv2.imshow("Filtered Image", f_img)
        cv2.waitKey(1)

        if num_detected > 30:
            prev_img.append(loc_val)
            pid_speed = pid(loc_val)
            print(pid.components())
            # print(pid_speed)
            found = True
        else:
            found = False

        if mean_distance < .17:
            r.drive(angSpeed=0, linSpeed=0)
            print("arrived")
        elif not found:
            r.drive(angSpeed=0.2, linSpeed=0)
        else:
            r.drive(angSpeed=pid_speed, linSpeed=0.3)
        time.sleep(0.1)
        #cv2.waitKey(1)

except Exception as e:
  print(e)
  rospy.loginto("node now terminated")
