import cv2
import numpy as np
import rospy,time,tf
from turtleAPI import robot
from filter import Filter


def PID_img(curr_img, end_img, prev_img):
    Kp = 0.0003
    Ekp = Kp*curr_img

    Ki = 0.0
    Eki = 0
    for i in range(0,len(prev_img)):
        Eki += prev_img[i]
    Eki = Ki*Eki

    Kd = 0
    Ekd = curr_img - prev_img[len(prev_img)-2]
    Ekd = Kd*Ekd

    return Ekp + Eki + Ekd

try:
    colors = ['red', 'blue', 'green', 'purple', 'yellow']
    print("creating robot")
    r = robot()
    color = raw_input("Choose your color: ")
    found = False
    popped = False

    if color not in colors:
        print("Please choose from one of the following colors (case sensitive):\n")
        print(colors)
        color = raw_input("Choose your color: ")
    if color not in colors:
        exit(1)

    prev_img = []
    while not popped and not rospy.is_shutdown():
        img = r.getImage()
        depth=r.getDepth()/5
        f_img, mask = Filter.get_filtered(img, color)
        curr_tup = Filter.get_PID_value(mask)

        mean_distance = np.nanmean(depth[mask != 0])
        print("distance", mean_distance)

        cv2.imshow("Filtered Image", f_img)
        # cv2.imshow("Depth",dpth)
        # print(dpth)
        cv2.waitKey(1)

        curr = curr_tup[0]
        detected = curr_tup[1]

        if detected > 30:
            prev_img.append(curr)
            pid_speed = PID_img(curr, 0, prev_img)
            print(pid_speed)
            found = True
        else:
            found = False

        if mean_distance < .4:
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
