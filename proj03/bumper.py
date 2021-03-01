# Gregory Polmatier
# For project 3 with Signorelli
import random
import math

import rospy
from turtleAPI import robot

try:
    print("creating robot")
    r = robot()
    # drive straight
    r.drive(angSpeed=0, linSpeed=.25)
    while not rospy.is_shutdown():
        bump = r.getBumpStatus()
        if bump['status'] == 1:
            if bump['bumper'] == 1:
                if random.choice([True, False]):
                    turn(math.pi/2)
                else:
                    turn(-math.pi/2)    
            elif bump['bumper'] == 0:
                turn(-math.pi/4)
            else:
                turn(math.pi/4)
        
except Exception as e:
    print(e)
    rospy.loginto("node now terminated")


def turn(angle):
    finalAngle = r.getAngle() + angle
    
    sign = 1
    if angle < 0:
        sign = -1

    if angle == 0:
        return

    speed = .3
    r.drive(angSpeed=speed*sign, linSpeed=0)
    while diff(finalAngle, r.getAngle()) > .1:
        print(r.getAngle, finalAngle)

    return 

def diff(a1, a2):
    return abs(a1-a2)