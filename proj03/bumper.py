# Gregory Polmatier
# For project 3 with Signorelli
import random
import math

import rospy
from turtleAPI import robot

R = robot()

def diff(afinal, acurr):
    x = afinal
    if afinal > math.pi:
        x = -1 * (afinal - math.pi)

    if afinal < -math.pi:
        x = -1 * (afinal + math.pi)
    return abs(acurr - x)

def turn(angle):
    currYaw = R.getAngle()[2]
    finalAngle = currYaw + angle
    
    sign = 1
    if angle < 0:
        sign = -1

    if angle == 0:
        return

    # print("deep in turn")
    RATE = rospy.Rate(10)
    speed = .4
    # print("here now")
    R.drive(angSpeed=speed*sign, linSpeed=0)
    # print("1")
    while diff(finalAngle, R.getAngle()[2]) > .1:
        RATE.sleep()
        # print("2")
        print(R.getAngle()[2], finalAngle)

    return 

try:
    print("creating robot")
    RATE = rospy.Rate(10)
    # drive straight
    R.drive(angSpeed=0, linSpeed=.15)
    while not rospy.is_shutdown():
        RATE.sleep()
        bump = R.getBumpStatus()
        print(bump)
        # print(bump['status'])
        if bump['state'] == 1:
            if bump['bumper'] == 1:
                if random.choice([True, False]):
                    print("got to turn\n\n\n")
                    turn(math.pi/2)
                else:
                    turn(-math.pi/2)    
            elif bump['bumper'] == 0:
                turn(-math.pi/4-.314)
                print("cw\n\n\n")
            else:
                print("ccw\n\n\n")
                turn(math.pi/4 + .314)
        R.drive(angSpeed=0, linSpeed=.25+.314)
        
except Exception as e:
    print(e)
    # rospy.loginto("node now terminated")




