# Gregory Polmatier
# For project 3 with Signorelli

import rospy
from turtleAPI import robot

try:
    print("creating robot")
    r = robot()
    # drive straight
    r.drive(angSpeed=0, linSpeed=.25)
    while not rospy.is_shutdown():
        print(r.getBumpStatus())
except Exception as e:
    print(e)
    rospy.loginto("node now terminated")
