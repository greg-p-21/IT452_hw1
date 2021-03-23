import numpy as np
import cv2
import rospy,time,tf
from turtleAPI import robot

from filter import Filter

def get_angle_pid(currAngle, targetAngle, k):
        angleVel = angleDiff - curr_yaw

        scaledAngle = AnglePID.scale_angle(angleVel)

        return scaledAngle*k

def scale_angle(angle):
	if angle < -math.pi/4 or angle > math.pi/4:
		if target_y < 0 and curr_y < target_y:
			return -2*math.pi + angle
		elif target_y >= 0 and curr_y > target_y:
			return 2*math.pi + angle 

	return angle

def turn(angle):
	currYaw = R.getAngle()[2]
	finalAngle = toScale(currYaw + angle)
	
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
	while abs(tfinalAngle - R.getAngle()[2]) > .1:
			RATE.sleep()
			# print("2")
			print(R.getAngle()[2], finalAngle)

	return 

if __name__ == "__main__":
