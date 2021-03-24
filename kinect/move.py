import rospy,time,tf
from turtleAPI import robot
import numpy as np
from random import random 

def scale_angle(angle):
	if angle < -math.pi/4 or angle > math.pi/4:
		if target_y < 0 and curr_y < target_y:
			return -2*math.pi + angle
		elif target_y >= 0 and curr_y > target_y:
			return 2*math.pi + angle 

	return angle

def turn(angle, robot):
	currYaw = robot.getAngle()[2]
	finalAngle = scale_angle(currYaw + angle)
	
	sign = np.sign(angle)

	# print("deep in turn")
	RATE = rospy.Rate(10)
	speed = .4 * sign
	# print("here now")
	R.drive(angSpeed=speed, linSpeed=0)
	print("speed", speed)
	error = abs(finalAngle - robot.getAngle()[2])
	while error > .1:
			RATE.sleep()
			print(robot.getAngle()[2], finalAngle)

	return 

def get_depth(depth):
	depth_T = depth.T # transpose

	half = len(depth_T)/2
	left = depth_T[:half]
	right = depth_T[half:]

	left_mean = np.nanmean(left)
	right_mean = np.nanmean(right)

	diff = left_mean - right_mean
	# center
	if diff < .2 and diff > -.2:
		
	# right
	elif diff < -.2:

	# left
	else:


if __name__ == "__main__":
	try:
		print("creating robot")
    	r = robot()

		while not rospy.is_shutdown():
			depth = r.getDepth()/5 

			cv2.imshow("depth", depth)
			cv2.waitKey(1)

			sign = random.choice([1,-1])
			turn(sign*np.pi/4, r.getAngle()[2], r)

	except Exception as e:
		print(e)
		rospy.loginto("node now terminated")