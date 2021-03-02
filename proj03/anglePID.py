from math import radians, copysign, pow, pi, atan2, sqrt
from turtleAPI import robot

R = robot()

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05

# values to reach
(goal_x, goal_y) = (2, 3)


linSpeed = 1
angSpeed = 1

previous_angle = 0
total_angle = 0

(start_x, start_y, start_yaw) = R.getPositionTup()
goalDistance = sqrt(pow(goal_x - start_x, 2) + pow(goal_y - start_y, 2))
distance = goalDistance

while distance > 0.05:
    (curr_x, curr_y, curr_yaw) = R.getPositionTup()
    angle = atan2(goal_y - curr_y, goal_x - curr_x)

    if angle < -pi/4 or angle > pi/4:
        if goal_y < 0 and curr_y < goal_y:
            angle = -2*pi + angle
        elif goal_y >= 0 and curr_y > goal_y:
            angle = 2*pi + angle
    
    total_angle += angle
    dif_angle = angle - previous_angle #for kd

    pid_angle = kp_angle*angle + ki_angle*total_angle + kd_angle*dif_angle


    #drive robot speed
    

    # do distance calcs
    #
    #
    #
    ##
    #

    # final distance 
    pid_distance = 0

