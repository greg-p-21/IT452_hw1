import math
import rospy
# from turtleAPI import robot

def distError(current, target):
        currentX = current[0]
        currentY = current[1]
        targetX = target[0]
        targetY = target[1]
    
        #middle work to simplify the distance calculation line
        difX = targetX - currentX
        difY = targetY - currentY

        #calculate the raw distance between the 2 points which represents the coordinate error
        dist = math.sqrt((math.pow(difX, 2.0)) + (math.pow(difY, 2.0)))
        
        return dist

def angleError(current, target):
        target_x = target[0]
        target_y = target[1]

        (curr_x, curr_y, curr_yaw) = current
        angle = math.atan2(target_y - curr_y, target_x - curr_x)
        angleVel = angle - curr_yaw

        while angleVel < -math.pi/2 or angleVel > math.pi/2:
            if angleVel > math.pi/2:
                angleVel = -2*math.pi + angleVel
            elif angleVel < -math.pi/2:
                angleVel = 2*math.pi + angleVel 
        
        return angleVel

def GoTo(R, target, dist_pid, ang_pid):
    start = R.getMCLPose()
    RATE = rospy.Rate(10)

    start_x = start[0]
    start_y = start[1]

    target_x = target[0]
    target_y = target[1]

    goalDistance = math.sqrt(pow( start_x- target_x, 2) + pow(start_y - target_y, 2))
    distance = goalDistance

    while distance > 0.5 and not rospy.is_shutdown():
        RATE.sleep()
        print("entered loop")

        current = R.getMCLPose()
        # self.distError(current, target)

        lspeed = -1* dist_pid(distError(current, target))
        aspeed = -1* ang_pid(angleError(current, target))

        print("distance", distance)
        print("dist and angle pid", (lspeed, aspeed))

        if aspeed > .1 or aspeed < -.1:
            R.drive(angSpeed=aspeed, linSpeed=0)
        else:
            R.drive(angSpeed=aspeed, linSpeed=lspeed)

        print("current position", R.getPositionTup())
        print("goal", target)

        distance = math.sqrt(pow( current[0] - target_x, 2) + pow(current[1] - target_y, 2))


    print("Reached point")
