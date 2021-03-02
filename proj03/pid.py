
import math
from turtleAPI import robot
import rospy

class PID:
    
    def __init__(self):
        self.Kp = .05
        self.Ki = 0.01
        self.Kd = 0.01
        
        self.kp_angle = .1
        self.ki_angle = .05
        self.kd_angle = .01

        self.distErrorTrack = list() #list representing previous distance errors
        self.angErrorTrack = list() #list representing previous distance errors
        self.stepTrack = 10 #how far into the past should we consider for the integration step

    #Quick sumation function to take care of the math required for the integration portion of the PID 
    #Pass in the errorList for either the distance list or the angle list
    def integralIsh(self, errorList):
        total = 0
        for i in errorList:
            total += i
        return total

    # Function to calculate the coordinate error between the turtlebot and its target coordinate
    # Input its current position as a tuple returned from getPositionTuple()
    # Input its target coordinate as a tuple given by user input
    def distError(self, current, target):
        currentX = current[0]
        currentY = current[1]
        targetX = target[0]
        targetY = target[1]
    
        #middle work to simplify the distance calculation line
        difX = targetX - currentX
        difY = targetY - currentY

        #calculate the raw distance between the 2 points which represents the coordinate error
        dist = math.sqrt((math.pow(difX, 2.0)) + (math.pow(difY, 2.0)))
        
        self.distErrorTrack.append(dist)
        
        #update the list of previous errors to only include up to stepTrack items
        if len(self.distErrorTrack) > self.stepTrack:
            trash = self.distErrorTrack.pop(0)

    # Input its current position as a tuple returned from getPositionTuple()
    # Input its target coordinate as a tuple given by user input
    def distPID(self, current, target):
        self.distError(current, target) #update the error log
        
        proportional = self.Kp * self.distErrorTrack[len(self.distErrorTrack)-1]

        integral = self.Ki * self.integralIsh(self.distErrorTrack)

        derivative = self.Kd * (self.distErrorTrack[len(self.distErrorTrack)-1] - self.distErrorTrack[len(self.distErrorTrack)-2])

        print("distance kp,ki,kd", (proportional, integral, derivative))

        pidVal = proportional + integral + derivative

        return pidVal

    def anglePID(self, current, target, last_rotation):
        target_x = target[0]
        target_y = target[1]

        (curr_x, curr_y, curr_yaw) = current
        angle = math.atan2(target_y - curr_y, target_x - curr_x)

        if angle < -math.pi/4 or angle > math.pi/4:
            if target_y < 0 and curr_y < target_y:
                angle = -2*math.pi + angle
            elif target_y >= 0 and curr_y > target_y:
                angle = 2*math.pi + angle 
        if last_rotation > math.pi-0.1 and curr_yaw <= 0:
            curr_yaw = 2*math.pi + curr_yaw
        elif last_rotation < -math.pi+0.1 and curr_yaw > 0:
            curr_yaw = -2*math.pi + curr_yaw

        angleVel = angle - curr_yaw
        
        self.angErrorTrack.append(angleVel)
        if len(self.angErrorTrack) > self.stepTrack:
            self.angErrorTrack.pop(0)

        dif_angle = angleVel - self.angErrorTrack[len(self.angErrorTrack)-2] #for kd

        proportional = self.kp_angle * self.angErrorTrack[len(self.angErrorTrack)-1]

        integral = self.ki_angle * self.integralIsh(self.angErrorTrack)

        derivative = self.kd_angle * dif_angle

        print("angle kp,ki,kd", (proportional, integral, derivative))

        pid_angle = proportional + integral + derivative

        return pid_angle

    def GoTo(self, target):
        R = robot()
        RATE = rospy.Rate(10)

        start = R.getPositionTup()
        start_x = start[0]
        start_y = start[1]

        target_x = target[0]
        target_y = target[1]

        goalDistance = math.sqrt(pow( start_x- target_x, 2) + pow(start_y - target_y, 2))
        distance = goalDistance

        last_rotation = 0

        while distance > 0.05:
            RATE.sleep()

            current = R.getPositionTup()
            # self.distError(current, target)

            distance_pid = self.distPID(current, target)
            angle_pid = self.anglePID(current, target, last_rotation)

            print("dist and angle pid", (distance_pid, angle_pid))

            # modify pid ie set max mins and put into a value
            # to insert into drive function below
            lspeed = min(distance_pid, 0.3)
            aspeed = angle_pid

            R.drive(angSpeed=aspeed, linSpeed=lspeed)

            print("current position", R.getPositionTup())

            distance = self.distErrorTrack[len(self.distErrorTrack)-1]
            last_rotation = current[2]

        print("Reached point")
        R.drive(angSpeed=0, linSpeed=0)

if __name__ == "__main__":
    for i in range(2):
        print("Enter desired x:")
        x = input()

        print("Enter desired y:")
        y = input()

        pid = PID()

        pid.GoTo((x, y))

    
