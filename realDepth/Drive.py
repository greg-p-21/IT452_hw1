import rospy
from turtleAPI import robot
import time

class drive:
    def __init__(self):
        print("creating robot")
        self.r = robot()
        self.sums = []

        return

    #grab image from the robot
    def getImage(self):
        return self.r.getImage()

    #chooses between continuous driving versus pid controller
    def protocol(self, target, l, r):
        if target == True:  #PID controller activated once target found
            self.pidDrive(l,r)
        else:               #Keep driving if target not found yet
            self.contDrive()
            
        return

    def contDrive(self):
        self.r.drive( angSpeed = 0.08, linSpeed = 0)
        print('cont')
        return

    def pidDrive(self, l, r):
        #proportional
        prop = self.turn(l, r)* .0008
        
        #integral
        if len(self.sums) < 7:
            self.sums.append(prop)
        elif len(self.sums) >= 7:
            self.sums.pop(0)
            self.sums.append(prop)
        integ = self.sum(self.sums)
        
        print("prop", prop, "integ", integ)
        self.r.drive(prop + .00007 * integ, 0.1)
        print('pid')
        return

    #determines if the turtlebot should be turning left(+) or rignt(-)
    def turn(self, l,r):
        if r > l:
            return -(r-l)
        elif l > r:
            return (l-r)
        else:
            return 1

    #Performs the summation of up to the last 7 pid driving values
    def sum(self, n):
        sum = 0 
        for i in range(len(n)):
            sum += n[i]
        return sum


