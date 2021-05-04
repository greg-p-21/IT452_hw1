import json
import copy
import math
class World:

    def __init__(self, world_file, robot_loc):
        self.located = {}
        self.on = {}
        self._create_world(world_file, robot_loc)
        self.cost = 0
        self.robot_loc = robot_loc

    def _create_world(self, file, robot_loc):
        #open json file for initial world
        f = open(file)
        fdata = json.load(f)

        #goes through each key from json file
        for key in fdata.keys():
            #set the world
            self.located[key] = fdata[key]
            self.on[key] = 'floor'

        #add a robot to the initial world
        self.located['Robot'] =  robot_loc

        #on floor
        #print(self.located, self.on)

    def drive(self,new_robot_loc):
        print("drive a robot to location ", new_robot_loc)
        self.located['Robot'] = new_robot_loc

    def pickup1(self,eachKey):
        print("pick up 1 ballon, the first one is ", eachKey)
        self.on[eachKey] = 'Robot'

    def pickup2(self,eachKey):
        print("pick up 2 ballon, the second one is ", eachKey)
        self.on[eachKey] = 'Robot'

    def checkRobot (self):
        if 'Robot' in self.located.keys():
            return True
        return False

    def putdown(self,eachKey):
        print("put down the ballon ", eachKey)
        self.on[eachKey] = 'floor'

    def getOn(self):
        return self.on

    def getLocated(self):
        return self.located

    def robotHasBalloon(self,lookAt):
        for eachKey in self.on.keys():
            if (eachKey is not lookAt) and (self.on[eachKey] == 'Robot'):
                return True
        return False

    def updateCost(self,number):
        self.cost = number

    def printWorld(self):
        print("this is where everything locate ", self.located)
        print("this is everything that are on ", self.on)
        print("cost to get to this world is ", self.cost)

############ end of world class ###############

def distance(tuple1,tuple2):
    return math.sqrt((tuple1[0]-tuple2[0])**2 + (tuple1[1]-tuple2[1])**2)

def getSuccessors(world):
    #list of all successors
    allSuccessors = []
    #check the state of each world to see what children they can create
    allLocations = world.getLocated()
    allOn = world.getOn()
    #1.Check there is robot in the world, so it can drive
    if world.checkRobot():
        #drive to each ballon locations
        curr_robot_loc = allLocations['Robot']
        #drive to each balloon's location
        for eachKey in allLocations.keys():
            if eachKey is not 'Robot' :
                #get location of the ballon
                new_robot_loc = allLocations[eachKey]
                #make copy of this world
                temp = copy.deepcopy(world)
                #update the location (simulate the drive)
                temp.drive(new_robot_loc)
                #cost actually distance to drive based on robot's location
                temp.updateCost(distance(curr_robot_loc,new_robot_loc))
                #append temp to allSuccessors
                allSuccessors.append(temp)

        #2.Check if robot can pick up 1 ballon (any color)
        #check if a balloon on the floor
        for eachKey in allLocations.keys():
            #
            print("robot at ", allLocations['Robot'])
            if (eachKey is not 'Robot') and (allOn[eachKey] is 'floor') and (allLocations['Robot'] == allLocations[eachKey]):
                #pick up ballon (update where balloon on)
                #make copy of this world
                temp = copy.deepcopy(world)
                #update the location (simulate the drive)
                temp.pickup1(eachKey)
                #update cost of picking up 1 ballon
                temp.updateCost(10)
                #append temp to allSuccessors
                allSuccessors.append(temp)

        #3.Check if robot can pick up a second balloon (any color)
        for eachKey in allLocations.keys():
            if (eachKey is not 'Robot') and (allOn[eachKey] is 'floor') and (allLocations['Robot'] == allLocations[eachKey]):
                #check if there any ballon on the robot beside the one that it looking at
                if (world.robotHasBalloon(eachKey)):
                    #pick up ballon the second balloon (update where the second balloon on)
                    #make copy of this world
                    temp = copy.deepcopy(world)
                    #update the location (simulate the drive)
                    temp.pickup2(eachKey)
                    #update cost of picking up 2 balloon
                    temp.updateCost(7)
                    #append temp to allSuccessors
                    allSuccessors.append(temp)

        #4.Check if robot can put down a balloon
        for eachKey in allLocations.keys():
            if (eachKey is not 'Robot') and (allOn[eachKey] is 'Robot') and (allLocations['Robot'] == allLocations[eachKey]):
                #make copy of this world
                temp = copy.deepcopy(world)
                #update the location (simulate the drive)
                temp.putdown(eachKey)
                #update cost of picking up 1 ballon
                temp.updateCost(7)
                #append temp to allSuccessors
                allSuccessors.append(temp)
        return allSuccessors


w = World('simple.json', (1,3))
w.printWorld()
allChildren = getSuccessors(w)
for i in allChildren:
    i.printWorld()
