from json import load
from copy import deepcopy
from math import sqrt

def parse_json(file):
    located = {}
    on = {}

    #open json file for initial world
    f = open(file)
    fdata = load(f)

    #goes through each key from json file 
    for key in fdata.keys():
        #set the world 
        located[key] = fdata[key]
        on[key] = 'floor'

    return located, on 

def distance(tuple1,tuple2): 
    return sqrt((tuple1[0]-tuple2[0])**2 + (tuple1[1]-tuple2[1])**2)

class World:

    def __init__(self, world_file, robot_loc):
        self.located, self.on = parse_json(world_file)
        self.located['Robot'] = robot_loc
        self.cost = 0
        self.onRobot = 0
        self.action = None
    
    # returns list of drive successors
    def drive(self):
        successors = []
        if self.checkRobot(): 
            #drive to each ballon locations
            curr_robot_loc = self.located['Robot']
            #drive to each balloon's location
            for eachKey in self.located.keys(): 
                if eachKey is not 'Robot' :
                    #get location of the ballon 
                    new_robot_loc = self.located[eachKey]
                    #make copy of this self  
                    temp = deepcopy(self)
                    #update the location (simulate the drive)
                    temp.located['Robot'] = new_robot_loc
                    # assign action to world
                    temp.action = ('drive', new_robot_loc)
                    #cost actually distance to drive based on robot's location 
                    temp.updateCost(distance(curr_robot_loc,new_robot_loc))
                    #append temp to allSuccessors 
                    successors.append(temp)
        
        return successors

    def pickup1(self):
        successors = []
        for eachKey in self.located.keys():
            if (eachKey is not 'Robot') and (self.on[eachKey] is 'floor') and (self.located['Robot'] == self.located[eachKey]) and self.onRobot == 1:
                #pick up ballon (update where balloon on)
                #make copy of this self  
                temp = deepcopy(self)
                # add to onRobot
                temp.onRobot += 1
                #change status of balloon
                temp.on[eachKey] = 'Robot'
                # assign action to world
                temp.action = ('pickup', NULL)
                #update cost of picking up a second balloon
                temp.updateCost(5)
                #append temp to allSuccessors 
                successors.append(temp)

        return successors
        
    def pickup2(self):
        successors = []
        for eachKey in self.located.keys():
            if (eachKey is not 'Robot') and (self.on[eachKey] is 'floor') and (self.located['Robot'] == self.located[eachKey]) and self.onRobot == 0:
                #pick up the second balloon (update where the second balloon on)
                #make copy of this self  
                temp = deepcopy(self)
                # add to onRobot
                temp.onRobot += 1
                # update on
                temp.on[eachKey] = 'Robot'
                # assign action to world
                temp.action = ('pickup', NULL)
                #update cost of picking up 2 balloon 
                temp.updateCost(7)
                #append temp to allSuccessors 
                successors.append(temp)

        return successors
    
    def putdown(self):
        successors = []
        for eachKey in self.located.keys():
            if (eachKey is not 'Robot') and (self.on[eachKey] is 'Robot') and (self.located['Robot'] == self.located[eachKey]) and self.onRobot > 0:
                #make copy of this self  
                temp = deepcopy(self)
                #update the location 
                temp.located[eachKey] = temp.located['Robot']
                # update status 
                temp.on[eachKey] = 'floor'
                # assign action to world
                temp.action = ('putdown', NULL)
                #update cost of picking up 1 ballon 
                temp.updateCost(5)
                #append temp to allSuccessors 
                successors.append(temp)

        return successors

    def checkRobot (self): 
        if 'Robot' in self.located.keys(): 
            return True 
        return False 

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

    def getSuccessors(self):
        #list of all successors 
        allSuccessors = [] 

        #1. add drive action successors 
        allSuccessors.extend(self.drive())

        #2.Check if robot can pick up a second ballon (any color)
        allSuccessors.extend(self.pickup1())
        
        #3.Check if robot can pick up a balloon (any color) from none
        allSuccessors.extend(self.pickup2())

        #4.Check if robot can put down a balloon
        allSuccessors.extend(self.putdown())

        return allSuccessors

    def distanceFromGoal(self, goal_locations):
        '''
        A hueristic that sums of the distance of the balloons with where they are and where they should be 
        
        goal_locations: a dictionary of the final balloon locations {['Red': (5,7), ...]}
        '''
        sum_ = 0
        for balloon in goal_locations.keys():
            sum_ += distance(self.located[balloon], goal_locations[balloon])
        
        return sum_

    def getAction(self):
        return self.action

    def printWorld(self):
        print("this is where everything locate ", self.located)
        print("this is everything that are on ", self.on)
        print("cost to get to this world is ", self.cost)
        

############ end of world class ###############




w = World('simple.json', (1,3))
w.printWorld()
allChildren = w.getSuccessors()
for i in allChildren:
    i.printWorld()



