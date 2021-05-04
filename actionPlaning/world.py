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

    def __init__(self, world_file, robot_loc,goal_file):
        self.located, self.on = parse_json(world_file)
        self.located['Robot'] = robot_loc
        self.cost = 0
        self.onRobot = 0
        self.actions = []
        self.goal = parse_json(goal_file)[0]

    # returns list of drive successors
    def drive(self):
        successors = []
        if self.checkRobot():
            #drive to each ballon locations
            curr_robot_loc = self.located['Robot']
            #drive to each balloon's location of this world
            for eachKey in self.located.keys():
                if eachKey is not 'Robot' and (self.located['Robot'] != self.located[eachKey]):
                    #get location of the ballon
                    new_robot_loc = self.located[eachKey]
                    #make copy of this self
                    temp = deepcopy(self)
                    #update the location (simulate the drive)
                    temp.located['Robot'] = new_robot_loc

                    for key in temp.on.keys():
                        if temp.on[key] == 'Robot':
                            temp.located[key] = new_robot_loc
                    # assign action to world
                    temp.actions.append(('drive', new_robot_loc))
                    if len(temp.actions) > 2 and temp.actions[-2][0] == 'drive':
                        # print(temp.action)
                        temp.increaseCost(200)
                    #cost actually distance to drive based on robot's location
                    temp.increaseCost(distance(curr_robot_loc,new_robot_loc)+ 2)
                    #append temp to allSuccessors
                    successors.append(temp)
            #drive to a ballon location of goal world
            for eachKey in self.goal.keys():
                if(self.located['Robot'] != self.goal[eachKey]):
                    #get location of the ballon
                    new_robot_loc = self.goal[eachKey]
                    #make copy of this self
                    temp = deepcopy(self)
                    #update the location (simulate the drive)
                    temp.located['Robot'] = new_robot_loc

                    for key in temp.on.keys():
                        if temp.on[key] == 'Robot':
                            temp.located[key] = new_robot_loc
                    # assign action to world
                    temp.actions.append(('drive', new_robot_loc))
                    # if temp.actions[:-1] == temp.actions[:-3] or temp.actions[:-1] == temp.actions[:-2]:
                    # if len(temp.actions) > 1:
                        # print("previous action", temp.actions[-2][0])
                    if len(temp.actions) > 2 and temp.actions[-2][0] == 'drive':
                        # print(temp.action)
                        temp.increaseCost(200)
                    #cost actually distance to drive based on robot's location
                    temp.increaseCost(distance(curr_robot_loc,new_robot_loc) + 2)
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
                temp.actions.append(('pickup', eachKey))
                #update cost of picking up a second balloon
                temp.increaseCost(5)
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
                temp.actions.append(('pickup', eachKey))
                #update cost of picking up 2 balloon
                temp.increaseCost(5)
                #append temp to allSuccessors
                successors.append(temp)

        return successors

    def putdown(self):
        successors = []
        for eachKey in self.located.keys():
            if (eachKey is not 'Robot') and (self.on[eachKey] is 'Robot') and self.onRobot > 0:
                # print('putdown')
                #make copy of this self
                temp = deepcopy(self)
                #update the location
                temp.located[eachKey] = temp.located['Robot']
                # update status
                temp.on[eachKey] = 'floor'
                # assign action to world
                temp.actions.append(('putdown', eachKey))
                temp.onRobot -= 1
                #update cost of picking up 1 ballon
                if temp.located[eachKey] != temp.goal[eachKey]:
                    temp.increaseCost(100)
                else:
                    temp.increaseCost(3)
                    # temp.cost = temp.cost / 2s
                #append temp to allSuccessors
                successors.append(temp)

        return successors

    def checkRobot (self):
        if 'Robot' in self.located.keys():
            return True
        return False

    def getOn(self):
        return self.on
    def getCost(self):
        return self.cost

    def getLocated(self):
        return self.located

    def robotHasBalloon(self,lookAt):
        for eachKey in self.on.keys():
            if (eachKey is not lookAt) and (self.on[eachKey] == 'Robot'):
                return True
        return False

    def updateCost(self,number):
        self.cost = number
    def increaseCost(self,number):
        self.cost += number
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

    def distanceFromGoal(self):
        '''
        A hueristic that sums of the distance of the balloons with where they are and where they should be

        goal_locations: a dictionary of the final balloon locations {['Red': (5,7), ...]}
        '''
        sum_ = 0
        for balloon in self.goal.keys():
            sum_ += distance(self.located[balloon], self.goal[balloon])

        # sum_ /= (1+self.onRobot)
        return sum_

    def getAction(self):
        return self.action

    def printWorld(self):
        print("this is where everything locate ", self.located)
        print("this is everything that are on ", self.on)
        print("cost to get to this world is ", self.cost)

    #return flase if the two world are the same
    def __ne__(self):
        #check locations of everything
        #locationFromWorld2 = world2.getLocated()
        for world2Key in self.goal.keys():
            #for world1Key in self.located.keys():
            if self.goal[world2Key] != self.located[world2Key]:
                #print(goal_locs[world2Key])
                #print(self.located[world2Key])
                return True
        #check where everything on
        #onFromWorld2 = world2.getOn()
        
        for key in self.on.keys():
            if self.on[key] == 'Robot':
                return True
                    
        return False



############ end of world class ###############


'''

w = World('simple.json', (1,3))
w.printWorld()
allChildren = w.getSuccessors()
for i in allChildren:
    i.printWorld()
'''
