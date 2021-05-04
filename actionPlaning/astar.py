#implementation of aStart
# from PriQue_1 import *
from heapque import PQueue
from world import *
def aStar(start):
    ##set up##
    closed = []
    opened = PQueue()
    #goal = goal_locs
    parent = {}
    path = []
    origin = start
    #numNodeExpanded = 0

    num = 0
    #compare two world
    while ( start.__ne__() ):
        num += 1
        print("in aStar")
        closed.append(start)
        allSuccessors = start.getSuccessors()
        # print(allSuccessors.print)
        for w in allSuccessors :       ### w is a world
            if w not in closed :       ### expand

                ###  calculate F(w) of each successor
                ###  F(w) = G(w) + H(w)
                ###  G(w) cost to get to this world
                ###  H(w) cost to get to the goal => Heuristic
                Fcost = w.getCost() + w.distanceFromGoal()

                #update cost of each world
                # w.updateCost(Fcost)

                ##point successor w back to its parent (start)
                parent[w] = start

                ##add eachSuccessor to opened
                opened.add(Fcost,w)

        start = opened.get() ####

    return start.actions

######################################################################################
