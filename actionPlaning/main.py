import time, argparse
import rospy
from turtleAPI import robot

from route import dijkstras
from adjMatrix import AdjMatrix
from pid import PID
from move import GoTo
from filter import Filter
from astar import aStar
from world import World, parse_json
from PriQue import *

def to_tuple(string):
    res = []
    for token in string.split(", "):
        num = float(token.replace("(", "").replace(")", "").replace('"', ""))
        res.append(num)
    return tuple(res)


if __name__ == "__main__":
    print("code running")
    parser = argparse.ArgumentParser()

    parser.add_argument("dotfile")
    parser.add_argument("startJSON")
    parser.add_argument("goalJSON", help='Final locations of balloons')
    # parser.add_argument("--start", help='starting point of robot: "(x, y)"')

    args = parser.parse_args()

    # if args.start:
    #     start_loc = to_tuple(args.start)

    #     end_loc = to_tuple(args.goal)

    #     # adjacency matrix
    #     adj = AdjMatrix(args.dotfile, start_loc, end_loc)

    #     # run dikj
    #     route, points = dijkstras(adj, ("start", start_loc), ("finish", end_loc))

    #     print(route, points)

    #     exit()

    try:
        #1. create a robot
        print("creating robot")
        r = robot()
        found = False
        tup = r.getMCLPose()

        #get robot's location
        start_loc = (tup[0], tup[1])

        #read in intial world
        w = World(args.startJSON, start_loc, args.goalJSON)

        # get astar actions list
        action_plan = aStar(w)
        print(action_plan)

        # set PIDS
        distPID = PID(kp = .15, output_limits=(-.4, .4))
        angPID = PID(kp = .25, output_limits=(-.5, .5))

        # loop and run dikj on drive commands or print either PICKUP or PUTDOWN and balloon
        for action, value in action_plan:
            if action == 'drive':
                print("drive to ", value)
                # run dikj
                tup = r.getMCLPose()
                s = (tup[0], tup[1])
                #adjacency matrix, load the map for navigation
                adj = AdjMatrix(args.dotfile, s, value)
                route, points = dijkstras(adj, ("start", s), ("finish", value))
                for p in points:
                    print(p)
                    GoTo(r, p, distPID, angPID, action_plan) # adjust for color
                r.drive(angSpeed=0, linSpeed=0)
                print("Completed dijk")
            else:
                print(action, value)
                time.sleep(2)

    except Exception as e:
        print(e)
        rospy.loginto("node now terminated")
