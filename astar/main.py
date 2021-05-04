import rospy, time, argparse

from turtleAPI import robot

from route import dijkstras
from adjMatrix import AdjMatrix
from pid import PID
from move import GoTo
from filter import Filter
from astar import astar
from world import World, parse_json

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
        print("creating robot")
        r = robot()
        found = False

        tup = r.getMCLPose()
        # get location
        start_loc = (tup[0], tup[1])

        # adjacency matrix
        adj = AdjMatrix(args.dotfile, start_loc, end_loc)

        # get astar actions list
        w = World(args.startJSON, start_loc)
        goal_locs, goal_on = parse_json(args.goalJSON)

        action_plan = astar(w, goal_locs)
        print(action_plan)
        # set PIDS
        distPID = PID(kp = .15, output_limits=(-.4, .4))
        angPID = PID(kp = .25, output_limits=(-.5, .5))
        
        # loop and run dikj on drive commands or print either PICKUP or PUTDOWN and balloon
        for action, loc in action_plan:
            if action == 'drive':
                print("drive to ", loc)
                # run dikj
                tup = r.getMCLPose()
                s = (tup[0], tup[1])
                route, points = dijkstras(adj, ("start", s), ("finish", loc))
                for p in points:
                    print(p)
                    GoTo(r, p, distPID, angPID) # adjust for color
                r.drive(angSpeed=0, linSpeed=0)
                print("Made it")
            else:
                print(action)
                time.sleep(2)



        
        

    except Exception as e:
        print(e)
        rospy.loginto("node now terminated")


