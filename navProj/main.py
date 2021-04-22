import argparse
import rospy,time,tf

from turtleAPI import robot

from route import dijkstras
from adjMatrix import AdjMatrix
from pid import PID

def to_tuple(string):
    res = []
    for token in string.split(", "):
        num = float(token.replace("(", "").replace(")", ""))
        res.append(num)
    
    return tuple(res)

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

        if angleVel < -math.pi/4 or angleVel > math.pi/4:
            if target_y < 0 and curr_y < target_y:
                angleVel = -2*math.pi + angleVel
            elif target_y >= 0 and curr_y > target_y:
                angleVel = 2*math.pi + angleVel 
        
        return angleVel

def GoTo(R, target, dist_pid, ang_pid):
    start = R.getMCLPose()

    start_x = start[0]
    start_y = start[1]

    target_x = target[0]
    target_y = target[1]

    goalDistance = math.sqrt(pow( start_x- target_x, 2) + pow(start_y - target_y, 2))
    distance = goalDistance

    while distance > 0.1:
        RATE.sleep()

        current = R.getMCLPose()
        # self.distError(current, target)

        lspeed = dist_pid(distError(current, target))
        aspeed = ang_pid(angleError(current, target))

        print("dist and angle pid", (distance_pid, angle_pid))

        R.drive(angSpeed=aspeed, linSpeed=lspeed)

        # print("current position", R.getPositionTup())

    print("Reached point")
    R.drive(angSpeed=0, linSpeed=0)



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    
    parser.add_argument("dotfile")
    parser.add_argument("xgoal")
    parser.add_argument("ygoal")
    parser.add_argument("--start", help="starting point of robot: (x, y)")

    args = parser.parse_args()

    if args.start:
        start_loc = args.start

        end_loc = to_tuple(args.goal)

        # adjacency matrix
        adj = AdjMatrix(args.dotfile, start_loc, end_loc)

        # run dikj
        route, points = dijkstras(adj, ("start", start_loc), ("finish", end_loc))

        print(route, points)
        
        exit()

    try:
        print("creating robot")
        r = robot()
        found = False

        tup = r.getMCLPose()
        # get location
        start_loc = (tup[0], tup[1])
        
        # end location
        end_loc = to_tuple(args.goal)

        # adjacency matrix
        adj = AdjMatrix(args.dotfile, start_loc, end_loc)

        # run dikj
        route, points = dijkstras(adj, ("start", start_loc), ("finish", end_loc))

        print(route, points)
        
        distPID = PID(kp = .05, output_limits=(-.3, .3))
        angPID = PID(kp = .1, output_limits=(-.3, .3))

        while not rospy.is_shutdown():
            for p in points:
                GoTo(R, p, distPID, angPID) 
        
        print("Final location found")

    except Exception as e:
        print(e)
        rospy.loginto("node now terminated")


