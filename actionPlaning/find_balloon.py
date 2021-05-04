from filter import Filter
import rospy
import cv2

def find_balloon(Robot, col):
    # start robot and ask for color
    # print("creating robot")
    r = Robot
    found = False
    color = col

    #create pid controller
    limits = -.3, .3
    pid = PID(kp=.0006, ki=.00003, kd=.006, output_limits=limits)
    
    while not rospy.is_shutdown():
        # get image and depth info
        img = r.getImage()
        depth=r.getDepth()
        # create filter object
        flter = Filter(img, color, depth)

        # get filter info 
        f_img = flter.get_filtered
        loc_val, num_detected = flter.get_PID_value
        mean_distance = flter.mean_distance
        print("distance", mean_distance)

        # show image 
        cv2.imshow("Filtered Image", f_img)
        cv2.waitKey(1)

        # make sure there is enough of the color there
        if num_detected > THRESHOLD:
            pid_angSpeed = pid(loc_val)
            print(pid.components)
            found = True
        else:
            found = False

        # check if arrived at balloon
        if mean_distance < MAX_DISTANCE:
            r.drive(angSpeed=0, linSpeed=0)
            print("arrived")
            rospy.signal_shutdown("made it to balloon")
        elif not found: # Balloon not in view
            r.drive(angSpeed=ANGLE_SPEED, linSpeed=0)
        else: # Correct Balloon in image 
            r.drive(angSpeed=pid_angSpeed, linSpeed=LINEAR_SPEED)
        time.sleep(0.1)
