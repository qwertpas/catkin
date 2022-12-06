#!/usr/bin/env python

import sys
import copy
from threading import Thread
import time
import rospy

import numpy as np
from lab6_header import *
from lab6_func import *
from blob_search import *

import numpy as np
from numpy.linalg import norm


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -50*PI/180.0, 90*PI/180.0, -131*PI/180.0, -90*PI/180.0, 135*PI/180.0]
home = [180*PI/180.0, -50*PI/180.0, 90*PI/180.0, -131*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of blocks
blob_dict = {}
mask_dict = {}
image = np.zeros((100, 3, 3))


# 20Hz
SPIN_RATE = 20

# UR3 home location
# home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False

calibration = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################


def move_block(pub_cmd, loop_rate, start_pos, end_pos, theta=0, vel=4, accel=4,):

    def move_xyz(x, y, z, yaw=0):
        print(f"moving to {x}, {y}, {z}")
        Q = lab_invk(x, y, z, yaw_WgripDegree=np.degrees(yaw))
        # print('Q', Q)
        move_arm(pub_cmd, loop_rate, Q, vel, accel)

    print()
    move_xyz(start_pos[0], start_pos[1], start_pos[2] + 0.04, -theta/2)
    move_xyz(start_pos[0], start_pos[1], start_pos[2], -theta/2)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(0.5)

    #check suction
    if(digital_in_0 == 0):
        rospy.loginfo(f"Error did not suck")
        gripper(pub_cmd, loop_rate, suction_off)
        # error
        return 1

    #move up a little
    move_xyz(start_pos[0], start_pos[1], start_pos[2] + 0.04, theta/2)

    #move to above target pos
    move_xyz(end_pos[0], end_pos[1], end_pos[2] + 0.04, theta/2)
    #go down to target
    move_xyz(end_pos[0], end_pos[1], end_pos[2], theta/2)
    #drop
    gripper(pub_cmd, loop_rate, suction_off)
    #move to above target pos
    move_xyz(end_pos[0], end_pos[1], end_pos[2] + 0.04, theta/2)

    return 0 #no error


class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):

        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)



        cv_image = cv2.flip(raw_image, -1)
        # cv_image = raw_image
        # cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        global blob_dict
        global mask_dict
        global image
        blob_dict, mask_dict, image = blob_search(cv_image)


def stack(pub_command, loop_rate, blocks):
    # move_arm(pub_command, loop_rate, home, vel=4, accel=4)
    for i in range(len(blocks)):
        a, b, c = blocks[i]
        ac = c-a
        theta = np.arctan2(ac[1], ac[0])

        start_pos = (b[0], b[1], 0.03)
        end_pos = (0.2, 0.0, 0.03*(i+1))

        move_block(pub_command, loop_rate, start_pos, end_pos, theta=-theta)
    print('done')
    exit()

"""
Program run from here
"""
def main():

    print("aasasa")

    global go_away

    # Initialize ROS node
    rospy.init_node('lab6node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)



    ic = ImageConverter(SPIN_RATE)

    gripper(pub_command, loop_rate, suction_off)

    # time.sleep(1)

    print("start loop")

    moving = False
    while(True):

        # for color in mask_dict:
        #     cv2.imshow(f"Mask View {color}", mask_dict[color])
        cv2.imshow("keypoints", image)
        if cv2.waitKey(1)& 0xFF == ord('q'):
            return
        # time.sleep(0.5)

        # print(blob_dict
        
        
        if not moving:
            time.sleep(2)
            cancel = False
            for color in blob_dict:
                if len(blob_dict[color]) != 2:
                    cancel = True
                    break
            if cancel:
                continue

            blob_dict_save = copy.deepcopy(blob_dict)



            def score(a, b, c):
                ab = b-a
                ac = c-a
                return norm(2*ab - ac) + norm(ab)

            blocks = []
            for a in blob_dict_save['green']:
                minscore = np.inf
                b_best = np.zeros(2)
                c_best = np.zeros(2)
                for b in blob_dict_save['teal']:
                    for c in blob_dict_save['pink']:
                        newscore = score(a,b,c)
                        if newscore < minscore:
                            minscore = newscore
                            b_best = b
                            c_best = c

                blocks.append(np.array([a, b_best, c_best]))

            print('\n', "blocks", blocks, '\n')
            moving = True
            print('start move')
            move_arm(pub_command, loop_rate, home, vel, accel)

            thread = Thread(target = stack, args = (pub_command, loop_rate, blocks))
            thread.start()
        



    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")

    print("Use Ctrl+C to exit program")
    # rospy.spin()


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
