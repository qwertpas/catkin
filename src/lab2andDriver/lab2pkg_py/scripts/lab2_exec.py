#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([154.77, -79.81, 98.57, -110.84, -89.84, 1.34])

# Hanoi tower location 1
Q1top = [136.72, -66.36, 102.86, -129.52, -91.46, 8.25]
Q11 = [136.39, -47.60, 103.90, -146.92, -89.45, 0.02]
Q12 = [136.57, -54.22, 103.64, -141, -90.26, 0.05]
Q13 = [136.99, -60.19, 103.75, -137.34, -89.58, 0.12]

Q2top=[156.29, -70.94, 106.43, -123.81, -93.40, 15.77]
Q21 = [154.70, -50.73, 110.84, -150.97, -89.43, 0.08]
Q22 = [154.71, -57.94, 110.78, -143.24, -89.45, 0.09]
Q23 = [154.71, -64.44, 11039, -138.59, -89.43, 0.12]

Q3top= [171.96, -68.71, 104.63, -127.23, -91.40, 16.98]
Q31 = [171.55, -48.08, 105.59, -150.63, -90.10, 0.12]
Q32 = [170.55, -54.45, 105.90, -143.25, -88.50 , 1.32] 
Q33 = [171.84, -60.41, 101.85, -131.99, -92.13, 0.17]


# Q11 = [136.39*pi/180.0, -47.60*pi/180.0, 103.90*pi/180.0, -146.92*pi/180.0, -89.45*pi/180.0, 0.02*pi/180.0]
# Q12 = [136.57*pi/180.0, -54.22*pi/180.0, 103.64*pi/180.0, -141*pi/180.0, -90.26*pi/180.0, 0.05*pi/180.0]
# Q13 = [136.99*pi/180.0, -60.19*pi/180.0, 103.75*pi/180.0, -137.34*pi/180.0, -89.58*pi/180.0, 0.12*pi/180.0]




thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
issuck = 0

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13, Q1top], \
      [Q21, Q22, Q23, Q2top], \
      [Q31, Q32, Q33, Q3top] ]

Q = np.array(Q) * pi/180.0

Q11 = Q[0][0]
Q12 = Q[0][1]
Q13 = Q[0][2]

Q21 = Q[1][0]
Q22 = Q[1][1]
Q23 = Q[1][2]

Q31 = Q[2][0]
Q32 = Q[2][1]
Q33 = Q[2][2]


# for position in Q:
#     for height in position:
#         height = np.array(height) * pi/180.0
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_callback(msg):
    global issuck
    issuck = msg.DIGIN


############### Your Code End Here ###############


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

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


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
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".

    error = 0



    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)



    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 0

    while(not input_done):
        input_string = input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):
            input_done = 1
            loop_count = 1
        elif (int(input_string) == 2):
            input_done = 1
            loop_count = 2
        elif (int(input_string) == 3):
            input_done = 1
            loop_count = 3
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")





    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input


    def grab(pos, height):
        rospy.loginfo(f"Grabbing {pos} {height}")

        move_arm(pub_command, loop_rate, Q[pos-1][3], 4.0, 4.0)
        gripper(pub_command, loop_rate, suction_on)
        move_arm(pub_command, loop_rate, Q[pos-1][height-1], 4.0, 4.0)
        time.sleep(0.3)
        rospy.loginfo(f"did grab? {issuck}")

        if(issuck == 0):
            rospy.loginfo(f"ERRORRRRRRR")
            gripper(pub_command, loop_rate, suction_off)
            sys.exit()


        move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    def place(pos, height):
        rospy.loginfo(f"Placing {pos} {height}")
        move_arm(pub_command, loop_rate, Q[pos-1][3], 4.0, 4.0)
        move_arm(pub_command, loop_rate, Q[pos-1][height-1], 4.0, 4.0)
        gripper(pub_command, loop_rate, suction_off)
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)




    while(loop_count > 0):

        move_arm(pub_command, loop_rate, home, 4.0, 4.0)
        grab(1,3)
        place(3,1)
        grab(1,2)
        place(2,1)
        grab(3,1)
        place(2,2)
        grab(1,1)
        place(3,1)
        grab(2,2)
        place(1,1)
        grab(2,1)
        place(3,2)
        grab(1,1)
        place(3,3)
        loop_count = loop_count - 1

    gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
