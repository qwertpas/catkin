#!/usr/bin/env python3
import numpy as np
from numpy import sin, cos, arctan2, tan, pi, degrees, arccos, radians, arcsin, sqrt
from scipy.linalg import expm
from lab6_header import *

PI = pi


def Get_MS():
    l1 = np.array([-150, 150, 162]) / 1000.
    l2 = np.array([0, 120, 0]) / 1000.
    l3 = np.array([244, 0, 0]) / 1000.
    l4 = np.array([213, -93, 0]) / 1000.
    l5 = np.array([0, 83, 0]) / 1000.
    l6 = np.array([83, 0, 0]) / 1000.
    l7 = np.array([0, 82+59, 53.5])/1000.

    w1 = np.array([0, 0, 1])
    q1 = l1
    v1 = np.cross(-w1, q1)

    w2 = np.array([0, 1, 0])
    q2 = q1 + l2
    v2 = np.cross(-w2, q2)

    w3 = np.array([0, 1, 0])
    q3 = q2 + l3
    v3 = np.cross(-w3, q3)

    w4 = np.array([0, 1, 0])
    q4 = q3 + l4
    v4 = np.cross(-w4, q4)

    w5 = np.array([1, 0, 0])
    q5 = q4 + l5

    v5 = np.cross(-w5, q5)

    w6 = np.array([0, 1, 0])
    q6 = q5 + l6
    v6 = np.cross(-w6, q6)

    W = np.array([w1, w2, w3, w4, w5, w6])
    V = np.array([v1, v2, v3, v4, v5, v6])

    S = np.zeros((6, 4, 4))
    for i in range(6):
        S[i] = np.array([
            [0, -W[i][2], W[i][1], V[i][0]],
            [W[i][2], 0, -W[i][0], V[i][1]],
            [-W[i][1], W[i][0], 0, V[i][2]],
            [0, 0, 0, 0]
        ])

    q7 = q6 + l7

    Mx = q7[0]
    My = q7[1]
    Mz = q7[2]

    M = np.array([
        [0, -1, 0, Mx],
        [0, 0, -1, My],
        [1, 0, 0, Mz],
        [0, 0, 0, 1]
    ])
    return M, S


"""
Function that calculates encoder numbers for each motor
"""


def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    return_value = [None, None, None, None, None, None]

    # print("Foward kinematics calculated:")
    M, S = Get_MS()

    T = expm(S[0]*(theta1)) @ expm(S[1]*theta2) @ expm(S[2]*(theta3)
                                                       ) @ expm(S[3]*theta4) @ expm(S[4]*theta5) @ expm(S[5]*theta6) @ M
    # print(f"T: {str(T)}\n")

    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6
    return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""


def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    
    yaw = np.radians(yaw_WgripDegree)
    # =================== Your code starts here ====================#

    L1 = 152/1000.
    L2 = 120/1000.
    L3 = 244/1000.
    L4 = 93/1000.
    L5 = 213/1000.
    L6 = 83/1000.
    L7 = 83/1000.
    L8 = 82/1000.
    L9 = 53.5/1000.
    L10 = 59/1000.

    # convert gripper in world frame to frame centered at base of main pivot (call it 0)
    x0grip = xWgrip + 150/1000.
    y0grip = yWgrip - 150/1000.
    z0grip = zWgrip - 10/1000.

    # center of rotation for the gripper at the last link
    xcen = x0grip - L9*cos(yaw)
    ycen = y0grip - L9*sin(yaw)
    zcen = z0grip
    theta_cen = arctan2(ycen, xcen)

    # rot_cen = np.array([ #rotation matrix from cen frame to base frame, cen frame has x axis parallel to vector from origin to (xcen, ycen)
    # 	[cos(-theta_cen), -sin(-theta_cen)],
    # 	[sin(-theta_cen), cos(-theta_cen)]
    # ])
    # cen_to_3end = rot_cen @ np.array([-L7, -(L6 + 0.027)]) #vector from xycen to xy3end in base frame

    # xy_cen = np.array(xcen, ycen)
    # xy_3end = xy_cen + cen_to_3end #add vectors to get (x_3end, y_3end)
    # x_3end = xy_3end[0]
    # y_3end = xy_3end[1]
    # theta1 = arctan2(y_3end, x_3end)

    L_cen = np.linalg.norm((xcen, ycen))
    L_little = L2 - L4 + L6
    theta1 = theta_cen - arcsin(L_little / L_cen)
    theta6 = theta1 + pi/2 - yaw
    
    z_3end = zcen + L10 + L8

    L_3end = sqrt(L_cen**2 - L_little**2) - L7
    x_3end = L_3end * cos(theta1)
    y_3end = L_3end * sin(theta1)

    L35 = np.linalg.norm((x_3end, y_3end, z_3end - L1))
    theta3 = pi - arccos((L3**2 + L5**2 - L35**2) / (2*L3*L5))

    theta2toppart = arcsin(L5*sin(pi - theta3) / L35)
    theta2 = -(arctan2(z_3end - L1, L_3end) + theta2toppart)

    theta4 = -(arctan2(L_3end, z_3end - L1) -
               (pi/2 - (theta3 - theta2toppart)))

    theta5 = -pi/2  # fixed so end effector is perpendicular to ground

    # print(f"L35: {L35}")
    # print(f"L_3end: {L_3end}")

    # print(f"theta1: {degrees(theta1)}")
    # print(f"theta2: {degrees(theta2)}")
    # print(f"theta3: {degrees(theta3)}")
    # print(f"theta4: {degrees(theta4)}")
    # print(f"theta5: {degrees(theta5)}")
    # print(f"theta6: {degrees(theta6)}")
    # print(f"x_3end: {x_3end}")
    # print(f"y_3end: {y_3end}")
    # print(f"z_3end: {z_3end}")

    # theta6 = 0.0
    # ==============================================================#
    # add some angle offsets so robot understands
    return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)


# lab_invk(0.27, 0.39, 0.08, yaw_WgripDegree=radians(45))
