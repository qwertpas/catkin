#!/usr/bin/env python
import numpy as np
from numpy import sin, cos, arctan2, tan, pi, degrees, arccos, radians, arcsin
from scipy.linalg import expm
# from lab4_header import *

PI = pi

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))




	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()




	# ==============================================================#

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
	# =================== Your code starts here ====================#

	L1 = 152/1000.
	L2 = 120/1000.
	L3 = 244/1000.
	L4 = 93/1000.
	L5 = 213/1000.
	L6 = 83/1000.
	L8 = 82/1000.
	L9 = 53.5/1000.
	L10 = 59/1000.

	x0grip = xWgrip + 150/1000.
	y0grip = yWgrip - 150/1000.
	z0grip = zWgrip - 10/1000.

	xcen = x0grip - L9*cos(yaw_WgripDegree)
	ycen = y0grip - L9*sin(yaw_WgripDegree)
	zcen = z0grip + L10

	theta_cen = arctan2(ycen, xcen)
	rot_cen = np.array([ #rotation matrix from cen frame to base frame, cen frame has x axis parallel to vector from origin to (xcen, ycen)
		[cos(theta_cen), -sin(theta_cen)],
		[sin(theta_cen), cos(theta_cen)]
	])
	cen_to_3end = rot_cen @ np.array([-83/1000, -(27+83)/1000]) #vector from xycen to xy3end in base frame
	
	xy_cen = np.array(xcen, ycen)
	xy_3end = xy_cen + cen_to_3end #add vectors to get (x_3end, y_3end)
	x_3end = xy_3end[0]
	y_3end = xy_3end[1]

	theta1 = arctan2(y_3end, x_3end)

	L_cen = np.linalg.norm((xcen, ycen))
	L_little = L2 - L4 + L6
	theta1_alt = theta_cen - arcsin(L_little / L_cen)

	theta6 = theta1 + pi/2 - yaw_WgripDegree

	z_3end = zcen + L10 + L8

	print(f"theta1: {degrees(theta1)}")
	print(f"theta1_alt: {degrees(theta1_alt)}")
	print(f"theta6: {degrees(theta6)}")
	print(f"x_3end: {x_3end}")
	print(f"y_3end: {y_3end}")
	print(f"z_3end: {z_3end}")

	L35 = np.linalg.norm((x_3end, y_3end, z_3end - L1))
	theta3 = pi - arccos((L3**2 + L5**2 - L35**2) / (2*L3*L5))

	print(f"L35: {L35}")
	print(f"theta3: {theta3}")


	# theta1 = 0.0
	theta2 = 0.0
	# theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	# theta6 = 0.0
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)


lab_invk(0.27, 0.39, 0.08, yaw_WgripDegree=radians(45))