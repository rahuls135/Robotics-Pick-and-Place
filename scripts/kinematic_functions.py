#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm


def Get_M():
    # Initialize return values of M (and S- s is unnecessary as per TA's announcement)
    M = np.eye(4)
    # S = np.zeros((6,6))
    ##### Your Code Starts Here #####
    # Fill in scripts from lab3 here
    T = np.matmul(T, DHtoA(0.150*np.sqrt(2), 0, 0, np.radians(135)))
    T = np.matmul(T, DHtoA(0, np.radians(-90), 0.162, 0-np.radians(135)))
    T = np.matmul(T, DHtoA(0.244, 0, 0.027, 0))
    T = np.matmul(T, DHtoA(0.213, 0, 0, 0))
    T = np.matmul(T, DHtoA(0, np.radians(-90), 0.083, 0-np.radians(90)))
    T = np.matmul(T, DHtoA(0, np.radians(90), 0.083, 0))
    T = np.matmul(T, DHtoA(0.0535, 0, 0.141, 0)) #end effector
    ##### Your Code Ends Here #####
    return M

def DHtoA(a, alpha, d, theta):
# =================== Your code starts here ====================#
# Write a script for returning the transformation matrix for the link from the DH parameters
    A = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha),
    np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
    [np.sin(theta), np.cos(theta)*np.cos(alpha), -
    np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
    [0, np.sin(alpha), np.cos(alpha), d],
    [0,0,0,1]])
    # ==============================================================#
    return A

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    # Initialize the return_value
    return_value = [None, None, None, None, None, None]
    print("Foward kinematics calculated:\n")
    ##### Your Code Starts Here #####
    # Fill in scripts from lab3 here
    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    T = np.eye(4)
    T = np.matmul(T, DHtoA(0.150*np.sqrt(2), 0, 0, np.radians(135)))
    T = np.matmul(T, DHtoA(0, np.radians(-90), 0.162, theta1-np.radians(135)))
    T = np.matmul(T, DHtoA(0.244, 0, 0.027, theta2))
    T = np.matmul(T, DHtoA(0.213, 0, 0, theta3))
    T = np.matmul(T, DHtoA(0, np.radians(-90), 0.083, theta4-np.radians(90)))
    T = np.matmul(T, DHtoA(0, np.radians(90), 0.083, theta5))
    T = np.matmul(T, DHtoA(0.0535, 0, 0.141, theta6)) #end effector
    ##### Your Code Ends Here #####
    print(str(T) + "\n")
    return_value[0] = theta1 + np.pi
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*np.pi)
    return_value[4] = theta5
    return_value[5] = theta6
    if T[2, 3] < 0:
        print("Calculated Z coordinate is less than 0. The robot joint angles will be set as 0.")
    return_value = np.array([0, 0, 0, 0, 0, 0])
    return return_value

def inverse_kinematics(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    return_value = np.array([0, 0, 0, 0, 0, 0])
    ##### Your Code Starts Here #####
    # Fill in scripts from lab4 here
    # Step 1: find gripper position relative to the base of UR3,
    # and set theta_5 equal to -pi/2
    xgrip = xWgrip + 0.15
    ygrip = yWgrip - 0.15
    zgrip = zWgrip - 0.01
    theta5 = -np.pi/2
    # Step 2: find x_cen, y_cen, z_cen
    x_cen = xgrip - 0.0535*np.cos(np.radians(yaw_WgripDegree))
    y_cen = ygrip - 0.0535*np.sin(np.radians(yaw_WgripDegree))
    z_cen = zgrip
    # Step 3: find theta_1
    phi = np.arctan(y_cen/x_cen)
    psi = np.arcsin(0.110 / np.sqrt(x_cen**2 + y_cen**2))
    theta1 = phi - psi
    # Step 4: find theta_6
    theta6 = (np.pi/2) - np.radians(yaw_WgripDegree) + theta1
    # Step 5: find x3_end, y3_end, z3_end
    p1_0 = np.array([[x_cen], [y_cen]])
    theta = np.radians(90) - theta1
    r1_0 = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta),
    np.cos(theta)]])
    p2_1 = np.array([[0.11], [-0.083]])
    p2_0 = p1_0 + np.dot(r1_0, p2_1)
    x3_end = p2_0[0][0]
    y3_end = p2_0[1][0]
    z3_end = z_cen + 0.052 + 0.083
    # Step 6: find theta_2, theta_3, theta_4
    dist = np.sqrt(x3_end**2 + y3_end**2 + (z3_end-0.152)**2)
    theta3 = np.radians(180) - np.arccos(((dist**2) - (0.244**2) - (0.213**2)) /
    (-2*0.244*0.213))
    phi = np.arccos((np.sqrt(x3_end**2 + y3_end**2)) / (dist))
    psi = np.arccos((dist**2 + (0.244**2) - (0.213**2)) / (2*0.244*dist))
    theta2 = -(phi + psi)
    theta4 = -(theta2+theta3)
    ##### Your Code Ends Here #####
    # print theta values (in degree) calculated from inverse kinematics
    print("Joint angles: ")
    print(str(theta1*180/np.pi) + " " + str(theta2*180/np.pi) + " " + \
    str(theta3*180/np.pi) + " " + str(theta4*180/np.pi) + " " + \
    str(theta5*180/np.pi) + " " + str(theta6*180/np.pi))
    # obtain return_value from forward kinematics function
    return_value = lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
    return return_value