#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def transform_matrix(alpha, a, d, q):
    """Build the modified DH transformation matrix based on the provided q, alpha, d and a values.
        :param alpha: Sympy symbol
        :param d: Sympy symbol
        :param a: Sympy symbol
        :param q: Sympy symbol
        :return: Sympy Matrix object of the DH transformation matrix
    """
    T = Matrix([[ 		   cos(q),           -sin(q),           0,             a],
        [ 		sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [       sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
        [                 	 	0,                 0,           0,             1]])
    return T

def rotate_x(r):
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,         cos(r),  -sin(r)],
                  [ 0,         sin(r),  cos(r)]])
    
    return R_x
    
def rotate_y(p):              
    R_y = Matrix([[ cos(p),        0,  sin(p)],
                  [      0,            1,       0],
                  [-sin(p),        0, cos(p)]])
    
    return R_y

def rotate_z(y):    
    R_z = Matrix([[ cos(y),  -sin(y),       0],
                  [ sin(y),   cos(y),       0],
                  [      0,        0,       1]])
    
    return R_z

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Create Modified DH parameters
        dh = {  alpha0:     0, d1:  0.75, a0:      0,
                alpha1: -pi/2, d2:     0, a1:   0.35, q2: (q2 - pi/2),
                alpha2:     0, d3:     0, a2:   1.25,
                alpha3: -pi/2, d4:  1.50, a3: -0.054,
                alpha4:  pi/2, d5:     0, a4:      0,
                alpha5: -pi/2, d6:     0, a5:      0,
                alpha6:     0, d7: 0.303, a6:      0, q7: 0
        }

        # Define Modified DH Transformation matrix
        # See transform_matrix() function
        
        # Create individual transformation matrices
        T0_1 = transform_matrix(alpha0, a0, d1, q1).subs(dh)
        T1_2 = transform_matrix(alpha1, a1, d2, q2).subs(dh)
        T2_3 = transform_matrix(alpha2, a2, d3, q3).subs(dh)
        T3_4 = transform_matrix(alpha3, a3, d4, q4).subs(dh)
        T4_5 = transform_matrix(alpha4, a4, d5, q5).subs(dh)
        T5_6 = transform_matrix(alpha5, a5, d6, q6).subs(dh)
        T6_EE = transform_matrix(alpha6, a6, d7, q7).subs(dh)

        # Composition of Homogenous (link) transformations
        # Transform from Base link to end effector (Gripper)
        T0_2 = (T0_1 * T1_2) 	## Link_0 (Base) to Link_2
        T0_3 = (T0_2 * T2_3) 	## Link_0 (Base) to Link_3
        T0_4 = (T0_3 * T3_4) 	## Link_0 (Base) to Link_4
        T0_5 = (T0_4 * T4_5) 	## Link_0 (Base) to Link_5
        T0_6 = (T0_5 * T5_6) 	## Link_0 (Base) to Link_6
        T0_EE = (T0_6 * T6_EE)	## Link_0 (Base) to End Effector
        
        #
        # Extract rotation matrices from the transformation matrices
        ###
        # Find EE rotation matrix RPY (Roll, Pitch, Yaw)
        r, p, y = symbols('r p y')
        
        R_x = rotate_x(r)       # Roll
        R_y = rotate_y(p)       # Pitch
        R_z = rotate_z(y)       # Yaw

        R_EE = R_z * R_y * R_x
        
        # Compensate for rotation discrepancy between DH parameters and Gripper link in URDF   
        R_err = R_z.subs(y, rad(180)) * R_y.subs(p, rad(-90))
        ROT_EE = R_EE * R_err
        
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            EE = Matrix([[px], [py], [pz]])

            WC = EE - (0.303) * ROT_EE[:,2]
            # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1], WC[0])

            # SSS triangle for theta2 and theta3
            side_a = 1.501  # d4
            side_b = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))   # d1: 0.75
            side_c = 1.25   # a2

            # Cosine Laws SSS to find angles of triangle
            angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a) / (2*side_b*side_c))
            angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b) / (2*side_a*side_c))
            angle_c = acos((side_a*side_a + side_b*side_c - side_c*side_c) / (2*side_b*side_b))

            theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)
            theta3 = pi/2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m


            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})

            R3_6 = R0_3.transpose() * ROT_EE

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
