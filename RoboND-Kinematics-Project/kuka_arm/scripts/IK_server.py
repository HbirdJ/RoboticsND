#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
from numpy import mean

import error_graphing

#degree to radian conversions
dtr = pi/180.
rtd = 180./pi

#Standard Rotation Matrices
def rot_x(q):
    R_x = Matrix([[1,       0,       0],
                  [0,  cos(q), -sin(q)],
                  [0,  sin(q),  cos(q)]])
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[cos(q), 0,  sin(q)],
                  [0,      1,       0],
                  [-sin(q), 0, cos(q)]])
    return R_y

def rot_z(q):    
    R_z = Matrix([[cos(q),  -sin(q), 0],
                  [sin(q),   cos(q), 0],
                  [     0,        0, 1]])
    return R_z

#Build rotation matrix for roll, pitch, yaw
roll = symbols('roll')
pitch = symbols('pitch')
yaw = symbols('yaw')

Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)

#Return Euler angles from rotation matrix
def eulerangles(rot_mat):
    beta = atan2(-rot_mat[2,0], sqrt(rot_mat[0,0]**2+rot_mat[1,0]**2)) #about Y-axis
    gamma = atan2(rot_mat[2,1], rot_mat[2,2]) #about X-axis
    alpha = atan2(rot_mat[1,0], rot_mat[0,0]) #about Z-axis
    return alpha, beta, gamma

#function to take D-H parameters and return individual transform matrix
def Tdh(alpha, a, d, q):
    DHtransform = Matrix([
        [            cos(q),           -sin(q),           0,             a],
        [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
        [                 0,                 0,           0,             1]])
    return DHtransform

#function to build simplified total transform matrix from a given DH table
def transformbuild(DHparam):
    totaltransform = eye(4)

    print "Building transforms from given DH parameter table"

    for x in range(0,DHparam.rows):
        transform = Tdh(DHparam[x,0], DHparam[x,1], DHparam[x,2], DHparam[x,3])
        totaltransform = totaltransform*transform
        print "Tranform matrix from frame %s to %s complete" %(x,x+1)
    
    print "Total DH tranform complete. Simplifying"
    totaltransform = simplify(totaltransform)
    print "Simplify and build complete."
    
    return totaltransform

## To save on calculation time,
## transformation matrices are built and simplified at start

# Define joint angle and DH param symbols
q1, q2, q3, q4, q5, q6 = symbols('q1:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Create Modified DH parameter matrix. Cols: (alpha, a, d, theta)
DHparam = Matrix([
             [    0,  0, d1, q1],
             [-pi/2, a1,  0, q2-pi/2],
             [    0, a2,  0, q3],
             [-pi/2, a3, d4, q4],
             [ pi/2,  0,  0, q5],
             [-pi/2,  0,  0, q6],
             [    0,  0, d7,  0]])

# Full DH table included here for comparison
s = { alpha0:     0, a0:     0, d1:  .75,
      alpha1: -pi/2, a1:   .35, d2:    0,
      alpha2:     0, a2:  1.25, d3:    0,
      alpha3: -pi/2, a3: -.054, d4:  1.5,
      alpha4:  pi/2, a4:     0, d1:    0,
      alpha5: -pi/2, a5:     0, d1:    0,
      alpha6:     0, a6:     0, d7: .303}


###Commented out to save calculation time
##Define Modified DH Transformation matrix
# T0_G = transformbuild(DHparam)
# T0_3 = transformbuild(DHparam[:3,:])

##Modify transform for URDF space
# Tdh_urdf = diag(rot_z(pi) * rot_y(-pi/2), 1)
# print "Applying and simplifying transform to URDF space"
# T0_URDF = simplify(T0_G * Tdh_urdf)

# T0_3urdf = T0_3 * Tdh_urdf
# R0_3 = simplify(T0_3urdf[:3,:3])

#Simplify takes too long on my machine
T0_URDF = Matrix([
[-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -d7*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) - cos(q1)*cos(q5)*cos(q2 + q3)) + (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*cos(q1)],
[-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -d7*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) - sin(q1)*cos(q5)*cos(q2 + q3)) + (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*sin(q1)],
[                                    -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + sin(q4)*cos(q6)*cos(q2 + q3),                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3) - d7*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5))],
[                                                                                       0,                                                                                                                                                            0,                                                                                                                                                            0,                                                                                                                                                             1]])

R0_3 = Matrix([
[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
[sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
[        cos(q2 + q3),        -sin(q2 + q3),        0]])

R3_6rhs = R0_3**-1 * Rrpy

print "URDF transform complete"

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

    #Initialize object to store and plot data from path
    path = error_graphing.kuka_path()
    path.poses = len(req.poses)

    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        error = []

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            print "Calculating pose %s of %s" %(x,len(req.poses))
            
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            Px = req.poses[x].position.x
            Py = req.poses[x].position.y
            Pz = req.poses[x].position.z    

            path.Px.append(Px)
            path.Py.append(Py)
            path.Pz.append(Pz)


            PointXYZ = Matrix([Px,Py,Pz])

            (rol, pit, ya) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate Wrist center and build transform matrix
            
            R_rpy = Rrpy.evalf(subs={roll: rol, pitch: pit, yaw: ya})

            IK_P = R_rpy.row_join(PointXYZ).col_join(Matrix([[0,0,0,1]]))

            WC = Matrix(IK_P[:,3]-(.303)*IK_P[:,0])
     
            ## Calculate joint angles using Geometric IK method

            # Variables for IK trig
            link1z = .75
            link1x = .35
            link2 = 1.25
            link3DH = sqrt(1.5**2 + .054**2)
            link3_4 = sqrt(.96**2 + .054**2)

            #theta1 IK
            theta1 = atan2(WC[1], WC[0])

            #theta3 DH IK
            zed = WC[2] - link1z
            r = sqrt(WC[0]**2 + WC[1]**2) - link1x
            el = sqrt(zed**2 + r**2)
            loc = -(link2**2 + link3DH**2 - el**2)/(2 * link2 * link3DH) #law of cos

            theta3DH = atan2(sqrt(1-loc**2),loc)

            #theta2 DH IK
            theta2 = pi/2 - atan2(zed, r) - atan2(link3DH*sin(theta3DH), link2 + link3DH*cos(theta3DH))

            #adjust theta3DH for proper reference frame
            theta3DH = theta3DH - pi/2 - atan2(.054, 1.5)
            theta3 = theta3DH - pi/2 - atan2(.054, .96)
    
            #Evaluates rotation matrix formed prior to loop.
            R3_6 = R3_6rhs.evalf(subs={q1: theta1, q2: theta2, q3: theta3DH, roll: rol, pitch: pit, yaw: ya})

            #Theta 4, 5, and 6 are calculated based on rotation from 3 to 6
            theta4 = atan2(R3_6[2,0], -R3_6[0,0])
            theta5 = atan2(sqrt(R3_6[1,1]**2 + R3_6[1,2]**2), R3_6[1,0])
            theta6 = atan2(R3_6[1,1], R3_6[1,2])

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3DH, theta4, theta5, theta6]

            path.theta1.append(theta1)
            path.theta2.append(theta2)
            path.theta3DH.append(theta3DH)
            path.theta4.append(theta4)
            path.theta5.append(theta5)
            path.theta6.append(theta6)

            joint_trajectory_list.append(joint_trajectory_point)

            sols = angles = {q1: theta1, q2: theta2, q3: theta3DH, q4: theta4, q5: theta5, q6: theta6, 
                             d1:  .75, a1:   .35, a2:  1.25, a3: -.054, d4:  1.5, d7: .303}

            #Calculate Forward Kinematics to determine error
            FK = T0_URDF.evalf(subs=sols)

            err_px = FK[0,3]-Px
            err_py = FK[1,3]-Py
            err_pz = FK[2,3]-Pz
            err_tot = sqrt(err_pz**2 + err_px**2 + err_py**2) #displacement distance
            error.append(err_tot)
            path.error.append(err_tot)

            print "Calculated (x,y,z, displacement) errors: (%f, %f, %f, %f)" %(err_px, err_py, err_pz, err_tot)

            rospy.loginfo("Sent %s points. Average displacement error: %f" %(len(joint_trajectory_list), mean(err_tot)))

            #generate error plot after final calculation
            if len(joint_trajectory_list) == len(req.poses):
                print "Finished path planning. Writing path to file."
                path.datadump()

        return CalculateIKResponse(joint_trajectory_list)



def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

