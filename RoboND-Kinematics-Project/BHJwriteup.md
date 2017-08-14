## Project: Kinematics Pick & Place
### Bradford Johnson
---

####Summary
This project 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/IK.png
[image3]: ./misc_images/sucess.png
[image4]: ./misc_images/sketch.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Using the DH methods suggested by Udacity, the following sketch was completed and a DH table was constructed:

![alt text][image4]

i | **alpha(i-1) | a(i-1) | d(i) | theta(i)**
--- | --- | --- | --- | ---
**1** | 0 | 0 | .75 | q1
**2** | -pi/2 | .35 | 0 | q2-pi/2
**3** | 0 | 1.25 | 0 | q3
**4** | -pi/2 | -.054 | 1.5 | q4
**5** | pi/2 | 0 | 0 | q5
**6** | -pi/2 | 0 | 0 | q6
**7** | 0 | 0 | .303 | 0




#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Inserting a single row from the DH table into the following matrix composes a single transform matrix.

`DHtransform(i-1 to i) = Matrix([
        [            cos(q),           -sin(q),           0,             a],
        [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
        [                 0,                 0,           0,             1]])`
        
To compose the total transform from base link to end effector, each individual transform matrix is multiplied in the following manner:

Total transform from 0 to Gripper = `[T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7]`

Because the resulting DH space does not match up with the URDF space as seen in the above sketch, the following rotation must be applied:

`Rdh_URDF = rot_z(pi) * rot_y(-pi/2)`

So, the final forward kinematics matrix is as follows:

`T0_URDF = Matrix([
[-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -d7*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) - cos(q1)*cos(q5)*cos(q2 + q3)) + (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*cos(q1)],
[-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -d7*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) - sin(q1)*cos(q5)*cos(q2 + q3)) + (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*sin(q1)],
[                                    -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + sin(q4)*cos(q6)*cos(q2 + q3),                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3) - d7*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5))],
[                                                                                       0,                                                                                                                                                            0,                                                                                                                                                            0,                                                                                                                                                             1]])`

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Recieving the orientation and gripper position allows us to calculate the wrist center postion by applying the gripper to wrist center offset distance along an orthonormal vector determined from the gripper orientation.

Using the wrist center and the law of cosines, the first three joint angles can be calculated as seen in the following sketch:

![alt text][image2]

`theta1 = atan2(WristCenter[y], WristCenter[x])
theta3 = atan2(sqrt(1-epsilon**2), epsilon)
theta2 = pi/2 - atan2(Zp, r) - atan2(link3*sin(theta3), link2 + link3*cos(theta3))`

It is possible to achieve an orientation which produces an epsilon value above 180º, but this case isn't considered here. The robot's geometry works best for the given calculations

The roll, pitch, and yaw recieved are used to calculate Theta 4, 5, and 6. 

`R3_6 = inv(R0_3) * Rrpy

R0_3 = Matrix([
[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
[sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
[        cos(q2 + q3),        -sin(q2 + q3),        0]])`

This creates the following matrix which allows calculation of the remaining theta angles from the result of the right hand side of the equation.

`R3_6 = Matrix([
[-sin(q5)*cos(q4),  sin(q4)*cos(q6) + sin(q6)*cos(q4)*cos(q5), -sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6)],
[         cos(q5),                            sin(q5)*sin(q6),                            sin(q5)*cos(q6)],
[ sin(q4)*sin(q5), -sin(q4)*sin(q6)*cos(q5) + cos(q4)*cos(q6), -sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4)]])

theta4 = atan2(R3_6[2,0], -R3_6[0,0])
theta5 = atan2(sqrt(R3_6[1,1]**2 + R3_6[1,2]**2), R3_6[1,0])
theta6 = atan2(R3_6[1,1], R3_6[1,2])`

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The math described above is the framework for `IK_server.py`. Through multiply iterations and debugging, the attached python code appears to succesfully pick up the and place it in the trash can on every attempt in which Rviz requests a reasonable path.

The code also performs the forward kinematics and checks the error between the FK and IK analysis for the gripper center. On all trials using up to date code, no measurable error was observed up to 10e-8.

A few further steps could improve the results. One would be to include a logic tree to ensure the provided joint angles aren't outside of the robot's bounds. Another would be to add the IK analysis for further orientations.

![alt text][image3]


