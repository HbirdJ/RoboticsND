# 6 Joint Robot Arm Kinematic Analysis

The most recent project in my Udacity nanodegree gave me a 6 jointed robot arm and a desired path for the robot's gripper. My job was to perform the inverse kinematic analysis to calculate the joint positions that will move the gripper along the requested path. After learning the forward kinematics and doing a full geometric analysis for the inverse kinematics, my robot arm was able to follow the paths it received with negligible error.

[image1]: ./misc_images/sketch.png
[grab]: ./misc_images/shelfgrab.jpg
[drop]: ./misc_images/candrop.jpg
[maxerrpoints]: ./plots/maxerrs.png
[thetaerrpoints]: ./plots/theterrs.png
[grabplot]: ./plots/2017-07-14%20184756.png
[dropplot]: ./plots/2017-07-14%20184915.png
[rightmid]: ./plots/2017-07-15%20091729.png

## Denavit–Hartenberg Analysis
A single free body in space requires definition of 6 variables to define its position, and most commonly used are x, y, z, roll, pitch, and yaw. Standard spacial translation and rotation matrices can be used to fully define a system with 6 degrees of freedom, but this would mean each 4x4 transform matrix would involve 6 variables for each  joint location. Because every transform matrix is multiplied together to complete the forward kinematic analysis, the computations required rapidly become complex.

The Denavit–Hartenberg (DH) parameters were developed to cut down the variables required at each joint from 6 to 4, thus lowering the overall computation time. Microprocessors everywhere breathed a sigh of relief. A high level outline of this method is shown below.

###### To use this method I sketched the robot arm and determined axis orientation based on the parameters:
![DH Diagram Sketch][image1]

###### Built a table of parameters based on the sketch and additional geometry information:
i | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
**1** | 0 | 0 | .75 | q1
**2** | -pi/2 | .35 | 0 | q2-pi/2
**3** | 0 | 1.25 | 0 | q3
**4** | -pi/2 | -.054 | 1.5 | q4
**5** | pi/2 | 0 | 0 | q5
**6** | -pi/2 | 0 | 0 | q6
**7** | 0 | 0 | .303 | 0

###### And built some functions to compile the required transformation matrix
``` python
def Tdh(alpha, a, d, q):
    DHtransform = Matrix([
        [            cos(q),           -sin(q),           0,             a],
        [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
        [                 0,                 0,           0,             1]])
    return DHtransform
    
def transformbuild(DHparam):
    totaltransform = eye(4)

    print "Building transforms from given DH parameter table"

    for x in range(0,DHparam.rows):
        transform = Tdh(DHparam[x,0], DHparam[x,1], DHparam[x,2], DHparam[x,3])
        totaltransform = totaltransform*transform
        print "Transform matrix from frame %s to %s complete" %(x,x+1)
    
    print "Total DH tranform complete. Simplifying"
    totaltransform = simplify(totaltransform)
    print "Simplify and build complete."
    
    return totaltransform
```

That allowed me to piece together the forward kinematics so I could calculate where in 3D space the robot's gripper was located and oriented based only on the joint positions.

## Inverse Kinematics
The inverse kinematics is used to calculate the joint positions after receiving a position and orientation of the robot's gripper. A full discussion of this requires its own report, but I can offer the following summary of the steps required:
1. Calculate the x,y,z position of Joint 5
2. Use a hearty helping of the law of cos alongside arctan to calculate the first three joint positions using only the x,y,z found in step 1.
3. Apply knowledge of Euler's angles and rotation matrices to perform a separate analysis to find the last three joint positions.
4. Pray that you didn't flub any rotations or misunderstand the orientations required.
5. Shove some inverse kinematic code into the ROS server and hope it works

Udacity left the students to figure out most of this informationon on their own for this project, which led to a lot of discussion in the community Slack as everyone struggled to debug together. After my own share of failed attempts, it was time to try a full test of the project.

| Grabbing Can | Dropping Can |
| :------------------------------: | :------------------------------: |
| ![DH Diagram Sketch][grab]     |     ![DH Diagram Sketch][drop] |

Success! My arm is picking and grabbing from all locations with ease! Well, with as much ease as the Udacity provided virtual machine could muster on my Macbook Air. Robot Operating System (ROS) only functions on Linux, and Udacity will only offer official support for their provided VM. I've read about some painful experiences other students have gone through to get projects running on a native Ubuntu install, so I'm sticking with the slower VM for the moment.

At this point my project met all the required criteria for submittal, so I turned it in and received my congratulatory code review the next morning. Except...I decided to go a little deeper.

## Error checking
While working on my analysis I was logging the cartesian distance between the path point requested and the path point my code returned. This ensured my forward kinematics and inverse kinematics matched, and it was super helpful for late stage debugging.

After submitting, I decided to refresh my limited experience with matplotlib and started building some plots to visualize the path and how much error I was getting.

| Grabbing Can | Dropping Can |
| :------------------------------: | :------------------------------: |
| ![Grabbing error plot][grabplot]    | ![Dropping error plot][dropplot] |

Let's take a closer look at another path:

![Right mid to trash can][rightmid]

I made these plots for 15 different paths and saw similar error ranges in each. The displacement error observed is hitting exactly where we expect typical floating point error to show up, showing that my math was performing as advertised.

### Trends
I'm not well versed in the details of floating point precision and rounding error, so I wanted to see if any robot orientations were consistently producing high error. I was concerned that there would be some near singularities for certain joint angles that don't play well with arctan.

The following two graphs were compiled to check for patterns of higher error.

![Plot of the max error location for each path][maxerrpoints]
![Plot of the theta values at each maximum error][thetaerrpoints]

No angles or positions seem to jump out as concerning, so it looks like my joint orientations worked out fairly well for this analysis.

## Conclusion
All in all, this shows that my code is running geometry calculations that produce the requested values with a very high degree of accuracy. If the real world application did not require precision to 10^-15, there are a variety of optimization options that would speed up the calculations but potentially increase error. The analysis tools I've built allow quick comparisons between inverse kinematic methods that may use different tools.

A few of the potential paths forward if I were continuing this project:
* Include orientation (roll/pitch/yaw) in the error calculation
* Build a new Inverse Kinematics server that uses more rounding to decrease calculation time.
* Observe the effect of using `chop = True` on SymPy's evalf function.
* Log the computation time for each calculation
* Compare the floating point error between SymPy (used for the matrix multiplication here) and NumPy.
