# Project: Pick and Place

## Engineer: Chase Johnson

---
[//]: # (Image References)

[image1]: ./misc_images/serial_manipulator.png
[image2]: ./misc_images/js_to_cs_fk_cs_js_ik.png
[image3]: ./misc_images/fk_homogeneous_transform.png
[image4]: ./misc_images/dh_method.png
[image5]: ./misc_images/6dof_dh.png
[image6]: ./misc_images/transform_eq.png
[image7]: ./misc_images/kuka_urdf_dh_diff.png
[image8]: ./misc_images/wc_eqn.png
[image9]: ./misc_images/theta1.png
[image10]: ./misc_images/cosine_laws.png


**Aim:**  The aim of the `Pick & Place` project is to understand kinematics (forward/inverse) and use these studies in conjunction with ROS to program a robtic arm to pick objects from a shelf and place them in a bin.

This project is inspired by the [Amazon Robotics Challenge](https://www.amazonrobotics.com/site/binaries/content/assets/amazonrobotics/arc/2017-amazon-robotics-challenge-rules-v3.pdf)

**Procedure:**  

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 

# Kinematic Analysis

"[Kinematics](https://en.wikipedia.org/wiki/Kinematics)" is a branch of classic mechanics that describes the motion of points, bodies (objects), and systems of bodies (groups of objects) without considering the forces that caused the motion" - wiki.

We learnt important theoretical concepts such as reference frames, generalized co-ordinates, degress of freedom and homogenous transforms.

We were educated on Rigid bodies ("links") which are connected by kinematic pairs ("joints"). These joints are commonly revolute or prismatic which have 1 degree of freedom. Joints can be group (image below) which increases the degree's of freedom which are known as kinematic chains (Serial Manipulator/Robotic Arm).

![alt text][image1]
In Robotics there are two important kinematic equations, 1) Forward Kinematics and 2) Inverse Kinematics.

Foward kinematics (fk) ("Joint Space") calculations the position of the end-effector from the known joint variables/parameters.

Conversly, Inverse kinematics (ik) has the known Cartesian co-ordinates (position and orientation) of the end-effect and the objective is the solve the joint variables.

As inverse kinematics "works backwards" multiple mathematicals solutions are possible which may not be real-world applicable.

![alt text][image2]

## Forward Kinematic Analysis
The  Unified Robot Description Format (URDF) is an XML document that defines the robot model. Reviewing the `kr210.urdf.xacro` the position data (xyz) and orientation (rpy) of each join cant be found.

Joint | Parent | Child | x  | y  | x  | roll  | pitch  | yaw  |
----- | ------ | ----- | -- | -- | -- | -- | -- | -- |
fixed_base_joint | base_footprint | base_link | 0 | 0 | 0 | 0 | 0 | 0
joint_1 | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0
joint_2 | link_1    | link_2 | 0.35 | 0 | 0.42 | 0 | 0 | 0
joint_3 | link_2    | link_3 | 0 | 0 | 0 | 1.25 | 0 | 0
joint_4 | link_3    | link_4 | 0.96 | 0 | -0.054 | 0 | 0 | 0
joint_5 | link_4    | link_5 | 0.54 | 0 | 0 | 0 | 0 | 0
joint_6 | link_5    | link_6 | 0.11 | 0 | 0 | 0 | 0 | 0
gripper_joint       | link_6 | gripper_link | 0.11 | 0 | 0 | 0 | 0 | 0
Total       |  |  | 2.153 | 0 | 1.946 | 0 | 0 | 0

## Denavit-Hartenberg Parameters
Determining the location of the end effector requires the homogeneous tranfrom from the fixed based through all links (link 1, link 2 etc) to the end effector.

![alt text][image3]

Jacques Denavit and Richard Hartenberg simplified the process where their method only requires four parameters to describe the position and orientation of neighboring reference frames.

![alt text][image4]

Where:
* α(alpha)​ = arm twist angle
* a = arm link length
* d​ = arm link offset
* θ​ = arm joint angle

Using the specifications from the Kuka KR210 `urdf` and the Denavit-Hartenberg method we can establish the DH parameters for a 6 Degree of Freedom (DoF) robotic arm as shown below.

![alt text][image5]

Joint | a(i-1) | Notes | d(i) | Notes
---   | ---    | ---   | ---  | ---
1 | a0 = 0  | Base link | d1 = 0.75 | -
2 | a1 = 0.35 | - | d2 = 0 | 0 as X1 and X2 are perpendicular
3 | a2 = 1.25 | - | d3 = 0 | X2 and X3 are coincident
4 | a3 = -0.054 | - | d4 = 1.50 | -
5 | a4 = 0 | O4/O5 are coincident | d5 = 0 | X4/X5 are coincident
6 | a5 = 0 | O5/O6 are coincident | d6 = 0 | X5/X6 are coincident
7 | a6 = 0 | O6/O7 are coincident | d7 = 0.303 |

Links | α(i-1) | a(i-1) | d(i) | θ​(i)
--- | --- | --- | --- | ---
0 | - | 0 | 0 | 0
1 | 0 | 0 | 0.75 | θ​1
2 | -90| 0.35 | 0 | -pi/2 + θ​2
3 | 0 | 1.25 | 0 | θ​3
4 | -90 | -0.054 | 1.50 | θ​4
5 | 90 | 0 | 0 | θ​5
6 | -90 | 0 | 0 | θ​6
7 (EE) | 0 | 0 | 0.303 | 07

The above parameters are combined together using **Homogenous transforms**. This will be achieved using the python template `IK_server.py`

## Homogenous Transform Implementation

### Create the symbols

```python
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
```

### Create Modified DH parameters

```python
# Create Modified DH parameters
print('Assigning DH Paramters')
dh = {  alpha0:     0, d1:  0.75, a0:      0,
        alpha1: -pi/2, d2:     0, a1:   0.35, q2: (q2 - pi/2),
        alpha2:     0, d3:     0, a2:   1.25,
        alpha3: -pi/2, d4:  1.50, a3: -0.054,
        alpha4:  pi/2, d5:     0, a4:      0,
        alpha5: -pi/2, d6:     0, a5:      0,
        alpha6:     0, d7: 0.303, a6:      0, q7: 0
}
```

### Define Modified DH Transformation matrix

```python
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
```

### Create individual joint transformation matrices

```python
T0_1 = transform_matrix(dh, alpha0, a0, d1, theta1)
T1_2 = transform_matrix(dh, alpha1, a1, d2, theta2)
T2_3 = transform_matrix(dh, alpha2, a2, d3, theta3)
T3_4 = transform_matrix(dh, alpha3, a3, d4, theta4)
T4_5 = transform_matrix(dh, alpha4, a4, d5, theta5)
T5_6 = transform_matrix(dh, alpha5, a5, d6, theta6)
T6_EE = transform_matrix(dh, alpha6, a6, d7, theta7)
```

### Composition of Homogenous (link) transformations

With the invdivdual link transforms known an overall transform between the base frame (base link) and the end effecotr (gripper) is composed by individual link transforms.

![alt text][image6]

```python
# Composition of Homogenous (link) transformations
TO_EE = simplify(T0_1 * T1_T2 * T2_T3 * T3_4 * T5_T6 * T6_EE)
```

Substituting for joint angles of zero (θ​ = 0) yields:

```bash
T_total = [[ 1.0,   0,   0, 2.153],
           [   0, 1.0,   0,     0],
           [   0,   0, 1.0, 1.946],
           [   0,   0,   0,     1]]

```

## Gripper Orientation Correction

Before calculating the inverse kinematics, we have to compensate for a different between the kuka `URDF` and `DH` parameters. It can be seen below yellow (`DH`) frame is different to the original green (`URDF`)

![alt text][image7]

To correct for these differences two rotational matrixes will be used. The first composed by a rotation of Z axis of 180° (π) followed by a rotation of the Y axis -90 (-π/2).

```python
R_x = rotate_x(r)       # Roll
R_y = rotate_y(p)       # Pitch
R_z = rotate_z(y)       # Yaw

R_EE = R_z * R_y * R_x

# Compensate for rotation discrepancy between DH parameters and Gripper link in URDF
R_err = R_z.subs(y, rad(180)) * R_y.subs(p, rad(-90))
ROT_EE = R_EE * R_err
```

## Decoupling the Inverse Kinematics Problem

As indicated earlier Inverse kinematics use pose (position/orientation) of the known end effort to calculate the joint angles of the manipulator.

As indicated in the lectures, when a robots last three joints are revolute and their joints intersect at a single point, we have a case of **spherical wrist** with `joint_5` being the **wrist center**

The wrist center (WC) calculation is given by the equation below. What this means is that the wrist center depends on the position (roll/pitch/yaw) of the end-effector (gripper)

![alt text][image8]

Programatically this is becomes:

```python
ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
EE = Matrix([[px], [py], [pz]])
WC = EE - (0.303) * ROT_EE[:,2]
```

where 0.303 is `d7` from the `DH Table`

With the wrist center determined `theta1` can be calculated as follows.

![alt text][image9]

```python
theta1 = atan2(WC[1], WC[0])
```

Calculation of `theta2` and `theta3` leverages [Cosine Laws](https://en.wikipedia.org/wiki/Law_of_cosines)

![alt text][image10]

```python
# SSS triangle for theta2 and theta3
side_a = 1.501  # d4 = sqrt(1.5^2 + -0.054^2) = 1.500971685275908 ~ 1.501
side_b = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))   # d1: 0.75
side_c = 1.25   # a2

# Cosine Laws SSS to find angles of triangle
angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a) / (2*side_b*side_c))
angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b) / (2*side_a*side_c))
angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c) / (2*side_a*side_b))

theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)
theta3 = pi/2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m
```

With `theta1, theta2, theta3` calculated now need to find the values of the final three joint variables.

Using the individual DH transforms we can obtain the resultant transform and hence resultant rotation by:

```python
R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6
```

We can substitute the values we calculated for joints 1 to 3 in their respective individual rotation matrices and pre-multiply both sides of the above equation by **`inv(R0_3)`** which leads to:

```python
R3_6 = inv(R0_3) * Rrpy
```

Note: Inverse or Transpostion can be used as the rotation matrices are orthogonal which means that its transpose is equal to its [inverse](https://en.wikipedia.org/wiki/Orthogonal_matrix)

```python
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})
R3_6 = R0_3.transpose() * ROT_EE

theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```


## Final Result

The project succesfully picked the object from the shelf and placed it in the bin. However, while the movement (trajectory) and action was successful the process was CPU intensive and planning (calculating) to actioning was significantly slow. It will be interesting to explore this topic more to see if the process can be optimised to perform faster (real-time)responses.

