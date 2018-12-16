# Project: Pick and Place

## Engineer: Chase Johnson

---
[//]: # (Image References)

[image1]: ./misc_images/serial_manipulator.png
[image2]: ./misc_images/js_to_cs_fk_cs_js_ik.png
[image3]: ./misc_images/fk_homogeneous_transform.png
[image4]: ./misc_images/dh_method.png
[image5]: ./misc_images/6dof_dh.png
[image6]: ./misc_images/.png
[image7]: ./misc_images/.png
[image8]: ./misc_images/.png
[image9]: ./misc_images/.png
[image10]: ./misc_images/.png

[image20]: ./misc_images/fk.png

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

"[Kinematics](https://en.wikipedia.org/wiki/Kinematics)) is a branch of classic mechanics that describes the motion of points, bodies (objects), and systems of bodies (groups of objects) without considering the forces that caused the motion" - wiki.