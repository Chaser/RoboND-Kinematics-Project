from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}

def transform_matrix(s, alpha, a, d, theta):
    """Build the modified DH transformation matrix based on the provided theta, alpha, d and a values.
        :param theta: Sympy symbol
        :param alpha: Sympy symbol
        :param d: Sympy symbol
        :param a: Sympy symbol
        :return: Sympy Matrix object of the DH transformation matrix
    """
    # Create the transformation matrix template
    print('Creating Transformation Matrix')
    T = Matrix([[ 		   cos(theta),           -sin(theta),           0,             a],
        [ 		sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [       sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
        [                 	 	    0,                 	   0,           0,             1]])
    return T

def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    print('Assigning DH Paramters')
    # Create symbols
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')     
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:theta8')
    
    # Create Modified DH parameters
    dh = {  alpha0:     0, d1:  0.75, a0:      0, theta1: 0,
            alpha1: -pi/2, d2:     0, a1:   0.35, theta2: (theta2 - pi/2),
            alpha2:     0, d3:     0, a2:   1.25, theta3: 0,
            alpha3: -pi/2, d4:  1.50, a3: -0.054, theta4: 0,
            alpha4:  pi/2, d5:     0, a4:      0, theta5: 0,
            alpha5: -pi/2, d6:     0, a5:      0, theta6: 0,
            alpha6:     0, d7: 0.303, a6:      0, theta7: 0
    }
    
    # Create individual transformation matrices
    # Note: also substitute in the DH parameters into the matrix 
    T0_1 = transform_matrix(dh, alpha0, a0, d1, theta1).subs(dh)
    T1_2 = transform_matrix(dh, alpha1, a1, d2, theta2).subs(dh)
    T2_3 = transform_matrix(dh, alpha2, a2, d3, theta3).subs(dh)
    T3_4 = transform_matrix(dh, alpha3, a3, d4, theta4).subs(dh)
    T4_5 = transform_matrix(dh, alpha4, a4, d5, theta5).subs(dh)
    T5_6 = transform_matrix(dh, alpha5, a5, d6, theta6).subs(dh)
    T6_EE = transform_matrix(dh, alpha6, a6, d7, theta7).subs(dh)
    
    # Composition of Homogenous (link) transformations
    # Transform from Base link to end effector (Gripper)
    T0_2 = simplify(T0_1 * T1_2) 	## Link_0 (Base) to Link_2
    T0_3 = simplify(T0_2 * T2_3) 	## Link_0 (Base) to Link_3
    T0_4 = simplify(T0_3 * T3_4) 	## Link_0 (Base) to Link_4
    T0_5 = simplify(T0_4 * T4_5) 	## Link_0 (Base) to Link_5
    T0_6 = simplify(T0_5 * T5_6) 	## Link_0 (Base) to Link_6
    T0_EE = simplify(T0_6 * T6_EE)	## Link_0 (Base) to End Effector
    
    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0
    theta5 = 0
    theta6 = 0

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [1,1,1] # <--- Load your calculated WC values in this array
    your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
