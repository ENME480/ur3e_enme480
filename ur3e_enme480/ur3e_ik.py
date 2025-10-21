import rclpy
from rclpy.node import Node
from ur3e_mrc_msgs.msg import PositionUR3e, CommandUR3e
import sys
import math
import numpy as np
from ur3e_enme480.submodules.kinematic_functions import KinematicFunctions
from ur3e_enme480.ur3e_fk import ForwardKinematicsUR3e           # ----> From your last studio 

class InverseKinematicsUR3e(Node):

  def __init__(self): 
    super().__init__('ur3e_fk_publisher')
    self.publisher_ = self.create_publisher(CommandUR3e, '/ur3e/command', 10)

  def inverse_kinematics(self, rob_pos):

    xWgrip, yWgrip, zWgrip, yaw_WgripDegree = rob_pos
    return_value = np.array([0, 0, 0, 0, 0, 0])

    ####################################### Your Code Starts Here #######################################
    
    # TODO: Function that calculates an elbow up inverse kinematics solution for the UR3

    # Step 1: find gripper position relative to the base of UR3,
    # and set theta_5 equal to -pi/2


    # Step 2: find x_cen, y_cen, z_cen


    # Step 3: find theta_1


    # Step 4: find theta_6 


    # Step 5: find x3_end, y3_end, z3_end


    # Step 6: find theta_2, theta_3, theta_4


    ##### Your Code Ends Here #####

    # print theta values (in degree) calculated from inverse kinematics
    
    print("Your Joint Angles: ")
    print(str(theta_1*180/np.pi) + " " + str(theta_2*180/np.pi) + " " + \
            str(theta_3*180/np.pi) + " " + str(theta_4*180/np.pi) + " " + \
            str(theta_5*180/np.pi) + " " + str(theta_6*180/np.pi))

    # obtain return_value from forward kinematics function
    return_value = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

    return return_value 

    
def main(args=None):

    rclpy.init(args=args)

    # Check command-line arguments
    if len(sys.argv) != 5:
        print("Usage: ros2 run <package_name> <script_name> <x> <y> <z> <yawAngle>")
        return

    try:
        # Read positions from command-line arguments
        robot_positions = [float(arg) for arg in sys.argv[1:5]]
    except ValueError:
        print("All positions must be numbers.")
        return

    node = InverseKinematicsUR3e()

    try:
        joint_positions = node.inverse_kinematics(robot_positions)
        print(robot_positions)
        print(joint_positions)
        xc, yc, zc, yawc = robot_positions
        correct_joint_positions = KinematicFunctions().correct_inverse_kinematics(xc, yc, zc, yawc)
        ct1, ct2, ct3, ct4, ct5, ct6 = correct_joint_positions
        for i in joint_positions:
            if abs(i) > 3.15:
                print("Check angles, convert to radians")
                break
        ForwardKinematicsUR3e().send_command(joint_positions)  # Send the command with updated joint positions
        # rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
    	node.destroy_node()
    	rclpy.shutdown()

if __name__ == '__main__':
  main()