import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur3e_mrc.msg import CommandUR3e 
import math

class JointTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('ur3e_joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Subscribe to the /ur3/command topic
        self.subscription = self.create_subscription(
            CommandUR3e,
            '/ur3/command',
            self.command_callback,
            10
        )
        self.get_logger().info('JointTrajectoryPublisher is ready and listening for commands.')

    def command_callback(self, msg):
        # Extract the joint positions from the incoming CommandUR3e message
        joint_positions = msg.destination  # The destination field contains the joint angles.

        # Ensure that we have 6 joint positions for the UR3e robot
        if len(joint_positions) != 6:
            self.get_logger().error('Received command does not have exactly 6 joint positions.')
            return

        joint_positions = [pos for pos in joint_positions]

        # Create and publish the trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 5  # Modify time as needed for smoother motion

        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)
        self.get_logger().info(f'Publishing trajectory: {traj_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
