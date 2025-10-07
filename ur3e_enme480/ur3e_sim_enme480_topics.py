import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from ur3e_mrc_msgs.msg import PositionUR3e  
import tf_transformations  # Make sure you have this package installed

class EndEffectorPositionPublisher(Node):

    def __init__(self):
        super().__init__('ur3e_end_effector_position_publisher')

        # Publisher for the /ur3/position topic
        self.publisher_ = self.create_publisher(PositionUR3e, '/ur3/position', 10)

        # Create a buffer and a TransformListener to listen for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define the frame names
        self.world_frame = 'base_link'  # Replace with your world frame name if different
        self.end_effector_frame = 'vac_gripper'  # Replace with your end-effector frame name if different

        # Set a timer to periodically publish the position
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('EndEffectorPositionPublisher is ready.')

    def timer_callback(self):
        try:
            # Lookup the transform from the world frame to the end-effector frame
            transform = self.tf_buffer.lookup_transform(self.world_frame, self.end_effector_frame, rclpy.time.Time())

            # Extract the position from the transform
            position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]

            # Extract the orientation (quaternion) from the transform
            quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            euler_angles = tf_transformations.euler_from_quaternion(quaternion)

            # Combine position and Euler angles into a single array
            position_with_orientation = position + list(euler_angles)

            # Create and publish the PositionUR3e message
            msg = PositionUR3e()
            msg.position = position_with_orientation  # Position now includes orientation as Euler angles
            msg.is_ready = True  # Set this based on any condition if needed

            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing end-effector position with orientation (Euler)....')

        except Exception as e:
            # If the transform is not available, set is_ready to False
            msg = PositionUR3e()
            msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Default position and orientation
            msg.is_ready = False

            self.publisher_.publish(msg)
            self.get_logger().warn(f'Could not retrieve transform: {e}')
            self.get_logger().info(f'Published default position with is_ready=False')


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPositionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
