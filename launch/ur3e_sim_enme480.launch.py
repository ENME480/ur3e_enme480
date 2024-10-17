from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur3e_enme480',  # Replace with your package name
            executable='ur3e_sim_enme480_ctrl',  # Replace with your joint trajectory publisher script name (without .py)
            name='ur3e_joint_trajectory_publisher',
            output='screen',
            parameters=[
                {'param_name': 'param_value'}  # Add any parameters here if needed
            ]
        ),
        Node(
            package='ur3e_enme480',  # Replace with your package name
            executable='ur3e_sim_enme480_topics',  # Replace with your position publisher script name (without .py)
            name='ur3e_end_effector_position_publisher',
            output='screen'
        ),
    ])
