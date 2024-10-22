from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur3e_enme480',  
            executable='ur3e_sim_enme480_ctrl',  
            name='ur3e_joint_trajectory_publisher',
            output='screen',
            parameters=[
                {'param_name': 'param_value'}  
            ]
        ),
        Node(
            package='ur3e_enme480',  
            executable='ur3e_sim_enme480_topics', 
            name='ur3e_end_effector_position_publisher',
            output='screen'
        ),
    ])
