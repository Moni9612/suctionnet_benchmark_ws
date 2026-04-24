import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_control',
            executable='gripper_control',  # This points to your node's entry point
            name='gripper_node',
            output='screen',
            parameters=[],
            remappings=[],
        )
    ])
