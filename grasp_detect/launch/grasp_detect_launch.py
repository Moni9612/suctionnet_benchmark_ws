import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='grasp_detect',
            executable='grasp_detect',  # This points to your node's entry point
            name='grasp_detect_node',
            output='screen',
            parameters=[],
            remappings=[],
        )
    ])

