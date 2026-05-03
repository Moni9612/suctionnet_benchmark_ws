import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='suction_grasp',
            executable='suction_grasp',  # This points to your node's entry point
            name='suction_grasp_node',
            output='screen',
            parameters=[],
            remappings=[],
        )
    ])

