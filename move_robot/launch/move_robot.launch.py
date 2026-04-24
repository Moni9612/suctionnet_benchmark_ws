import launch
from launch_ros.actions import Node

def generate_launch_description():
    demo_node = Node(
        package="move_robot",
        executable="move_robot",
        name="move_robot",
        output="screen",
        # Remove robot description and kinematics parameters; assume they are already provided.
        parameters=[],
    )

    return launch.LaunchDescription([demo_node])
