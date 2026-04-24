import launch
from launch_ros.actions import Node

def generate_launch_description():
    demo_node = Node(
        package="suction_move_robot",
        executable="suction_move_robot",
        name="suction_move_robot",
        output="screen",
        # Remove robot description and kinematics parameters; assume they are already provided.
        parameters=[],
    )

    return launch.LaunchDescription([demo_node])
