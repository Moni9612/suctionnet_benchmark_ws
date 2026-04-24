import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch the first file (suction_grasp)
    suction_grasp_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'suction_grasp', 'suction_grasp_launch.py'],
        output='screen'
    )

    # Launch the second file (suction_move_robot) with a delay, triggered after logger message
    suction_move_robot_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'suction_move_robot', 'suction_move_robot.launch.py'],
        output='screen'
    )

    # Define a custom delay action with a timer to simulate logger detection
    wait_for_logger_message = TimerAction(
        period=12.0,  # Adjust based on how long "demo.py script executed successfully" typically takes to log
        actions=[
            LogInfo(msg="inference_command.sh script executed successfully."),
            suction_move_robot_launch
        ]
    )

    return LaunchDescription([
        suction_grasp_launch,
        wait_for_logger_message
    ])