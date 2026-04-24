import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch the first file (grasp_detect)
    grasp_detect_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'grasp_detect', 'grasp_detect_launch.py'],
        output='screen'
    )

    # Launch the second file (move_robot) with a delay, triggered after logger message
    move_robot_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'move_robot', 'move_robot.launch.py'],
        output='screen'
    )

    # Define a custom delay action with a timer to simulate logger detection
    wait_for_logger_message = TimerAction(
        period=12.0,  # Adjust based on how long "demo.py script executed successfully" typically takes to log
        actions=[
            LogInfo(msg="Logger detected. Launching move_robot..."),
            move_robot_launch
        ]
    )

    return LaunchDescription([
        grasp_detect_launch,
        wait_for_logger_message
    ])