import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo, OpaqueFunction

# Counter for iterations
MAX_ITERATIONS = 10
current_iteration = 0


def iterate_process(context, *args):
    global current_iteration
    if current_iteration >= MAX_ITERATIONS:
        return [LogInfo(msg="Iteration limit reached. Shutting down.")]
    
    current_iteration += 1

    # Launch the first file (grasp_detect)
    suction_grasp_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'suction_grasp', 'suction_grasp_launch.py'],
        output='screen'
    )

    # Launch the second file (move_robot) with an initial logger-based delay
    suction_move_robot_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'suction_move_robot', 'suction_move_robot.launch.py'],
        output='screen',
        on_exit=[
            LogInfo(msg="Shutting down ROS node detected."),
            TimerAction(
                period=1.0,
                actions=[
                    LogInfo(msg=f"Iteration {current_iteration} completed. Restarting..."),
                    OpaqueFunction(function=iterate_process)
                ]
            )
        ]
    )

    # Define a custom delay action with a timer to simulate logger detection
    wait_for_logger_message = TimerAction(
        period=13.0,  # was 10s before. Adjust based on how long "demo.py script executed successfully" typically takes to log
        actions=[
            LogInfo(msg="inference_command.sh script executed successfully."),
            suction_move_robot_launch
        ]
    )

    return [suction_grasp_launch, wait_for_logger_message]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=iterate_process)
    ])

