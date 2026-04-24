from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the static_tf package
    static_tf_package_path = get_package_share_directory('static_tf')
    static_tf_launch_path = os.path.join(static_tf_package_path, 'launch', 'static_tf_launch.py')

    return LaunchDescription([
        # Launch UR robot driver
        # New (adds URScript logging)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ur_robot_driver'), 'launch', 'ur_control.launch.py')
            ),
            launch_arguments={
                'ur_type': 'ur10',
                'robot_ip': '10.0.0.89',
                'launch_rviz': 'true',
            }.items()
        ),


        # Launch UR MoveIt configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ur_moveit_config'), 'launch', 'ur_moveit.launch.py')
            ),
            launch_arguments={
                'ur_type': 'ur10',
                'launch_rviz': 'true'
            }.items()
        ),

        # Launch RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'pointcloud.enable': 'true',
                # 'color.enable': 'true',
                # 'depth.enable': 'true',
                # 'align_depth.enable': 'true'
            }.items()
        ),

        # Launch static TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_tf_launch_path)
        )
    ])