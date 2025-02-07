import os
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform publisher 1
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg="Launching static_transform_publisher for base_link to laser"),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_transform_publisher_1',
                    arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser']
                )
            ]
        ),
        # Static transform publisher 2
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg="Launching static_transform_publisher for odom to base_link"),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_transform_publisher_2',
                    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
                )
            ]
        ),
        # RPLidar node
        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg="Launching RPLidar node"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                get_package_share_directory('rplidar_ros'),
                                'launch',
                                'rplidar_a1_launch.py'
                                         )
                        ]
                    )
                )
            ]
        ),
        # Laser odometry node
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="Launching laser odometry node"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                get_package_share_directory('rf2o_laser_odometry'),
                                'launch',
                                'rf2o_laser_odometry.launch.py'
                                         )
                        ]
                    )
                )
            ]
        ),
        # SLAM Toolbox node
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg="Launching SLAM Toolbox node"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                get_package_share_directory('slam_toolbox'),
                                'launch',
                                'online_async_launch.py'
                                         )
                        ]
                    )
                )
            ]
        )
    ])