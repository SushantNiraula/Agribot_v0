from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('agribot_bringup')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    sllidar_launch = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        # 1. Micro-ROS Agent (Pico 2)
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0', '-v6'],
            output='screen'
        ),

        # 2. RPLidar A1
        # LiDAR launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                'serial_port': lidar_port,
                'serial_baudrate': lidar_baud,
                'frame_id': 'base_laser'
            }.items()
        ),

        # 3. Pi Camera V1.3
        Node(
            package='camera_ros',
            executable='camera_node',
            parameters=[{'width': 640, 'height': 480}],
            output='screen'
        ),

        # 4. Robot State Publisher (URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('agribot_description'), 'launch', 'description.launch.py')
            )
        )
    ])