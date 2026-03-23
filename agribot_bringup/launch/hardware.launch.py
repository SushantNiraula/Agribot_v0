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
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),

        # 2. IMU Filter (Takes /imu/data_raw, outputs /imu/data)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'reverse_tf': False
            }],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/data', '/imu/data')
            ]
        ),

        # 3. Tick to Odom (Takes /left_ticks & /right_ticks, outputs /odom)
        Node(
            package='agribot_base',
            executable='tick_to_odom',
            name='tick_to_odom',
            output='screen'
        ),

        # 4. RPLidar A1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                'serial_port': lidar_port,
                'serial_baudrate': lidar_baud,
                'frame_id': 'base_laser'
            }.items()
        ),
        # # 5. Pi Camera V1.3 (Updated with remapping)
        # Node(
        #     package='camera_ros',
        #     executable='camera_node',
        #     parameters=[{'width': 640, 'height': 480}],
        #     remappings=[('/cmd_vel', '/cmd_vel_raw')], # Divert camera output
        #     output='screen'
        # ),

        # 5. Pi Camera V1.3 
        Node(
            package='camera_ros',
            executable='camera_node',
            parameters=[{
                'width': 640, 
                'height': 480,
                'fps': 5.0  # Limit the camera to 5 Frames Per Second!
            }],
            # Removed the cmd_vel remapping because the camera doesn't use it
            output='screen'
        ),
        Node(
            package='crop_row_nav',
            executable='image_throttler',
            name='image_throttler',
            # Removed the cmd_vel remapping because the camera doesn't use it
            output='screen'
        ),

        # # 7. Obstacle Avoidance Multiplexer
        # Node(
        #     package='agribot_vision', # Assuming you place the script here
        #     executable='obstacle_avoider',
        #     name='obstacle_avoider',
        #     output='screen'
        # ),

        # 6. Robot State Publisher (URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('agribot_description'), 'launch', 'description.launch.py')
            )
        )
    ])