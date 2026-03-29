#!/usr/bin/env python3
"""
agribot.launch.py  —  Single launch file for the entire AgriBot system.

Starts everything needed to run the bot:
  1. Micro-ROS Agent        (Pico 2 via /dev/ttyACM0)
  2. IMU Madgwick Filter    (/imu/data_raw → /imu/data)
  3. Wheel Tick → Odometry  (/left_ticks + /right_ticks → /wheel/odometry)
  4. EKF                    (/wheel/odometry + /imu/data → /odom)
  5. RPLidar A1
  6. ESP32-CAM stream
  7. Image Throttler
  8. Robot State Publisher  (URDF)
  9. Arm Controller
  10. Navigation node        (crop_row | green_follower — pick one at launch)

Usage:
  ros2 launch agribot_bringup agribot.launch.py
  ros2 launch agribot_bringup agribot.launch.py nav_type:=crop_row
  ros2 launch agribot_bringup agribot.launch.py esp32_ip:=192.168.0.50 lidar_port:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------ #
    #  Launch Arguments                                                    #
    # ------------------------------------------------------------------ #
    esp32_ip_arg = DeclareLaunchArgument(
        'esp32_ip', default_value='192.168.1.100',
        description='IP address of the ESP32-CAM'
    )
    nav_type_arg = DeclareLaunchArgument(
        'nav_type', default_value='crop_row',
        description='Navigator to run: "crop_row" or "green_follower"'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB1',
        description='Serial port for RPLidar A1'
    )
    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baud', default_value='115200',
        description='Baud rate for RPLidar A1'
    )

    # Resolve substitutions to variables for reuse below
    esp32_ip   = LaunchConfiguration('esp32_ip')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')

    # ------------------------------------------------------------------ #
    #  Package paths                                                       #
    # ------------------------------------------------------------------ #
    sllidar_launch = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch', 'sllidar_a1_launch.py'
    )
    description_launch = os.path.join(
        get_package_share_directory('agribot_description'),
        'launch', 'description.launch.py'
    )
    ekf_config = os.path.join(
        get_package_share_directory('agribot_bringup'),
        'config', 'ekf.yaml'
    )

    # ------------------------------------------------------------------ #
    #  1. Micro-ROS Agent  (Pico 2 over USB-Serial)                       #
    # ------------------------------------------------------------------ #
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', '/dev/ttyACM0'
        ],
        output='screen',
        name='micro_ros_agent'
    )

    # ------------------------------------------------------------------ #
    #  2. IMU Madgwick Filter                                              #
    #     /imu/data_raw  →  /imu/data                                     #
    # ------------------------------------------------------------------ #
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag':     False,  # No magnetometer on this robot
            'publish_tf':  False,  # EKF owns the TF tree
            'world_frame': 'enu',
            'reverse_tf':  False,
        }],
        remappings=[
            ('imu/data_raw', 'imu/data_raw'),
            ('imu/data',     'imu/data'),
        ]
    )

    # ------------------------------------------------------------------ #
    #  3. Wheel Tick → Odometry                                           #
    #     /left_ticks + /right_ticks  →  /wheel/odometry                 #
    # ------------------------------------------------------------------ #
    tick_to_odom_node = Node(
        package='agribot_base',
        executable='tick_to_odom',
        name='tick_to_odom',
        output='screen',
    )

    # ------------------------------------------------------------------ #
    #  4. EKF  (robot_localization)                                        #
    #     /wheel/odometry + /imu/data  →  /odometry/filtered → /odom     #
    # ------------------------------------------------------------------ #
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', '/odom')]
    )

    # ------------------------------------------------------------------ #
    #  5. RPLidar A1                                                       #
    # ------------------------------------------------------------------ #
    sllidar_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch),
        launch_arguments={
            'serial_port':     lidar_port,
            'serial_baudrate': lidar_baud,
            'frame_id':        'base_laser',
        }.items()
    )
    # ------------------------------------------------------------------ #
    #  6.1. Pi Camera V1.3                                                 #
    # ------------------------------------------------------------------ #
    pi_camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        parameters=[{
            'width': 640, 
            'height': 480,
            'fps': 5.0  # Limit the camera to 5 FPS natively!
        }],
        output='screen'
    )

    # ------------------------------------------------------------------ #
    #  6.2. ESP32-CAM Stream                                                 #
    # ------------------------------------------------------------------ #
    esp32_cam_node = Node(
        package='esp_cam_work',
        executable='esp32_cam_node',
        name='esp32_cam',
        output='screen',
        parameters=[{'esp32_ip': esp32_ip}]
    )

    # ------------------------------------------------------------------ #
    #  7. Image Throttler                                                  #
    #     Reduces camera topic rate before feeding into the nav pipeline  #
    # ------------------------------------------------------------------ #
    image_throttler_node = Node(
        package='crop_row_nav',
        executable='image_throttler',
        name='image_throttler',
        output='screen',
    )

    # ------------------------------------------------------------------ #
    #  8. Robot State Publisher  (URDF)                                    #
    # ------------------------------------------------------------------ #
    description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch)
    )

    # ------------------------------------------------------------------ #
    #  9. Arm Controller                                                   #
    # ------------------------------------------------------------------ #
    arm_controller_node = Node(
        package='arm_control',
        executable='arm_action_controller',
        name='arm_action_controller',
        output='screen',
    )

    # ------------------------------------------------------------------ #
    #  10. Navigation  (mutually exclusive — pick one at launch time)      #
    # ------------------------------------------------------------------ #
    crop_navigator_node = Node(
        package='crop_row_nav',
        executable='crop_row_navigator',
        name='crop_row_navigator',
        output='screen',
        condition=LaunchConfigurationEquals('nav_type', 'crop_row')
    )

    green_follower_node = Node(
        package='crop_row_nav',
        executable='green_follower',
        name='green_follower',
        output='screen',
        condition=LaunchConfigurationEquals('nav_type', 'green_follower')
    )

    # Log which navigator came up (handler is guarded by same condition
    # so it won't crash when the node is not launched)
    log_crop_nav = RegisterEventHandler(
        OnProcessStart(
            target_action=crop_navigator_node,
            on_start=[LogInfo(msg='[NAV] Crop Row Navigator is online.')]
        ),
        condition=LaunchConfigurationEquals('nav_type', 'crop_row')
    )

    log_green_nav = RegisterEventHandler(
        OnProcessStart(
            target_action=green_follower_node,
            on_start=[LogInfo(msg='[NAV] Green Follower is online.')]
        ),
        condition=LaunchConfigurationEquals('nav_type', 'green_follower')
    )

    # ------------------------------------------------------------------ #
    #  Assemble — args first, then nodes in startup order                  #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        # --- Args -------------------------------------------------------
        esp32_ip_arg,
        nav_type_arg,
        lidar_port_arg,
        lidar_baud_arg,

        LogInfo(msg='--- AgriBot Bringup Starting ---'),

        # --- Hardware ---------------------------------------------------
        micro_ros_agent,            # 1. Micro-ROS (Pico 2)
        imu_filter_node,            # 2. IMU filter
        tick_to_odom_node,          # 3. Wheel odometry
        ekf_node,                   # 4. EKF fusion → /odom
        sllidar_launch_include,     # 5. LiDAR
        pi_camera_node,
        esp32_cam_node,             # 6. ESP32-CAM
        image_throttler_node,       # 7. Image throttler
        description_launch_include, # 8. URDF / RSP
        arm_controller_node,        # 9. Arm

        # --- Navigation (one of two) ------------------------------------
        crop_navigator_node,        # 10a. Model-based crop row nav
        green_follower_node,        # 10b. Simple green follower

        # --- Event handlers ---------------------------------------------
        log_crop_nav,
        log_green_nav,
    ])