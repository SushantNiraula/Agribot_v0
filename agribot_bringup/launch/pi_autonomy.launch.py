import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from launch.conditions import LaunchConfigurationEquals # <-- Added for if/else logic

def generate_launch_description():
    esp32_ip_arg = DeclareLaunchArgument(
        'esp32_ip', default_value='192.168.1.100', description='IP address of the ESP32-CAM'
    )

    # --- NEW: Launch argument to select your navigator ---
    nav_type_arg = DeclareLaunchArgument(
        'nav_type', default_value='green_follower', 
        description='Which navigation to use: "crop_row" or "green_follower"'
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('agribot_bringup'), 'launch', 'hardware.launch.py'])
        ])
    )

    odom_node = Node(package='agribot_base', executable='tick_to_odom', name='tick_to_odom')
    
    ekf_config_path = PathJoinSubstitution([
        FindPackageShare('agribot_base'), 'config', 'ekf.yaml'
    ])
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('/odometry/filtered', '/odom')]
    )

    arm_controller_node = Node(package='arm_control', executable='arm_controller', name='arm_controller')
    
    esp32_cam_node = Node(
        package='esp_cam_work', executable='esp32_cam_node', name='esp32_cam',
        parameters=[{'esp32_ip': LaunchConfiguration('esp32_ip')}]
    )

    image_throttler_node = Node(package='crop_row_nav', executable='image_throttler', name='image_throttler')
    
    # --- The Navigation Nodes (Mutually Exclusive) ---
    crop_navigator_node = Node(
        package='crop_row_nav', executable='crop_row_navigator', name='crop_row_navigator', output='screen',
        condition=LaunchConfigurationEquals('nav_type', 'crop_row')
    )

    green_follower_node = Node(
        package='crop_row_nav', executable='green_follower', name='green_follower', output='screen',
        condition=LaunchConfigurationEquals('nav_type', 'green_follower')
    )

    # Event handlers to let you know which one successfully booted
    log_crop_nav = RegisterEventHandler(
        OnProcessStart(target_action=crop_navigator_node, on_start=[LogInfo(msg="🟢 SUCCESS: Model Crop Row Navigator is online!")])
    )
    
    log_green_nav = RegisterEventHandler(
        OnProcessStart(target_action=green_follower_node, on_start=[LogInfo(msg="🟢 SUCCESS: Basic Green Follower is online!")])
    )

    return LaunchDescription([
        esp32_ip_arg,
        nav_type_arg, # <-- Added the argument
        hardware_launch,
        odom_node,
        ekf_node,               
        arm_controller_node,
        esp32_cam_node,
        image_throttler_node,
        # crop_navigator_node,   # Only runs if nav_type=crop_row
        # green_follower_node,   # Only runs if nav_type=green_follower
        log_crop_nav,
        log_green_nav
    ])