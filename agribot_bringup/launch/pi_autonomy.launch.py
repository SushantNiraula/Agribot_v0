import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    esp32_ip_arg = DeclareLaunchArgument(
        'esp32_ip', default_value='192.168.1.100', description='IP address of the ESP32-CAM'
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('agribot_bringup'), 'launch', 'hardware.launch.py'])
        ])
    )

    odom_node = Node(package='agribot_base', executable='tick_to_odom', name='tick_to_odom')
    arm_controller_node = Node(package='arm_control', executable='arm_controller', name='arm_controller')
    
    esp32_cam_node = Node(
        package='esp_cam_work', executable='esp32_cam_node', name='esp32_cam',
        parameters=[{'esp32_ip': LaunchConfiguration('esp32_ip')}]
    )

    image_throttler_node = Node(package='crop_row_nav', executable='image_throttler', name='image_throttler')
    
    crop_navigator_node = Node(
        package='crop_row_nav', executable='crop_row_navigator', name='crop_row_navigator', output='screen'
    )

    log_navigator_start = RegisterEventHandler(
        OnProcessStart(target_action=crop_navigator_node, on_start=[LogInfo(msg="🟢 SUCCESS: Crop Row Navigator is online!")])
    )

    return LaunchDescription([
        esp32_ip_arg,
        hardware_launch,
        odom_node,
        arm_controller_node,
        esp32_cam_node,
        image_throttler_node,
        crop_navigator_node,
        log_navigator_start
    ])