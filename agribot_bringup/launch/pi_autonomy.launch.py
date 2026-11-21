"""Autonomous crop-row task on the Raspberry Pi 4.

Starts: hardware.launch.py (micro-ROS agent, Madgwick filter, tick_to_odom,
camera, image throttler, robot description), the EKF, the arm action server,
the ESP32-CAM node and one navigator.

  ros2 launch agribot_bringup pi_autonomy.launch.py esp32_ip:=192.168.1.100
  ros2 launch agribot_bringup pi_autonomy.launch.py nav_type:=green_follower
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    esp32_ip = LaunchConfiguration('esp32_ip')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('agribot_bringup'), 'launch', 'hardware.launch.py'])),
        launch_arguments={'use_lidar': 'false'}.items(),
    )

    # tick_to_odom is already started by hardware.launch.py.
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution(
            [FindPackageShare('agribot_bringup'), 'config', 'ekf.yaml'])],
        remappings=[('/odometry/filtered', '/odom')],
    )

    arm_node = Node(
        package='arm_control', executable='arm_action_controller',
        name='arm_action_controller', output='screen',
        parameters=[{'esp32_ip': esp32_ip}],
    )

    esp32_cam_node = Node(
        package='esp_cam_work', executable='esp32_cam_node', name='esp32_cam',
        parameters=[{'esp32_ip': esp32_ip}],
    )

    crop_row_nav = Node(
        package='crop_row_nav', executable='crop_row_navigator',
        name='crop_row_navigator', output='screen',
        condition=LaunchConfigurationEquals('nav_type', 'crop_row'),
    )
    green_follower = Node(
        package='crop_row_nav', executable='green_follower',
        name='green_follower', output='screen',
        condition=LaunchConfigurationEquals('nav_type', 'green_follower'),
    )

    return LaunchDescription([
        DeclareLaunchArgument('esp32_ip', default_value='192.168.1.100',
                              description='IP address of the ESP32-CAM'),
        DeclareLaunchArgument('nav_type', default_value='crop_row',
                              description='"crop_row" (HSV strip navigator) or "green_follower"'),
        hardware_launch,
        ekf_node,
        arm_node,
        esp32_cam_node,
        crop_row_nav,
        green_follower,
    ])
