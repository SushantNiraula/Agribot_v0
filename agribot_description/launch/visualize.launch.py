import launch
from launch.substitutions import Command, PathJoinSubstitution,LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    urdf_path = PathJoinSubstitution(
        [FindPackageShare('agribot_description'), 'urdf/robots', 'agribot.urdf.xacro']
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('agribot_description'), 'rviz', 'description.rviz']
    )
    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
        output="screen"
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments= ['-d', rviz_config_path]
    )
    return launch.LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])