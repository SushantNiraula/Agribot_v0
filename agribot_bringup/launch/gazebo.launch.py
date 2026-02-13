import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument



def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('agribot_description')
    bringup_share = get_package_share_directory('agribot_bringup')


    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf/robots', 'agribot.urdf.xacro')
    ros_gz_bridge_config = os.path.join(bringup_share, 'config', 'ros_gz_bridge_gazebo.yaml')
    
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

   
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

   
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items()
    )

    spawn_robot = TimerAction(
        period=5.0,  
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", "/robot_description",
                "-name", "agribot",
                "-allow_renaming", "false",  # prevents "_1" duplicate
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.32",
                "-Y", "0.0"
            ],
            output='screen'
        )]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )
    spawn_controllers = TimerAction(
    period=8.0,
    actions=[
        ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner",
                 "joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner",
                 "diff_drive_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ],
    )
    cmd_vel_bridge = Node(
        package="cmd_vel_bridge",
        executable="cmd_vel_bridge",
        name="cmd_vel_bridge",
        output="screen",
        parameters=[
            {"in_topic": "/cmd_vel"},
            {"out_topic": "/diff_drive_controller/cmd_vel"},
            {"frame_id": "base_link"},
        ],
    )


    return LaunchDescription([
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        robot_state_publisher,
        spawn_controllers,
        cmd_vel_bridge,
    ])
