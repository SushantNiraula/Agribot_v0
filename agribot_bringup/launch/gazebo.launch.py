from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join
import os
import xacro

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('agribot_description')
    bringup_share = get_package_share_directory('agribot_bringup')

    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf/robots', 'agribot.urdf.xacro')
    ros_gz_bridge_config = os.path.join(bringup_share, 'config', 'ros_gz_bridge_gazebo.yaml')

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    use_sim_time = {"use_sim_time": True}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, use_sim_time],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}, use_sim_time],
        output='screen'
    )

    # 1) Spawn robot (process exits when done)
    spawn_robot = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-topic", "/robot_description",
            "-name", "agribot",
            "-allow_renaming", "false",
            "-x", "0.0", "-y", "0.0", "-z", "0.32", "-Y", "0.0",
        ],
        output="screen"
    )

    # 2) Spawn controllers only AFTER spawn_robot finishes
    spawn_joint_state = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner",
             "joint_state_broadcaster", "--controller-manager", "/controller_manager","--wait"],
        output="screen",
    )

    spawn_diff_drive = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner",
             "diff_drive_controller", "--controller-manager", "/controller_manager",],
        output="screen",
    )

    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawn_joint_state, spawn_diff_drive],
        )
    )
    cmd_vel_bridge = Node(
        package="cmd_vel_bridge",
        executable="cmd_vel_bridge",
        name="cmd_vel_bridge",
        output="screen",
        parameters=[
            use_sim_time,
            {"in_topic": "/cmd_vel"},
            {"out_topic": "/diff_drive_controller/cmd_vel"},
        ],

    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'ekf.yaml')],
    )


    return LaunchDescription([
        gazebo,
        ros_gz_bridge,
        robot_state_publisher,
        spawn_robot,
        controllers_after_spawn,
        cmd_vel_bridge,
        ekf_node,
    ])
