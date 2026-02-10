from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hello = Node(
        package="agribot_sandbox",
        executable="hello_node",
        output="screen",
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen",
        # Default port is 8765. We'll keep it default for now.
    )

    return LaunchDescription([hello, foxglove])