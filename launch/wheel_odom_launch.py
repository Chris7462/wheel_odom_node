from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    wheel_odom_node = Node(
        package='wheel_odom_node',
        executable='wheel_odom_node',
        name='wheel_odom_node'
    )

    return LaunchDescription([
        wheel_odom_node
    ])
