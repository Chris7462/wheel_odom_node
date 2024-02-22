from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    wheel_odom_node = Node(
        package='wheel_odom_node',
        executable='wheel_odom_node',
        name='wheel_odom_node'
    )

    trajectory_server_node = Node(
        package='trajectory_server',
        executable='trajectory_server_node',
        name='trajectory_server_node',
        namespace='wheel_odom',
        parameters=[{
            'target_frame_name': 'odom',
            'source_frame_name': 'wheel_link',
            'trajectory_update_rate': 10.0,
            'trajectory_publish_rate': 10.0
        }]
    )

    return LaunchDescription([
        wheel_odom_node,
        trajectory_server_node
    ])
