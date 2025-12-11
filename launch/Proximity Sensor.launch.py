from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ecan_driver',
            executable='ethernet_node',
            name='ethernet_node',
            output='screen',
            parameters=[
                # Example: override defaults if your node declares them
                # {'can_server_ip': '192.168.0.223', 'can_server_port': 4001}
            ],
        ),
        Node(
            package='ecan_driver',
            executable='processing_node',
            name='processing_node',
            output='screen',
        ),
    ])
