from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('ecan_driver'),
        'config',
        'config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='ecan_driver',
            executable='ethernet_node',
            name='ethernet_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='ecan_driver',
            executable='processing_node',
            name='processing_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
