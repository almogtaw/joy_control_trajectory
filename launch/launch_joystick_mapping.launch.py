import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('joy_control_trajectory'),
        'config',
        'joy_velocity.yaml'
    )

    return LaunchDescription([
        Node(
            package='joy_control_trajectory',
            executable='joy_to_multiarray',
            name='joy_to_multiarray',
            parameters=[{'config_file': config}]
        )
    ])
