import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('joy_control_trajectory'),
        'config',
        'joystick_mapping.yaml'
    )

    return LaunchDescription([
        Node(
            package='joy_control_trajectory',
            executable='joy_velocity_mapper',
            name='joystick_to_trajectory',
            parameters=[{'config_file': config}]
        )
    ])
