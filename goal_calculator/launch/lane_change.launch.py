from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

config_yaml = os.path.join(get_package_share_directory('goal_calculator'), 'config','config.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='goal_calculator',
            executable='lane_change',
            name='lane_change',
            output='screen',
            parameters=[{
                'config_file_path': config_yaml
            }]),
    ])