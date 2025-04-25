from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='transforms',
            executable='lidar_tf',
            name='lidar_tf',
            output='screen'
        ),
        Node(
            package='transforms',
            executable='odom_pc',
            name='odom_pc',
            output='screen'
        ),
        Node(
            package='transforms',
            executable='zed_tf',
            name='zed_tf',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'robot/base_link', 'os_sensor'],
            output='screen'
        )
    ])
