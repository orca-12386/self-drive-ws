from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='filterer',
            executable='lane_filter_node',
            name='lane_filter_node',
            output='screen',
            parameters=[{
                'higher_h': 180,
                'higher_s': 60,
                'higher_v': 255,
                'lower_h': 0,
                'lower_s': 0,
                'lower_v': 200,
                'target_v': 140,
            }],
            remappings=[
                ('rgb_image_topic_sub', '/zed/zed_node/rgb/image_rect_color'),
                ('mask_topic_pub', '/lane_filter/mask'),
            ],
        )
    ])