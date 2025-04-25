from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision",
                executable="pc_mapper_node",
                name="mapper_node",
                output="screen",
                parameters=[
                    {
                        "prior": 0.5,
                        "prob_hit": 0.8,
                        "prob_miss": 0.2,
                        "min_prob": 0.12,
                        "max_prob": 0.99,
                        "obstacle_threshold": 0.55,
                        "resolution": 0.04,
                        "width": 3500,
                        "height": 3500,
                    }
                ],
                remappings=[
                    ("pointcloud_topic_sub", "nav/point_cloud"),
                    ("odom_topic_sub", "dlo/odom_node/odom"),
                    ("mask_topic_sub", "lane_filter/mask"),
                    ("map_modify_sub", "nav/needs_modify"),
                    ("map_modify_pub", "nav/needs_modify"),
                    ("goal_sleep_pub", "nav/goal_sleep"),
                    ("map_pub", "nav/global_map"),
                    ("debug_cloud_pub", "lane_filter/debug_cloud"),
                ],
            )
        ]
    )
