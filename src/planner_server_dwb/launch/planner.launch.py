from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

nav2_yaml = os.path.join(get_package_share_directory('planner_server_dwb'), 'config','planner_server.yaml')
controller_yaml = os.path.join(get_package_share_directory('planner_server_dwb'), 'config','controller.yaml')
bt_navigator_yaml = os.path.join(get_package_share_directory('planner_server_dwb'), 'config','bt_navigator.yaml')
recovery_yaml = os.path.join(get_package_share_directory('planner_server_dwb'), 'config','recovery.yaml')
smoother_yaml = os.path.join(get_package_share_directory('planner_server_dwb'), 'config','smoother.yaml')
amcl_yaml = os.path.join(get_package_share_directory('planner_server_dwb'), 'config','amcl.yaml')
bt_xml_path = os.path.join(get_package_share_directory('planner_server_dwb'),'launch','behaviour.xml')
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml]),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[smoother_yaml]),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml]),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'global_frame': 'map',
                'robot_base_frame': 'base_link',
                'odom_topic': '/odom',
                'bt_loop_duration': 10,
                'default_server_timeout': 20,
                'default_nav_to_pose_bt_xml': bt_xml_path,
                'plugin_lib_names': [
                    'nav2_compute_path_to_pose_action_bt_node',
                    'nav2_compute_path_through_poses_action_bt_node',
                    'nav2_smooth_path_action_bt_node',
                    'nav2_follow_path_action_bt_node',
                    'nav2_spin_action_bt_node',
                    'nav2_wait_action_bt_node',
                    'nav2_back_up_action_bt_node',
                    'nav2_drive_on_heading_bt_node',
                    'nav2_clear_costmap_service_bt_node',
                    'nav2_is_stuck_condition_bt_node',
                    'nav2_goal_reached_condition_bt_node',
                    'nav2_goal_updated_condition_bt_node',
                    'nav2_globally_updated_goal_condition_bt_node',
                    'nav2_is_path_valid_condition_bt_node',
                    'nav2_initial_pose_received_condition_bt_node',
                    'nav2_reinitialize_global_localization_service_bt_node',
                    'nav2_rate_controller_bt_node',
                    'nav2_distance_controller_bt_node',
                    'nav2_speed_controller_bt_node',
                    'nav2_truncate_path_action_bt_node',
                    'nav2_truncate_path_local_action_bt_node',
                    'nav2_goal_updater_node_bt_node',
                    'nav2_recovery_node_bt_node',
                    'nav2_pipeline_sequence_bt_node',
                    'nav2_round_robin_node_bt_node',
                    'nav2_transform_available_condition_bt_node',
                    'nav2_time_expired_condition_bt_node',
                    'nav2_path_expiring_timer_condition',
                    'nav2_distance_traveled_condition_bt_node',
                    'nav2_single_trigger_bt_node',
                    'nav2_goal_updated_controller_bt_node',
                    'nav2_is_battery_low_condition_bt_node',
                    'nav2_navigate_through_poses_action_bt_node',
                    'nav2_navigate_to_pose_action_bt_node',
                    'nav2_remove_passed_goals_action_bt_node',
                    'nav2_planner_selector_bt_node',
                    'nav2_controller_selector_bt_node',
                    'nav2_goal_checker_selector_bt_node',
                    'nav2_controller_cancel_bt_node',
                    'nav2_path_longer_on_approach_bt_node',
                    'nav2_wait_cancel_bt_node',
                    'nav2_spin_cancel_bt_node',
                    'nav2_back_up_cancel_bt_node',
                    'nav2_drive_on_heading_cancel_bt_node'
                ]
            }]
        ),
        
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[recovery_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['planner_server',
                                        'amcl',
                                        'smoother_server',
                                        'controller_server', 
                                        'bt_navigator',
                                        'recoveries_server'
                                        ]}])
    ])
