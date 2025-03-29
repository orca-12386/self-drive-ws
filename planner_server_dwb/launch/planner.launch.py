from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

nav2_yaml = os.path.join(get_package_share_directory('planner_server_dwb'), 'config','planner_server.yaml')
controller_yaml = os.path.join(get_package_share_directory('planner_server_dwb'), 'config','controller.yaml')
bt_navigator_yaml = os.path.join(get_package_share_directory('planner_server_dwb'), 'config','bt_navigator.yaml')
recovery_yaml = os.path.join(get_package_share_directory('planner_server_dwb'), 'config','recovery.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml]),
        
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
            parameters=[bt_navigator_yaml]),
        
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
                                        'controller_server', 
                                        'bt_navigator',
                                        'recoveries_server'
                                        ]}])
    ])