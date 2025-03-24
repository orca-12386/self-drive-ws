import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from scripts import GazeboRosPaths

def generate_launch_description():
    package_share_dir = get_package_share_directory("self_drive_course")
    print("Package Share Directory:", package_share_dir)
    robo_desc_dir = get_package_share_directory("steve_description")
    gazebo_ros_package = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_package, 'launch', 'gazebo.launch.py')
    world_file = os.path.join(package_share_dir, 'world', 'self_drive_course_lights.world')
    robot_file = os.path.join(robo_desc_dir,'launch','robot.launch.py')

    launch_world_robot = [
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(package_share_dir, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={
                'world': world_file,
                'robot': robot_file,
                'pause': 'false',
                'reset': 'true'
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_file)
        )
    ]

    launch_lane_masker = [
        Node(
            package='lane_mapper',
            executable='lane_mask_publisher_node',
            name='lane_mask_publisher_node'
        )
    ]

    launch_lane_mapper = [
        Node(
            package='lane_mapper',
            executable='lane_mapper_node',
            name='lane_mapper_node',
            parameters=[{
                'mask_sub_topic': '/mask/white',
                'map_pub_topic': '/map/white'
            }]
        ),
        Node(
            package='lane_mapper',
            executable='lane_mapper_node',
            name='lane_mapper_node',
            parameters=[{
                'mask_sub_topic': '/mask/yellow',
                'map_pub_topic': '/map/yellow'
            }]
        ),
        Node(
            package='lane_mapper',
            executable='nearest_lane_mapper_node',
            name='nearest_lane_mapper_node'
        )
    ]

    launch_map_ensemble = [
        Node(
            package='multimap_assembler',
            executable='multimap_assembler_node',
            name='multimap_assembler_node',
            parameters=[{
                'map1_sub_topic': '/map/white/local',
                'map2_sub_topic': '/map/yellow/local',
                'map_pub_topic': '/map',
                'assemble_mode': 1
            }]
        ),
        Node(
            package='multimap_assembler',
            executable='multimap_assembler_node',
            name='multimap_assembler_node',
            parameters=[{
                'map1_sub_topic': '/map/white/local',
                'map2_sub_topic': '/map/white/local/near',
                'map_pub_topic': '/map/white/local/far',
                'assemble_mode': 0
            }]
        ),
        Node(
            package='multimap_assembler',
            executable='multimap_assembler_node',
            name='multimap_assembler_node',
            parameters=[{
                'map1_sub_topic': '/map/white/local/near',
                'map2_sub_topic': '/map/yellow/local',
                'map_pub_topic': '/map/current',
                'assemble_mode': 1
            }]
        )
    ]

    launch_local_map = [
        Node(
            package='local_costmap',
            executable='local_costmap_node',
            name='local_costmap_node',
            parameters=[{
                'map_sub_topic': '/map/yellow'
            }]
        ),
        Node(
            package='local_costmap',
            executable='local_costmap_node',
            name='local_costmap_node',
            parameters=[{
                'map_sub_topic': '/map/white'
            }]
        )
    ]

    launch_transforms =  [
        # Node(
        #     package='height_mapper',
        #     executable='height_mask_publisher_node',
        #     name='height_mask_publisher_node'
        # ),
        # Node(
        #     package='height_mapper',
        #     executable='height_mapper_node',
        #     name='height_mapper_node'
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'link_base']
        ),
        Node(
            package='robot_pose_publisher',
            executable='robot_pose_publisher_node',
            name='robot_pose_publisher'
        ),
        Node(
            package='tf_odom_link_base',
            executable='tf_odom',
            name='tf_odom'
        )
    ]
    
    launch_description = list()

    launch_description.extend(launch_world_robot)
    launch_description.extend(launch_lane_masker)
    launch_description.extend(launch_lane_mapper)
    launch_description.extend(launch_local_map)
    launch_description.extend(launch_map_ensemble)
    launch_description.extend(launch_transforms)

    return LaunchDescription(launch_description)