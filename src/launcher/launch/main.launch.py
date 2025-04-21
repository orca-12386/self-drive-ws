import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import yaml


def generate_launch_description():
    package_share_dir = get_package_share_directory("self_drive_course")
    print("Package Share Directory:", package_share_dir)
    robo_desc_dir = get_package_share_directory("steve_description")
    gazebo_ros_package = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_package, 'launch', 'gazebo.launch.py')
    world_file = os.path.join(package_share_dir, 'world', 'self_drive_course_custom.world')
    robot_file = os.path.join(robo_desc_dir,'launch','robot.launch.py')

    motion_planner_package = get_package_share_directory("planner_server_dwb")
    motion_planner_launch_file = os.path.join(motion_planner_package, "launch", "planner.launch.py")

    config_launcher_path = os.path.join(get_package_share_directory("launcher"), "config", "config.yaml")

    with open(config_launcher_path) as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    SIM = config['sim']
    MOVEMENT = config['movement']

    if SIM:
        depth_sub_topic = "/zed_node/stereocamera/depth/image_raw"
        color_sub_topic = "/zed_node/stereocamera/image_raw"
        camera_info_sub_topic = "/zed_node/stereocamera/camera_info"
    else:
        depth_sub_topic = "/zed/zed_node/depth/depth_registered"
        color_sub_topic = "/zed/zed_node/rgb/image_rect_color"
        camera_info_sub_topic = "/zed/zed_node/rgb/camera_info"

    if SIM:
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
    else:
        # Define arguments for the included launch files
        sensor_hostname = "os-122220002210.local"
        pointcloud_topic = LaunchConfiguration('pointcloud_topic')
        imu_topic = LaunchConfiguration('imu_topic')

        # Declare arguments with default values
        declare_sensor_hostname = DeclareLaunchArgument(
            'sensor_hostname', default_value='os-122220002210.local'
        )
        declare_pointcloud_topic = DeclareLaunchArgument(
            'pointcloud_topic', default_value='/lidar/points'
        )
        declare_imu_topic = DeclareLaunchArgument(
            'imu_topic', default_value='/lidar/imu'
        )

        zed_wrapper_dir = get_package_share_directory('zed_wrapper')

        dlo_dir= get_package_share_directory('direct_lidar_odometry')

        driver_launch = ExecuteProcess(
            cmd=['ros2', 'launch', 'ouster_ros', 'sensor.launch.xml', f'sensor_hostname:={sensor_hostname}'],
            output='screen'
        )

        dlo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(dlo_dir, 'launch', 'dlo.launch.py')),
            launch_arguments={
                'rviz': 'false',
                'pointcloud_topic': pointcloud_topic,
                'imu_topic': imu_topic
            }.items()
        )

        zed_no_tf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')),
            launch_arguments={
                "camera_model": "zed2i"
            }.items()
        )

        # transform_launch = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(transformer_dir, 'launch', 'transform.launch.py'))
        # )

        launch_sensors = [
            declare_sensor_hostname,
            declare_pointcloud_topic,
            declare_imu_topic,
            driver_launch,
            dlo_launch,
            zed_no_tf_launch,
            # transform_launch,
        ]

    launch_motion_control = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(motion_planner_launch_file),
        )
    ]

    launch_lane_masker = [
        Node(
            package='lane_mapper',
            executable='lane_mask_publisher_node',
            name='lane_mask_publisher_node',
            parameters=[{
                'sim': SIM,
                'depth_sub_topic': depth_sub_topic,
                'color_sub_topic': color_sub_topic,
            }]
        )
    ]

    launch_lane_mapper = [
        Node(
            package='lane_mapper',
            executable='lane_mapper_node',
            name='lane_mapper_node',
            parameters=[{
                'depth_sub_topic': depth_sub_topic,
                'camera_info_sub_topic': camera_info_sub_topic,
                'mask_sub_topic': '/mask/white',
                'map_pub_topic': '/map/white'
            }]
        ),
        Node(
            package='lane_mapper',
            executable='lane_mapper_node',
            name='lane_mapper_node',
            parameters=[{
                'depth_sub_topic': depth_sub_topic,
                'camera_info_sub_topic': camera_info_sub_topic,
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

    interpolation = [
        Node(
            package='spline_interp',
            executable='linear_interp',
            name='linear_interp',
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
                'map2_sub_topic': '/map/yellow/local/interp',
                'map_pub_topic': '/map/current',
                'assemble_mode': 1
            }]
        )
    ]

    launch_local_map = [
        Node(
            package='map_localiser',
            executable='map_localiser_node',
            name='map_localiser_node',
            parameters=[{
                'map_sub_topic': '/map/yellow'
            }]
        ),
        Node(
            package='map_localiser',
            executable='map_localiser_node',
            name='map_localiser_node',
            parameters=[{
                'map_sub_topic': '/map/white'
            }]
        )
    ]


    launch_goal_calculators = [
        # Lane Follow
        Node(
            package='goal_calculator',
            executable='lane_follow',
            name='lane_follow',
            parameters=[{
                'config_file_path': os.path.join(get_package_share_directory('goal_calculator'), 'config','config.yaml')
            }]),
        # Lane Change
        Node(
            package='goal_calculator',
            executable='lane_change_yellow',
            name='lane_change_yellow',
            output='screen',
            parameters=[{
                'config_file_path': os.path.join(
                    get_package_share_directory('goal_calculator'), 
                    'config', 
                    'config.yaml'
                )
            }]
        ),
        # Stop
        Node(
            package='goal_calculator',
            executable='stop_server',
            name='stop_server',
        ),
        # Turn
        Node(
            package='goal_calculator',
            executable='right_turn',
            name='right_turn'
        ),
        Node(
            package='goal_calculator',
            executable='left_turn',
            name='left_turn'
        )
    ]


    launch_detector = [
        # Node(
        #     package='detective',
        #     executable='drum_detector_node',
        #     name='drum_detector_node'
        # ),
        Node(
            package='detective',
            executable='pedestrian_detector_node',
            name='pedestrian_detector_node'
        ),
        Node(
            package='detective',
            executable='stop_sign_detector_node',
            name='stop_sign_detector_node'
        ),
        Node(
            package='intersection_detector',
            executable='intersection_detector_node',
            name='intersection_detector_node'
        )
    ]


    launch_behaviour_manager = [
        Node(
            package='behaviour_manager',
            executable='behaviour_manager_node',
            name='behaviour_manager_node'
        )
    ]

    launch_pose_publishers = [
        Node(
            package='robot_pose_publisher',
            executable='robot_pose_publisher_node',
            name='robot_pose_publisher',
            parameters=[{
                'map_sub_topic': '/map/white'
            }]       
        ),
        Node(
            package='robot_pose_publisher',
            executable='robot_pose_publisher_node',
            name='robot_pose_publisher',
            parameters=[{
                'map_sub_topic': '/map'
            }]       
        ),
    ]

    launch_topic_remapper = [
        Node(
            package='topic_remapper',
            executable='motion_control_topic_remapper_node',
            name='motion_control_topic_remapper_node',
            parameters=[{
                'default_sub_topic': '/map/current',
                'pub_topic': '/map/motion_control'
            }]       
        ),
        Node(
            package='topic_remapper',
            executable='odom_topic_remapper_node',
            name='odom_topic_remapper_node',
            parameters=[{
                'default_sub_topic': '/zed/zed_node/odom',
                'pub_topic': '/odom'
            }]       
        ),
    ]

    launch_transforms =  [
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
            package='tf_odom_link_base',
            executable='tf_odom',
            name='tf_odom'
        )
    ]
    
    launch_description = list()

    if SIM:
        launch_description.extend(launch_world_robot)
    else:
        launch_description.extend(launch_sensors)


    if MOVEMENT:
        launch_description.extend(launch_motion_control)

    launch_description.extend(launch_lane_masker)
    launch_description.extend(launch_lane_mapper)
    launch_description.extend(interpolation)
    launch_description.extend(launch_local_map)
    launch_description.extend(launch_map_ensemble)
    
    launch_description.extend(launch_goal_calculators)
    
    launch_description.extend(launch_behaviour_manager)
  
    launch_description.extend(launch_detector)
  
    launch_description.extend(launch_topic_remapper)

    launch_description.extend(launch_pose_publishers)
    launch_description.extend(launch_transforms)


    return LaunchDescription(launch_description)
