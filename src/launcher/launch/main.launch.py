from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('planner_server'),
                'launch',
                'planner.launch.py'
            ])
        ])
    )

    goal_calc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('goal_calculator'),
                'launch',
                'goalcalc.launch.py'
            ])
        ])
    )

    mapper_node = Node(
        package='mapping',
        executable='Mapper',
        name='mapper_node'
    )

    ramp_goals_node = Node(
        package='ramp_goals',
        executable='ramp_goals',
        name='ramp_goals_node'
    )

    robot_pose_publisher_node = Node(
        package='robot_pose_publisher',
        executable='robot_pose_publisher_node',
        name='robot_pose_publisher'
    )

    local_mapper_node = Node(
        package='local_costmap',
        executable='local_costmap_node',
        name='local_costmap_node'
    )

    ld = LaunchDescription()
    
    ld.add_action(mapper_node)
    ld.add_action(local_mapper_node)
    ld.add_action(planner_launch)
    ld.add_action(ramp_goals_node)
    ld.add_action(robot_pose_publisher_node)
    ld.add_action(goal_calc_launch)
    
    return ld