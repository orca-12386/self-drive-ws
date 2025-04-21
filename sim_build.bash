colcon build --packages-ignore direct_lidar_odometry direct_lidar_inertial_odometry zed_ros2 zed_components zed_wrapper ouster_ros behaviour_manager
. install/setup.bash
colcon build --packages-select behaviour_manager
