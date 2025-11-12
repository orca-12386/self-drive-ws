source_setup () {
    if [ -n "$ZSH_VERSION" ]; then
        . install/setup.zsh
    elif [ -n "$BASH_VERSION" ]; then
        . install/setup.bash
    fi
}
export MAKEFLAGS="-j 2"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select interfaces topic_remapper intersection_detector
source_setup
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore interfaces direct_lidar_odometry direct_lidar_inertial_odometry zed_ros2 zed_components zed_wrapper ouster_ros ouster_ros --executor sequential
source_setup

