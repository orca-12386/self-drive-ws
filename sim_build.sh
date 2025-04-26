source_setup () {
    if [ -n "$ZSH_VERSION" ]; then
        . install/setup.zsh
    elif [ -n "$BASH_VERSION" ]; then
        . install/setup.bash
    fi
}
export MAKEFLAGS="-j 2"
colcon build --packages-select interfaces
source_setup
colcon build --packages-ignore interfaces direct_lidar_odometry direct_lidar_inertial_odometry zed_ros2 zed_components zed_wrapper ouster_ros behaviour_manager ouster_ros --executor sequential
source_setup

