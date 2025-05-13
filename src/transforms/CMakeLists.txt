cmake_minimum_required(VERSION 3.8)
project(transforms)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Executables
# add_executable(lidar_tf src/lidar_tf.cpp)
add_executable(zed_tf src/zed_tf.cpp)
add_executable(odom_pc src/odom_pc.cpp)
add_executable(lidar_tf src/lidar_tf2.cpp)
add_executable(sim_odom_pc src/sim_odom_pc.cpp)
add_executable(odom_tf src/odom_tf.cpp)

# Common dependencies for all targets
foreach(target
    lidar_tf
    zed_tf
    odom_pc
    sim_odom_pc
    odom_tf
)
    ament_target_dependencies(${target}
        rclcpp
        sensor_msgs
        std_msgs
        nav_msgs
        tf2_ros
        geometry_msgs
        pcl_conversions
        pcl_ros
        tf2_sensor_msgs
    )
endforeach()

# Installation
install(TARGETS
    lidar_tf
    zed_tf
    odom_pc
    sim_odom_pc
    odom_tf
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


# Export dependencies
ament_export_dependencies(
    rclcpp
    sensor_msgs
    std_msgs
    nav_msgs
    tf2_ros
    geometry_msgs
    pcl_conversions
    pcl_ros
    tf2_sensor_msgs
)

ament_package()
