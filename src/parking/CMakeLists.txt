cmake_minimum_required(VERSION 3.8)
project(parking)

# Ensure C++17 compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



add_executable(pullout src/pullout.cpp)

ament_target_dependencies(pullout
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  geometry_msgs
)

add_executable(pullin src/pullin.cpp)

ament_target_dependencies(pullin
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  geometry_msgs
)


add_executable(parallel src/parallel.cpp)

ament_target_dependencies(parallel
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  geometry_msgs
)

# Install targets
install(TARGETS  pullout pullin parallel
  DESTINATION lib/${PROJECT_NAME})

# Package export
ament_package()
