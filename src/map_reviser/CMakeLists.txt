cmake_minimum_required(VERSION 3.8)
project(map_reviser)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)


# Create executable
add_executable(yellow_lane_interpolator src/yellow_lane_interpolator.cpp)
ament_target_dependencies(yellow_lane_interpolator rclcpp nav_msgs geometry_msgs example_interfaces)

add_executable(nearest_lane_mapper_node src/nearest_lane_mapper_node.cpp)
ament_target_dependencies(nearest_lane_mapper_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  geometry_msgs
)

# Install
install(TARGETS
  yellow_lane_interpolator
  nearest_lane_mapper_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()