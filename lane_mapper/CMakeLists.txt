cmake_minimum_required(VERSION 3.8)
project(lane_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Create the executable
add_executable(lane_mapper_node src/lane_mapper.cpp)
ament_target_dependencies(lane_mapper_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
)

add_executable(lane_mask_publisher_node src/lane_mask_publisher_node.cpp)
ament_target_dependencies(lane_mask_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  cv_bridge
  OpenCV
)

# Install the executable
install(TARGETS
  lane_mapper_node
  lane_mask_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
