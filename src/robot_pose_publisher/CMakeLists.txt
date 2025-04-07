cmake_minimum_required(VERSION 3.8)
project(robot_pose_publisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Create the executable
add_executable(robot_pose_publisher_node src/robot_pose_publisher.cpp)
ament_target_dependencies(robot_pose_publisher_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

# Install the executable
install(TARGETS
  robot_pose_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have any
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
#   PATTERN "*.launch.py"
# )

ament_package()