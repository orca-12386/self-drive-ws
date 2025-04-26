cmake_minimum_required(VERSION 3.8)
project(goal_calculator_cpp)

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
find_package(interfaces REQUIRED)

# Create the executable
add_executable(lane_follower_node src/lane_follow.cpp)
ament_target_dependencies(lane_follower_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  interfaces
)

# Install the executable
install(TARGETS
  lane_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
