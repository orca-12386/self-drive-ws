cmake_minimum_required(VERSION 3.8)
project(goal_calculator_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
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
find_package(rosidl_default_generators REQUIRED)
find_package(interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

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

add_library(find_intersection_action_server SHARED
  src/find_intersection_action_server.cpp
)

target_include_directories(find_intersection_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(find_intersection_action_server
  PRIVATE "GOAL_CALCULATOR_BUILDING_DLL"
)

ament_target_dependencies(find_intersection_action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "interfaces"
)

install(TARGETS
  find_intersection_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install the executable
install(TARGETS
  lane_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
