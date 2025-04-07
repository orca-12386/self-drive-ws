cmake_minimum_required(VERSION 3.8)
project(behaviour_manager)

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

find_package(interfaces REQUIRED)
find_package(topic_remapper REQUIRED)

add_executable(behaviour_manager_node src/behaviour_manager_node.cpp)
ament_target_dependencies(behaviour_manager_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  "rclcpp_action"
  "rclcpp_components"
  "interfaces"
  "topic_remapper"
)

target_include_directories(behaviour_manager_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# Install the executable
install(TARGETS
  behaviour_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
