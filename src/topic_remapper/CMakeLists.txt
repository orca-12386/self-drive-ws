cmake_minimum_required(VERSION 3.8)
project(topic_remapper)

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

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeTopic.srv"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)

add_executable(occupancy_grid_topic_remapper_node src/occupancy_grid_topic_remapper_node.cpp)
ament_target_dependencies(occupancy_grid_topic_remapper_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

add_executable(odom_topic_remapper_node src/odom_topic_remapper_node.cpp)
ament_target_dependencies(odom_topic_remapper_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

add_executable(pose_stamped_topic_remapper_node src/pose_stamped_topic_remapper_node.cpp)
ament_target_dependencies(pose_stamped_topic_remapper_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

target_link_libraries(occupancy_grid_topic_remapper_node "${cpp_typesupport_target}")
target_link_libraries(odom_topic_remapper_node "${cpp_typesupport_target}")
target_link_libraries(pose_stamped_topic_remapper_node "${cpp_typesupport_target}")


# Install the executable
install(TARGETS
  occupancy_grid_topic_remapper_node
  odom_topic_remapper_node
  pose_stamped_topic_remapper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
