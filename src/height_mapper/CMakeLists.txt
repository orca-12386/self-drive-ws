cmake_minimum_required(VERSION 3.8)
project(height_mapper)

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
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED) 
find_package(PCL REQUIRED)
include_directories(
  ${PCL_INCLUDE_DIRS}
)

# Create the executable
add_executable(height_mapper_node src/height_mapper_node.cpp)

ament_target_dependencies(height_mapper_node
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
  OpenCV
)

add_executable(height_mask_publisher_node src/height_mask_publisher_node.cpp)
ament_target_dependencies(height_mask_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  OpenCV
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

add_executable(pc_odom src/pc_odom.cpp)
ament_target_dependencies(pc_odom
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)



# Install the executable
install(TARGETS
  height_mapper_node
  height_mask_publisher_node
  pc_odom
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
