cmake_minimum_required(VERSION 3.8)
project(vision)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# Add definitions
add_definitions(
  ${PCL_DEFINITIONS}
)
# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# Add definitions
add_definitions(
  ${PCL_DEFINITIONS}
)

# Declare the executable
add_executable(pc_mapper_node src/pc_pub.cpp)
target_link_libraries(pc_mapper_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Add dependencies
ament_target_dependencies(pc_mapper_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  pcl_conversions
)

# Install executables
install(TARGETS
  pc_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# # Install config files
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )



# Export package dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  PCL
  pcl_conversions
)

ament_package()