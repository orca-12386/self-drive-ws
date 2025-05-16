cmake_minimum_required(VERSION 3.8)
project(lane_mask_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create the executable
add_executable(lane_mask_publisher_node src/lane_mask_publisher_node.cpp)
ament_target_dependencies(lane_mask_publisher_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(tuner src/tuner.cpp)
ament_target_dependencies(tuner
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(lane_mask_publisher_node
  ${OpenCV_LIBRARIES}
)

# Install the executable
install(TARGETS lane_mask_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(tuner
  ${OpenCV_LIBRARIES}
)

# Install the executable
install(TARGETS tuner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(rclcpp std_msgs sensor_msgs cv_bridge OpenCV yaml-cpp)

ament_package()