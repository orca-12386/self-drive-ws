cmake_minimum_required(VERSION 3.8)
project(goal_calculator_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

# Create executable
add_executable(goal_calculator_node src/goal_calc.cpp)
target_include_directories(goal_calculator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Specify dependencies
ament_target_dependencies(goal_calculator_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
  std_msgs
  Eigen3
)

# Link against yaml-cpp explicitly
target_link_libraries(goal_calculator_node
  yaml-cpp
)

# Install targets
install(TARGETS goal_calculator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  # launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
  std_msgs
  yaml-cpp
  Eigen3
)

ament_package()