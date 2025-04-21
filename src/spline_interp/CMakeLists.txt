cmake_minimum_required(VERSION 3.5)
project(spline_interp)

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
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build
add_executable(linear_interp src/linear_interp.cpp)
ament_target_dependencies(linear_interp
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Install
install(TARGETS
  linear_interp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()