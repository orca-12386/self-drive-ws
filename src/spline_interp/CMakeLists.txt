cmake_minimum_required(VERSION 3.8)
project(spline_interp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(spline_interp src/spline_interp.cpp)
add_executable(linear_interp src/linear_interp.cpp)
ament_target_dependencies(spline_interp rclcpp nav_msgs OpenCV)
ament_target_dependencies(linear_interp rclcpp nav_msgs OpenCV)

# Install the executable
install(TARGETS spline_interp
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS linear_interp
  DESTINATION lib/${PROJECT_NAME})

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
