cmake_minimum_required(VERSION 3.8)
project(ramp_goals)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Declare a C++ executable
add_executable(ramp_goals src/ramp_goals.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(ramp_goals
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Install the executable
install(TARGETS
  ramp_goals
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have any
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Uncomment and modify if you have header files to install
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

# This line is needed to make sure rosidl_generate_interfaces can find the messages
ament_package()