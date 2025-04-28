cmake_minimum_required(VERSION 3.8)
project(vision)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)

# (Optional) If you plan to use OpenCV in future
# find_package(OpenCV REQUIRED)
# find_package(cv_bridge REQUIRED)

add_executable(depth_mapper src/depth_mapper.cpp)

ament_target_dependencies(depth_mapper
  rclcpp
  sensor_msgs
  nav_msgs
  message_filters
)

# (Optional) If OpenCV/cv_bridge is needed later
# target_link_libraries(depth_processor_node ${OpenCV_LIBS})

install(TARGETS
  depth_mapper
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
