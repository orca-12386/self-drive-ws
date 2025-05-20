cmake_minimum_required(VERSION 3.8)
project(map_reviser)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)


add_library(find_intersection_action_server SHARED
  src/find_intersection_action_server.cpp
)

target_include_directories(find_intersection_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(find_intersection_action_server
  PRIVATE "MAP_REVISER_BUILDING_DLL"
)

ament_target_dependencies(find_intersection_action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "interfaces"
)


rclcpp_components_register_node(find_intersection_action_server
  PLUGIN "FindIntersectionActionServer"
  EXECUTABLE find_intersection_action_server_node
)

# Install
install(TARGETS
  find_intersection_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


add_executable(intersection_clearing_service src/intersection_clearer.cpp)

ament_target_dependencies(intersection_clearing_service
  rclcpp
  nav_msgs
  std_msgs
  example_interfaces
  tf2
  visualization_msgs
)

install(TARGETS
  intersection_clearing_service
  DESTINATION lib/${PROJECT_NAME}
)


# Create executable
add_executable(yellow_lane_interpolator src/yellow_lane_interpolator.cpp)
ament_target_dependencies(yellow_lane_interpolator rclcpp nav_msgs geometry_msgs example_interfaces)

add_executable(nearest_lane_mapper_node src/nearest_lane_mapper_node.cpp)
ament_target_dependencies(nearest_lane_mapper_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  geometry_msgs
)

# Install
install(TARGETS
  yellow_lane_interpolator
  nearest_lane_mapper_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
