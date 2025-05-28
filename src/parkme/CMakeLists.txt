cmake_minimum_required(VERSION 3.8)
project(parkme)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_library(lane_server SHARED
  src/park_lanes.cpp)
target_include_directories(lane_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(lane_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(lane_server
  "interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "cv_bridge"
  "OpenCV"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs")



rclcpp_components_register_node(lane_server PLUGIN "parkme::ParkingActionServer" EXECUTABLE lane_parking)
install(TARGETS
  lane_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


add_library(no_lane_server SHARED
src/park_no_lanes.cpp)
target_include_directories(no_lane_server PRIVATE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_definitions(no_lane_server
PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(no_lane_server
  "interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "cv_bridge"
  "OpenCV"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs")



rclcpp_components_register_node(no_lane_server PLUGIN "parkme::ParkingActionServer" EXECUTABLE no_lane_parking)
install(TARGETS
  no_lane_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

## Client

add_library(action_client SHARED
src/parking_action_client.cpp)
target_include_directories(action_client PRIVATE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "parkme::ParkingActionClient" EXECUTABLE parking_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
