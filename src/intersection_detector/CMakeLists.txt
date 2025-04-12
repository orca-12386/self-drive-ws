cmake_minimum_required(VERSION 3.8)
project(intersection_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/DetectIntersection.srv"
                           DEPENDENCIES std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(intersection_detector_node src/intersection_detector_node.cpp)
ament_target_dependencies(
  intersection_detector_node
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
)

target_link_libraries(intersection_detector_node "${cpp_typesupport_target}")

install(TARGETS intersection_detector_node DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
