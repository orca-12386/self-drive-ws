cmake_minimum_required(VERSION 3.8)
project(local_costmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Add executable
add_executable(local_costmap_node src/local_costmap_node.cpp)
target_include_directories(local_costmap_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIR}
)
target_compile_features(local_costmap_node PUBLIC c_std_99 cxx_std_17)

# Link against dependencies
ament_target_dependencies(
  local_costmap_node
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros"
  "ament_index_cpp"
)
target_link_libraries(local_costmap_node
  yaml-cpp
)

# Install
install(TARGETS local_costmap_node
  DESTINATION lib/${PROJECT_NAME})

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()