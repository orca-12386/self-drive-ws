cmake_minimum_required(VERSION 3.8)
project(multimap_assembler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Create the executable
add_executable(multimap_assembler_node src/multimap_assembler_node.cpp)
ament_target_dependencies(multimap_assembler_node
  rclcpp
  nav_msgs
)

# Install the executable
install(TARGETS
  multimap_assembler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
