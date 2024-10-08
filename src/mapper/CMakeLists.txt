cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# include directories for PCL
include_directories(${PCL_INCLUDE_DIRS})

# link directories for PCL
link_directories(${PCL_LIBRARY_DIRS})

# add the executable
add_executable(Mapper src/mapper.cpp)

# link libraries for PCL and ROS 2
ament_target_dependencies(Mapper rclcpp std_msgs nav_msgs pcl_ros)
target_link_libraries(Mapper ${PCL_LIBRARIES})

# install the executable
install(TARGETS Mapper
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
