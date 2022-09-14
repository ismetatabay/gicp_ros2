cmake_minimum_required(VERSION 3.5)
project(gicp_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(gicp src/gicp.cpp)
add_executable(viewer src/viewer.cpp)

target_link_libraries (gicp ${PCL_LIBRARIES})
target_link_libraries (viewer ${PCL_LIBRARIES})

ament_target_dependencies(gicp rclcpp std_msgs sensor_msgs)
ament_target_dependencies(viewer rclcpp std_msgs sensor_msgs)

install(
  TARGETS gicp
  DESTINATION lib/${PROJECT_NAME})
install(
  TARGETS viewer
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
ament_package()
