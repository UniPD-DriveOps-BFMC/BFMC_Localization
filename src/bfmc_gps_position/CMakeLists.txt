cmake_minimum_required(VERSION 3.8)
project(bfmc_gps_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(gps_tag_to_base_node
  src/gps_tag_to_base_node.cpp
)

ament_target_dependencies(gps_tag_to_base_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS
  gps_tag_to_base_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
