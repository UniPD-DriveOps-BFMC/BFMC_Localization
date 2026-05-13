cmake_minimum_required(VERSION 3.8)
project(bfmc_isaac_visual_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(visual_odom_republisher
  src/visual_odom_republisher.cpp
)

ament_target_dependencies(visual_odom_republisher
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  visual_odom_republisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()