cmake_minimum_required(VERSION 3.8)
project(bfmc_odometry_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_localization REQUIRED)

add_executable(odom_stats_node src/odom_stats_node.cpp)
ament_target_dependencies(odom_stats_node rclcpp nav_msgs std_msgs)

install(TARGETS
  odom_stats_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
