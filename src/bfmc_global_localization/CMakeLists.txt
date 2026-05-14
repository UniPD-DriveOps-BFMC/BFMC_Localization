cmake_minimum_required(VERSION 3.8)
project(bfmc_global_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)

add_executable(sign_localization_node src/sign_localization_node.cpp)
ament_target_dependencies(sign_localization_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

add_executable(position_publisher_node src/position_publisher_node.cpp)
ament_target_dependencies(position_publisher_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  sign_localization_node
  position_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
