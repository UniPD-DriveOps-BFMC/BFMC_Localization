cmake_minimum_required(VERSION 3.8)
project(bfmc_state_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(encoder_odometry_node
  src/encoder_odometry_node.cpp
)

ament_target_dependencies(encoder_odometry_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(car_imu_republisher_node
  src/car_imu_republisher_node.cpp
)

ament_target_dependencies(car_imu_republisher_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  encoder_odometry_node
  car_imu_republisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
