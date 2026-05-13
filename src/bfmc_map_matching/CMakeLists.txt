cmake_minimum_required(VERSION 3.8)
project(bfmc_map_matching)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(rgb_map_matching_node
  src/rgb_map_matching_node.cpp
)

ament_target_dependencies(rgb_map_matching_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

target_link_libraries(rgb_map_matching_node
  ${OpenCV_LIBS}
)

install(TARGETS
  rgb_map_matching_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
