cmake_minimum_required(VERSION 3.8)
project(bfmc_car_description)

find_package(ament_cmake REQUIRED)

foreach(dir launch meshes textures urdf config)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(
      DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

ament_package()
