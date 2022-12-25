cmake_minimum_required(VERSION 3.5)
project(ghost_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
