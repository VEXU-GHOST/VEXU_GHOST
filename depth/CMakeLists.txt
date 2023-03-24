cmake_minimum_required(VERSION 3.5)
project(depth)

#########################
#### Compile Options ####
#########################
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compile time flags to add debug symbols and disable code optimization (for using debugger)
# set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_BUILD_TYPE "RELEASE")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######################
#### Dependencies ####
######################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ghost_msgs REQUIRED)

include_directories(include)

####################
#### Depth Node ####
####################
add_executable(depth_calculations
  src/depth_calculations.cpp
)
ament_target_dependencies(depth_calculations
  rclcpp
  sensor_msgs
  std_msgs
  ghost_msgs
)
install(
  TARGETS depth_calculations
  DESTINATION lib/${PROJECT_NAME}
)

#################
#### Install ####
#################
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
