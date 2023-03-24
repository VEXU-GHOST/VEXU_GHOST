cmake_minimum_required(VERSION 3.5)
project(sentry_control)

#########################
#### Compile Options ####
#########################
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######################
#### Dependencies ####
######################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ghost_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

#############################
#### Sentry Control Node ####
#############################
add_executable(sentry_control
  src/sentry_control.cpp
)
ament_target_dependencies(sentry_control
  rclcpp
  geometry_msgs
  ghost_msgs
  nav_msgs
)
install(
  TARGETS sentry_control
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
