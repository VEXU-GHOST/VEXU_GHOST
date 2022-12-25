cmake_minimum_required(VERSION 3.5)
project(ghost_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom Msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorCommands.msg"
  "msg/SensorUpdate.msg"
 )
ament_export_dependencies(rosidl_default_runtime)

ament_package()
