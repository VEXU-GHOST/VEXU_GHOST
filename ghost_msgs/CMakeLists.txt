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
find_package(std_msgs REQUIRED)

# Custom Msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/util/LabeledFloat.msg"
  "msg/util/LabeledFloatMap.msg"
  "msg/util/LabeledVector.msg"
  "msg/util/LabeledVectorMap.msg"
  "msg/v5_msgs/V5MotorCommand.msg"
  "msg/v5_msgs/V5EncoderState.msg"
  "msg/GhostRobotState.msg"
  "msg/V5ActuatorCommand.msg"
  "msg/V5SensorUpdate.msg"
  "msg/CVDisc.msg"
  DEPENDENCIES std_msgs
 )
ament_export_dependencies(rosidl_default_runtime)

ament_package()
