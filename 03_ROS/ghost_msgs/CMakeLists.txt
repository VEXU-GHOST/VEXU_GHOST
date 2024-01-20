cmake_minimum_required(VERSION 3.5)
project(ghost_msgs)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{HOME}/VEXU_GHOST/cmake")
find_package(Ghost CONFIG REQUIRED)

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
  "msg/util/Pose2D.msg"
  "msg/util/Twist2D.msg"
  "msg/device_msgs/V5DeviceHeader.msg"
  "msg/device_msgs/commands/V5MotorCommand.msg"
  "msg/device_msgs/states/V5CompetitionStatus.msg"
  "msg/device_msgs/states/V5JoystickState.msg"
  "msg/device_msgs/states/V5MotorState.msg"
  "msg/device_msgs/states/V5RotationSensorState.msg"
  "msg/V5ActuatorCommand.msg"
  "msg/V5SensorUpdate.msg"
  "msg/DrivetrainCommand.msg"
  "msg/MotorTrajectory.msg"
  "msg/RobotTrajectory.msg"
  "srv/BroadcastJacobian.srv"
  DEPENDENCIES std_msgs
 )
ament_export_dependencies(rosidl_default_runtime)

ament_package()
