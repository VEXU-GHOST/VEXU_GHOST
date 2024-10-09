cmake_minimum_required(VERSION 3.5)
project(ghost_msgs)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Custom Msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/util/LabeledDouble.msg"
  "msg/util/LabeledDoubleMap.msg"
  "msg/util/LabeledVector.msg"
  "msg/util/LabeledVectorMap.msg"
  "msg/device_msgs/V5DeviceHeader.msg"
  "msg/device_msgs/commands/V5MotorCommand.msg"
  "msg/device_msgs/states/V5CompetitionStatus.msg"
  "msg/device_msgs/states/V5JoystickState.msg"
  "msg/device_msgs/states/V5MotorState.msg"
  "msg/device_msgs/states/V5RotationSensorState.msg"
  "msg/device_msgs/states/DiffDriveState.msg"
  "msg/device_msgs/states/V5InertialSensorState.msg"
  "msg/V5ActuatorCommand.msg"
  "msg/V5SensorUpdate.msg"
  "msg/DrivetrainCommand.msg"
  "msg/Trajectory.msg"
  "msg/RobotTrajectory.msg"
  "srv/BroadcastJacobian.srv"
  "srv/StartRecorder.srv"
  "srv/StopRecorder.srv"
  "srv/StartTimer.srv"
  "srv/CheckTimer.srv"
  DEPENDENCIES std_msgs geometry_msgs
 )
ament_export_dependencies(rosidl_default_runtime)

ament_package()
