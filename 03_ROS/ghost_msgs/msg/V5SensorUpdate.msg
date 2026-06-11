std_msgs/Header header

uint32 msg_id

# Competition Status
ghost_msgs/V5CompetitionStatus competition_status

# Joystick States
ghost_msgs/V5JoystickState[] joysticks

# Rotation Sensor States
ghost_msgs/V5RotationSensorState[] rotation_sensors

# Inertial Sensor States
ghost_msgs/V5InertialSensorState[] inertial_sensors

# Motor States
ghost_msgs/V5MotorState[] motors

# Digital Inputs
bool[] digital_io

# VEXLink inter-robot received pose (V5 receives from partner radio; Jetson reads this)
ghost_msgs/V5VexlinkChannelState[] vexlink_channels

