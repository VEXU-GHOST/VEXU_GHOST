std_msgs/Header header

uint32 msg_id

# Motor Commands
ghost_msgs/V5MotorCommand[] motor_commands

# Digital IO
bool[] digital_io

# VEXLink inter-robot pose to broadcast (Jetson sets this; V5 transmits over radio)
ghost_msgs/V5VexlinkChannelCommand[] vexlink_channels