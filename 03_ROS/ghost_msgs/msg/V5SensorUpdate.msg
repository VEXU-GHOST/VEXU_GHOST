std_msgs/Header header
uint32 msg_id

# Encoder Data (Both motors and rotation sensors)
# Typically we won't use all ports or may send a subset of them.
ghost_msgs/V5EncoderState[21] encoders

# Digital Ports
bool[8] digital_port_vector

# Competition state
ghost_msgs/V5CompetitionState competition_state_msg

ghost_msgs/V5Joystick joystick_msg