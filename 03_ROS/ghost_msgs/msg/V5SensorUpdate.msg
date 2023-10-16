std_msgs/Header header
uint32 msg_id

# Encoder Data (Both motors and rotation sensors)
# Typically we won't use all ports or may send a subset of them.
ghost_msgs/V5EncoderState[21] encoders