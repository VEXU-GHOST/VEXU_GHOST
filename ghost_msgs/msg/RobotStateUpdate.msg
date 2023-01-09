std_msgs/Header header

# Encoder Data (Both motors and rotation sensors)
# Typically we won't use all ports or may send a subset of them. Regardless, we will 
ghost_msgs/V5EncoderState[21] encoders


# Digital Outputs
bool[8] digital_out_vector