std_msgs/Header header

# Encoder Data (Both motors and rotation sensors)
# Typically we won't use all ports or may send a subset of them. Regardless, we will 
ghost_msgs/V5EncoderState[21] encoders

# Joystick Channels
int32 joystick_left_x       # 0 -> 127
int32 joystick_left_y       # 0 -> 127
int32 joystick_right_x      # 0 -> 127
int32 joystick_right_y      # 0 -> 127

# Joystick Buttons
bool joystick_btn_a
bool joystick_btn_b
bool joystick_btn_x
bool joystick_btn_y
bool joystick_btn_up
bool joystick_btn_down
bool joystick_btn_left
bool joystick_btn_right
bool joystick_btn_l1
bool joystick_btn_l2
bool joystick_btn_r1
bool joystick_btn_r2

# Competition state
bool is_disabled
bool is_autonomous
bool is_connected

# Digital Outputs
bool[8] digital_out_vector