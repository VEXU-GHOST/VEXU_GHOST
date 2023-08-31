std_msgs/Header header
uint32 msg_id

# Encoder Data (Both motors and rotation sensors)
# Typically we won't use all ports or may send a subset of them.
ghost_msgs/V5EncoderState[21] encoders

# Digital Ports
bool[8] digital_port_vector

# Competition state
bool is_disabled
bool is_autonomous
bool is_connected

# Joystick Channels
float32 joystick_left_x       # 0 -> 1.0
float32 joystick_left_y       # 0 -> 1.0
float32 joystick_right_x      # 0 -> 1.0
float32 joystick_right_y      # 0 -> 1.0

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