std_msgs/Header header

# Drivetrain Motors
int32 drive_lf_angle            # Degrees
int32 drive_lb_angle            # Degrees
int32 drive_rf_angle            # Degrees
int32 drive_rb_angle            # Degrees
int32 drive_bl_angle            # Degrees
int32 drive_br_angle            # Degrees

float32 drive_lf_velocity       # RPM
float32 drive_lb_velocity       # RPM
float32 drive_rf_velocity       # RPM
float32 drive_rb_velocity       # RPM
float32 drive_bl_velocity       # RPM
float32 drive_br_velocity       # RPM

# Turret Encoder
int32 turret_angle              # Degrees
float32 turret_velocity         # RPM

# Drivetrain Steering Encoders
int32 steering_left_angle       # Degrees
int32 steering_right_angle      # Degrees
int32 steering_back_angle       # Degrees

int32 steering_left_velocity    # RPM
int32 steering_right_velocity   # RPM
int32 steering_back_velocity    # RPM

# Joystick Channels
int32 joystick_left_x           # 0 -> 127
int32 joystick_left_y           # 0 -> 127
int32 joystick_right_x          # 0 -> 127
int32 joystick_right_y          # 0 -> 127

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
bool digital_out_1
bool digital_out_2
bool digital_out_3
bool digital_out_4
bool digital_out_5
bool digital_out_6
bool digital_out_7
bool digital_out_8
