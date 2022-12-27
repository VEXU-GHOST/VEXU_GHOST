# Header
std_msgs/Header header

# Wheel Trajectory
int32 wheel_left_angle_cmd
int32 wheel_right_angle_cmd
int32 wheel_back_angle_cmd

float32 wheel_left_velocity_cmd
float32 wheel_right_velocity_cmd
float32 wheel_back_velocity_cmd

int16 wheel_left_torque_cmd
int16 wheel_right_torque_cmd
int16 wheel_back_torque_cmd

# Steering Trajectory
int32 steering_left_angle_cmd
int32 steering_right_angle_cmd
int32 steering_back_angle_cmd

float32 steering_left_velocity_cmd
float32 steering_right_velocity_cmd
float32 steering_back_velocity_cmd

int16 steering_left_torque_cmd
int16 steering_right_torque_cmd
int16 steering_back_torque_cmd

# Turret Motor
int32 turret_angle_cmd
float32 turret_velocity_cmd
int16 turret_torque_cmd

# Flywheel Motors
float32 flywheel_left_velocity_cmd
float32 flywheel_right_velocity_cmd

int16 flywheel_right_torque_cmd
int16 flywheel_left_torque_cmd

# Intake
int16 intake_torque_cmd

# Indexer
int16 indexer_torque_cmd

# Digital Outputs
bool digital_out_1
bool digital_out_2
bool digital_out_3
bool digital_out_4
bool digital_out_5
bool digital_out_6
bool digital_out_7
bool digital_out_8