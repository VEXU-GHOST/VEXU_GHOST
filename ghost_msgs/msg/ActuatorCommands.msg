# Header
std_msgs/Header header

# Wheel Trajectory
int32 wheel_left_angle_cmd              # Degrees
int32 wheel_right_angle_cmd             # Degrees
int32 wheel_back_angle_cmd              # Degrees

float32 wheel_left_velocity_cmd         # RPM
float32 wheel_right_velocity_cmd        # RPM
float32 wheel_back_velocity_cmd         # RPM

float32 wheel_left_voltage_cmd          # 0.0 -> 1.0
float32 wheel_right_voltage_cmd         # 0.0 -> 1.0
float32 wheel_back_voltage_cmd          # 0.0 -> 1.0

# Steering Trajectory
int32 steering_left_angle_cmd           # Degrees
int32 steering_right_angle_cmd          # Degrees
int32 steering_back_angle_cmd           # Degrees

float32 steering_left_velocity_cmd      # RPM
float32 steering_right_velocity_cmd     # RPM
float32 steering_back_velocity_cmd      # RPM

float32 steering_left_voltage_cmd       # 0.0 -> 1.0
float32 steering_right_voltage_cmd      # 0.0 -> 1.0
float32 steering_back_voltage_cmd       # 0.0 -> 1.0

# Turret Motor
int32 turret_angle_cmd # Degrees
float32 turret_velocity_cmd             # RPM
float32 turret_voltage_cmd              # 0.0 -> 1.0

# Flywheel Motors
float32 flywheel_left_velocity_cmd      # RPM
float32 flywheel_right_velocity_cmd     # RPM

float32 flywheel_right_voltage_cmd      # 0.0 -> 1.0
float32 flywheel_left_voltage_cmd       # 0.0 -> 1.0

# Intake
float32 intake_voltage_cmd              # 0.0 -> 1.0

# Indexer
float32 indexer_voltage_cmd             # 0.0 -> 1.0

# Digital Outputs
bool digital_out_1
bool digital_out_2
bool digital_out_3
bool digital_out_4
bool digital_out_5
bool digital_out_6
bool digital_out_7
bool digital_out_8