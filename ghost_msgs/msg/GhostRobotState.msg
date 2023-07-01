# Header
std_msgs/Header header
uint32 msg_id

## DRIVETRAIN ##
# Robot Pose
float32 x
float32 y
float32 theta

# Robot Twist
float32 x_vel
float32 y_vel
float32 theta_vel

# Robot Accel
float32 x_accel
float32 y_accel
float32 theta_accel

## SWERVE ##
float32 left_steering_angle
float32 right_steering_angle
float32 back_steering_angle

float32 left_steering_vel
float32 right_steering_vel
float32 back_steering_vel