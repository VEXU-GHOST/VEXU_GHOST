std_msgs/Header header

# Raw data from the sensor
## 16 bit value
uint16 raw_r
uint16 raw_g
uint16 raw_b
uint16 raw_c

# Processed data
float32 h # [0, 360)
float32 s # [0, 1] 
float32 v # [0, 1] 