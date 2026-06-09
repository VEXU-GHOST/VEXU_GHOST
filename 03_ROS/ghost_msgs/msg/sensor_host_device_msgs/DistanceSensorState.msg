string name

uint16_t distance_mm            # measured distance in mm
uint8_t  range_status           # status of measurements, 0 = valid
uint16_t sigma_mm               # estimated standard deviation of distance in mm
uint32_t signal_rate_kcps       # measured signal of the target in kcps/SPAD
uint32_t ambient_rate_kcps      # measured ambient noise in kcps/SPAD