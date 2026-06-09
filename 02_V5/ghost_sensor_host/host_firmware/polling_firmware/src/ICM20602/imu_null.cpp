/*
Original IMU driver code by Martin Budden: https://github.com/martinbudden/Library-Sensors
*/

#include "imu_null.h"


int ImuNull::init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex)
{
    (void)gyro_sensitivity;
    (void)acc_sensitivity;
    (void)bus_mutex;

    _gyro_id_msp = MSP_GYRO_ID_VIRTUAL;
    _acc_id_msp = MSP_ACC_ID_VIRTUAL;

    return static_cast<int>(target_output_data_rate_hz);
}

ImuBase::xyz_int32_t ImuNull::read_gyro_raw()
{
    return _gyro_raw;
}

ImuBase::xyz_int32_t ImuNull::read_acc_raw()
{
    return _acc_raw;
}
