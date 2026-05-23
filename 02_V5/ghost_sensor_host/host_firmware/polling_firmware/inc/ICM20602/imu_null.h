/*
Original IMU driver code by Martin Budden: https://github.com/martinbudden/Library-Sensors
*/

#pragma once

#include "imu_base.h"

/*!
NULL IMU, useful for test code.
*/
class ImuNull : public ImuBase {
public:
    virtual ~ImuNull() override = default;
    ImuNull(const ImuNull&) = delete;
    ImuNull& operator=(const ImuNull&) = delete;
    ImuNull(ImuNull&&) = delete;
    ImuNull& operator=(ImuNull&&) = delete;

    explicit ImuNull(uint8_t axis_order) : ImuBase(axis_order) {}
    ImuNull() : ImuNull(ImuBase::XPOS_YPOS_ZPOS) {}

// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
    virtual int init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) override;
    virtual xyz_int32_t read_gyro_raw() override;
    virtual xyz_int32_t read_acc_raw() override;
// NOLINTEND(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
public: // for testing
    void set_gyro_raw(const xyz_int32_t& gyro_raw) { _gyro_raw = gyro_raw; }
    void set_acc_raw(const xyz_int32_t& acc_raw) { _acc_raw = acc_raw; }
private:
    xyz_int32_t _gyro_raw {};
    xyz_int32_t _acc_raw {};
};
