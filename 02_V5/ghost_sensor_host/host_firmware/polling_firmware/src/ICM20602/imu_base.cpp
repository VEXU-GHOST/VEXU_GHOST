/*
Original IMU driver code by Martin Budden: https://github.com/martinbudden/Library-Sensors
*/

#include "imu_base.h"
#include <cassert>


#if defined(FRAMEWORK_RPI_PICO)
#include <pico/time.h>
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#endif


ImuBase::ImuBase(uint8_t axis_order, BusBase& bus_base, uint8_t flags) :
    _bus_base(&bus_base),
    _axis_order(axis_order),
    _flags(flags)
{
}

ImuBase::ImuBase(uint8_t axis_order, BusBase& bus_base) :
    ImuBase(axis_order, bus_base, 0)
{
}

ImuBase::ImuBase(uint8_t axis_order, uint8_t flags):
    _bus_base(nullptr),
    _axis_order(axis_order),
    _flags(flags)
{
}

ImuBase::ImuBase(uint8_t axis_order) :
    ImuBase(axis_order, 0)
{
}

int ImuBase::init(uint32_t target_output_data_rate_hz, void* bus_mutex)
{
    return init(target_output_data_rate_hz, GYRO_FULL_SCALE_MAX, ACC_FULL_SCALE_MAX, bus_mutex);
}

int ImuBase::init(uint32_t target_output_data_rate_hz)
{
    return init(target_output_data_rate_hz, nullptr);
}

int ImuBase::init(void* bus_mutex)
{
    return init(TARGET_OUTPUT_DATA_RATE_MAX, GYRO_FULL_SCALE_MAX, ACC_FULL_SCALE_MAX, bus_mutex);
}

int ImuBase::init()
{
    return init(nullptr);
}

void ImuBase::set_interrupt_driven()
{
}

xyz_t ImuBase::get_gyro_offset() const
{
    return _gyro_offset;
}

void ImuBase::set_gyro_offset(const xyz_t& gyro_offset)
{
    _gyro_offset = gyro_offset;
}

xyz_t ImuBase::get_acc_offset() const
{
    return _acc_offset;
}

void ImuBase::set_acc_offset(const xyz_t& acc_offset)
{
    _acc_offset = acc_offset;
}

xyz_t ImuBase::get_gyro_offset_mapped() const
{
    return map_axes(get_gyro_offset(), get_axis_order());
}

void ImuBase::set_gyro_offset_mapped(const xyz_t& offset)
{
    set_gyro_offset(ImuBase::map_axes(offset, axis_order_inverse(get_axis_order())));
}

xyz_t ImuBase::get_acc_offset_mapped() const
{
    return ImuBase::map_axes(get_acc_offset(), get_axis_order());
}

void ImuBase::set_acc_offset_mapped(const xyz_t& offset)
{
    set_acc_offset(map_axes(offset, axis_order_inverse(get_axis_order())));
}

xyz_t ImuBase::read_gyro_rps()
{
    const xyz_int32_t gyro_raw = read_gyro_raw();
    return map_axes({
        .x = static_cast<float>(gyro_raw.x) * _gyro_resolution_rps - _gyro_offset.x,
        .y = static_cast<float>(gyro_raw.y) * _gyro_resolution_rps - _gyro_offset.y,
        .z = static_cast<float>(gyro_raw.z) * _gyro_resolution_rps - _gyro_offset.z
    });
}

xyz_t ImuBase::read_gyro_dps()
{
    const xyz_int32_t gyro_raw = read_gyro_raw();
    return map_axes({
        .x = static_cast<float>(gyro_raw.x) * _gyro_resolution_dps - _gyro_offset.x,
        .y = static_cast<float>(gyro_raw.y) * _gyro_resolution_dps - _gyro_offset.y,
        .z = static_cast<float>(gyro_raw.z) * _gyro_resolution_dps - _gyro_offset.z
    });
}

xyz_t ImuBase::read_acc()
{
    const xyz_int32_t acc_raw = read_acc_raw();
    return map_axes({
        .x = static_cast<float>(acc_raw.x) * _acc_resolution - _acc_offset.x,
        .y = static_cast<float>(acc_raw.y) * _acc_resolution - _acc_offset.y,
        .z = static_cast<float>(acc_raw.z) * _acc_resolution - _acc_offset.z
    });
}

acc_gyro_rps_t ImuBase::read_acc_gyro_rps()
{
    return acc_gyro_rps_t {
        .gyro_rps = read_gyro_rps(),
        .acc = read_acc()
    };
}

acc_gyro_rps_t ImuBase::get_acc_gyro_rps() const
{
    return acc_gyro_rps_t {};
}

Quaternion ImuBase::read_orientation()
{
    return Quaternion {};
}

xyz_t ImuBase::map_axes(const xyz_t& data, uint8_t axis_order)
{
// NOLINTBEGIN(bugprone-branch-clone) false positive
    switch (axis_order) {
    case XPOS_YPOS_ZPOS:
        return data;
    case YPOS_XNEG_ZPOS: // rotate  90 degrees anticlockwise
        // .x =   data.x*cos90(0) + data.y*sin90(1)
        // .y =  -data.x*sin90(1) + data.y*cos90(0)
        return xyz_t {
            .x =  data.y,
            .y = -data.x,
            .z =  data.z
        };
    case XNEG_YNEG_ZPOS: // rotate 180 degrees
        // .x =   data.x*cos180(-1) + data.y*sin180(0)
        // .y =  -data.x*sin180(0) + data.y*cos180(-1)
        return xyz_t {
            .x = -data.x,
            .y = -data.y,
            .z =  data.z
        };
    case YNEG_XPOS_ZPOS: // rotate 270 degrees anticlockwise
        // .x =   data.x*cos270(0) + data.y*sin270(-1)
        // .y =  -data.x*sin270(-1) + data.y*cos270(0)
        return xyz_t {
            .x = -data.y,
            .y =  data.x,
            .z =  data.z
        };
    case XPOS_YNEG_ZNEG:
        return xyz_t {
            .x =  data.x,
            .y = -data.y,
            .z = -data.z
        };
    case YPOS_XPOS_ZNEG:
        return xyz_t {
            .x =  data.y,
            .y =  data.x,
            .z = -data.z
        };
    case XNEG_YPOS_ZNEG:
        return xyz_t {
            .x = -data.x,
            .y =  data.y,
            .z = -data.z
        };
    case YNEG_XNEG_ZNEG:
        return xyz_t {
            .x = -data.y,
            .y = -data.x,
            .z = -data.z
        };
    case ZPOS_YNEG_XPOS:
        return xyz_t {
            .x =  data.z,
            .y = -data.y,
            .z =  data.x
        };
    case YPOS_ZPOS_XPOS:
        return xyz_t {
            .x =  data.y,
            .y =  data.z,
            .z =  data.x
        };
    case ZNEG_YPOS_XPOS:
        return xyz_t {
            .x = -data.z,
            .y =  data.y,
            .z =  data.x
        };
    case YNEG_ZNEG_XPOS:
        return xyz_t {
            .x = -data.y,
            .y = -data.z,
            .z =  data.x
        };
    case ZPOS_YPOS_XNEG:
        return xyz_t {
            .x =  data.z,
            .y =  data.y,
            .z = -data.x
        };
    case YPOS_ZNEG_XNEG:
        return xyz_t {
            .x =  data.y,
            .y = -data.z,
            .z = -data.x
        };
    case ZNEG_YNEG_XNEG:
        return xyz_t {
            .x = -data.z,
            .y = -data.y,
            .z = -data.x
        };
    case YNEG_ZPOS_XNEG:
        return xyz_t {
            .x = -data.y,
            .y =  data.z,
            .z = -data.x
        };
    case ZPOS_XPOS_YPOS:
        return xyz_t {
            .x =  data.z,
            .y =  data.x,
            .z =  data.y
        };
    case XNEG_ZPOS_YPOS:
        return xyz_t {
            .x = -data.x,
            .y =  data.z,
            .z =  data.y
        };
    case ZNEG_XNEG_YPOS:
        return xyz_t {
            .x = -data.z,
            .y = -data.x,
            .z =  data.y
        };
    case XPOS_ZNEG_YPOS:
        return xyz_t {
            .x =  data.x,
            .y = -data.z,
            .z =  data.y
        };
    case ZPOS_XNEG_YNEG:
        return xyz_t {
            .x =  data.z,
            .y = -data.x,
            .z = -data.y
        };
    case XNEG_ZNEG_YNEG:
        return xyz_t {
            .x = -data.x,
            .y = -data.z,
            .z = -data.y
        };
    case ZNEG_XPOS_YNEG:
        return xyz_t {
            .x = -data.z,
            .y =  data.x,
            .z = -data.y
        };
    case XPOS_ZPOS_YNEG:
        return xyz_t {
            .x =  data.x,
            .y =  data.z,
            .z = -data.y
        };
    case XPOS_YPOS_ZPOS_45: // 45
        return xyz_t {
            .x =  data.x*cos45f + data.y*sin45f,
            .y = -data.x*sin45f + data.y*cos45f,
            .z =  data.z
        };
    case YPOS_XNEG_ZPOS_45: {// 135
        static constexpr float sin135f =  sin45f;
        static constexpr float cos135f = -cos45f;
        return xyz_t {
            .x =  data.x*cos135f + data.y*sin135f,
            .y = -data.x*sin135f + data.y*cos135f,
            .z =  data.z
        };
    }
    case XNEG_YNEG_ZPOS_45: {// 225
        static constexpr float sin225f = -sin45f;
        static constexpr float cos225f = -cos45f;
        return xyz_t {
            .x =  data.x*cos225f + data.y*sin225f,
            .y = -data.x*sin225f + data.y*cos225f,
            .z =  data.z
        };
    }
    case YNEG_XPOS_ZPOS_45: {// 315
        static constexpr float sin315f = -sin45f;
        static constexpr float cos315f =  cos45f;
        return xyz_t {
            .x =  data.x*cos315f + data.y*sin315f,
            .y = -data.x*sin315f + data.y*cos315f,
            .z =  data.z
        };
    }
    default:
        assert(false && "IMU axis order not supported"); // NOLINT(readability-implicit-bool-conversion,readability-simplify-boolean-expr)
        break;
    } // end switch
// NOLINTEND(bugprone-branch-clone)

    return data;
}

uint8_t ImuBase::axis_order_inverse(uint8_t axis_order)
{
    switch (axis_order) {
    case YPOS_XNEG_ZPOS:
        return YNEG_XPOS_ZPOS;
    case YNEG_XPOS_ZPOS:
        return YPOS_XNEG_ZPOS;
    case YPOS_ZPOS_XPOS:
        return ZPOS_XPOS_YPOS;
    case ZNEG_YPOS_XPOS:
        return ZPOS_YPOS_XNEG;
    case YNEG_ZNEG_XPOS:
        return ZPOS_XNEG_YNEG;
    case ZPOS_YPOS_XNEG:
        return ZNEG_YPOS_XPOS;
    case YPOS_ZNEG_XNEG:
        return ZNEG_XPOS_YNEG;
    case YNEG_ZPOS_XNEG:
        return ZNEG_XNEG_YPOS;
    case ZPOS_XPOS_YPOS:
        return YPOS_ZPOS_XPOS;
    case ZNEG_XNEG_YPOS:
        return YNEG_ZPOS_XNEG;
    case XPOS_ZNEG_YPOS:
        return XPOS_ZPOS_YNEG;
    case ZPOS_XNEG_YNEG:
        return YNEG_ZNEG_XPOS;
    case ZNEG_XPOS_YNEG:
        return YPOS_ZNEG_XNEG;
    case XPOS_ZPOS_YNEG:
        return XPOS_ZNEG_YPOS;
    case XPOS_YPOS_ZPOS_45: // 45
        return YNEG_XPOS_ZPOS_45; // 315
    case YPOS_XNEG_ZPOS_45: // 135
        return XNEG_YNEG_ZPOS_45; // 225
    case XNEG_YNEG_ZPOS_45: // 225
        return YPOS_XNEG_ZPOS_45; // 135
    case YNEG_XPOS_ZPOS_45: // 315
        return XPOS_YPOS_ZPOS_45; // 45
    default:
        // other axis orders are self-inverting
        return axis_order;
    }
}

ImuBase::xyz_alignment_t ImuBase::alignment_from_axis_order(uint8_t axis_order)
{
    (void)axis_order;
    xyz_alignment_t alignment {
        .x = 0,
        .y = 0,
        .z = 0
    };
    return alignment;
}

uint8_t ImuBase::axis_order_from_alignment(const xyz_alignment_t& alignment)
{
    (void)alignment;
    return XPOS_YPOS_ZPOS;
}

void ImuBase::calibrate(uint8_t calibration_type, size_t calibration_count)
{
    int64_t gyroX = 0;
    int64_t gyroY = 0;
    int64_t gyroZ = 0;
    int64_t accX = 0;
    int64_t accY = 0;
    int64_t accZ = 0;

    for (size_t ii = 0; ii < calibration_count; ++ii) {
        delay_ms(1);

        const xyz_int32_t gyro32 = read_gyro_raw();
        gyroX += gyro32.x;
        gyroY += gyro32.y;
        gyroZ += gyro32.x;

        const xyz_int32_t acc32 = read_acc_raw();
        accX += acc32.x;
        accY += acc32.y;
        accZ += acc32.z;
    }

    const xyz_t gyro_offset = xyz_t {
        .x = static_cast<float>(gyroX) / static_cast<float>(calibration_count),
        .y = static_cast<float>(gyroY) / static_cast<float>(calibration_count),
        .z = static_cast<float>(gyroZ) / static_cast<float>(calibration_count)
    } *  get_acc_resolution();

    xyz_t acc_offset = {
        .x = static_cast<float>(accX) / static_cast<float>(calibration_count),
        .y = static_cast<float>(accY) / static_cast<float>(calibration_count),
        .z = static_cast<float>(accZ) / static_cast<float>(calibration_count)
    };

    const float oneG = 1.0F / get_acc_resolution();
    const float halfG = oneG / 2.0F;
    if (acc_offset.x > halfG) {
        acc_offset.x -= oneG;
    } else if (acc_offset.x < - halfG) {
        acc_offset.x += oneG;
    } else if (acc_offset.y > halfG) {
        acc_offset.y -= oneG;
    } else if (acc_offset.y < - halfG) {
        acc_offset.y += oneG;
    } else if (acc_offset.z > halfG) {
        acc_offset.z -= oneG;
    } else if (acc_offset.z < - halfG) {
        acc_offset.z += oneG;
    }
    acc_offset *= get_acc_resolution();

    set_gyro_offset(gyro_offset);
    if (calibration_type == CALIBRATE_ACC_AND_GYRO) {
        set_acc_offset(acc_offset);
    }
}
