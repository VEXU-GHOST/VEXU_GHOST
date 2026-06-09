/*
Original IMU driver code by Martin Budden: https://github.com/martinbudden/Library-Sensors
*/

#pragma once

#include "bus_base.h"

#include "../lib/Library-VectorQuaternionMatrix/src/quaternion.h"


/*!
IMU virtual base class.

Base class for an IMU (Inertial Management Unit) including a gyroscope and accelerometer.

Gyroscope readings can be returned as raw values, in RPS (Radians Per Second), or in DPS (Degrees Per Second).

Accelerometer readings are returned in units of standard gravity (g - 9.80665 meters per second squared).

The gyro and accelerometer can be read together using read_acc_gyro_rps. For typical IMUs this involves reading 12 bytes of data:
if the IMU is read via a 10 MHz  SPI bus this takes approximately 10 microseconds,
if the IMU is read via a 400 kHz I2C bus this takes approximately 270 microseconds.
*/
class ImuBase {
public:
    virtual ~ImuBase() = default;
    ImuBase(uint8_t axis_order, BusBase& bus_base, uint8_t flags);
    ImuBase(uint8_t axis_order, BusBase& bus_base);
    ImuBase(uint8_t axis_order, uint8_t flags);
    explicit ImuBase(uint8_t axis_order);
public:
    static constexpr int32_t NOT_DETECTED = -1;
    /*!
    Axes order describing the sensor axes relative to the body axes.
    For example, if the sensor is rotated relative to the body so that the
    sensor X-axis points right, the sensor Z-axis points forward, and the sensor Y-axis points down,
    then the axis order is XPOS_ZPOS_YNEG.

    For example, if the sensor is rotated relative to the body so that the
    sensor y-axis points right, the sensor X-axis points back, and the sensor Z-axis points up,
    then the axis order is YPOS_XNEG_ZPOS
    */
    static constexpr uint8_t XPOS_YPOS_ZPOS = 0;
    static constexpr uint8_t YPOS_XNEG_ZPOS = 1; // rotate  90 degrees anticlockwise
    static constexpr uint8_t XNEG_YNEG_ZPOS = 2; // rotate 180 degrees
    static constexpr uint8_t YNEG_XPOS_ZPOS = 3; // rotate 270 degrees anticlockwise

    static constexpr uint8_t XPOS_YNEG_ZNEG = 4;
    static constexpr uint8_t YPOS_XPOS_ZNEG = 5;
    static constexpr uint8_t XNEG_YPOS_ZNEG = 6;
    static constexpr uint8_t YNEG_XNEG_ZNEG = 7;

    static constexpr uint8_t ZPOS_YNEG_XPOS = 8;
    static constexpr uint8_t YPOS_ZPOS_XPOS = 9;
    static constexpr uint8_t ZNEG_YPOS_XPOS = 10;
    static constexpr uint8_t YNEG_ZNEG_XPOS = 11;

    static constexpr uint8_t ZPOS_YPOS_XNEG = 12;
    static constexpr uint8_t YPOS_ZNEG_XNEG = 13;
    static constexpr uint8_t ZNEG_YNEG_XNEG = 14;
    static constexpr uint8_t YNEG_ZPOS_XNEG = 15;

    static constexpr uint8_t ZPOS_XPOS_YPOS = 16;
    static constexpr uint8_t XNEG_ZPOS_YPOS = 17;
    static constexpr uint8_t ZNEG_XNEG_YPOS = 18;
    static constexpr uint8_t XPOS_ZNEG_YPOS = 19;

    static constexpr uint8_t ZPOS_XNEG_YNEG = 20;
    static constexpr uint8_t XNEG_ZNEG_YNEG = 21;
    static constexpr uint8_t ZNEG_XPOS_YNEG = 22;
    static constexpr uint8_t XPOS_ZPOS_YNEG = 23;

    static constexpr uint8_t XPOS_YPOS_ZPOS_45 = 24; // rotate  45 degrees anticlockwise
    static constexpr uint8_t YPOS_XNEG_ZPOS_45 = 25; // rotate 135 degrees anticlockwise
    static constexpr uint8_t XNEG_YNEG_ZPOS_45 = 26; // rotate 225 degrees anticlockwise
    static constexpr uint8_t YNEG_XPOS_ZPOS_45 = 27;  // rotate 315 degrees anticlockwise

    static constexpr uint8_t XPOS_YPOS_ZPOS_135 = YPOS_XNEG_ZPOS_45;
    static constexpr uint8_t XPOS_YPOS_ZPOS_225 = XNEG_YNEG_ZPOS_45;
    static constexpr uint8_t XPOS_YPOS_ZPOS_315 = YNEG_XPOS_ZPOS_45;

    // North East Down (NED) equivalents
    static constexpr uint8_t XPOS_YPOS_ZPOS_NED = YPOS_XPOS_ZNEG;
    static constexpr uint8_t YPOS_XNEG_ZPOS_NED = XNEG_YPOS_ZNEG;
    static constexpr uint8_t XNEG_YNEG_ZPOS_NED = YNEG_XNEG_ZNEG;
    static constexpr uint8_t YNEG_XPOS_ZPOS_NED = XPOS_YNEG_ZNEG;

    static constexpr uint8_t XPOS_YNEG_ZNEG_NED = YNEG_XPOS_ZPOS;
    static constexpr uint8_t YPOS_XPOS_ZNEG_NED = XPOS_YPOS_ZPOS;
    static constexpr uint8_t XNEG_YPOS_ZNEG_NED = YPOS_XNEG_ZPOS;
    static constexpr uint8_t YNEG_XNEG_ZNEG_NED = XNEG_YNEG_ZPOS;

    static constexpr uint8_t ZPOS_YNEG_XPOS_NED = YNEG_ZPOS_XNEG;
    static constexpr uint8_t YPOS_ZPOS_XPOS_NED = ZPOS_YPOS_XNEG;
    static constexpr uint8_t ZNEG_YPOS_XPOS_NED = YPOS_ZNEG_XNEG;
    static constexpr uint8_t YNEG_ZNEG_XPOS_NED = ZNEG_YNEG_XNEG;

    static constexpr uint8_t ZPOS_YPOS_XNEG_NED = YPOS_ZPOS_XPOS;
    static constexpr uint8_t YPOS_ZNEG_XNEG_NED = ZNEG_YPOS_XPOS;
    static constexpr uint8_t ZNEG_YNEG_XNEG_NED = YNEG_ZNEG_XPOS;
    static constexpr uint8_t YNEG_ZPOS_XNEG_NED = ZPOS_YNEG_XPOS;

    static constexpr uint8_t ZPOS_XPOS_YPOS_NED = XPOS_ZPOS_YNEG;
    static constexpr uint8_t XNEG_ZPOS_YPOS_NED = ZPOS_XNEG_YNEG;
    static constexpr uint8_t ZNEG_XNEG_YPOS_NED = XNEG_ZNEG_YNEG;
    static constexpr uint8_t XPOS_ZNEG_YPOS_NED = ZNEG_XPOS_YNEG;

    static constexpr uint8_t ZPOS_XNEG_YNEG_NED = XNEG_ZPOS_YPOS;
    static constexpr uint8_t XNEG_ZNEG_YNEG_NED = ZNEG_XNEG_YPOS;
    static constexpr uint8_t ZNEG_XPOS_YNEG_NED = XPOS_ZNEG_YPOS;
    static constexpr uint8_t XPOS_ZPOS_YNEG_NED = ZPOS_XPOS_YPOS;

    static constexpr uint8_t TARGET_OUTPUT_DATA_RATE_MAX = 0;

    static constexpr uint8_t GYRO_FULL_SCALE_MAX = 0;
    static constexpr uint8_t GYRO_FULL_SCALE_125_DPS = 1;
    static constexpr uint8_t GYRO_FULL_SCALE_250_DPS = 2;
    static constexpr uint8_t GYRO_FULL_SCALE_500_DPS = 3;
    static constexpr uint8_t GYRO_FULL_SCALE_1000_DPS = 4;
    static constexpr uint8_t GYRO_FULL_SCALE_2000_DPS = 5;
    static constexpr uint8_t GYRO_FULL_SCALE_4000_DPS = 6;

    static constexpr uint8_t ACC_FULL_SCALE_MAX = 0;
    static constexpr uint8_t ACC_FULL_SCALE_1G = 1;
    static constexpr uint8_t ACC_FULL_SCALE_2G = 2;
    static constexpr uint8_t ACC_FULL_SCALE_4G = 3;
    static constexpr uint8_t ACC_FULL_SCALE_8G = 4;
    static constexpr uint8_t ACC_FULL_SCALE_16G = 5;
    static constexpr uint8_t ACC_FULL_SCALE_32G = 6;

    // Values for reporting gyro and acc type back to MSP (MultiWii Serial Protocol)
    static constexpr uint8_t MSP_GYRO_ID_NONE = 0;
    static constexpr uint8_t MSP_GYRO_ID_DEFAULT = 1;
    static constexpr uint8_t MSP_GYRO_ID_VIRTUAL = 20;
    static constexpr uint8_t MSP_ACC_ID_DEFAULT = 0;
    static constexpr uint8_t MSP_ACC_ID_NONE = 1;
    static constexpr uint8_t MSP_ACC_ID_VIRTUAL = 21;

    static constexpr uint8_t CALIBRATE_ACC_AND_GYRO = 0;
    static constexpr uint8_t CALIBRATE_GYRO_ONLY = 1;

    // IMU characteristics flag values
    static constexpr uint32_t IMU_AUTO_CALIBRATES = 0x01;
    static constexpr uint32_t IMU_PERFORMS_SENSOR_FUSION = 0x02;

    static constexpr float sin45f = 0.7071067811865475F;
    static constexpr float cos45f = 0.7071067811865475F;
    const std::array<Quaternion, 24> axisOrientations = {{
        Quaternion(  1.0F,    0.0F,    0.0F,    0.0F ),
        Quaternion(  sin45f,  0.0F,    0.0F,    sin45f ),
        Quaternion(  0.0F,    0.0F,    0.0F,    1.0F ),
        Quaternion(  sin45f,  0.0F,    0.0F,   -sin45f ),
        Quaternion(  0.0F,    0.0F,   -1.0F,    0.0F ),
        Quaternion(  0.0F,   -sin45f, -sin45f,  0.0F ),
        Quaternion(  0.0F,   -1.0F,    0.0F,    0.0F ),
        Quaternion(  0.0F,   -sin45f,  sin45f,  0.0F ),
        Quaternion(  0.0F,    0.0F,   -sin45f, sin45f ),
        Quaternion(  0.5F,   -0.5F,   -0.5F,    0.5F ),
        Quaternion(  sin45f, -sin45f,  0.0F,    0.0F ),
        Quaternion(  0.5F,   -0.5F,    0.5F,   -0.5F ),
        Quaternion(  sin45f, -sin45f,  0.0F,    0.0F ),
        Quaternion( -0.5F,   -0.5F,   -0.5F,   -0.5F ),
        Quaternion(  0.0F,    0.0F,   -sin45f, -sin45f ),
        Quaternion(  0.5F,    0.5F,   -0.5F,   -0.5F ),
        Quaternion( -0.5F,   -0.5F,   -0.5F,    0.5F ),
        Quaternion(  0.0F,   -sin45f,  0.0F,    sin45f ),
        Quaternion(  0.5F,   -0.5F,    0.5F,    0.5F ),
        Quaternion( -sin45f,  0.0F,   -sin45f,  0.0F ),
        Quaternion(  0.5F,    0.5F,   -0.5F,    0.5F ),
        Quaternion(  0.0F,   -sin45f,  0.0F,   -sin45f ),
        Quaternion(  0.5F,   -0.5F,   -0.5F,   -0.5F ),
        Quaternion(  sin45f,  0.0F,   -sin45f,  0.0F )
    }};
public:
    struct xyz_alignment_t {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct xyz_int32_t {
        int32_t x;
        int32_t y;
        int32_t z;
    };
    static constexpr float DEGREES_TO_RADIANS = static_cast<float>(3.14159265358979323846 / 180.0);
    static constexpr float RADIANS_TO_DEGREES = static_cast<float>(180.0 / 3.14159265358979323846);
public:
    static void delay_ms(int ms) { BusBase::delay_ms(ms); }
    virtual int init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) = 0;
    virtual int init(uint32_t target_output_data_rate_hz, void* bus_mutex) final;
    virtual int init(uint32_t target_output_data_rate_hz) final;
    virtual int init(void* bus_mutex) final;
    virtual int init() final;

    float get_gyro_resolution_dps() const { return _gyro_resolution_dps; }
    float get_acc_resolution() const { return _acc_resolution; }
    uint32_t get_gyro_sample_rate_hz() const { return _gyro_sample_rate_hz; }
    uint32_t get_acc_sample_rate_hz() const { return _acc_sample_rate_hz; }
    uint16_t get_gyro_id_msp() const { return _gyro_id_msp; }
    uint16_t get_acc_id_msp() const { return _acc_id_msp; }
    void calibrate(uint8_t calibration_type, size_t calibration_count);

    virtual void set_interrupt_driven();
    int32_t WAIT_IMU_DATA_READY() { return _bus_base->WAIT_DATA_READY(); }
    int32_t WAIT_IMU_DATA_READY(uint32_t ticksToWait) { return _bus_base->WAIT_DATA_READY(ticksToWait); }
    void SIGNAL_IMU_DATA_READY_FROM_ISR() { _bus_base->SIGNAL_DATA_READY_FROM_ISR(); } //! should not be used directly, made public for debugging purposes

    xyz_t get_gyro_offset() const;
    void set_gyro_offset(const xyz_t& gyro_offset);
    xyz_t get_acc_offset() const;
    void set_acc_offset(const xyz_t& acc_offset);
    xyz_t get_gyro_offset_mapped() const;
    void set_gyro_offset_mapped(const xyz_t& offset);
    xyz_t get_acc_offset_mapped() const;
    void set_acc_offset_mapped(const xyz_t& offset);

    virtual xyz_int32_t read_gyro_raw() = 0;
    virtual xyz_int32_t read_acc_raw() = 0;

    // read functions have default implementations in the base class for convenience,
    // but should be reimplemented in derived classes for efficiency
    virtual xyz_t read_gyro_rps();
    virtual xyz_t read_gyro_dps();
    virtual xyz_t read_acc();
    virtual acc_gyro_rps_t read_acc_gyro_rps();
    virtual acc_gyro_rps_t get_acc_gyro_rps() const;

    virtual Quaternion read_orientation();

    uint8_t get_axis_order() const { return _axis_order; }
    void set_axis_order(uint8_t axis_order) { _axis_order = axis_order; }
    static xyz_t map_axes(const xyz_t& data, uint8_t axis_order);
    xyz_t map_axes(const xyz_t& data) const { return map_axes(data, _axis_order); }
    static uint8_t axis_order_inverse(uint8_t axis_order);
    static xyz_alignment_t alignment_from_axis_order(uint8_t axis_order);
    static uint8_t axis_order_from_alignment(const xyz_alignment_t& alignment);

    uint8_t get_flags() const { return _flags; }
#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(LIBRARY_SENSORS_IMU_BUS_MUTEX_REQUIRED)
    void bus_semaphore_take() const { xSemaphoreTake(_bus_mutex, portMAX_DELAY); }
    void bus_semaphore_give() const { xSemaphoreGive(_bus_mutex); }
#else
    void bus_semaphore_take() const {}
    void bus_semaphore_give() const {}
#endif
    // functions to allow an IMU implementation to do run time checking if a mutex is required. Used by M5Stack implementations.
    void bus_semaphore_take(SemaphoreHandle_t bus_mutex) const { if (bus_mutex) {xSemaphoreTake(bus_mutex, portMAX_DELAY);}  }
    void bus_semaphore_give(SemaphoreHandle_t bus_mutex) const { if (bus_mutex) {xSemaphoreGive(bus_mutex);} }
    SemaphoreHandle_t _bus_mutex {};
#else
    void bus_semaphore_take() const {}
    void bus_semaphore_give() const {}
    void bus_semaphore_take(void* bus_mutex) const { (void)bus_mutex; }
    void bus_semaphore_give(void* bus_mutex) const { (void)bus_mutex; }
    void* _bus_mutex {};
#endif // FRAMEWORK_USE_FREERTOS
protected:
    BusBase* _bus_base;
    float _gyro_resolution_rps {};
    float _gyro_resolution_dps {};
    float _acc_resolution { 8.0F / 32768.0F };
    uint32_t _gyro_sample_rate_hz {};
    uint32_t _acc_sample_rate_hz {};
    xyz_t _gyro_offset {};
    xyz_t _acc_offset {};
    uint16_t _gyro_id_msp {};
    uint16_t _acc_id_msp {};
    uint8_t _axis_order;
    const uint8_t _flags; //!< Flags for describing IMU characteristics
};
