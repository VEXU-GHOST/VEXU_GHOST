#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include "hardware/i2c.h"
#include "isl29124.h"
#include "TCA9536.h"
#include "platform.h"      // VL53L4CD (from inc/VL53L4CD/)
#include "imu_icm20602.h"  // ImuIcm20602, ImuBase, xyz_t, acc_gyro_rps_t
#include "bus_i2c.h"       // BusI2c::i2c_pins_t
#include "I2CBus.h"

// ---- Device type identifier -------------------------------------------------

enum class DeviceType : uint8_t {
    ISL29124 = 0,   // RGB light sensor
    ICM20602 = 1,   // 6-axis IMU
    VL53L4CD = 2,   // ToF distance sensor
    TCA9536  = 3    // 4-bit GPIO expander
};

// ---- Sensor data templates --------------------------------------------------
// Primary template is intentionally undefined; only specializations are valid.

template<DeviceType T>
struct SensorData;

template<>
struct SensorData<DeviceType::ISL29124> {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint32_t lux;
    uint32_t cct;
    bool     valid = false;
};

template<>
struct SensorData<DeviceType::ICM20602> {
    xyz_t acc;          // g-units (9.80665 m/s² per g)
    xyz_t gyro_rps;     // rad/s
    float temperature;  // °C
    bool  valid = false;
};

template<>
struct SensorData<DeviceType::VL53L4CD> {
    uint16_t distance_mm;
    uint8_t  range_status;
    uint16_t sigma_mm;
    uint32_t signal_rate_kcps;
    uint32_t ambient_rate_kcps;
    bool     valid = false;
};

template<>
struct SensorData<DeviceType::TCA9536> {
    uint8_t gpio_state;  // raw 4-bit port value (pins 0–3)
    bool    valid = false;
};

// ---- Abstract base Device class ---------------------------------------------

class Device {
protected:
    const uint8_t  device_id;
    const uint8_t  i2c_addr;
    I2CBus         *i2c_bus;
    bool           initialized = false;

    Device(uint8_t device_id, uint8_t i2c_addr, I2CBus *i2c_bus)
        : device_id(device_id), i2c_addr(i2c_addr), i2c_bus(i2c_bus) {}

public:
    virtual ~Device() = default;

    virtual bool       init()           = 0;
    virtual DeviceType get_type() const = 0;

    // Stops the hardware and marks the device uninitialized.
    // Each concrete class overrides to perform device-specific shutdown.
    virtual bool destroy() { initialized = false; return true; }

    // // Convenience: destroy() then init(). Override if a cold-reset sequence
    // // differs from a plain destroy + re-init.
    // virtual bool reset() { return destroy() && init(); }

    uint8_t      get_device_id()  const { return device_id; }
    uint8_t      get_i2c_addr()   const { return i2c_addr; }
    I2CBus       *get_i2c_bus()   const { return i2c_bus; }
    bool         is_initialized() const { return initialized; }

    // Factory. Returns nullptr for ICM20602; use ICM20602Device::create() instead
    // (ICM20602 requires SDA/SCL pin numbers that are not part of the common interface).
    static std::unique_ptr<Device> create(DeviceType type, uint8_t i2c_addr, I2CBus *i2c_bus);
};

// ---- Typed intermediate base ------------------------------------------------
// Binds the return type of get_data() to the correct SensorData specialization.

template<DeviceType TYPE>
class TypedDevice : public Device {
protected:
    TypedDevice(uint8_t i2c_addr, I2CBus *i2c_bus)
        : Device(static_cast<uint8_t>(TYPE), i2c_addr, i2c_bus) {}

public:
    DeviceType get_type() const override { return TYPE; }
    virtual SensorData<TYPE> get_data() = 0;
};

// ---- Concrete device classes ------------------------------------------------

class ISL29124Device : public TypedDevice<DeviceType::ISL29124> {
    ISL29124 sensor_;
public:
    ISL29124Device(uint8_t i2c_addr, I2CBus *i2c_bus)
        : TypedDevice(i2c_addr, i2c_bus) {}

    bool                             init()     override;
    bool                             destroy()  override;
    SensorData<DeviceType::ISL29124> get_data() override;
};

class ICM20602Device : public TypedDevice<DeviceType::ICM20602> {
    std::optional<ImuIcm20602> sensor_;
    uint8_t sda_pin_;
    uint8_t scl_pin_;
public:
    ICM20602Device(uint8_t i2c_addr, I2CBus *i2c_bus, uint8_t sda_pin, uint8_t scl_pin)
        : TypedDevice(i2c_addr, i2c_bus), sda_pin_(sda_pin), scl_pin_(scl_pin) {}

    bool                             init()     override;
    bool                             destroy()  override;
    SensorData<DeviceType::ICM20602> get_data() override;

    static std::unique_ptr<ICM20602Device> create(uint8_t i2c_addr, I2CBus *i2c_bus,
                                                   uint8_t sda_pin, uint8_t scl_pin);
};

class VL53L4CDDevice : public TypedDevice<DeviceType::VL53L4CD> {
    VL53L4CD sensor_;
public:
    VL53L4CDDevice(uint8_t i2c_addr, I2CBus *i2c_bus)
        : TypedDevice(i2c_addr, i2c_bus) {}

    bool                             init()     override;
    bool                             destroy()  override;
    SensorData<DeviceType::VL53L4CD> get_data() override;
};

class TCA9536Device : public TypedDevice<DeviceType::TCA9536> {
    TCA9536 sensor_;
public:
    TCA9536Device(uint8_t i2c_addr, I2CBus *i2c_bus)
        : TypedDevice(i2c_addr, i2c_bus) {}

    bool                            init()     override;
    SensorData<DeviceType::TCA9536> get_data() override;

    // GPIO-specific operations not part of the common polling API.
    bool    pin_mode(uint8_t pin, uint8_t mode);
    bool    digital_write(uint8_t pin, uint8_t value);
    uint8_t digital_read(uint8_t pin);
};
