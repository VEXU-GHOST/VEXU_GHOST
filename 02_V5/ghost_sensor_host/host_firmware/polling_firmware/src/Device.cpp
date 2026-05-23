#include "Device.h"
#include "bus_base.h"  // BusBase::BUS_INDEX_0/1

// ---- Factory ----------------------------------------------------------------

std::unique_ptr<Device> Device::create(DeviceType type, uint8_t i2c_addr, I2CBus *i2c_bus) {
    switch (type) {
        case DeviceType::ISL29124: return std::make_unique<ISL29124Device>(i2c_addr, i2c_bus);
        case DeviceType::VL53L4CD: return std::make_unique<VL53L4CDDevice>(i2c_addr, i2c_bus);
        case DeviceType::TCA9536:  return std::make_unique<TCA9536Device>(i2c_addr, i2c_bus);
        case DeviceType::ICM20602: return nullptr;  // use ICM20602Device::create() instead
        default:                   return nullptr;
    }
}

// ---- ISL29124Device ---------------------------------------------------------

bool ISL29124Device::init() {
    initialized = sensor_.init(i2c_bus, i2c_addr);
    return initialized;
}

bool ISL29124Device::destroy() {
    sensor_.set_mode(RGB_OP_PWDN_MODE_SET);
    initialized = false;
    return true;
}

SensorData<DeviceType::ISL29124> ISL29124Device::get_data() {
    SensorData<DeviceType::ISL29124> data{};
    if (!initialized) return data;
    if (sensor_.read_rgb(data.r, data.g, data.b) < 0) return data;
    sensor_.autorange(data.g);
    int cct = 0;
    data.lux   = sensor_.cal_lux(cct);
    data.cct   = static_cast<uint32_t>(cct);
    data.valid = true;
    return data;
}

// ---- ICM20602Device ---------------------------------------------------------

bool ICM20602Device::init() {
    sensor_.emplace(ImuBase::XPOS_YPOS_ZPOS,
                    i2c_bus,
                    BusI2c::i2c_pins_t{.sda = sda_pin_, .scl = scl_pin_, .irq = BusI2c::IRQ_NOT_SET},
                    i2c_addr);
    int status = sensor_->init(ImuBase::TARGET_OUTPUT_DATA_RATE_MAX,
                               ImuBase::GYRO_FULL_SCALE_MAX,
                               ImuBase::ACC_FULL_SCALE_MAX, nullptr);
    initialized = (status == 500);
    return initialized;
}

bool ICM20602Device::destroy() {
    sensor_.reset();  // destroys the ImuIcm20602 in-place object
    initialized = false;
    return true;
}

SensorData<DeviceType::ICM20602> ICM20602Device::get_data() {
    SensorData<DeviceType::ICM20602> data{};
    if (!initialized || !sensor_) return data;
    const acc_gyro_rps_t raw = sensor_->read_acc_gyro_rps();
    data.acc         = raw.acc;
    data.gyro_rps    = raw.gyro_rps * ImuBase::RADIANS_TO_DEGREES;
    data.temperature = sensor_->read_temperature();
    data.valid       = true;
    return data;
}

std::unique_ptr<ICM20602Device> ICM20602Device::create(uint8_t i2c_addr, I2CBus *i2c_bus,
                                                        uint8_t sda_pin, uint8_t scl_pin) {
    return std::make_unique<ICM20602Device>(i2c_addr, i2c_bus, sda_pin, scl_pin);
}

// ---- VL53L4CDDevice ---------------------------------------------------------

bool VL53L4CDDevice::init() {
    if (sensor_.init(i2c_bus, i2c_addr) != VL53L4CD_ERROR_NONE) return false;
    sensor_.SetRangeTiming(50, 0);
    sensor_.StartRanging();
    initialized = true;
    return true;
}

bool VL53L4CDDevice::destroy() {
    sensor_.StopRanging();
    initialized = false;
    return true;
}

SensorData<DeviceType::VL53L4CD> VL53L4CDDevice::get_data() {
    SensorData<DeviceType::VL53L4CD> data{};
    if (!initialized) return data;
    uint8_t data_ready = 0;
    sensor_.CheckForDataReady(&data_ready);
    if (!data_ready) return data;
    VL53L4CD_ResultsData_t result{};
    if (sensor_.GetResult(&result) != VL53L4CD_ERROR_NONE) return data;
    sensor_.ClearInterrupt();
    data.distance_mm       = result.distance_mm;
    data.range_status      = result.range_status;
    data.sigma_mm          = result.sigma_mm;
    data.signal_rate_kcps  = result.signal_rate_kcps;
    data.ambient_rate_kcps = result.ambient_rate_kcps;
    data.valid             = true;
    return data;
}

// ---- TCA9536Device ----------------------------------------------------------

bool TCA9536Device::init() {
    initialized = (sensor_.TCA9536_init(i2c_bus, i2c_addr) == TCA9536_ERROR_SUCCESS);
    return initialized;
}

SensorData<DeviceType::TCA9536> TCA9536Device::get_data() {
    SensorData<DeviceType::TCA9536> data{};
    if (!initialized) return data;
    data.gpio_state = sensor_.readReg();
    data.valid      = true;
    return data;
}

bool TCA9536Device::pin_mode(uint8_t pin, uint8_t mode) {
    return sensor_.pinMode(pin, mode) == TCA9536_ERROR_SUCCESS;
}

bool TCA9536Device::digital_write(uint8_t pin, uint8_t value) {
    return sensor_.digitalWrite(pin, value) == TCA9536_ERROR_SUCCESS;
}

uint8_t TCA9536Device::digital_read(uint8_t pin) {
    return sensor_.read(pin);
}
