/*
Original IMU driver code by Martin Budden: https://github.com/martinbudden/Library-Sensors
*/

#pragma once

#include "bus_i2c.h"
#include "bus_spi.h"
#include "imu_base.h"


class ImuIcm20602 : public ImuBase {
public:
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_ImuIcm20602_USE_SPI_BUS)
    // SPI constructors
    ImuIcm20602(uint8_t axis_order, uint32_t frequency, uint8_t spi_index, const BusSpi::stm32_spi_pins_t& pins);
    ImuIcm20602(uint8_t axis_order, uint32_t frequency, uint8_t spi_index, const BusSpi::spi_pins_t& pins);
#else
    // I2C constructors
    ImuIcm20602(uint8_t axis_order, I2CBus* i2c, const BusI2c::stm32_i2c_pins_t& pins, uint8_t i2c_address);
    ImuIcm20602(uint8_t axis_order, const BusI2c::stm32_i2c_pins_t& pins, uint8_t i2c_address) : ImuIcm20602(axis_order, nullptr, pins, i2c_address) {}
    ImuIcm20602(uint8_t axis_order, const BusI2c::stm32_i2c_pins_t& pins) : ImuIcm20602(axis_order, pins, I2C_ADDRESS) {}

    ImuIcm20602(uint8_t axis_order, I2CBus* i2c, const BusI2c::i2c_pins_t& pins, uint8_t i2c_address);
    ImuIcm20602(uint8_t axis_order, const BusI2c::i2c_pins_t& pins, uint8_t i2c_address) : ImuIcm20602(axis_order, nullptr, pins, i2c_address) {}
    ImuIcm20602(uint8_t axis_order, const BusI2c::i2c_pins_t& pins) : ImuIcm20602(axis_order, pins, I2C_ADDRESS) {}
#endif
    virtual int init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) override;
public:
    static constexpr uint8_t I2C_ADDRESS = 0x68;
#pragma pack(push, 1)
    union mems_sensor_data_t {
        static constexpr size_t DATA_SIZE = 6;
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            uint8_t x_h;
            uint8_t x_l;
            uint8_t y_h;
            uint8_t y_l;
            uint8_t z_h;
            uint8_t z_l;
        } value;
    };
private:
    union acc_temperature_gyro_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        static constexpr size_t DATA_SIZE = 14;
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            uint8_t acc_x_h;
            uint8_t acc_x_l;
            uint8_t acc_y_h;
            uint8_t acc_y_l;
            uint8_t acc_z_h;
            uint8_t acc_z_l;
            uint8_t temperature_h;
            uint8_t temperature_l;
            uint8_t gyro_x_h;
            uint8_t gyro_x_l;
            uint8_t gyro_y_h;
            uint8_t gyro_y_l;
            uint8_t gyro_z_h;
            uint8_t gyro_z_l;
        } value;
    };
#pragma pack(pop)
    struct spi_acc_temperature_gyro_data_t {
        std::array<uint8_t, BusBase::SPI_PRE_READ_BUFFER_SIZE> pre_read_buffer; // buffer for the transmit byte sent as part of a read
        acc_temperature_gyro_data_t accGyro;
    };
public:
    virtual void set_interrupt_driven() override;

    virtual xyz_int32_t read_gyro_raw() override;
    virtual xyz_int32_t read_acc_raw() override;

    virtual xyz_t read_gyro_rps() override;
    virtual xyz_t read_gyro_dps() override;
    virtual xyz_t read_acc() override;
    virtual acc_gyro_rps_t read_acc_gyro_rps() override;
    virtual acc_gyro_rps_t get_acc_gyro_rps() const override;

    float read_temperature() const;
    int32_t read_temperature_raw() const;
private:
    xyz_t gyro_rps_from_raw(const mems_sensor_data_t::value_t& data) const;
    xyz_t acc_from_raw(const mems_sensor_data_t::value_t& data) const;
    acc_gyro_rps_t acc_gyro_rps_from_raw(const acc_temperature_gyro_data_t::value_t& data) const;
private:
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_ImuIcm20602_USE_SPI_BUS)
    BusSpi _bus; //!< SPI bus interface
#else
    BusI2c _bus; //!< I2C bus interface
#endif
    spi_acc_temperature_gyro_data_t _spi_acc_temperature_gyro_data {};
};
