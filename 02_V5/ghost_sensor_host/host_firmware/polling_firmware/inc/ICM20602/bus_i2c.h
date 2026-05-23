/*
Original IMU driver code by Martin Budden: https://github.com/martinbudden/Library-Sensors
*/

#pragma once

#include "bus_base.h"
#include "I2CBus.h"

#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal_i2c.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal_i2c.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal_i2c.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal_i2c.h>
#endif
#endif

#if defined(FRAMEWORK_RPI_PICO)
typedef struct i2c_inst i2c_inst_t;
#elif defined(FRAMEWORK_ESPIDF)
#include <driver/i2c.h>
#include <driver/i2c_master.h>
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Wire.h>
#if defined(FRAMEWORK_ARDUINO_RPI_PICO)
#elif defined(FRAMEWORK_ARDUINO_ESP32)
#elif defined(FRAMEWORK_ARDUINO_STM32)
#endif // FRAMEWORK_ARDUINO
#endif // FRAMEWORK

#if !defined(IRAM_ATTR)
#define IRAM_ATTR
#endif


class BusI2c  : public BusBase {
public:
    struct i2c_pins_t {
        uint8_t sda;
        uint8_t scl;
        uint8_t irq;
    };
    struct stm32_i2c_pins_t {
        port_pin_t sda;
        port_pin_t scl;
        port_pin_t irq;
    };
public:
    BusI2c(uint8_t i2c_address, I2CBus* i2c);
    explicit BusI2c(uint8_t i2c_address) : BusI2c(i2c_address, nullptr) {}
    BusI2c(uint8_t i2c_address, I2CBus* i2c, const i2c_pins_t& pins);
    BusI2c(uint8_t i2c_address, const i2c_pins_t& pins) : BusI2c(i2c_address, nullptr, pins) {}

    BusI2c(uint8_t i2c_address, I2CBus* i2c, const stm32_i2c_pins_t& pins);
    BusI2c(uint8_t i2c_address, const stm32_i2c_pins_t& pins) : BusI2c(i2c_address, nullptr, pins) {}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) &&!defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
    BusI2c(uint8_t i2c_address, TwoWire& wire, const i2c_pins_t& pins);
#endif
public:
    void init();
    void set_interrupt_driven(uint8_t irq_level);
    bool read_device_data();
    uint8_t read_register(uint8_t reg) const;
    uint8_t read_register_with_timeout(uint8_t reg, uint32_t timeout_ms) const;
    bool read_register(uint8_t reg, uint8_t* data, size_t length) const;
    bool read_bytes(uint8_t* data, size_t length) const;
    bool read_bytes_with_timeout(uint8_t* data, size_t length, uint32_t timeout_ms) const;
    uint8_t write_register(uint8_t reg, uint8_t data);
    uint8_t write_register(uint8_t reg, const uint8_t* data, size_t length);
    uint8_t write_bytes(const uint8_t* data, size_t length);
private:
    static BusI2c* self; //!< alias of `this` to be used in interrupt service routine
    I2CBus* _i2c_ {};
    stm32_i2c_pins_t _pins {};
#if defined(FRAMEWORK_RPI_PICO)
    static constexpr bool RETAIN_CONTROL_OF_BUS = true;
    static constexpr bool DONT_RETAIN_CONTROL_OF_BUS = false;
    static void data_ready_isr(unsigned int gpio, uint32_t events);
#elif defined(FRAMEWORK_ESPIDF)
    static void data_ready_isr(); // cppcheck-suppress unusedPrivateFunction
    //i2c_master_bus_handle_t _bus_handle {};
    //i2c_master_dev_handle_t _dev_handle {};
#elif defined(FRAMEWORK_STM32_CUBE)
    static void data_ready_isr(); // cppcheck-suppress unusedPrivateFunction
    mutable I2C_HandleTypeDef _i2c {};
#elif defined(FRAMEWORK_TEST)
    static void data_ready_isr(); // cppcheck-suppress unusedPrivateFunction
#else // defaults to FRAMEWORK_ARDUINO
    static void data_ready_isr(); // cppcheck-suppress unusedPrivateFunction
    TwoWire& _wire;
#endif
    uint8_t _i2c_address {};
};
