/*
Original IMU driver code by Martin Budden: https://github.com/martinbudden/Library-Sensors
*/

#include "bus_i2c.h"

#include <cassert>
#if defined(FRAMEWORK_RPI_PICO)
//#include <boards/pico.h> // for PICO_DEFAULT_LED_PIN
#include <hardware/i2c.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdio.h>
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#endif


BusI2c* BusI2c::self {nullptr};

/*!
Data ready interrupt service routine (ISR)

Currently support only one interrupt, but could index action off the gpio pin
*/
#if defined(FRAMEWORK_RPI_PICO)
void __not_in_flash_func(BusI2c::data_ready_isr)(unsigned int gpio, uint32_t events) // NOLINT(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
{
    (void)gpio;
    (void)events;
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
    // reading the IMU register resets the interrupt
    self->read_device_data();
    self->SIGNAL_DATA_READY_FROM_ISR();
}
#else
FAST_CODE void BusI2c::data_ready_isr()
{
    // reading the IMU register resets the interrupt
    self->read_device_data();
    self->SIGNAL_DATA_READY_FROM_ISR();
}
#endif

BusI2c::BusI2c(uint8_t i2c_address, I2CBus* i2c, const stm32_i2c_pins_t& pins) :
    _i2c_(i2c),
    _pins(pins),
#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    _wire(Wire),
#endif
    _i2c_address(i2c_address)
{
#if defined(FRAMEWORK_STM32_CUBE)
    _i2c.Instance = (i2c_index == BUS_INDEX_1) ? I2C2 : I2C1;
#endif
    init();
}

BusI2c::BusI2c(uint8_t i2c_address, I2CBus* i2c, const i2c_pins_t& pins) :
    _i2c_(i2c),
#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    _wire(Wire),
#endif
    _i2c_address(i2c_address)
{
#if defined(FRAMEWORK_STM32_CUBE)
    _i2c.Instance = (i2c_index == BUS_INDEX_1) ? I2C2 : I2C1;
#endif

    _pins.sda = {0,pins.sda};
    _pins.scl = {0,pins.scl};
    _pins.irq = {0,pins.irq};
    init();
}

void BusI2c::init() // NOLINT(readability-make-member-function-const)
{
#if defined(FRAMEWORK_RPI_PICO)
    static_assert(static_cast<int>(IRQ_LEVEL_LOW) == GPIO_IRQ_LEVEL_LOW);
    static_assert(static_cast<int>(IRQ_LEVEL_HIGH) == GPIO_IRQ_LEVEL_HIGH);
    static_assert(static_cast<int>(IRQ_EDGE_FALL) == GPIO_IRQ_EDGE_FALL);
    static_assert(static_cast<int>(IRQ_EDGE_RISE) == GPIO_IRQ_EDGE_RISE);

    // enum { BUS_400_KHZ = 400000 };
    // i2c_init(_i2c, BUS_400_KHZ);
    // gpio_set_function(_pins.sda.pin, GPIO_FUNC_I2C); // PICO_DEFAULT_I2C_SDA_PIN is GP4
    // gpio_set_function(_pins.scl.pin, GPIO_FUNC_I2C); // PICO_DEFAULT_I2C_SCL_PIN is GP5
    // gpio_pull_up(_pins.sda.pin);
    // gpio_pull_up(_pins.scl.pin);
    // Make the I2C pins _available to pictooof
    // bi_decl(bi_2pins_with_func(_pins.sda.pin, _pins.scl.pin, GPIO_FUNC_I2C));

#elif defined(FRAMEWORK_ESPIDF)
/*
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = 0,//TEST_I2C_PORT,
        .sda_io_num = static_cast<gpio_num_t>(_pins.sda.pin),
        .scl_io_num = static_cast<gpio_num_t>(_pins.scl.pin),
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags {
            .enable_internal_pullup = true,
            .allow_pd = true
        }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &_bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _i2c_address,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags {
            .disable_ack_check = true
        }
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus_handle, &dev_cfg, &_dev_handle));
*/
#elif defined(FRAMEWORK_STM32_CUBE)

    // STM32 indices are 1-based
    if (_i2c_index == BUS_INDEX_0) {
        _i2c.Instance = I2C1;
#if defined(I2C2)
    } else if (_i2c_index == BUS_INDEX_1) {
        _i2c.Instance = I2C2;
#endif
#if defined(I2C3)
    } else if (_i2c_index == BUS_INDEX_2) {
        _i2c.Instance = I2C3;
#endif
#if defined(I2C4)
    } else if (_i2c_index == BUS_INDEX_3) {
        _i2c.Instance = I2C4;
#endif
    }
    if (gpioPort(_pins.sda) == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
#if defined(GPIOB)
    if (gpioPort(_pins.sda) == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
#endif
#if defined(GPIOC)
    if (gpioPort(_pins.sda) == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
#endif
#if defined(GPIOD)
    if (gpioPort(_pins.sda) == GPIOD) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
#endif
#if defined(GPIOE)
    if (gpioPort(_pins.sda) == GPIOE) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
#endif
#if defined(FRAMEWORK_STM32_CUBE_F1)
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#elif defined(FRAMEWORK_STM32_CUBE_F4)
    _i2c.Init.ClockSpeed = 100000;
    _i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    _i2c.Init.OwnAddress1 = 0;
    _i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    _i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    _i2c.Init.OwnAddress2 = 0;
    _i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    _i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&_i2c);
#endif

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO

#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
    _wire.begin(_pins.sda.pin, _pins.scl.pin);
#else
    _wire.begin();
#endif

#if defined(FRAMEWORK_USE_FREERTOS)
    _data_ready_queue = xQueueCreateStatic(IMU_DATA_READY_QUEUE_LENGTH, sizeof(_data_ready_queue_item), &_data_ready_queueStorageArea[0], &_data_ready_queue_static);
    configASSERT(_data_ready_queue);
    const UBaseType_t message_count = uxQueueMessagesWaiting(_data_ready_queue);
    assert(message_count == 0);
    (void)message_count;
#endif

#endif // FRAMEWORK
}

#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
BusI2c::BusI2c(uint8_t i2c_address, TwoWire& wire, const i2c_pins_t& pins) :
    _i2c_index(BUS_INDEX_0),
    _wire(wire),
    _i2c_address(i2c_address)
{
    _pins.sda = {0,pins.sda};
    _pins.scl = {0,pins.scl};
    _pins.irq = {0,pins.irq};
#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
    _wire.begin(pins.sda, pins.scl);
#else
    _wire.begin();
#endif
#if defined(FRAMEWORK_USE_FREERTOS)
    _data_ready_queue = xQueueCreateStatic(IMU_DATA_READY_QUEUE_LENGTH, sizeof(_data_ready_queue_item), &_data_ready_queueStorageArea[0], &_data_ready_queue_static);
    configASSERT(_data_ready_queue);
    const UBaseType_t message_count = uxQueueMessagesWaiting(_data_ready_queue);
    assert(message_count == 0);
    (void)message_count;
#endif
}
#endif

BusI2c::BusI2c(uint8_t i2c_address, I2CBus* i2c)
#if defined(FRAMEWORK_RPI_PICO)
    : BusI2c(i2c_address, i2c, i2c_pins_t{.sda=PICO_DEFAULT_I2C_SDA_PIN, .scl=PICO_DEFAULT_I2C_SCL_PIN, .irq=IRQ_NOT_SET})
#elif defined(FRAMEWORK_ESPIDF)
    : BusI2c(i2c_address, i2c_index, i2c_pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET})
#elif defined(FRAMEWORK_STM32_CUBE)
    : BusI2c(i2c_address, i2c_index, i2c_pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET})
#elif defined(FRAMEWORK_TEST)
    : BusI2c(i2c_address, i2c_index, i2c_pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET})
#else // defaults to FRAMEWORK_ARDUINO
    : BusI2c(i2c_address, i2c_index, i2c_pins_t{.sda=0, .scl=0, .irq=IRQ_NOT_SET})
#endif // FRAMEWORK
{
}

void BusI2c::set_interrupt_driven(uint8_t irq_level) // NOLINT(readability-make-member-function-const)
{
    self = this;

    assert(_pins.irq.pin != IRQ_NOT_SET);

#if defined(FRAMEWORK_RPI_PICO)
    gpio_init(_pins.irq.pin);
    enum { IRQ_ENABLED = true };
    gpio_set_irq_enabled_with_callback(_pins.irq.pin, irq_level, IRQ_ENABLED, &data_ready_isr);
#elif defined(FRAMEWORK_ARDUINO_ESP32)
    //pinMode(_pins.irq.pin, INPUT);
    // map to ESP32 constants
    enum { LEVEL_LOW = 0x04, LEVEL_HIGH = 0x05, EDGE_FALL = 0x02, EDGE_RISE = 0x01, EDGE_CHANGE = 0x03 };
    const uint8_t level =
        (irq_level == IRQ_LEVEL_LOW) ? LEVEL_LOW :
        (irq_level == IRQ_LEVEL_HIGH) ? LEVEL_HIGH :
        (irq_level == IRQ_EDGE_FALL) ? EDGE_FALL :
        (irq_level == IRQ_EDGE_RISE) ? EDGE_RISE : EDGE_CHANGE;
    (void)level;
    //attachInterrupt(digitalPinToInterrupt(_pins.irq), &data_ready_isr, irq_level); // esp32-hal-gpio.h
#else
    enum { LEVEL_LOW = 0x04, LEVEL_HIGH = 0x05, EDGE_FALL = 0x02, EDGE_RISE = 0x01, EDGE_CHANGE = 0x03 };
    const uint8_t level =
        (irq_level == IRQ_LEVEL_LOW) ? LEVEL_LOW :
        (irq_level == IRQ_LEVEL_HIGH) ? LEVEL_HIGH :
        (irq_level == IRQ_EDGE_FALL) ? EDGE_FALL :
        (irq_level == IRQ_EDGE_RISE) ? EDGE_RISE : EDGE_CHANGE;
    (void)level;
#endif
}

FAST_CODE uint8_t BusI2c::read_register(uint8_t reg) const
{
    uint8_t ret = 0; // NOLINT(misc-const-correctness)
#if defined(FRAMEWORK_RPI_PICO)
    _i2c_->write(_i2c_address, &reg, 1, RETAIN_CONTROL_OF_BUS);
    _i2c_->read(_i2c_address, &ret, 1, DONT_RETAIN_CONTROL_OF_BUS);
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
#elif defined(FRAMEWORK_STM32_CUBE)
    HAL_I2C_Master_Transmit(&_i2c, _i2c_address, &reg, 1, HAL_MAX_DELAY); //Sending in Blocking mode
    HAL_I2C_Master_Receive (&_i2c, _i2c_address, &ret, 1, HAL_MAX_DELAY);
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_i2c_address);
    _wire.write(reg);
    _wire.endTransmission();

    if (_wire.requestFrom(_i2c_address, 1U)) {
        return static_cast<uint8_t>(_wire.read());
    }
#endif
    return ret;
}

FAST_CODE uint8_t BusI2c::read_register_with_timeout(uint8_t reg, uint32_t timeout_ms) const
{
    uint8_t ret = 0; // NOLINT(misc-const-correctness)
#if defined(FRAMEWORK_RPI_PICO)
    _i2c_->write(_i2c_address, &reg, 1, RETAIN_CONTROL_OF_BUS);
    enum { MILLISECONDS_TO_MICROSECONDS = 1000 };
    _i2c_->read_timeout_us(_i2c_address, &ret, 1, DONT_RETAIN_CONTROL_OF_BUS, timeout_ms * MILLISECONDS_TO_MICROSECONDS);
    return ret;
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)timeout_ms;
#elif defined(FRAMEWORK_STM32_CUBE)
    HAL_I2C_Master_Transmit(&_i2c,_i2c_address, &reg, 1, timeout_ms); //Sending in Blocking mode
    HAL_I2C_Master_Receive (&_i2c,_i2c_address, &ret, 1, timeout_ms);
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)timeout_ms;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_i2c_address);
    _wire.write(reg);
    _wire.endTransmission(false);

    _wire.requestFrom(_i2c_address, 1U);
    for (uint32_t ii = 0; ii < timeout_ms; ++ii) {
        if (_wire.available() > 0) {
            return static_cast<uint8_t>(_wire.read());
        }
        delay(1);
    }
#endif
    return ret;
}

FAST_CODE bool BusI2c::read_device_data()
{
    return read_register(_device_data_register, _device_read_buf, _device_read_length);
}

FAST_CODE bool BusI2c::read_register(uint8_t reg, uint8_t* data, size_t length) const // NOLINT(readability-non-const-parameter)
{
#if defined(FRAMEWORK_RPI_PICO)
    int bits_written = _i2c_->write(_i2c_address, &reg, 1, RETAIN_CONTROL_OF_BUS);
    const int bytes_read = _i2c_->read(_i2c_address, data, length, DONT_RETAIN_CONTROL_OF_BUS);
    if (bytes_read <= 0) {
        return false;
    }
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_STM32_CUBE)
    HAL_I2C_Master_Transmit(&_i2c,_i2c_address, &reg, 1, HAL_MAX_DELAY); //Sending in Blocking mode
    const HAL_StatusTypeDef status = HAL_I2C_Master_Receive (&_i2c,_i2c_address, data, length, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return false;
    }
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_i2c_address);
    _wire.write(reg);
    const uint8_t err = _wire.endTransmission(false);
    if (err == 0) {
        return read_bytes(data, length);
    }
#endif
    return false;
}

FAST_CODE bool BusI2c::read_bytes(uint8_t* data, size_t length) const // NOLINT(readability-non-const-parameter)
{
#if defined(FRAMEWORK_RPI_PICO)
    _i2c_->read(_i2c_address, data, length, false);
    return true;
#elif defined(FRAMEWORK_ESPIDF)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_STM32_CUBE)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    if (_wire.requestFrom(_i2c_address, static_cast<uint8_t>(length))) {
        //while (_wire._available() < 1) { delay(1); }
        for (size_t ii = 0; ii < length; ++ii) {
            data[ii] = static_cast<uint8_t>(_wire.read()); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
        return true;
    }
#endif
    return false;
}

FAST_CODE bool BusI2c::read_bytes_with_timeout(uint8_t* data, size_t length, uint32_t timeout_ms) const // NOLINT(readability-non-const-parameter)
{
#if defined(FRAMEWORK_RPI_PICO)
    enum { MILLISECONDS_TO_MICROSECONDS = 1000 };
    _i2c_->read_timeout_us(_i2c_address, data, length, false, timeout_ms * MILLISECONDS_TO_MICROSECONDS);
    return true;
#elif defined(FRAMEWORK_ESPIDF)
    *data = 0;
    (void)length;
    (void)timeout_ms;
#elif defined(FRAMEWORK_STM32_CUBE)
    const HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&_i2c,_i2c_address, data, length, timeout_ms);
    if (status != HAL_OK) {
        return false;
    }
#elif defined(FRAMEWORK_TEST)
    (void)data;
    (void)length;
    (void)timeout_ms;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.requestFrom(_i2c_address, static_cast<uint8_t>(length));
    for (uint32_t ii = 0; ii < timeout_ms; ++ii) {
        if (_wire.available() > 0) {
            for (size_t jj = 0; jj < length; ++jj) {
                data[jj] = static_cast<uint8_t>(_wire.read()); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            }
            return true;
        }
        delay(1);
    }
    return false;
#endif
    return false;
}

FAST_CODE uint8_t BusI2c::write_register(uint8_t reg, uint8_t data)
{
#if defined(FRAMEWORK_RPI_PICO)
    std::array<uint8_t, 2> buf = {{ reg, data }};
    return _i2c_->write(_i2c_address, &buf[0], sizeof(buf), false);
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
#elif defined(FRAMEWORK_STM32_CUBE)
    std::array<uint8_t, 2> buf = { reg, data };
    HAL_I2C_Master_Transmit(&_i2c,_i2c_address, &buf[0], sizeof(buf), HAL_MAX_DELAY); //Sending in Blocking mode
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_i2c_address);
    _wire.write(reg);
    _wire.write(data);
    return _wire.endTransmission();
#endif
    return 0;
}

FAST_CODE uint8_t BusI2c::write_register(uint8_t reg, const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_RPI_PICO)
    _i2c_->write(_i2c_address, &reg, 1, false);
    return _i2c_->write(_i2c_address, data, length, false);
#elif defined(FRAMEWORK_ESPIDF)
    (void)reg;
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_STM32_CUBE)
    HAL_I2C_Master_Transmit(&_i2c,_i2c_address, &reg, 1, HAL_MAX_DELAY); //Sending in Blocking mode
    HAL_I2C_Master_Transmit(&_i2c,_i2c_address, const_cast<uint8_t*>(data), length, HAL_MAX_DELAY); //Sending in Blocking mode
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_i2c_address);
    _wire.write(reg);
    _wire.write(data, length);
    return _wire.endTransmission();
#endif
    return 0;
}

FAST_CODE uint8_t BusI2c::write_bytes(const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_RPI_PICO)
    return _i2c_->write(_i2c_address, data, length, false);
#elif defined(FRAMEWORK_ESPIDF)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_STM32_CUBE)
    HAL_I2C_Master_Transmit(&_i2c,_i2c_address, const_cast<uint8_t*>(data), length, HAL_MAX_DELAY); //Sending in Blocking mode
#elif defined(FRAMEWORK_TEST)
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _wire.beginTransmission(_i2c_address);
    _wire.write(data, length);
    return _wire.endTransmission();
#endif
    return 0;
}
