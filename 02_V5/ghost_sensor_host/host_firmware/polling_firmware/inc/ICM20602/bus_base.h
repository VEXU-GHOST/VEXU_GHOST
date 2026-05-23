/*
Original IMU driver code by Martin Budden: https://github.com/martinbudden/Library-Sensors
*/

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#define FRAMEWORK_RPI_PICO

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#endif
#endif

#if defined(FRAMEWORK_RPI_PICO)

#elif defined(FRAMEWORK_ESPIDF)

#if !defined(FAST_CODE)
#define FAST_CODE IRAM_ATTR
#endif
#if !defined(IRAM_ATTR)
#define IRAM_ATTR
#endif

#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)

#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_gpio.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_gpio.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_gpio.h>
#endif

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_RPI_PICO)
#elif defined(FRAMEWORK_ARDUINO_ESP32)
#endif // FRAMEWORK_ARDUINO

#endif // FRAMEWORK

#if !defined(FAST_CODE)
#define FAST_CODE
#endif


/*!
Base class for BUS_I2C and BUS_SPI.

Note
bus_index is zero-based.

RPI Pico bus index is zero-based so, for SPI, BUS_INDEX_0 corresponds to spi0

ESPIDF bus index is one-based so, for SPI, BUS_INDEX_0 corresponds to SPI1_HOST

STM32 bus index is one-based so, for SPI, BUS_INDEX_0 corresponds to SPI1
*/
class BusBase {
public:
    static constexpr uint8_t BUS_INDEX_0 = 0;
    static constexpr uint8_t BUS_INDEX_1 = 1;
    static constexpr uint8_t BUS_INDEX_2 = 2;
    static constexpr uint8_t BUS_INDEX_3 = 3;
    static constexpr uint8_t BUS_INDEX_4 = 4;
    static constexpr uint8_t BUS_INDEX_5 = 5;
    static constexpr uint8_t BUS_INDEX_6 = 6;
    static constexpr uint8_t BUS_INDEX_7 = 7;
    static constexpr uint8_t READ_BIT = 0x80U;
    static constexpr uint8_t SPI_PRE_READ_BUFFER_SIZE = 4; // to reserve space for the transmit instruction that occurs before a read
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    static constexpr uint8_t SPI_PRE_READ_BUFFER_OFFSET = SPI_PRE_READ_BUFFER_SIZE;
#else
    static constexpr uint8_t SPI_PRE_READ_BUFFER_OFFSET = SPI_PRE_READ_BUFFER_SIZE - 1;
#endif
    static constexpr uint8_t IRQ_NOT_SET = 0xFF;
    static constexpr uint8_t IRQ_LEVEL_LOW = 0x01;
    static constexpr uint8_t IRQ_LEVEL_HIGH = 0x02;
    static constexpr uint8_t IRQ_EDGE_FALL = 0x04;
    static constexpr uint8_t IRQ_EDGE_RISE = 0x08;
    static constexpr uint8_t IRQ_EDGE_CHANGE = 0x04|0x08;
    struct port_pin_t {
        uint8_t port;
        uint8_t pin;
    };
public:
    /*!
    The device_data_register is the register that is read to provide the data of primary interest.
    This sets up a pre-read buffer for the register, so the register can be read using a single SPI transmit/receive instruction.
    This can be used, in particular, to simplify DMA on some architectures.
    */
    void set_device_data_register(uint8_t device_data_register, uint8_t* read_buf, size_t read_length) {
        _device_data_register = device_data_register | READ_BIT;
        _device_read_buf = read_buf + SPI_PRE_READ_BUFFER_OFFSET;
        *_device_read_buf = _device_data_register;
        _device_read_length = read_length - SPI_PRE_READ_BUFFER_OFFSET;
    }
    static void delay_ms(int ms);
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    static GPIO_TypeDef* gpioPort(port_pin_t portPin) { return reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + portPin.port*(GPIOB_BASE - GPIOA_BASE)); }
    static uint16_t gpioPin(port_pin_t portPin) { return static_cast<uint16_t>(1U << portPin.pin); }
#endif
protected:
    uint8_t _device_data_register {}; // the device register that is read in the read_device_data() function
    uint8_t* _device_read_buf {};
    size_t _device_read_length {};
#if defined(FRAMEWORK_USE_FREERTOS)
    uint32_t _data_ready_queue_item {}; // this is just a dummy item whose value is not used
    BaseType_t _data_ready_queue_higher_priority_task_woken = pdFALSE;
    static constexpr uint8_t IMU_DATA_READY_QUEUE_LENGTH = 1;
    std::array<uint8_t, IMU_DATA_READY_QUEUE_LENGTH * sizeof(_data_ready_queue_item)> _data_ready_queueStorageArea {};
    StaticQueue_t _data_ready_queue_static {};
    QueueHandle_t _data_ready_queue {};
public:
    int32_t WAIT_DATA_READY() { return xQueueReceive(_data_ready_queue, &_data_ready_queue_item, portMAX_DELAY); }
    int32_t WAIT_DATA_READY(uint32_t ticksToWait) { return xQueueReceive(_data_ready_queue, &_data_ready_queue_item, ticksToWait); } // returns pdPASS(1) if queue read, pdFAIL(0) if timeout
    void SIGNAL_DATA_READY_FROM_ISR() {
        _data_ready_queue_higher_priority_task_woken = pdFALSE;
        xQueueOverwriteFromISR(_data_ready_queue, &_data_ready_queue_item, &_data_ready_queue_higher_priority_task_woken);
        portYIELD_FROM_ISR(_data_ready_queue_higher_priority_task_woken); // or portEND_SWITCHING_ISR() depending on the port.
    }
#else
public:
    int32_t WAIT_DATA_READY() { return 0; }
    int32_t WAIT_DATA_READY(uint32_t ticksToWait) { (void)ticksToWait; return 0; }
    void SIGNAL_DATA_READY_FROM_ISR() {}
#endif
};

inline void BusBase::delay_ms(int ms)
{
#if defined(FRAMEWORK_USE_FREERTOS)
    vTaskDelay(pdMS_TO_TICKS(ms));
#else
    (void)ms;
#endif
}
