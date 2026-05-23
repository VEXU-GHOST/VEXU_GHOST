/*
Original IMU driver code by Martin Budden: https://github.com/martinbudden/Library-Sensors
*/

#pragma once

#include "bus_base.h"

//#define LIBRARY_SENSORS_USE_SPI_HARDWARE_CHIP_SELECT //!!TODO: make this based on member data, not build flag

#if defined(FRAMEWORK_RPI_PICO)
#if defined(LIBRARY_SENSORS_USE_SPI_DMA)
#include "hardware/dma.h"
#endif
typedef struct spi_inst spi_inst_t;
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <driver/gpio.h>
#include <driver/spi_master.h>
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal_spi.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal_spi.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal_spi.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal_spi.h>
#endif
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <SPI.h>
#endif // FRAMEWORK


class BusSpi : public BusBase {
public:
    static constexpr uint8_t BITS_PER_BYTE = 8;
    struct spi_pins_t {
        uint8_t cs;
        uint8_t sck;
        uint8_t cipo; // RX, CIPO, MISO, POCI
        uint8_t copi; // TX, COPI, MOSI, PICO
        uint8_t irq; // interrupt pin
    };
    struct stm32_spi_pins_t {
        port_pin_t cs;
        port_pin_t sck;
        port_pin_t cipo; // RX, CIPO, MISO, POCI
        port_pin_t copi; // TX, COPI, MOSI, PICO
        port_pin_t irq; // interrupt pin
    };

    static constexpr uint8_t BUS_READY = 0;
    static constexpr uint8_t BUS_BUSY = 1;
    static constexpr uint8_t BUS_ABORT = 2;

    struct segment_t {
        union {
            struct {
                uint8_t* tx_data; // Transmit buffer
                uint8_t* rx_data; // Receive buffer
            } buffers;
            struct {
                const BusSpi* dev; // Link to the device associated with the next transfer
                volatile segment_t* segments; // Segments to process in the next transfer.
            } link;
        } u;
        int len;
        bool negate_cs; // Should CS be negated at the end of this segment
        uint8_t (*callbackFn)(uint32_t arg);
    };
public:
    virtual ~BusSpi();
    BusSpi(uint32_t frequency, uint8_t spi_index, const stm32_spi_pins_t& pins);
    BusSpi(uint32_t frequency, uint8_t spi_index, const spi_pins_t& pins);
public:
    void init();
    void configure_dma();
    void set_interrupt_driven(uint8_t irq_level);

    uint16_t calculate_clock_divider(uint32_t frequency_hz);
    uint32_t calculate_clock(uint16_t clock_divisor);
    void set_clock_divisor(uint16_t divisor);
    void set_clock_phase_polarity(bool leading_edge);
    void dma_enable(bool enable);
    void dma_sequence(segment_t* segments);
    void dma_wait();
    void dma_release();
    bool dma_is_busy();

    bool read_device_data();
    bool read_device_data_dma();
    uint8_t read_register(uint8_t reg) const;
    uint8_t read_register_with_timeout(uint8_t reg, uint32_t timeout_ms) const;
    bool read_register(uint8_t reg, uint8_t* data, size_t length) const;
    bool read_bytes(uint8_t* data, size_t length) const;
    bool read_bytes_with_timeout(uint8_t* data, size_t length, uint32_t timeout_ms) const;
    uint8_t write_register(uint8_t reg, uint8_t data);
    uint8_t write_register(uint8_t reg, const uint8_t* data, size_t length);
    uint8_t write_bytes(const uint8_t* data, size_t length);
    static void cs_select(const BusSpi& bus);
    static void cs_deselect(const BusSpi& bus);
    uint16_t get_irq_pin() const { return _pins.irq.pin; }
public:
    static BusSpi* self; //!< alias of `this` to be used in interrupt service routine
private:
    uint32_t _clock_divider {1};
    uint32_t _frequency_hz;
    uint8_t _spi_index;
    stm32_spi_pins_t _pins;
#if defined(FRAMEWORK_RPI_PICO)
    spi_inst_t* _spi {};
    uint32_t _dma_interrupt_number {};
    uint32_t _dma_rx_channel {};
    uint32_t _dma_tx_channel {};
    static constexpr bool START_NOW = true;
    static constexpr bool DONT_START_YET = false;
    static void data_ready_isr(unsigned int gpio, uint32_t events);
    static void dma_rx_complete_isr();
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    static void data_ready_isr(); // cppcheck-suppress unusedPrivateFunction
    spi_device_handle_t _spi {}; // set by spi_bus_add_device in init()
    mutable spi_transaction_t _spi_transaction {};
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    mutable SPI_HandleTypeDef _spi {};
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    static void data_ready_isr(); // cppcheck-suppress unusedPrivateFunction
    mutable volatile uint32_t* _cs_out {};
    uint32_t _cs_bit {};
    SPIClass& _spi;
#endif // FRAMEWORK
#if defined(LIBRARY_SENSORS_USE_SPI_HARDWARE_CHIP_SELECT)
    mutable std::array<uint8_t, 256> _write_read_buf {};
#endif
};
