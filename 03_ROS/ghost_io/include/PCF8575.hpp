#pragma once

#include <cstdint>
#include <string>

class PCF8575 {
public:
    PCF8575(){};
    /**
     * @brief Construct a new PCF8575 object.
     * 
     * @param i2cBus Path to the I2C bus device (e.g. "/dev/i2c-1").
     * @param address I2C address of the PCF8575 (typically between 0x20 and 0x27).
     */
    bool open(const std::string &i2cBus, uint8_t address, uint16_t initVal = 0xFFFF);

    /// Clean up and close the bus.
    ~PCF8575();

    /**
     * @brief Set a single pin to HIGH or LOW.
     * 
     * @param pin Pin number (0..15).
     * @param value true for HIGH, false for LOW.
     * @return true on success, false on error.
     */
    bool setPin(uint8_t pin, bool value);

    /**
     * @brief Read a single pin value.
     * 
     * @param pin Pin number (0..15).
     * @param value Reference to store the pin value (true for HIGH, false for LOW).
     * @return true on success, false on error.
     */
    bool getPin(uint8_t pin, bool &value);

    bool poll();

private:
    /**
     * @brief Read 16 bits from the expander.
     * 
     * @return uint16_t 16-bit input value.
     */
    uint16_t read16();

    /**
     * @brief Write 16 bits to the expander.
     * 
     * @param value 16-bit output value.
     * @return true on success, false on error.
     */
    bool write16(uint16_t value);


    int _fd;           // File descriptor for the I2C bus.
    uint8_t _address;  // I2C address of the PCF8575.

    uint16_t current_in;
    uint16_t current_out;
};