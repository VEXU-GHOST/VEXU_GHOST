#ifndef DRIVER_APDS9960_H
#define DRIVER_APDS9960_H

#include <stdint.h>
#include <memory>
#include "I2C_interfacing.h"

namespace ghost_sensing {

class ISL29125 {
public:
    // Register definitions
    // static const uint8_t ADDRESS   = 0x39; // for reference
    static const uint8_t STATUS_REG                     = 0x08;
    static const uint8_t HHigh_Threshold                 = 0x07;
    static const uint8_t HLow_Threshold                 = 0x06;
    static const uint8_t LHigh_Threshold                 = 0x05;
    static const uint8_t LLow_Threshold                 = 0x04; // 7-bit address
    static const uint8_t DEV_ADDRESS             = 0x88;
    static const uint8_t ID             = 0x00;
    static const uint8_t CONFIG_REG        = 0x01;
    static const uint8_t OFFSET_REG    = 0x02;
    static const uint8_t ALLCOLORS         = 0B101;
    static const uint8_t LARGE_SENSOR_RANGE      = 0B1000;
    static const uint8_t DEFAULT_OFFSET    = 0x00;
    static const uint8_t GREEN_LOW         = 0x09;
    static const uint8_t GREEN_HIGH        = 0x0A;
    static const uint8_t RED_LOW           = 0x0B;
    static const uint8_t RED_HIGH          = 0x0C;
    static const uint8_t BLUE_LOW          = 0x0D;
    static const uint8_t BLUE_HIGH         = 0x0E;
    uint8_t CONFIG_REG_VALUE = 0B101; // Default value for all colors and large 
    static const uint8_t INTERUPT_REG      = 0x03;
    // Constructor / destructor
    ISL29125(std::shared_ptr<I2C_interfacing> iface, uint8_t address);
    ~ISL29125();

    // Initialization and deinitialization
    bool init();
    bool deinit();

    // Power control
    bool enablePower();
    bool disablePower();

    // Light sensor functions
    bool enableLightSensor(bool interrupts = false);
    bool disableLightSensor();
    bool readAmbientLight(uint16_t &clear);
    bool readRedLight(uint16_t &red);
    bool readGreenLight(uint16_t &green);
    bool readBlueLight(uint16_t &blue);

    // Proximity sensor functions
    bool enableProximitySensor(bool interrupts = false);
    bool disableProximitySensor();
    bool readProximity(uint8_t &prox);

    // Gesture sensor functions
    // bool enableGestureSensor(bool interrupts = true);
    // bool disableGestureSensor();
    // bool isGestureAvailable();
    // int  readGesture();

    bool checkErrors();
    bool CheckConversion();
    bool CheckInterrupt();
    bool setIRCompensationRange(uint8_t comp);
    bool setConversionTime(uint8_t range);
    bool setSensingRange(uint8_t range);

    bool setResolution(uint8_t range);

    bool SetInterupts(uint8_t interupts);

    bool SetHighThreshold(uint16_t threshold);
    bool SetLowThreshold(uint16_t threshold);
    struct sensor_data_t {
        uint16_t red, green, blue, clear;
        uint8_t proximity;
        bool valid;  // Add valid field
    } sensor_data_;

    // Add readAllSensors declaration
    bool readAllSensors(sensor_data_t& data);
    bool readRegister(uint8_t reg, uint8_t &data);

private:
    std::shared_ptr<I2C_interfacing> m_i2c_communication;

    // Helper functions for register access
    bool writeRegister(uint8_t reg, uint8_t data);
    bool readRegisters(uint8_t reg, uint8_t *buf, uint16_t len);

    uint8_t address;

    // Add gesture data structure but keep it minimal
    struct gesture_data_t {
        uint8_t u_data[4];  // Simplified from 32 to 4
        uint8_t d_data[4];
        uint8_t l_data[4];
        uint8_t r_data[4];
        uint8_t index;
    } gesture_data_;

};

} // namespace ghost_sensing

#endif
