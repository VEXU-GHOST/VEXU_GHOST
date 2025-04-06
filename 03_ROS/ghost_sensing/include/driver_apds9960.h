#ifndef DRIVER_APDS9960_H
#define DRIVER_APDS9960_H

#include <stdint.h>
#include <memory>
#include "driver_tcs34725_interface.h"

namespace ghost_sensing {

class color_sensor_apds9960 {
public:
    // Register definitions
    // APDS9960 I2C address is just for reference, encode in robot_hardware config.yaml
    //static const uint8_t ADDRESS        = 0x39;

    static const uint8_t ENABLE         = 0x80;
    static const uint8_t ATIME          = 0x81;
    static const uint8_t WTIME          = 0x83;
    static const uint8_t AILTL          = 0x84;
    static const uint8_t AILTH          = 0x85;
    static const uint8_t AIHTL          = 0x86;
    static const uint8_t AIHTH          = 0x87;
    static const uint8_t PILT           = 0x89;
    static const uint8_t PIHT           = 0x8B;
    static const uint8_t PERS           = 0x8C;
    static const uint8_t CONFIG1        = 0x8D;
    static const uint8_t PPULSE         = 0x8E;
    static const uint8_t CONTROL        = 0x8F;
    static const uint8_t CONFIG2        = 0x90;
    static const uint8_t ID             = 0x92;
    static const uint8_t STATUS         = 0x93;
    static const uint8_t CDATAL         = 0x94;
    static const uint8_t CDATAH         = 0x95;
    static const uint8_t RDATAL         = 0x96;
    static const uint8_t RDATAH         = 0x97;
    static const uint8_t GDATAL         = 0x98;
    static const uint8_t GDATAH         = 0x99;
    static const uint8_t BDATAL         = 0x9A;
    static const uint8_t BDATAH         = 0x9B;
    static const uint8_t PDATA          = 0x9C;
    static const uint8_t CONFIG3        = 0x9F;  // Added CONFIG3 register
    
    // Gesture registers
    static const uint8_t GCONFIG1       = 0xA2;
    static const uint8_t GSTATUS        = 0xAF;
    static const uint8_t GFLVL          = 0xAE;
    static const uint8_t GOFFSET_U      = 0xA4;
    static const uint8_t GOFFSET_D      = 0xA5;
    static const uint8_t GOFFSET_L      = 0xA7;
    static const uint8_t GOFFSET_R      = 0xA9;
    static const uint8_t GPULSE         = 0xA6;
    static const uint8_t GCONF3         = 0xAA;
    static const uint8_t GCONF4         = 0xAB;
    static const uint8_t GFLVL_L        = 0xAE;
    static const uint8_t GFLVL_H        = 0xAF;
    static const uint8_t GDATA          = 0xFC;  // Gesture data registers start

    // Status bits
    static const uint8_t STATUS_AVALID  = 0x01;  // ALS Valid
    static const uint8_t STATUS_PVALID  = 0x02;  // Proximity Valid
    static const uint8_t STATUS_GINT    = 0x04;  // Gesture Interrupt
    static const uint8_t STATUS_AINT    = 0x10;  // ALS Interrupt
    static const uint8_t STATUS_PINT    = 0x20;  // Proximity Interrupt
    static const uint8_t STATUS_PGSAT   = 0x40;  // Gesture Saturation
    static const uint8_t STATUS_CPSAT   = 0x80;  // Clear Photodiode Saturation

    // Default values
    static const uint8_t DEFAULT_ATIME  = 255;    // 2.78ms
    static const uint8_t DEFAULT_WTIME  = 246;    // 27ms
    static const uint8_t DEFAULT_PPULSE = 0x87;   // 16us, 8 pulses
    static const uint8_t DEFAULT_CONFIG1 = 0x60;
    static const uint8_t DEFAULT_PILT   = 0;      // Proximity low threshold
    static const uint8_t DEFAULT_PIHT   = 50;     // Proximity high threshold
    static const uint8_t DEFAULT_PERS   = 0x11;   // 2 consecutive prox or ALS for int.
    static const uint8_t DEFAULT_CONFIG2 = 0x01;  // No saturation interrupts  
    static const uint8_t DEFAULT_CONFIG3 = 0x00;  // Enable all photodiodes

    // Bit fields
    static const uint8_t POWER_ON       = 0x01;
    static const uint8_t ALS_ENABLE     = 0x02;
    static const uint8_t PROX_ENABLE    = 0x04;
    static const uint8_t WAIT_ENABLE    = 0x08;
    static const uint8_t GESTURE_ENABLE = 0x40;  // Add gesture enable bit
    static const uint8_t GVALID         = 0x01;  // Add gesture valid bit

    // Gesture directions (returned by readGesture())
    static const int8_t DIR_NONE  = 0;
    static const int8_t DIR_LEFT  = 1;
    static const int8_t DIR_RIGHT = 2;
    static const int8_t DIR_UP    = 3;
    static const int8_t DIR_DOWN  = 4;

    // Constructor / destructor
    color_sensor_apds9960(std::shared_ptr<tcs_i2c_interface> iface, uint8_t address);
    ~color_sensor_apds9960();

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
    bool enableGestureSensor(bool interrupts = true);
    bool disableGestureSensor();
    bool isGestureAvailable();
    int  readGesture();

    struct sensor_data_t {
        uint16_t red, green, blue, clear;
        uint8_t proximity;
        bool valid;  // Add valid field
    } sensor_data_;

    // Add readAllSensors declaration
    bool readAllSensors(sensor_data_t& data);

private:
    std::shared_ptr<tcs_i2c_interface> m_i2c_communication;

    // Helper functions for register access
    bool writeRegister(uint8_t reg, uint8_t data);
    bool readRegister(uint8_t reg, uint8_t &data);
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

    // ...additional private members and helper functions as needed...
};

} // namespace ghost_sensing

#endif
