#ifndef DRIVER_APDS9960_H
#define DRIVER_APDS9960_H

#include <stdint.h>
#include <memory>
#include "I2C_interfacing.h"

namespace ghost_sensing {
class ICM20602
{
    

    private:
    std::shared_ptr<I2C_interfacing> m_i2c_communication;
    uint8_t m_i2c_address;

    bool writeRegister(uint8_t reg, uint8_t data);
    bool writeRegisters(uint8_t reg, uint8_t *buf, uint16_t len);
    bool readRegister(uint8_t reg, uint8_t &data);   
    bool readRegisters(uint8_t reg, uint8_t *buf, uint16_t len);
    
    public:


    struct GyroOffset {
        int16_t GyroOffsetX;
        int16_t GyroOffsetY;
        int16_t GyroOffsetZ;
    };

    struct ICM_data {
        int16_t AccelX;
        int16_t AccelY;
        int16_t AccelZ;
        int16_t Temp;
        int16_t GyroX;
        int16_t GyroY;
        int16_t GyroZ;
    };

    struct ICM_config {
        uint8_t sample_rate_divider;
        bool set_gyro_lowpass_filter;
        uint8_t gyro_lowpass_filter;
        bool set_accel_lowpass_filter;
        uint8_t accel_lowpass_filter;
        bool gyro_lowpower_mode;
        uint8_t gyro_sensitivity;
        uint8_t accel_sensitivity;
        uint8_t accel_data_rate;
    };

    struct Accel_Offset {
        int16_t AccelOffsetX;
        int16_t AccelOffsetY;
        int16_t AccelOffsetZ;
    };

    ICM20602(std::shared_ptr<I2C_interfacing> iface, uint8_t address);
    ~ICM20602();

    bool init();
    bool deinit();
    bool fineTuneOffset(int16_t GyroOffsetX, int16_t GyroOffsetY, int16_t GyroOffsetZ, int8_t sOffsetX, int8_t sOffsetY, int8_t sOffsetZ);
    GyroOffset getFTDeviceOffset();
    bool setOffset(int16_t GyroOffsetX, int16_t GyroOffsetY, int16_t GyroOffsetZ);
    GyroOffset getOffset();
    bool sampleRateDivider(uint8_t divider);
    ICM_data readSensorData();
    bool setAccelerometerOffset(int16_t AccelOffsetX, int16_t AccelOffsetY, int16_t AccelOffsetZ);
    bool resetData();

    #define GYRO_CONFIG_REGISTER 0x1B
    #define ACCEL_CONFIG_REGISTER 0x1C
    #define ACCEL_CONFIG2_REGISTER 0x1D
    #define SAMPLE_RATE_DIVIDER_REGISTER 0x19
    #define OFFSET_REGISTERx1 0x77
    #define OFFSET_REGISTERy1 0x7A
    #define OFFSET_REGISTERz1 0x7D
    #define OFFSET_X_HIGH 0x13
    #define OFFSET_Y_HIGH 0x15
    #define OFFSET_Z_HIGH 0x0A
    #define CONFIG_REGISTER 0x17
    #define ID 0x75
    #define ICM_DEFAULT_CONFIG  
    #define ICM_CONFIG_REGISTER 0x1A
    #define I2C_CTRL_REG 0x37
    #define ACCEL_INTEL_CTRL_REG 0x69
    #define GYRO_LP_CONFIG 0x1E
    #define FIFO_EN_REGISTER 0x23
    #define ACCEL_XOUT_H 0x3B
    #define USER_CTRL_REGISTER 0x6A
    #define PWR_MGMT_1 0x6B
    #define PWR_MGMT_2 0x6C
    #define I2C_CTL_REGISTER 0x70
    #define FIFO_COUNTH_REGISTER 0x72
    #define FIFO_COUNTL_REGISTER 0x73
    #define FIFO_R_W_REGISTER 0x74
    #define WHO_AM_I_REGISTER 0x75
    #define ACCEL_OFFSETXH_REG 0x77
    #define ACCEL_OFFSETXL_REG 0x78
    #define ACCEL_OFFSETYH_REG 0x7A
    #define ACCEL_OFFSETYL_REG 0x7B
    #define ACCEL_OFFSETZH_REG 0x7D
    #define ACCEL_OFFSETZL_REG 0x7E

    static ICM_config DEFAULTCONFIG{
        .sample_rate_divider = 0,
        .set_gyro_lowpass_filter = false,
        .gyro_lowpass_filter = 0,
        .set_accel_lowpass_filter = false,
        .accel_lowpass_filter = 0,
        .gyro_lowpower_mode = false,
        .gyro_sensitivity = 0, // 
        .accel_sensitivity = 0, // 
        .accel_data_rate = 0 // 
    };

}

}