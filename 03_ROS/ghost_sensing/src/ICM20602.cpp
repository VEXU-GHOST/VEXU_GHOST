#include "ICM20602.h"
#include <string.h>
#include <cstdint>
#include <memory>
#include "I2C_interfacing.h"

namespace ghost_sensing
{

    ICM20602::ICM20602(std::shared_ptr<I2C_interfacing> iface, uint8_t address)
        : m_i2c_communication(iface), address(address)
    {
        m_i2c_communication->init();
    }

    ICM20602::~ICM20602()
    {
    }

    bool ICM20602::init()
    {
        uint8_t id = 0;
        // Read and validate ID
        if (!readRegister(ID, id))
        {
            return false;
        }
        if ((id != 0xAB) && (id != 0x9C))
        {
            return false;
        }
        if (!writeRegister(ICM_CONFIG_REGISTER, ICM_DEFAULT_CONFIG))
        {
            return false;
        }
        // config setup
        if (!writeRegister(SAMPLE_RATE_DIVIDER_REGISTER, DEFAULTCONFIG.sample_rate_divider))
        {
            return false;
        }

        uint8_t gyro_config = 0;
        uint8_t accel_config = 0;
        uint8_t accel_config2 = 0;
        uint8_t config = 0;
        if (!DEFAULTCONFIG.set_gyro_lowpass_filter)
        {
            gyro_config++;
        }
        else
        {
            config | DEFAULTCONFIG.gyro_lowpass_filter;
        }
        if (!DEFAULTCONFIG.set_accel_lowpass_filter)
        {
            accel_config2 || 8;
        }
        else
        {
            accel_config2 |= DEFAULTCONFIG.accel_lowpass_filter;
        }
        gyro_config |= (DEFAULTCONFIG.gyro_sensitivity << 3);
        accel_config |= (DEFAULTCONFIG.accel_sensitivity << 3);
        accel_config2 |= (DEFAULTCONFIG.accel_data_rate << 4);
        if (!writeRegister(GYRO_CONFIG_REGISTER, gyro_config))
        {
            return false;
        }
        if (!writeRegister(ACCEL_CONFIG_REGISTER, accel_config))
        {
            return false;
        }
        if (!writeRegister(ACCEL_CONFIG2_REGISTER, accel_config2))
        {
            return false;
        }
        uint8_t power_config = DEFAULTCONFIG.gyro_lowpower_mode;
        power_config = power_config << 7;
        if (!writeRegister(GYRO_LP_CONFIG, power_config))
        {
            return false;
        }
        uint8_t fifo_config = readRegister(FIFO_EN_REGISTER);
        fifo_config |= 0x18; // enable all gyro and accel axes to be written
        if (!writeRegister(FIFO_EN_REGISTER, fifo_config)){ return false;}
        if (!writeRegister(ACCEL_INTEL_CTRL_REGISTER, 0x00)){ return false;} // enable accel data ready interrupt
        if (!writeRegister(I2C_CTL_REGISTER, 0x00)){ return false;} // 0 FOR I2C, 1 for SPI
        //TODO: double check later, determines how much data is stored in fifo
        if(!writeRegister(FIFO_COUNTH_REGISTER, 0x00)){ return false;}
        if(!writeRegister(FIFO_COUNTL_REGISTER, 0x01)){ return false;}
        if(!writeRegister(PWR_MGMT_1, 0x00);){ return false;} // turn on device, set clock source to gyro
        return true;

    }

    bool ICM20602::writeRegister(uint8_t reg, uint8_t data)
    {
        return (m_i2c_communication->write(reg, &data, 1) == 0);
    }

    bool ICM20602::writeRegisters(uint8_t reg, uint8_t *buf, uint16_t len)
    {
        return (m_i2c_communication->write(reg, buf, len) == 0);
    }

    bool ICM20602::readRegister(uint8_t reg, uint8_t &data)
    {
        return (m_i2c_communication->read(reg, &data, 1) == 0);
    }

    bool ICM20602::readRegisters(uint8_t reg, uint8_t *buf, uint16_t len)
    {
        return (m_i2c_communication->read(reg, buf, len) == 0);
    }

    bool ICM20602::fineTuneOffset(int16_t GyroOffsetX, int16_t GyroOffsetY, int16_t GyroOffsetZ, int8_t sOffsetX, int8_t sOffsetY, int8_t sOffsetZ)
    {
        uint8_t x1 = GyroOffsetX >> 8 + sOffsetX << 2;
        uint8_t x2 = GyroOffsetX & 0xFF;
        uint8_t y1 = GyroOffsetY >> 8 + sOffsetY << 2;
        uint8_t y2 = GyroOffsetY & 0xFF;
        uint8_t z1 = GyroOffsetZ >> 8 + sOffsetZ << 2;
        uint8_t z2 = GyroOffsetZ & 0xFF;
        uint8_t bufx[2] = {x1, x2};
        uint8_t bufy[2] = {y1, y2};
        uint8_t bufz[2] = {z1, z2};
        if (!writeRegisters(OFFSET_REGISTERx1, bufx, 2))
        {
            return false;
        }
        if (!writeRegisters(OFFSET_REGISTERy1, bufy, 2))
        {
            return false;
        }
        if (!writeRegisters(OFFSET_REGISTERz1, bufz, 2))
        {
            return false;
        }
        return true;
    }

    ICM20602::GyroOffset ICM20602::getFTDeviceOffset()
    {
        GyroOffset offset;
        uint8_t bufx[2];
        uint8_t bufy[2];
        uint8_t bufz[2];
        if (!readRegisters(OFFSET_REGISTERx1, bufx, 2))
        {
            return {0, 0, 0};
        }
        if (!readRegisters(OFFSET_REGISTERy1, bufy, 2))
        {
            return {0, 0, 0};
        }
        if (!readRegisters(OFFSET_REGISTERz1, bufz, 2))
        {
            return {0, 0, 0};
        }
        offset.GyroOffsetX = (bufx[0] << 8) | bufx[1];
        offset.GyroOffsetY = (bufy[0] << 8) | bufy[1];
        offset.GyroOffsetZ = (bufz[0] << 8) | bufz[1];
        offset.sOffsetX = (bufx[0] >> 2) & 0xFF;
        offset.sOffsetY = (bufy[0] >> 2) & 0xFF;
        offset.sOffsetZ = (bufz[0] >> 2) & 0xFF;
        return offset;
    }

    bool setOffset(int16_t GyroOffsetX, int16_t GyroOffsetY, int16_t GyroOffsetZ)
    {
        uint8_t buf[6];
        buf[0] = (GyroOffsetX >> 8) & 0xFF;
        buf[1] = GyroOffsetX & 0xFF;
        buf[2] = (GyroOffsetY >> 8) & 0xFF;
        buf[3] = GyroOffsetY & 0xFF;
        buf[4] = (GyroOffsetZ >> 8) & 0xFF;
        buf[5] = GyroOffsetZ & 0xFF;
        if (!writeRegisters(OFFSET_X_HIGH, buf, 6))
        {
            return false;
        }
        return true;
    }

    ICM20602::GyroOffset ICM20602::getOffset()
    {
        GyroOffset offset;
        uint8_t buf[6];
        if (!readRegisters(OFFSET_X_HIGH, buf, 6))
        {
            // debug error statement        return {0, 0, 0};
        }
        offset.GyroOffsetX = (buf[0] << 8) | buf[1];
        offset.GyroOffsetY = (buf[2] << 8) | buf[3];
        offset.GyroOffsetZ = (buf[4] << 8) | buf[5];
        return offset;
    }

    bool sampleRateDivider(uint8_t divider)
    {
        return writeRegister(SAMPLE_RATE_DIVIDER_REGISTER, divider);
    }

    ICM20602::ICM_data ICM20602::readSensorData()
    {
        ICM_data data;
        uint8_t buf[14];
        if (!readRegisters(ACCEL_XOUT_H, buf, 14))
        {
            return {0, 0, 0, 0, 0, 0};
        }
        data.AccelX = (buf[0] << 8) | buf[1];
        data.AccelY = (buf[2] << 8) | buf[3];
        data.AccelZ = (buf[4] << 8) | buf[5];
        data.Temp = (buf[6] << 8) | buf[7];
        data.GyroX = (buf[8] << 8) | buf[9];
        data.GyroY = (buf[10] << 8) | buf[11];
        data.GyroZ = (buf[12] << 8) | buf[13];
        
        return data;
    }

    bool ICM20602::setAccelerometerOffset(int16_t AccelOffsetX, int16_t AccelOffsetY, int16_t AccelOffsetZ) {
        uint8_t buf[6];
        buf[0] = (AccelOffsetX >> 8) & 0xFF;
        buf[1] = AccelOffsetX & 0xFF;
        buf[2] = (AccelOffsetY >> 8) & 0xFF;
        buf[3] = AccelOffsetY & 0xFF;
        buf[4] = (AccelOffsetZ >> 8) & 0xFF;
        buf[5] = AccelOffsetZ & 0xFF;
        if (!writeRegisters(ACCEL_OFFSETXH_REG, buf, 6)) {
            return false;
        }
        return true;
    }

    ICM20602::Accel_Offset ICM20602::getAccelerometerOffset() {
        Accel_Offset offset;
        uint8_t buf[6];
        if (!readRegisters(ACCEL_OFFSETXH_REG, buf, 6)) {
            return {0, 0, 0};
        }
        offset.AccelOffsetX = (buf[0] << 8) | buf[1];
        offset.AccelOffsetY = (buf[2] << 8) | buf[3];
        offset.AccelOffsetZ = (buf[4] << 8) | buf[5];
        return offset;
    }

    bool ICM20602::deviceDeactivation(bool accelx, bool accely, bool accelz, bool gyrox, bool gyroy, bool gyroz) {
        uint8_t = gyroz+(gyroy<<1)+(gyrox<<2)+(accelz<<3)+(accely<<4)+(accelx<<5);
        return writeRegister(PWR_MGMT_2, config);
    }

    bool ICM20602::deinit() {
        return writeRegister(PWR_MGMT_1, 0x40); // set sleep
    }

    bool resetData() {
        writeRegister(USER_CTRL_REGISTER, 0x80); // set reset bit
        writeRegister(USER_CTRL_REGISTER, 0x00); // clear reset bit
        return true;
    }

    bool ISL29125::writeRegister(uint8_t reg, uint8_t data) {
    return (m_i2c_communication->write(reg, &data, 1) == 0);
    }

    bool ISL29125::readRegister(uint8_t reg, uint8_t &data) {
     return (m_i2c_communication->read(reg, &data, 1) == 0);
    }

    bool ISL29125::readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
        return (m_i2c_communication->read(reg, buf, len) == 0);
    }

    // bool ISL29125::writeWord(uint8_t reg, uint8_t data) {
    //     return (m_i2c_communication->write(reg, &data, 4) == 0);
    // }
    // bool ISL29125::readWord(uint8_t reg, uint8_t &data) {
    //     return (m_i2c_communication->read(reg, &data, 4) == 0);
    // }
} // namespace ghost_sensing