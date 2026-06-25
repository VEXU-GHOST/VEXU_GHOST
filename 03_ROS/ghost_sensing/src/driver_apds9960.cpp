#include <driver_apds9960.h>
#include <string.h>

namespace ghost_sensing {

color_sensor_apds9960::color_sensor_apds9960(std::shared_ptr<I2C_interfacing> iface, uint8_t address)
: m_i2c_communication(iface), address(address)
{
}

color_sensor_apds9960::~color_sensor_apds9960()
{
}

bool color_sensor_apds9960::writeRegister(uint8_t reg, uint8_t data) {
    return (m_i2c_communication->write(address, reg, &data, 1) == 0);
}

bool color_sensor_apds9960::readRegister(uint8_t reg, uint8_t &data) {
    return (m_i2c_communication->read(address, reg, &data, 1) == 0);
}

bool color_sensor_apds9960::readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
    return (m_i2c_communication->read(address, reg, buf, len) == 0);
}

bool color_sensor_apds9960::init() {
    uint8_t id = 0;
    
    // Read and validate ID
    if (!readRegister(ID, id)) {
        return false;
    }
    if ((id != 0xAB) && (id != 0x9C)) {
        return false;
    }
    
    // Initialize with defaults
    if (!writeRegister(ENABLE, 0x00) ||          // Disable all features
        !writeRegister(ATIME, DEFAULT_ATIME) ||  // Set integration time
        !writeRegister(WTIME, DEFAULT_WTIME) ||  // Set wait time
        !writeRegister(WTIME, DEFAULT_WTIME) ||  // Set wait time
        !writeRegister(PPULSE, DEFAULT_PPULSE) || // Set proximity pulse count
        !writeRegister(CONFIG1, DEFAULT_CONFIG1) ||
        !writeRegister(CONTROL, DEFAULT_CONTROL) ||
        !writeRegister(PILT, DEFAULT_PILT) ||
        !writeRegister(PIHT, DEFAULT_PIHT) ||
        !writeRegister(PERS, DEFAULT_PERS) ||
        !writeRegister(CONFIG2, DEFAULT_CONFIG2) ||
        !writeRegister(CONFIG3, DEFAULT_CONFIG3)) {
        return false;
    }
    return true;
}

bool color_sensor_apds9960::deinit() {
    return disablePower(); // power off sensor
}

bool color_sensor_apds9960::enablePower() {
    uint8_t regVal;
    if (!readRegister(ENABLE, regVal)) {
        return false;
    }
    regVal |= POWER_ON;
    return writeRegister(ENABLE, regVal);
}

bool color_sensor_apds9960::disablePower() {
    uint8_t regVal;
    if (!readRegister(ENABLE, regVal)) {
        return false;
    }
    regVal &= ~POWER_ON;
    return writeRegister(ENABLE, regVal);
}

bool color_sensor_apds9960::enableLightSensor(bool interrupts) {
    uint8_t regVal;
    if (!readRegister(ENABLE, regVal)) {
        return false;
    }
    regVal |= ALS_ENABLE;
    return writeRegister(ENABLE, regVal);
}

bool color_sensor_apds9960::disableLightSensor() {
    uint8_t regVal;
    if (!readRegister(ENABLE, regVal)) {
        return false;
    }
    regVal &= ~ALS_ENABLE; // clear AEN bit
    return writeRegister(ENABLE, regVal);
}

bool color_sensor_apds9960::readAllSensors(sensor_data_t& data) {
    uint8_t buf[11];  // ID, STATUS, CDATAL/H, RDATAL/H, GDATAL/H, BDATAL/H, PDATA
    
    if (!readRegisters(ID, buf, sizeof(buf))) {
        return false;
    }
    
    // Verify device ID
    if ((buf[0] != 0xAB) && (buf[0] != 0x9C)) {
        return false;
    }
    
    // Check if data is valid (STATUS register)
    data.valid = (buf[1] & STATUS_AVALID) != 0;
    if (!data.valid) {
        return false;
    }
    
    // Parse the data
    data.clear = ((uint16_t)buf[3] << 8) | buf[2];
    data.red = ((uint16_t)buf[5] << 8) | buf[4];
    data.green = ((uint16_t)buf[7] << 8) | buf[6];
    data.blue = ((uint16_t)buf[9] << 8) | buf[8];
    data.proximity = buf[10];
    
    return true;
}

bool color_sensor_apds9960::readAmbientLight(uint16_t &clear) {
    sensor_data_t data;
    if (!readAllSensors(data)) {
        return false;
    }
    clear = data.clear;
    return true;
}

bool color_sensor_apds9960::readRedLight(uint16_t &red) {
    sensor_data_t data;
    if (!readAllSensors(data)) {
        return false;
    }
    red = data.red;
    return true;
}

bool color_sensor_apds9960::readGreenLight(uint16_t &green) {
    sensor_data_t data;
    if (!readAllSensors(data)) {
        return false;
    }
    green = data.green;
    return true;
}

bool color_sensor_apds9960::readBlueLight(uint16_t &blue) {
    sensor_data_t data;
    if (!readAllSensors(data)) {
        return false;
    }
    blue = data.blue;
    return true;
}

bool color_sensor_apds9960::readProximity(uint8_t &prox) {
    sensor_data_t data;
    if (!readAllSensors(data)) {
        return false;
    }
    prox = data.proximity;
    return true;
}

bool color_sensor_apds9960::enableProximitySensor(bool interrupts) {
    uint8_t regVal;
    if (!readRegister(ENABLE, regVal)) {
        return false;
    }
    // Set PEN (Proximity Enable, bit 2)
    regVal |= PROX_ENABLE;
    return writeRegister(ENABLE, regVal);
}

bool color_sensor_apds9960::disableProximitySensor() {
    uint8_t regVal;
    if (!readRegister(ENABLE, regVal)) {
        return false;
    }
    regVal &= ~PROX_ENABLE; // clear PEN bit
    return writeRegister(ENABLE, regVal);
}

bool color_sensor_apds9960::enableGestureSensor(bool interrupts) {
    uint8_t regVal;
    if (!readRegister(ENABLE, regVal)) {
        return false;
    }
    // Set GEN (Gesture Enable, bit 6) as an example.
    regVal |= GESTURE_ENABLE;
    // Optionally configure gesture-specific registers
    // ...existing code...
    return writeRegister(ENABLE, regVal);
}

bool color_sensor_apds9960::disableGestureSensor() {
    uint8_t regVal;
    if (!readRegister(ENABLE, regVal)) {
        return false;
    }
    regVal &= ~GESTURE_ENABLE; // clear GEN bit
    return writeRegister(ENABLE, regVal);
}

bool color_sensor_apds9960::isGestureAvailable() {
    uint8_t status;
    if (!readRegister(STATUS, status)) {
        return false;
    }
    // For example, assume GVALID is bit 0 (placeholder)
    return (status & GVALID) != 0;
}

int color_sensor_apds9960::readGesture() {
    if (!isGestureAvailable()) {
        return DIR_NONE;
    }
    
    uint8_t buf[4];
    if (!readRegisters(GFLVL, buf, 4)) {
        return DIR_NONE;
    }
    
    uint8_t max_val = 0;
    int direction = DIR_NONE;
    
    for(int i = 0; i < 4; i++) {
        if(buf[i] > max_val) {
            max_val = buf[i];
            switch(i) {
                case 0: direction = DIR_UP; break;
                case 1: direction = DIR_DOWN; break;
                case 2: direction = DIR_LEFT; break;
                case 3: direction = DIR_RIGHT; break;
            }
        }
    }
    
    return direction;
}

bool TCA9536::writeRegister(uint8_t reg, uint8_t data) {
    return (m_i2c_communication->write(reg, &data, 1) == 0);
}

bool TCA9536::readRegister(uint8_t reg, uint8_t &data) {
    return (m_i2c_communication->read(reg, &data, 1) == 0);
}

bool TCA9536::readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
    return (m_i2c_communication->read(reg, buf, len) == 0);
}
bool TCA9536::writeWord(uint8_t reg, uint8_t data) {
    return (m_i2c_communication->write(reg, &data, 4) == 0);
}
bool TCA9536::readWord(uint8_t reg, uint8_t &data) {
    return (m_i2c_communication->read(reg, &data, 4) == 0);
}
} // namespace ghost_sensing