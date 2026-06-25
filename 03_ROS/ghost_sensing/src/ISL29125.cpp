#include "ISL29125.h"
#include <string.h>
#include <cstdint>
#include <memory>
#include "I2C_interfacing.h"

namespace ghost_sensing {

ISL29125::ISL29125(std::shared_ptr<I2C_interfacing> iface, uint8_t address)
: m_i2c_communication(iface), address(address)
{m_i2c_communication->init();
}

ISL29125::~ISL29125()
{
}

bool ISL29125::init() {
     uint8_t id = 0;
     // Read and validate ID
     if (!readRegister(ID, id)) {
         return false;
     }
     if ((id != 0xAB) && (id != 0x9C)) {
         return false;
     }
    
     //Initialize with defaults

     uint8_t colors = ALLCOLORS; // Default configuration value
     uint8_t sensor_range = LARGE_SENSOR_RANGE;
     uint8_t total = colors + sensor_range; // Default configuration value

     if (!writeRegister(CONFIG_REG, total)){return false;}          //set colors and range
     if (!writeRegister(OFFSET_REG, DEFAULT_OFFSET)){ return false;}  //set integration time
    return true;
}

bool ISL29125::deinit() {
     return disablePower(); // power off sensor
}

bool ISL29125::setSensingRange(uint8_t range) { //0 =  low; 1 = high
    int mask = 1<<3;
    int val = CONFIG_REG_VALUE & (mask); val = val >>3;
    if(val!=range){
        CONFIG_REG_VALUE = CONFIG_REG_VALUE ^ mask; //changes range bit if different
    }
     return writeRegister(CONFIG_REG, CONFIG_REG_VALUE); // Enable with default config
}

bool ISL29125::setResolution(uint8_t range) { //0 =  16 bits; 1 = 12 bits
    int mask = 1<<4;
    int val = CONFIG_REG_VALUE & (mask); val = val >>4;
    if(val!=range){
        CONFIG_REG_VALUE = CONFIG_REG_VALUE ^ mask; //changes range bit if different
    }
     return writeRegister(CONFIG_REG, CONFIG_REG_VALUE); // Enable with default config
}

bool ISL29125::setConversionTime(uint8_t range) { //changes number of bits of resolution, higher resolution = longer conversion time. 0 = 4ms; 1 = 16ms
    int mask = 1<<5;
    int val = CONFIG_REG_VALUE & (mask); val = val >>5;
    if(val!=range){
        CONFIG_REG_VALUE = CONFIG_REG_VALUE ^ mask; //changes range bit if different
    }
     return writeRegister(CONFIG_REG, CONFIG_REG_VALUE); // Enable with default config
}

bool ISL29125::setIRCompensationRange(uint8_t comp) {  //ranges: 0-63 and 106-169
    if(comp<63){
        return writeRegister(OFFSET_REG, comp);
    }
    if(comp>=106 && comp<=169){
        return writeRegister(OFFSET_REG, comp-106+128); //-106+128(bit 7)
    }
    return false; // Enable with default config
}
// bool ISL29125::enablePower() {
//     // uint8_t regVal;
//     // if (!readRegister(ENABLE, regVal)) {
//     //     return false;
//     // }
//     // regVal |= POWER_ON;
//     return writeRegister(CONFIG_REG, 0x0D); // Enable power
// }

 bool ISL29125::disablePower() {
     // uint8_t regVal;
     // if (!readRegister(ENABLE, regVal)) {
     //     return false;
     // }
     // regVal &= ~POWER_ON;
     return writeRegister(CONFIG_REG, 0x00); // Set to power down mode
}

bool ISL29125::SetInterupts(uint8_t interupts) {//0 = disable; 1 = 1 interupt; 2 = 2 interupts; 3 = 4 interupts; 4 = 8 interupts
    uint8_t regVal;
    readRegister(INTERUPT_REG, regVal);
    if(interupts == 0){
        regVal &= 0xF7;
        return writeRegister(INTERUPT_REG, regVal);
    }
    if(interupts <=4){
        regVal &= 0xF8;
        interupts--;
        regVal |= interupts;
    }
    else{
        return false;
    }
    return writeRegister(INTERUPT_REG, regVal);
}

bool ISL29125::SetLowThreshold(uint16_t threshold) {
    uint8_t low = threshold & 0x00FF;
    uint8_t high = (threshold & 0xFF00) >> 8;
    if (!writeRegister(LLow_Threshold, low)) {
        return false;
    }
    if (!writeRegister(LHigh_Threshold, high)) {
        return false;
    }
    return true;
}
bool ISL29125::SetHighThreshold(uint16_t threshold) {
    uint8_t low = threshold & 0x00FF;
    uint8_t high = (threshold & 0xFF00) >> 8;
    if (!writeRegister(HLow_Threshold, low)) {
        return false;
    }
    if (!writeRegister(HHigh_Threshold, high)) {
        return false;
    }
    return true;
}

bool ISL29125::CheckInterrupt() {
    uint8_t status;
    if (!readRegister(STATUS_REG, status)) {
        //rip idk what to do
        //console.log("Failed to read STATUS register");
        return false;
    }
    // For example, assume AINT is bit 4
    return (status & 0x01);
}

bool ISL29125::CheckConversion() {
    uint8_t status;
    if (!readRegister(STATUS_REG, status)) {
        //rip idk what to do
        //console.log("Failed to read STATUS register");
        return false;
    }
    // For example, assume AINT is bit 4
    return (status & 0x02)>>1;
}

bool ISL29125::checkErrors() { //if error true
    uint8_t status;
    if (!readRegister(STATUS_REG, status)) {
        //rip idk what to do
        //console.log("Failed to read STATUS register");
    }
    status ^= 0x02;
    status &= 0x07;
    return status!=0;
}
// bool ISL29125::enableLightSensor(bool interrupts) {
//     uint8_t regVal;
//     if (!readRegister(ENABLE, regVal)) {
//         return false;
//     }
//     regVal |= ALS_ENABLE;
//     return writeRegister(ENABLE, regVal);
// }

// bool ISL29125::disableLightSensor() {
//     uint8_t regVal;
//     if (!readRegister(ENABLE, regVal)) {
//         return false;
//     }
//     regVal &= ~ALS_ENABLE; // clear AEN bit
//     return writeRegister(ENABLE, regVal);
// }

 bool ISL29125::readAllSensors(sensor_data_t& data) {
     uint8_t buf[6];  //RDATAL/H, GDATAL/H, BDATAL/H, 

     if (!readRegisters(0x09, buf, sizeof(buf))) {
         return false;
     }
    
     // Verify device ID
    //  if ((buf[0] != 0xAB) && (buf[0] != 0x9C)) {
    //      return false;
    //  }
    
     // Check if data is valid (STATUS register)
    //  data.valid = (buf[1] & STATUS_AVALID) != 0;
    //  if (!data.valid) {
    //      return false;
    //  }
    
     // Parse the data
     //data.clear = ((uint16_t)buf[3] << 8) | buf[2];
     data.green = ((uint16_t)buf[1] << 8) | buf[0];
     data.red = ((uint16_t)buf[3] << 8) | buf[2];
     data.blue = ((uint16_t)buf[5] << 8) | buf[4];
     //data.proximity = buf[10];
  
     return true;
 }

// bool ISL29125::readAmbientLight(uint16_t &clear) {
//     sensor_data_t data;
//     if (!readAllSensors(data)) {
//         return false;
//     }
//     clear = data.clear;
//     return true;
// }

 bool ISL29125::readRedLight(uint16_t &red) {
     sensor_data_t data;
     if (!readAllSensors(data)) {
        return false;
     }
     red = data.red;
     return true;
 }

bool ISL29125::readGreenLight(uint16_t &green) {
     sensor_data_t data;
     if (!readAllSensors(data)) {
         return false;
     }
     green = data.green;
     return true;
}

bool ISL29125::readBlueLight(uint16_t &blue) {
     sensor_data_t data;
     if (!readAllSensors(data)) {
         return false;
     }
     blue = data.blue;
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