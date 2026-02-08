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
    
     Initialize with defaults

     uint8_t colors = ALLCOLORS; // Default configuration value
     uint8_t sensor_range = LARGE_SENSOR_RANGE;
     uint8_t total = colors + sensor_range; // Default configuration value

     if (!writeRegister(CONFIG_REG, total) ||          //set colors and range
         !writeRegister(OFFSET_REG, DEFAULT_OFFSET) ||  // Set integration time
      
         return false;
     }
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

bool ISL29125::setConversionTime(uint8_t range) { //0 = start at i2c write 0x01; 1 = start at rising edge of INT
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
        regVal &= 0xF7
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
