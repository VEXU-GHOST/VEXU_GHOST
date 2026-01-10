#include <stdint.h>
#include <memory>
#include "I2C_interfacing.h"

namespace ghost_sensing {


//Defines for shorthand access of registers/names/etc for TCA9536
#define input 0x00  // Register that Reads inputs
#define output 0x01 // Register that sets outputs (HIGH/LOW)
#define polarity 0x02   // Register that swaps Polarity of inputs. (What is low is high vice versa)
#define configuration 0x03 // Register that sets pin direction (Input(1)/Output(0))
#define special_function 0x50 // Register S7 sets Interrupt and S6 sets Pullup Disable(1)/Enable(0)
#define TCA9536_ 0x41  // Default Address for TCA9536
#define TCA9536A 0x40
#define TCA9536B 0x43
#define TCA9536C 0x42   // Address options for TCA9536, Default is 0x41 Statements set up so no address needed for 0x41

class TCA9536{
    private:
    std::shared_ptr<I2C_interfacing> m_i2c_communication;
    uint8_t address;
    uint16_t leng;
    public:
    
    // Has options for manual, semi-auto, and full auto configuration on instantiation       



    // TCA9536 p(iface, TCA9536_, length);
 TCA9536(std::shared_ptr<I2C_interfacing> iface, uint8_t address, uint16_t len);



    // Example setup: out = xxxx0101 --> Pins 1 and 3 are output LOW, Pins 0 and 2 are output HIGH
    //                config = xxxx0101 --> Pins 0 and 2 are inputs, Pins 1 and 3 are outputs
    // TCA9536 p(iface, TCA9536_, 0b00000101, 0b00000101, length);
    TCA9536(std::shared_ptr<I2C_interfacing> iface, uint8_t address, uint8_t out, uint8_t config, uint16_t len);                                   
    



    // Example setup: polarity = xxxx0101 --> Pins 0 and 2 are inverted polarity, Pins 1 and 3 are normal polarity
    //                special = 11xxxxxx --> Pin 3 is set as interrupt, Pin 2 has pullup resistor disabled
    // TCA9536 p(iface, TCA9536_, 0b00000101, 0b00000101, 0b00000101, 0b11000000, length);
   TCA9536(std::shared_ptr<I2C_interfacing> iface, uint8_t address, uint8_t out, uint8_t pol, uint8_t config, uint8_t special, uint16_t len);


    
    //destructor
    ~TCA9536();


bool writeRegister(uint8_t reg, uint8_t data);
bool readRegister(uint8_t reg, uint8_t &data);
bool readRegisters(uint8_t reg, uint8_t *buf, uint16_t len);


};

} // namespace ghost_sensing