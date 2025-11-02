#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include "ghost_sensing/TCA9536.hpp"

namespace ghost_sensing {

class driver_TCA9536 {
    private:
    vector<uint8_t> inputs (4,0); // 0x01, 0x02, 0x03, 0x50
    uint8_t I2c_address;
    public:
    driver_TCA9536(I2c_address, std::vector<uint8_t> initial_input_states){
        inputs = initial_input_states;
        write_register(0x03, inputs[2]);    //Sets I/O      (1111XXXX 1 for input 0 for output)
        write_register(0x01, inputs[0]);    //Sets Output high/Low  (1111XXXX 1 for output high 0 for output low)
        write_register(0x02, inputs[1]);    //Sets Polarity (0000XXXX 1 for Invert 0 for normal)
        write_register(0x50, inputs[3]);    //Sets P3/Int and PU (XX000000 N8 = 0 disable P3/Int, 1 enable P3/Int; N7 = 0 disable PU, 1 enable PU)
    }

    void write_register(uint8_t reg, uint8_t value){
        return (m_i2c_communication->write(I2c_address, reg, &value, 1) == 0);
    }
    uint8_t read_register(uint8_t reg){
        uint8_t value;
        m_i2c_communication->read(I2c_address, reg, &value, 1);
        return value;

}

} // namespace ghost_sensing
}