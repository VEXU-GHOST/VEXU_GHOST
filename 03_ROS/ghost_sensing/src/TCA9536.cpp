#include <stdint.h>
#include <memory>
#include "I2C_interfacing.h"

namespace ghost_sensing {

class TCA9536 : public I2C_interfacing {
    
    public:
        TCA9536(std::shared_ptr<I2C_interfacing> iface, uint8_t address, uint16_t len) : I2C_interfacing(iface->filename, iface->logger), addr(address), leng(len){ init(); }
                                               
        TCA9536(std::shared_ptr<I2C_interfacing> iface, uint8_t address, uint8_t out, uint8_t config, uint16_t len) : I2C_interfacing(iface->filename, iface->logger), addr(address), leng(len){
            init();
            write(configuration, &config, leng);    
            write(output, &out, leng);              
        }    
        TCA9536(std::shared_ptr<I2C_interfacing> iface, uint8_t address, uint8_t out, uint8_t pol, uint8_t config, uint8_t special, uint16_t len) : I2C_interfacing(iface->filename, iface->logger), addr(address), leng(len){
            init();
            write(configuration, &config, leng);
            write(output, &out, leng);  
            write(polarity, &pol, leng); 
            write(special_function, &special, leng); 
        }

        ~TCA9536() {deinit();}




}


}