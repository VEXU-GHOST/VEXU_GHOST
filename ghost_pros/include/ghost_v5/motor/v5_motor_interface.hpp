#pragma once
#include "ghost_control/motor_controller.hpp"
#include "pros/motors.hpp"
#include "pros/apix.h"
#include "pros/error.h"

#include <unordered_map>
namespace ghost_v5
{
    class V5MotorInterface : public ghost_control::MotorController
    {
    public:
        V5MotorInterface(int port, bool reversed, const ghost_v5_config::MotorConfigStruct &config);

        void updateInterface();

        bool getDeviceIsConnected()
        {
            return device_connected_;
        }

        std::shared_ptr<pros::Motor> getMotorInterfacePtr(){
            return motor_interface_ptr_;
        }

    private:
        std::shared_ptr<pros::Motor> motor_interface_ptr_;
        bool device_connected_;
    };

} // namespace ghost_v5