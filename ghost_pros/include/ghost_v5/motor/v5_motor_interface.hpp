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

        float getInterfaceVelocity() const;
        float getInterfacePosition() const;

        bool getDeviceIsConnected();
        float getCurrentDraw() const;
        float getTemperature() const;
        float getPower() const;

        void setCurrentLimit(float limit);

    private:
        void setInterfaceCommand(float voltage);

        bool device_connected_;

        std::unique_ptr<pros::Motor> motor_interface_ptr_;
    };

} // namespace ghost_v5