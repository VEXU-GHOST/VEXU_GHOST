#pragma once

namespace ghost_common
{

    class V5RobotBase
    {
    public:
        V5RobotBase();
        virtual void autonomous(double current_time) = 0;
        virtual void teleop(double current_time) = 0;

    protected:
    private:
    };

} // namespace ghost_common