#include "ghost_util/angle_util.hpp"

namespace ghost_util
{
    float WrapAngle360(float angle){
        angle = fmod(angle, 360.0);

        if(angle < 0.0){
            angle += 360.0;
        }

        return angle;
    }

    float WrapAngle180(float angle){
        angle = WrapAngle360(angle);

        if(angle > 180.0){
            angle -= 360.0;
        }

        return angle;
    }

    float WrapAngle2PI(float angle){
        return DEG_TO_RAD * WrapAngle360(angle * RAD_TO_DEG);
    }

    float WrapAnglePI(float angle){
        return DEG_TO_RAD * WrapAngle180(angle * RAD_TO_DEG);
    }

    float FlipAngle180(float angle){
        return WrapAngle180(angle + 180.0);
    }

    float FlipAnglePI(float angle){
        return WrapAnglePI(angle + M_PI);
    }

    float SmallestAngleDistDeg(float a2, float a1){
        a1 = WrapAngle360(a1);
        a2 = WrapAngle360(a2);

        float diff = a2 - a1;
        if(diff > 180.0){
            diff = -(360.0 - diff);
        }
        else if(diff < -180.0){
            diff = 360.0 + diff;
        }
        return diff;
    }

    float SmallestAngleDistRad(float a2, float a1){
        return DEG_TO_RAD * SmallestAngleDistDeg(RAD_TO_DEG * a2, RAD_TO_DEG * a1);
    }
}


