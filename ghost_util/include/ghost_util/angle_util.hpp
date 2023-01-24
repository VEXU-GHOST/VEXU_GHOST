#ifndef GHOST_UTILS__ANGLE_UTIL_HPP
#define GHOST_UTILS__ANGLE_UTIL_HPP

#include <cmath>

namespace ghost_util
{
    float WrapAngle360(float angle);
    float WrapAngle180(float angle);

    float FlipAngle180(float angle);

    float SmallestAngleDist(float a1, float a2);
}

#endif