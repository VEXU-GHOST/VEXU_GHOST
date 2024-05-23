/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#pragma once

#include <cmath>

namespace ghost_util
{

double WrapAngle360(double angle);
double WrapAngle180(double angle);
double FlipAngle180(double angle);

double WrapAngle2PI(double angle);
double WrapAnglePI(double angle);
double FlipAnglePI(double angle);

double SmallestAngleDistDeg(double a2, double a1);
double SmallestAngleDistRad(double a2, double a1);

double quaternionToYawDeg(double w, double x, double y, double z);
void yawToQuaternionDeg(double yaw, double & w, double & x, double & y, double & z);

double quaternionToYawRad(double w, double x, double y, double z);
void yawToQuaternionRad(double yaw, double & w, double & x, double & y, double & z);

} // namespace ghost_util
