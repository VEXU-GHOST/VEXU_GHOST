#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

a = -8/27
b = 20/9
c = -32/9
d = 71/27

def position(t):
    return a*t**3 + b*t**2 + c*t + d

def velocity(t):
    return 3*a*t**2 + 2*b*t + c

def accel(t):
    return 6*a*t + 2*b

t = np.arange(1, 4, 0.01)

plt.figure()
plt.subplot(3, 1, 1)
plt.ylabel("Position")
plt.xlabel("Time")
plt.plot(t, position(t))
plt.subplot(3, 1, 2)
plt.ylabel("Velocity")
plt.xlabel("Time")
plt.plot(t, velocity(t))
plt.subplot(3, 1, 3)
plt.ylabel("Acceleration")
plt.xlabel("Time")
plt.plot(t, accel(t))
plt.show()