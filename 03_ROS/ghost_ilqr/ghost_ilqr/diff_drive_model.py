import numpy as np
from typing import Tuple
import rclpy
from rclpy.node import Node
import math
import sympy as sp
from sympy import Symbol, Matrix, sin, cos
from nav_msgs.msg import Path, Odometry

import sys
sys.path.append('../')

from ghost_ilqr.ilqr import iLQR

"""
Kinematic Differential Drive model
In world frame

TODO: run iLQR multiple times and give it non zero wheel velocity initial trajectory
Ak and Bk linearized about vl == vr == 0 make rows of Ak zero (0.5 * vl * vr)
"""
DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 180 / math.pi
IN_TO_M = 1/39.37
LBS_TO_KG = 1 / 2.205
       
L = 10 * IN_TO_M  # Length between wheels
r_b = L / 2       # Radius of base to wheel
r_w = 1 * IN_TO_M # Wheel radius

# TODO : ask
tau_stall = 2.42     # Stall torque Nm
max_volt = 10        # Volts
free_speed = 4110    # RPM
m = 20 * LBS_TO_KG   # kg
I = 2                # kg m^2
M = sp.diag(m, m, I) # Mass matrix

class DiffDrive():
    def __init__(self, Ts: float, n: int, m: int):
        """
        Initialize the Differential Drive model
        Inputs:
        Ts: float (the simulation step size)
        """

        self.Ts = Ts
        self.n = n # number of states
        self.m = m # number of controls

    def cosd(self, tht):
        return math.cos(tht * self.DEG_TO_RAD)

    def spy_cosd(self, tht):
        return cos(tht * sp.pi / 180)
   
    def sind(self, tht):
        return math.sin(tht * self.DEG_TO_RAD)
   
    def spy_sind(self, tht):
        return sin(tht * sp.pi / 180)

    def f_syms(self):
        x = Symbol('x')
        y = Symbol('y')
        tht = Symbol('tht')
        vl = Symbol('vl')
        vr = Symbol('vr')

        l_volts = Symbol('l_volts')
        r_volts = Symbol('r_volts')

        x_ = Matrix([x, y, tht, vl, vr])
        y_ = Matrix([l_volts, r_volts])
        # wheel states to base states Jacobian, J
        J = 1 / 2 * Matrix([[cos(tht), cos(tht)],
                               [sin(tht), sin(tht)],
                               [-1 / L, 1 / L]])
        # base states to wheel states Jacobian, J_plus
        J_plus = Matrix([[cos(tht), sin(tht), -L],
                        [cos(tht), sin(tht),  L]])
                         
        dJ_plus = Matrix([[-1 * sin(tht), cos(tht), 0],
                         [-1 * sin(tht), cos(tht), 0]])
        up_right = J
        low_right = dJ_plus * J - J_plus * M ** -1 * J_plus.T * tau_stall / (r_w ** 2 * free_speed)

        # Padding
        up_left = Matrix(np.zeros((self.n - low_right.shape[0], self.n - up_right.shape[1])))
        low_left = Matrix(np.zeros((self.n - up_right.shape[0], self.n - low_right.shape[1])))
        H = Matrix(sp.BlockMatrix([[up_left, up_right],
                            [low_left, low_right]]))
       
        C = Matrix(sp.BlockMatrix([[low_left.T],
                            [J_plus * M ** -1 * J_plus.T * tau_stall / (r_w ** 2 * free_speed)]]))

        return H * x_ + C * y_

    """
    Nonlinear Dynamics
    """
    def f(self, x_data: np.ndarray, u_data: np.ndarray):
        H = np.zeros(self.n)
       
        x = x_data[0]
        y = x_data[1]
        tht = x_data[2]
        vl = x_data[3]
        vr = x_data[4]
        l_volts = u_data[0]
        r_volts = u_data[1]
       
        x_ = Matrix([x, y, tht, vl, vr])
        y_ = Matrix([l_volts, r_volts])
        # wheel states to base states Jacobian, J
        J = 1 / 2 * Matrix([[cos(tht), cos(tht)],
                               [sin(tht), sin(tht)],
                               [-1 / L, 1 / L]])
        # base states to wheel states Jacobian, J_plus
        J_plus = Matrix([[cos(tht), sin(tht), -L],
                        [cos(tht), sin(tht),  L]])
                         
        dJ_plus = Matrix([[-1 * sin(tht), cos(tht), 0],
                         [-1 * sin(tht), cos(tht), 0]])


        up_right = J
        low_right = dJ_plus * J - J_plus * M ** -1 * J_plus.T * tau_stall / (r_w ** 2 * free_speed)

        # Padding
        up_left = Matrix(np.zeros((self.n - low_right.shape[0], self.n - up_right.shape[1])))
        low_left = Matrix(np.zeros((self.n - up_right.shape[0], self.n - low_right.shape[1])))
        H = Matrix(sp.BlockMatrix([[up_left, up_right],
                            [low_left, low_right]]))
       
        C = Matrix(sp.BlockMatrix([[low_left.T],
                            [J_plus * M ** -1 * J_plus.T * tau_stall / (r_w ** 2 * free_speed)]]))
       
        return H * x_ + C * y_
 
    def approx_A_B(self, x_bar: np.ndarray, u_bar: np.ndarray) -> Tuple[np.ndarray]:
        """
        For the given state and control, returns approximations of the A
        matrices
        Inputs:
        x: 2D array of shape (n, 1)
        u: 2D array of shape (m, 1)

        Returns:
        A: 2D array of shape (n, n)
        B: 2D array of shape (n, m)
        """
        x = Symbol('x')
        y = Symbol('y')
        tht = Symbol('tht')
        vl = Symbol('vl')
        vr = Symbol('vr')

        l_volts = Symbol('l_volts')
        r_volts = Symbol('r_volts')
        f_syms = self.f_syms()

        # Define A jacobian
        X = Matrix([x, y, tht, vl, vr])
        A_j = f_syms.jacobian(X)
        A_j_n = A_j.subs({"x": x_bar[0], "y": x_bar[1], "tht": x_bar[2], "vl": x_bar[3], "vr": x_bar[4]})

        # Define B jacobian
        U = Matrix([l_volts, r_volts])
        B_j = f_syms.jacobian(U)
        B_j_n = B_j.subs({"x": x_bar[0], "y": x_bar[1], "tht": x_bar[2], "vl": x_bar[3], "vr": x_bar[4]})

        Ak = np.eye(self.n) + self.Ts * A_j_n
        Bk = self.Ts * B_j_n

        return Ak, Bk

    def next_step(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        For the given state and control, returns the next state
        Inputs:
        x: 2D array of shape (n, 1) [q qd tht thtd]
        u: 2D array of shape (m, 1)
        Returns:
        x_next: 2D array of shape (n, 1)
        """
        fx = (self.f(x, u) * self.Ts).flat()
        x_next = x + fx

        return x_next
   
    def sympy_to_np(self, m):
        return np.array(m).astype(np.float64)

