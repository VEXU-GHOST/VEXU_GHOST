import numpy as np
import sympy as sp
import math
class iLQR:
    def __init__(self, Ts: float, t_total, diff_drive, x_0: np.ndarray, u_0: np.ndarray):
        DEG_TO_RAD = math.pi / 180
       
        self.Ts = Ts
        self.t_total = t_total
        self.diff_drive = diff_drive
        self.x_0 = x_0
        self.u_0 = u_0
        self.if_debug = False

        self.x_0_data = np.ndarray(shape = (x_0.shape[0], round(t_total / Ts)))
        self.u_0_data = np.ndarray(shape = (u_0.shape[0], round(t_total / Ts)))
        self.A_data = {}
        self.B_data = {}
        self.f_data = {}

        # Tunable Parameters
        self.Qk = sp.diag(1000, 1000, 0, 0, 0)
        self.Rk = sp.diag(0.001, 0.001)

    """
    Provides solver with better initial guess compared to a forward rollout with no control.
    To test performance of solver
    """
    def initial_forward_rollout(self):
        # Define some initial trajectory using NL dynamics and no control
        if self.if_debug:
            print("Performing Initial Forward Rollout")
        i = 0
        for t in np.arange(0, self.t_total, self.Ts):
            if i == 0:
                self.x_0_data[:, i] = np.zeros_like(self.x_0)
                self.u_0_data[:, i] = np.zeros_like(self.u_0)

            else:
                x_next = np.zeros_like(self.x_0)
                self.x_0_data[:, i] = x_next
                self.u_0_data[:, i] = np.zeros_like(self.u_0)
            i += 1
       
        # Linearize at every Ts
        if self.if_debug:
            print("Performing Linearization")
        i = 0
        for t in np.arange(0, self.t_total, self.Ts):
            Ak, Bk = self.diff_drive.approx_A_B(self.x_0_data[:, i], self.u_0_data[:, i])
            self.A_data[t] = Ak
            self.B_data[t] = Bk

            i += 1

    def get_linear_dyn(self):
        i = 0
        for t in np.arange(0, self.t_total, self.Ts):
            Ak, Bk = self.diff_drive.approx_A_B(self.x_0_data[:, i], self.u_0_data[:, i])
            self.A_data[t] = Ak
            self.B_data[t] = Bk

            i += 1
    """
    Approximate quadratic cost at terminal state
    """
    def approx_VN(self, x_bar):
        if self.if_debug:
            print("Approximating quadratic terminal cost.")
        x = sp.Symbol('x')
        y = sp.Symbol('y')
        tht = sp.Symbol('tht')
        vl = sp.Symbol('vl')
        vr = sp.Symbol('vr')

        l_volts = sp.Symbol('l_volts')
        r_volts = sp.Symbol('r_volts')
        X = sp.Matrix([x, y, tht, vl, vr])

        x_bar_m = sp.Matrix([x_bar[0], x_bar[1], x_bar[2], x_bar[3], x_bar[4]])

        V_N = (X - x_bar_m).T * self.Qk * (X - x_bar_m)

        dV_N = 0.5 * (V_N.jacobian(X)).subs({"x": x_bar[0], "y": x_bar[1], "tht": x_bar[2], "vl": x_bar[3], "vr": x_bar[4]})
        ddV_N = 0.5 * (sp.hessian(V_N, (x, y, tht, vl, vr))).subs({"x": x_bar[0], "y": x_bar[1], "tht": x_bar[2], "vl": x_bar[3], "vr": x_bar[4]})

        return dV_N, ddV_N
   
    def approx_c(self, x_bar: np.ndarray, u_bar: np.ndarray):
        x = sp.Symbol('x')
        y = sp.Symbol('y')
        tht = sp.Symbol('tht')
        vl = sp.Symbol('vl')
        vr = sp.Symbol('vr')

        l_volts = sp.Symbol('l_volts')
        r_volts = sp.Symbol('r_volts')
        X = sp.Matrix([x, y, tht, vl, vr])
        U = sp.Matrix([l_volts, r_volts])

        C =  (X.T * self.Qk * X)[0] + (U.T * self.Rk * U)[0]

        # lx = (C.diff(x)).subs({"q": x_bar[0], "qd": x_bar[1], "tht": x_bar[2], "thtd": x_bar[3], "F" : u_bar})
        # lu = (C.diff(u)).subs({"q": x_bar[0], "qd": x_bar[1], "tht": x_bar[2], "thtd": x_bar[3], \
        #                                  "u" : u_bar})
        # lxx = sp.hessian(C, x).subs({"q": x_bar[0], "qd": x_bar[1], "tht": x_bar[2], "thtd": x_bar[3], \
        #                                  "u" : u_bar})

        # luu = sp.hessian(C, [u]).subs({"q": x_bar[0], "qd": x_bar[1], "tht": x_bar[2], "thtd": x_bar[3], \
        #                                  "u" : u_bar})

       
        # lxu = ((C.diff(x)).diff(u)).subs({"q": x_bar[0], "qd": x_bar[1], "tht": x_bar[2], "thtd": x_bar[3], \
        #                                  "u" : u_bar})

        lx = ((self.Qk @ X)).subs({"x": x_bar[0], "y": x_bar[1], "tht": x_bar[2], "vl": x_bar[3], "vr": x_bar[4]})
        lu = ((self.Rk @ U)).subs({"x": x_bar[0], "y": x_bar[1], "tht": x_bar[2], "vl": x_bar[3], "vr": x_bar[4], "l_volts": u_bar[0], "r_volts": u_bar[1]})
        lxx = self.Qk
        luu = self.Rk
        # hessian(f, (x, y)
        # This is zero for this cost function
        lxu =  (((C.diff(X))[0]).diff(U)).subs({"x": x_bar[0], "y": x_bar[1], "tht": x_bar[2], "vl": x_bar[3], "vr": x_bar[4]})
        # lxu = sp.hessian(C, (X, U))

        return lx, lu, lxx, luu, lxu

    def back_pass(self, x_bar_data: np.ndarray, u_bar_data: np.ndarray):
        if self.if_debug:
            print("Performing Backward Pass")
        t_range = np.arange(0, self.t_total, self.Ts)
        if_term_cost = True
        stage_n_1 = True
        V_data = {}

        x = sp.Symbol('x')
        y = sp.Symbol('y')
        tht = sp.Symbol('tht')
        vl = sp.Symbol('vl')
        vr = sp.Symbol('vr')

        del_x = sp.Matrix([x, y, tht, vl, vr])
        del_u_star_data = {}

    # Traversing backwards
        for n in t_range[::-1]:
            n_index = np.where(t_range == n)[0][0]

            # Stage N
            if if_term_cost:
                sn, Sn = self.approx_VN(x_bar_data[:, n_index])
                x_N = sp.Matrix(x_bar_data[:, n_index])
                V_data[n] = (0.5 * x_N.T * Sn * (x_N) + sn * (x_N))[0]
                if_term_cost = False
           
            # If we are at last index do not run
            if n_index != t_range.size - 1:
                # Stage N - 1
                lx, lu, lxx, luu, lxu = self.approx_c(x_bar_data[:, n_index-1], u_bar_data[:, n_index - 1])

                x_bar = x_bar_data[:, n_index-1]
                u_bar = u_bar_data[:, n_index - 1]

                Ak = self.A_data[n]
                Bk = self.B_data[n]
                Qx = lx + Ak @ sn.T
                Qu = lu + (Bk.T * sn.T)
                Qxx = lxx + Ak.T * Sn * Ak
                Quu = luu + (Bk.T * Sn * Bk)
               
                # TODO: lxu is zeros with vanilla cost function. But change when adding bilinear terms
                lxu = np.zeros_like(Bk.T * Sn * Ak)

                Qux = lxu + Bk.T * Sn * Ak
               
                # Solve for V_data[n+1].
                d = -1 * Quu.inv() * Qu
                Kk = -1 * Quu.inv() * Qux
                del_u_star = d + (Kk * del_x)
                del_V = 0.5 * d.T * Quu * d + d.T * Qu
                print(del_V)
                V_data[round(n, 1)] = V_data[round(n+self.Ts, 1)] + del_V.flat()[0]
                del_u_star_data[round(n, 1)] = del_u_star

                sn = (Qx + Kk.T * Quu * d + Kk.T * Qu + Qux.T * d).T
                Sn = (Qxx + Kk.T * Quu * Kk + Kk.T * Qux + Qux.T * Kk).T

        if self.if_debug:
            print("Completed Backward Pass")
        return del_u_star_data, V_data

    def for_pass(self, x_data, del_u_star_data, old_u_data):
        if self.if_debug:
            print("Performing Forward Pass")
        k = 0
        new_x_data = np.ndarray(shape = (self.x_0.size, round(self.t_total / self.Ts)))
        new_u_data = np.ndarray(shape = (2, round(self.t_total / self.Ts)))
        # t = 0
        # use f(x0, u*) = x1
        # t = 1
        # f(x1, u*(x1-x0)) = x2
        for t in np.arange(0, self.t_total - self.Ts, self.Ts):
            if k == 0:
                new_x_data[:, k] = self.x_0
                u_star = self.u_0_data[k]
                new_x_data[:, k + 1] = self.diff_drive.next_step(self.x_0, u_star)
                new_u_data[k] = u_star

            else:
                del_x = new_x_data[:, k] - x_data[:, k]
                u_star = sp.Matrix((old_u_data[:, k])) + (del_u_star_data[k]).subs({"x": del_x[0], "y": del_x[1], "tht": del_x[2], "vl": del_x[3], "vr": del_x[4]})
               
                new_x_data[:, k + 1] = self.diff_drive.next_step(new_x_data[:, k], u_star)

                new_u_data[:, k] = u_star.flat()

            k += 1
        return new_x_data, new_u_data
