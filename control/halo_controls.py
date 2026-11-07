"""Flight code: reference trajectory, Kalman filter, LQR, and roll command.

This module does not import the simulator. Vehicle numbers come from a
HaloParameters object. `lqr` comes from the installed python-control library.
control/__init__.py loads that library before this file runs, because this
folder uses the same name as the library.
"""

import numpy as np
import scipy as sp
from control import lqr

from halo_parameters import HaloParameters


class Controls():
    def __init__(self,
                 H=50.0,
                 V=4.0,
                 Ti=21.0,
                 Th=10.0,
                 Tf=54.0,
                 Q=np.diag([0.1, 350, 0.1, 320, 20, 50]),
                 R=np.diag([2000, 1536]),
                 control_frequency=15,
                 vehicle=None):
        self.H = H
        self.V = V
        self.Ti = Ti
        self.Th = Th
        self.Tf = Tf
        self.R = R
        self.control_frequency = control_frequency
        self.dt = 1/control_frequency
        # Breakpoints of the altitude reference. x1 and x2 bound the climb,
        # Ti starts the hover, x4 ends it, and x6 starts the landing flare.
        self.Z = 0.83
        self.x1 = self.V / self.Z
        self.x2 = (2/self.V) * (self.H+self.V * self.x1 - 0.5* self.Z * (self.x1**2) ) - self.Ti
        self.x4 = self.Ti +self.Th
        self.x5 = self.x4 + (self.Ti - self.x2)
        self.X = self.V/(2*(self.Ti-self.x2))
        self.x6 = (2/self.V) * (self.H+self.V * self.x5 - self.X* ((self.x5-self.x4) **2) ) - self.Tf
        self.Y = self.V/(2*(self.Tf-self.x6))
        self.Q = Q
        self.R_kalman = np.diag([1, 1, 1, 1, 1, 1]) * 1e-5
        if vehicle is None:
            vehicle = HaloParameters()
        self.vehicle = vehicle
        self._build_model(R)

    def set_params(self, H, V, Ti, Th, Tf, Z, Q, R, control_frequency, vehicle):
        # H is accepted and not stored. The hover height set in __init__ stays.
        self.V = V
        self.Ti = Ti
        self.Th = Th
        self.Tf = Tf
        self.R = R
        self.control_frequency = control_frequency
        self.dt = 1/control_frequency
        self.Z = Z
        self.x1 = self.V / self.Z
        self.x2 = (2/self.V) * (self.H+self.V * self.x1 - 0.5* self.Z * (self.x1**2) ) - self.Ti
        self.x4 = self.Ti +self.Th
        self.x5 = self.x4 + (self.Ti - self.x2)
        self.X = self.V/(2*(self.Ti-self.x2))
        self.x6 = (2/self.V) * (self.H+self.V * self.x5 - self.X* ((self.x5-self.x4) **2) ) - self.Tf
        self.Y = self.V/(2*(self.Tf-self.x6))
        self.Q = Q
        self.R_kalman = np.diag([1, 1, 1, 1, 1, 1]) * 1e-5
        self.vehicle = vehicle
        self._build_model(R)

    def _build_model(self, R):
        self.Q_d = np.identity(self.vehicle.Q_kalman.shape[0]) + self.vehicle.Q_kalman * self.dt
        self.R_d = self.R_kalman / self.dt
        self.Iyy = (1/12) * self.vehicle.m0 * (self.vehicle.l ** 2)
        self.Izz = (1/2) * self.vehicle.m0 * (self.vehicle.r ** 2)

        # Onboard model. Six states: x, z, xdot, zdot, theta, thetadot.
        self.Atc = np.zeros([6, 6])
        self.Atc[0, 2] = 1
        self.Atc[1, 3] = 1
        self.Atc[2, 4] = -self.vehicle.g
        self.Atc[4, 5] = 1

        self.Btc = np.zeros([6, 2])
        self.Btc[2, 0] = -self.vehicle.g
        self.Btc[3, 1] = -self.vehicle.T/self.vehicle.m0
        self.Btc[5, 0] = self.vehicle.dNoz*self.vehicle.m0*self.vehicle.g/self.Iyy

        Concatenated = np.concatenate((np.concatenate((self.Atc, self.Btc), axis=1), np.zeros((2, 8))), axis=0)
        Exp_Concat = np.round(sp.linalg.expm(Concatenated * self.dt), decimals=4)
        self.Atd = Exp_Concat[:6, :6]
        self.Btd = Exp_Concat[:6, 6:]

        self.K = lqr(self.Atc, self.Btc, self.Q, R)[0]

    def z_ref(self, t):
        if t < self.x1:
            return 0.5 * self.Z * (t ** 2)
        elif (t > self.x1) and (t < self.x2):
            return self.V * (t - self.x1) + 0.5 * self.Z * (self.x1 ** 2)
        elif (t > self.x2) and (t < self.Ti):
            return -self.X * ((t - self.Ti) ** 2) + self.H
        elif (t > self.Ti) and (t < self.x4):
            return self.H
        elif (t > self.x4) and (t < self.x5):
            return -self.X * ((t - self.x4) ** 2) + self.H
        elif (t > self.x5) and (t < self.x6):
            return -self.V * (t - self.x5) + self.H - self.X * ((self.x5 - self.x4) ** 2)
        elif (t > self.x6) and (t < self.Tf):
            return self.Y * ((t - self.Tf) ** 2)
        else:
            return 0

    def dz_ref(self, t):
        return (self.z_ref(t + 0.001) - self.z_ref(t)) / 0.001

    def LQR(self, t_hat, State_hat, u):
        x_hat, z_hat, dx_hat, dz_hat, theta_hat, dtheta_hat = State_hat
        gamma0, P0 = u

        gamma, P = -self.K @ [x_hat, z_hat-self.z_ref(t_hat), dx_hat, dz_hat-self.dz_ref(t_hat), theta_hat, dtheta_hat]

        bP = 1.0
        dP = P-P0
        P = P0 + np.clip((dP/self.dt), -0.5, 0.5)*self.dt / bP
        P = np.clip(P, 0, self.vehicle.P_max)

        bgamma = 1.0
        dgamma = gamma-gamma0
        gamma = gamma0 + np.clip((dgamma/self.dt), -0.5, 0.5)*self.dt / bgamma
        gamma = np.clip(gamma, -0.22, 0.22)

        return [gamma, P]

    def Kalman_Filter(self, observed_state, kalman_state, kalman_covariance, u):
        apriori_state = self.Atd@kalman_state + self.Btd@u
        apriori_covariance = self.Atd@kalman_covariance@self.Atd.T + self.Q_d
        kalman_gain = (apriori_covariance) @ np.linalg.inv(apriori_covariance + self.R_d)

        aposteriori_state = apriori_state + kalman_gain@(observed_state - apriori_state)
        aposteriori_covariance = (np.identity(6) - kalman_gain)@apriori_covariance@(np.identity(6) - kalman_gain).T + kalman_gain@self.R_d@kalman_gain.T

        return [aposteriori_state, aposteriori_covariance]

    def Roll_control(self, psi, dpsi, f_prev):
        ta = ( (self.Izz/self.vehicle.Tau0) * dpsi * np.sign(psi) + np.sqrt( (self.Izz/self.vehicle.Tau0) * np.abs(psi) + 0.5 * (((self.Izz/self.vehicle.Tau0) * dpsi) ** 2) ) )
        tb = ( (self.Izz/self.vehicle.Tau0) * dpsi * np.sign(psi) + 2 * np.sqrt( (self.Izz/self.vehicle.Tau0) * np.abs(psi) + 0.5 * (((self.Izz/self.vehicle.Tau0) * dpsi) ** 2) ) )
        k = 2
        e_on = 0.02
        e_off = 0.02 * 0.3
        if f_prev == 0:
            if abs(psi+k*dpsi) > e_on:
                if np.ceil(ta/(self.dt*1))*(self.dt*1) > 0:
                    f = -np.sign(psi)
                elif np.ceil(tb/(self.dt*1))*(self.dt*1) > 0:
                    f = np.sign(psi)
                else:
                    f = 0
            else:
                f = 0
        else:
            if abs(psi+k*dpsi) < e_off:
                f = 0
            else:
                if np.ceil(ta/(self.dt*1))*(self.dt*1) > 0:
                    f = -np.sign(psi)
                elif np.ceil(tb/(self.dt*1))*(self.dt*1) > 0:
                    f = np.sign(psi)
                else:
                    f = 0
        return f

    def LQR_control_algo(self, t, state, control_t_prev, control_state_prev, u_prev):
        # Hold the previous command until one control period (dt) has passed.
        if (t - control_t_prev > self.dt):
            # Planar measurements taken from the 13-state vector.
            # x-z uses x, z, dx, dz, theta, dtheta. y-z uses y, z, dy, dz, phi, dphi.
            observed_state_x = [state[i] for i in [0, 2, 3, 5, 7, 10]] * np.array([1, 1, 1, 1, -1, -1])
            observed_state_y = [state[i] for i in [1, 2, 4, 5, 6, 9]] * np.array([-1, 1, -1, 1, -1, -1])
            observed_roll = [state[i] for i in [8, 11]]

            v = np.random.multivariate_normal(np.zeros(6), self.R_kalman)
            observed_state_x += self.R_d@v
            observed_state_y += self.R_d@v

            kalman_state_x, kalman_state_y, kalman_roll, kalman_cov_x, kalman_cov_y = control_state_prev
            P, gx, gy, f = u_prev

            aposteriori_state_x, aposteriori_covariance_x = self.Kalman_Filter(observed_state_x, kalman_state_x, kalman_cov_x, [gy, P])
            aposteriori_state_x = aposteriori_state_x * np.array([1, 1, 1, 1, -1, -1])
            kalman_state_x = aposteriori_state_x
            kalman_cov_x = aposteriori_covariance_x

            aposteriori_state_y, aposteriori_covariance_y = self.Kalman_Filter(observed_state_y, kalman_state_y, kalman_cov_y, [gx, P])
            aposteriori_state_y = aposteriori_state_y * np.array([-1, 1, -1, 1, -1, -1])
            kalman_state_y = aposteriori_state_y
            kalman_cov_y = aposteriori_covariance_y

            gy, P1 = self.LQR(t, kalman_state_x * np.array([1, 1, 1, 1, -1, -1]), [gy, P])
            gx, P2 = self.LQR(t, kalman_state_y * np.array([-1, 1, -1, 1, -1, -1]), [gx, P])
            P = (P1+P2)/2

            kalman_roll = np.array(observed_roll)
            psi = observed_roll[0]
            dpsi = observed_roll[1]
            f = self.Roll_control(psi, dpsi, f)

            return [[t, kalman_state_x, kalman_state_y, kalman_roll, kalman_cov_x, kalman_cov_y], [P, gx, gy, f]]
        else:
            kalman_state_x, kalman_state_y, kalman_roll, kalman_cov_x, kalman_cov_y = control_state_prev
            return [[control_t_prev, kalman_state_x, kalman_state_y, kalman_roll, kalman_cov_x, kalman_cov_y], u_prev]
