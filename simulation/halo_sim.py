"""Plant: the 13-state rocket. This module does not call the controller.

The runner stores the current command on `sim.command` before each integrator
step. `update` only returns the derivative for the state it is given.
Vehicle numbers come from a params dict supplied by the notebook.
"""

import numpy as np

class RocketSim:
    def __init__(self, params):
        self.params = params
        self.command = [0, 0, 0, 0]

    def configure_wind(self, wind_speed=0.0, wind_direction=0.0,
                           gust_variance=0.0, seed=None):
            """Configure steady horizontal wind and a repeatable filtered gust path.
    
            ``wind_direction`` is measured in radians from +x toward +y.
            ``gust_variance`` is the long-run variance in (m/s)^2 of each horizontal
            component.  The one-second Ornstein-Uhlenbeck filter avoids feeding
            discontinuous white noise into the adaptive BDF solver.  Pre-generating
            the path also guarantees that repeated derivative evaluations at the
            same time see the same wind.
            """
            if wind_speed < 0:
                raise ValueError("wind_speed must be non-negative")
            if gust_variance < 0:
                raise ValueError("gust_variance must be non-negative")
            self.wind_speed = float(wind_speed)
            self.wind_direction = float(wind_direction)
            self.gust_variance = float(gust_variance)
            self.wind_seed = seed
            self.gust_time_constant = 1.0
            self.gust_dt = 0.05
    
            end_time = max(float(self.t_final), self.gust_dt)
            self.wind_times = np.arange(0.0, end_time + self.gust_dt, self.gust_dt)
            self.gust_cache = np.zeros((len(self.wind_times), 2))
            if self.gust_variance > 0:
                rng = np.random.default_rng(seed)
                decay = np.exp(-self.gust_dt / self.gust_time_constant)
                noise_scale = np.sqrt(self.gust_variance * (1.0 - decay**2))
                for i in range(1, len(self.wind_times)):
                    self.gust_cache[i] = decay * self.gust_cache[i-1] + noise_scale * rng.standard_normal(2)
    
            self.steady_wind = self.wind_speed * np.array([
                np.cos(self.wind_direction), np.sin(self.wind_direction)
            ])
    
    def wind_at(self, t):
        """Return the horizontal steady-plus-gust wind vector at time ``t``."""
        gust = np.array([
            np.interp(t, self.wind_times, self.gust_cache[:, 0]),
            np.interp(t, self.wind_times, self.gust_cache[:, 1]),
        ])
        return np.array([*(self.steady_wind + gust), 0.0])

    def update(self, t, state):
        # state: x, y, z, dx, dy, dz, phi, theta, psi, dphi, dtheta, dpsi, m
        x, y, z, dx, dy, dz, phi, theta, psi, dphi, dtheta, dpsi, m = (state)
        P, gx, gy, f = self.command # for t = 0, it sets all of them equal to 0
        vehicle = self.params

        dm = (-vehicle["m_dot"]/vehicle["T"]) * vehicle["T"] * (1-P)

        Ixx = (1/12) * m * (vehicle["l"] ** 2)
        Iyy = (1/12) * m * (vehicle["l"] ** 2)
        Izz = (1/2) * m * (vehicle["r"] ** 2)
        dIxx = (1/12) * dm * (vehicle["l"] ** 2)
        dIyy = (1/12) * dm * (vehicle["l"] ** 2)
        dIzz = (1/2) * dm * (vehicle["r"] ** 2)

        ddx = -vehicle["T"]*(P - 1)*(((np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))*np.cos(gy) + np.sin(gy)*np.cos(psi)*np.cos(theta))*np.cos(gx) - (np.sin(phi)*np.sin(theta)*np.cos(psi) - np.sin(psi)*np.cos(phi))*np.sin(gx))/m
        ddy = vehicle["T"]*(P - 1)*(((np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi))*np.cos(gy) - np.sin(gy)*np.sin(psi)*np.cos(theta))*np.cos(gx) + (np.sin(phi)*np.sin(psi)*np.sin(theta) + np.cos(phi)*np.cos(psi))*np.sin(gx))/m
        ddz = (vehicle["T"]*(P - 1)*((np.sin(gy)*np.sin(theta) - np.cos(gy)*np.cos(phi)*np.cos(theta))*np.cos(gx) + np.sin(gx)*np.sin(phi)*np.cos(theta)) - vehicle["g"]*m)/m
        ddphi = (Ixx*Iyy*Izz*dtheta*(dphi*np.tan(theta) + dpsi/np.cos(theta)) - Ixx*Iyy*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + Iyy*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + dIzz*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)))*np.cos(phi)*np.tan(theta) + Ixx*Izz*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + Izz*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + vehicle["T"]*vehicle["dNoz"]*(P - 1)*np.sin(gy)*np.cos(gx) - dIyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)))*np.sin(phi)*np.tan(theta) + Iyy*Izz*(Iyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) - Izz*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + vehicle["T"]*vehicle["dNoz"]*(P - 1)*np.sin(gx) - dIxx*(dphi - dpsi*np.sin(theta))))/(Ixx*Iyy*Izz)
        ddtheta = (-Iyy*Izz*dphi*dpsi*np.cos(theta) + Iyy*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + Iyy*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + dIzz*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)))*np.sin(phi) + Izz*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + Izz*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + vehicle["T"]*vehicle["dNoz"]*(P - 1)*np.sin(gy)*np.cos(gx) - dIyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)))*np.cos(phi))/(Iyy*Izz)
        ddpsi = (Iyy*Izz*dtheta*(dphi + dpsi*np.sin(theta)) - Iyy*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + Iyy*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + dIzz*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)))*np.cos(phi) + Izz*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + Izz*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + vehicle["T"]*vehicle["dNoz"]*(P - 1)*np.sin(gy)*np.cos(gx) - dIyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)))*np.sin(phi))/(Iyy*Izz*np.cos(theta)) + (vehicle["Tau0"]/Izz) * f

        return [dx, dy, dz, ddx, ddy, ddz, dphi, dtheta, dpsi, ddphi, ddtheta, ddpsi, dm]
