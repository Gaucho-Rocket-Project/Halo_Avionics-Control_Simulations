"""Plant: the 13-state rocket. This module does not call the controller.

The runner stores the current command on `sim.command` before each integrator
step. `update` only returns the derivative for the state it is given.
Vehicle numbers come from a params dict supplied by the notebook.
"""

import numpy as np

class RocketSim:
    def __init__(self, params : dict):
        self.T : float = params["T"]
        self.P_max : float = params["P_max"]
        self.m0 : float = params["m0"]
        self.m_dot : float = params["m_dot"]
        self.g : float = params["g"]
        self.dNoz : float = params["dNoz"]
        self.t_final : float = params["t_final"]
        self.l : float = params["l"]
        self.r : float = params["r"]
        self.Tau0 : float = params["Tau0"]
        self.Cd0 : float = params["Cd0"]
        self.Cl0 : float = params["Cl0"]
        self.S0 : float = params["S0"]
        self.dCoP : float = params["dCoP"]
        self.rho : float = params["rho"]
        self.State0 : list = params["State0"]

        # wind
        self.wind_speed : float = params["wind_speed"]
        self.wind_direction : float = params["wind_direction"]
        self.gust_variance : float = params["gust_variance"]
        self.wind_seed : int | None = params["wind_seed"]
        self.gust_time_constant = 1.0
        self.gust_dt = 0.05

        self.Q_kalman : np.ndarray = params["Q_kalman"]

        self.command = [0, 0, 0, 0]
        self.configure_wind(params["wind_speed"], params["wind_direction"], params["gust_variance"], params["wind_seed"])

    def configure_wind(self, wind_speed, wind_direction, gust_variance, seed):
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
        x, y, z, dx, dy, dz, phi, theta, psi, dphi, dtheta, dpsi, m = (state)
        P, gx, gy, f = self.command

        # Aerodynamic loads depend on velocity relative to the air. Preserve
        # ground-relative velocity for the position derivatives returned below.
        ground_dx, ground_dy, ground_dz = dx, dy, dz
        wind_x, wind_y, wind_z = self.wind_at(t)
        dx, dy, dz = dx-wind_x, dy-wind_y, dz-wind_z
        
        dm = (-self.m_dot/self.T) * self.T * (1-P)

        air_speed = np.sqrt(dx**2 + dy**2 + dz**2)
        if air_speed == 0:
            alpha = 0
        else:
            body_axis_projection = (dx*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi)) + dy*(-np.sin(phi)*np.cos(psi) + np.sin(psi)*np.sin(theta)*np.cos(phi)) + dz*np.cos(phi)*np.cos(theta))/air_speed
            alpha = np.arccos(np.clip(body_axis_projection, -1.0, 1.0))

        Ixx = (1/12) * m * (self.l ** 2)
        Iyy = (1/12) * m * (self.l ** 2)
        Izz = (1/2) * m * (self.r ** 2)
        dIxx = (1/12) * dm * (self.l ** 2)
        dIyy = (1/12) * dm * (self.l ** 2)
        dIzz = (1/2) * dm * (self.r ** 2)

        ddx = (-self.Cd0*self.S0*dx*self.rho*np.sqrt(dx**2 + dy**2 + dz**2) - self.Cl0*alpha*self.S0*self.rho*(dy*(dx*(-np.sin(phi)*np.cos(psi) + np.sin(psi)*np.sin(theta)*np.cos(phi)) - dy*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) - dz*(-dx*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi)))) - 2*self.T*(P - 1)*(((np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))*np.cos(gy) + np.sin(gy)*np.cos(psi)*np.cos(theta))*np.cos(gx) - (np.sin(phi)*np.sin(theta)*np.cos(psi) - np.sin(psi)*np.cos(phi))*np.sin(gx)))/(2*m)
        ddy = (-self.Cd0*self.S0*dy*self.rho*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*self.S0*self.rho*(-dx*(-dx*(-np.sin(phi)*np.cos(psi) + np.sin(psi)*np.sin(theta)*np.cos(phi)) + dy*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) + dz*(-dy*np.cos(phi)*np.cos(theta) + dz*(-np.sin(phi)*np.cos(psi) + np.sin(psi)*np.sin(theta)*np.cos(phi)))) - 2*self.T*(P - 1)*((-(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi))*np.cos(gy) + np.sin(gy)*np.sin(psi)*np.cos(theta))*np.cos(gx) - (np.sin(phi)*np.sin(psi)*np.sin(theta) + np.cos(phi)*np.cos(psi))*np.sin(gx)))/(2*m)
        ddz = (-self.Cd0*self.S0*dz*self.rho*np.sqrt(dx**2 + dy**2 + dz**2)/2 - self.Cl0*alpha*self.S0*self.rho*(dx*(-dx*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) - dy*(dy*np.cos(phi)*np.cos(theta) - dz*(-np.sin(phi)*np.cos(psi) + np.sin(psi)*np.sin(theta)*np.cos(phi))))/2 + self.T*(P - 1)*((np.sin(gy)*np.sin(theta) - np.cos(gy)*np.cos(phi)*np.cos(theta))*np.cos(gx) + np.sin(gx)*np.sin(phi)*np.cos(theta)) - self.g*m)/m
        ddphi = (Ixx*Iyy*Izz**2*dtheta*(dphi*np.tan(theta) + dpsi/np.cos(theta)) + Ixx*Iyy*(Izz*(Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) - Iyy*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) - dIzz*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi))) + self.Tau0*f)*np.cos(phi)*np.tan(theta) - Ixx*Izz**2*(2*Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) - 2*Izz*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + self.S0*self.dCoP*self.rho*((-self.Cd0*dx*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dy*(dx*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)) + dy*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) + dz*(-dx*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi)))))*np.cos(psi)*np.cos(theta) - (self.Cd0*dy*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dx*(dx*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)) + dy*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) + dz*(dy*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)))))*np.sin(psi)*np.cos(theta) + (self.Cd0*dz*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dx*(-dx*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) - dy*(dy*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)))))*np.sin(theta)) - 2*self.T*self.dNoz*(P - 1)*np.sin(gy)*np.cos(gx) + 2*dIyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)))*np.sin(phi)*np.tan(theta)/2 + Iyy*Izz**2*(2*Iyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) - 2*Izz*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) - self.S0*self.dCoP*self.rho*(-(-self.Cd0*dx*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dy*(dx*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)) + dy*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) + dz*(-dx*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi)))))*(np.sin(phi)*np.sin(theta)*np.cos(psi) - np.sin(psi)*np.cos(phi)) + (self.Cd0*dy*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dx*(dx*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)) + dy*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) + dz*(dy*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)))))*(np.sin(phi)*np.sin(psi)*np.sin(theta) + np.cos(phi)*np.cos(psi)) + (self.Cd0*dz*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dx*(-dx*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) - dy*(dy*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)))))*np.sin(phi)*np.cos(theta)) + 2*self.T*self.dNoz*(P - 1)*np.sin(gx) - 2*dIxx*(dphi - dpsi*np.sin(theta)))/2)/(Ixx*Iyy*Izz**2)
        ddtheta = (-Iyy*Izz**2*dphi*dpsi*np.cos(theta) - Iyy*(Izz*(Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) - Iyy*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) - dIzz*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi))) + self.Tau0*f)*np.sin(phi) - Izz**2*(2*Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) - 2*Izz*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + self.S0*self.dCoP*self.rho*((-self.Cd0*dx*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dy*(dx*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)) + dy*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) + dz*(-dx*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi)))))*np.cos(psi)*np.cos(theta) - (self.Cd0*dy*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dx*(dx*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)) + dy*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) + dz*(dy*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)))))*np.sin(psi)*np.cos(theta) + (self.Cd0*dz*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dx*(-dx*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) - dy*(dy*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)))))*np.sin(theta)) - 2*self.T*self.dNoz*(P - 1)*np.sin(gy)*np.cos(gx) + 2*dIyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)))*np.cos(phi)/2)/(Iyy*Izz**2)
        ddpsi = (Iyy*Izz**2*dtheta*(dphi + dpsi*np.sin(theta)) + Iyy*(Izz*(Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) - Iyy*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) - dIzz*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi))) + self.Tau0*f)*np.cos(phi) - Izz**2*(2*Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) - 2*Izz*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + self.S0*self.dCoP*self.rho*((-self.Cd0*dx*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dy*(dx*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)) + dy*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) + dz*(-dx*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi)))))*np.cos(psi)*np.cos(theta) - (self.Cd0*dy*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dx*(dx*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)) + dy*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) + dz*(dy*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)))))*np.sin(psi)*np.cos(theta) + (self.Cd0*dz*np.sqrt(dx**2 + dy**2 + dz**2) + self.Cl0*alpha*(dx*(-dx*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))) - dy*(dy*np.cos(phi)*np.cos(theta) + dz*(np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi)))))*np.sin(theta)) - 2*self.T*self.dNoz*(P - 1)*np.sin(gy)*np.cos(gx) + 2*dIyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)))*np.sin(phi)/2)/(Iyy*Izz**2*np.cos(theta))
        
        return [ground_dx, ground_dy, ground_dz, ddx, ddy, ddz, dphi, dtheta, dpsi, ddphi, ddtheta, ddpsi, dm]
    
