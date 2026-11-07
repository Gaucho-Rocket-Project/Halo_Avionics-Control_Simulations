"""Vehicle numbers shared by the plant and the controller.

Both sides hold this object and read it at call time. The controller does not
keep its own copy of thrust, mass, or the actuator limits.
"""

import numpy as np


class HaloParameters:
    # T: maximum thrust (N)
    # P_max: largest throttle command the controller may use
    # m0: initial mass (kg), used by the onboard model
    # m_dot: propellant flow at full thrust (kg/s)
    # g: gravitational acceleration (m/s^2)
    # dNoz: nozzle-to-center-of-mass distance (m)
    # t_final: simulation stop time (s). The runner reads this.
    # l, r: cylinder length and radius used for inertia (m)
    # Tau0: roll-torque magnitude (N·m)
    # Q_kalman: process-noise covariance used by the runner and by Q_d
    def __init__(self,
                 T=3000,
                 P_max=0.4,
                 m0=285,
                 m_dot=1.75,
                 dNoz=0.7,
                 t_final=65,
                 l=2.5,
                 r=0.2,
                 Tau0=3.23,
                 g=9.8):
        self.T = T
        self.P_max = P_max
        self.m0 = m0
        self.m_dot = m_dot
        self.dNoz = dNoz
        self.t_final = t_final
        self.l = l
        self.r = r
        self.Tau0 = Tau0
        self.g = g
        self.position_variance = 1e-10
        self.velocity_variance = 1e-5
        self.Q_kalman = np.diag([
            self.position_variance,
            self.position_variance,
            self.velocity_variance,
            self.velocity_variance,
            1e-10,
            1e-7,
        ])

    def set_vehicle(self, T, P_max, m0, m_dot, dNoz, t_final, l, r, Tau0):
        """Write the numbers RocketSim.set_params used to store on the sim."""
        self.T = T
        self.P_max = P_max
        self.m0 = m0
        self.m_dot = m_dot
        self.dNoz = dNoz
        self.t_final = t_final
        self.l = l
        self.r = r
        self.Tau0 = Tau0
