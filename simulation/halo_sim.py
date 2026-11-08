"""Plant: the 13-state rocket. This module does not call the controller.

The runner stores the current command on `sim.command` before each integrator
step. `update` only returns the derivative for the state it is given.
Vehicle numbers come from a params dict supplied by the notebook.
"""

import numpy as np


class RocketSim:
    def __init__(self, params):
        self.params = params
        # Command held for the integrator: [P, gx, gy, f].
        # P is the throttle fraction used by update, gx and gy are gimbal angles (rad),
        # and f is the roll-torque command.
        self.command = [0, 0, 0, 0]

    def update(self, t, state):
        # state: x, y, z, dx, dy, dz, phi, theta, psi, dphi, dtheta, dpsi, m
        x, y, z, dx, dy, dz, phi, theta, psi, dphi, dtheta, dpsi, m = (state)
        P, gx, gy, f = self.command
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
