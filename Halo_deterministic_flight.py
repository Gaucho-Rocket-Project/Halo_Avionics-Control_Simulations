from simulation.halo_sim import RocketSim
from controls.halo_controls import Controls
from runner.halo_runner import Runner
import numpy as np
import matplotlib.pyplot as plt
import hashlib

## PARAMETERS ##
control_params = {
    "H": 50.0,
    "V": 4.0,
    "Ti": 21.0,
    "Th": 10.0,
    "Tf": 54.0,
    "Z": 0.83,
    "Q": np.diag([0.1, 350, 0.1, 320, 20, 50]),
    "R": np.diag([2000, 1536]),
    "R_kalman": np.diag([1, 1, 1, 1, 1, 1]) * 1e-5,
    "control_frequency": 15, # control frequency, dt = 1 / frequency 
}

sim_params = {
    "T" : 3000,                   #Max thrust
    "P_max" : 0.4,               #Max throttle percentage
    "m0" : 285,                  #Initial mass
    "m_dot" : 1.75,               #Mass flow at max thrust
    "g" : 9.8,                     #Gravity
    "dNoz" : 0.7,                #Distance from CoM to Nozzle
    "t_final" : 65,             #Final simulation time
    "l" : 2.5,                   #Effective length
    "r" : 0.2,                   #Effective radius
    "Tau0" : 3.23,                #RCS torque magnitude
    "Cd0" : 0.8,                 #Coeff of drag NOTE: ADD THIS TO halo_control script (it's only here right now)

    "Cl0" : 6.28,                 #Coeff of lift
    "S0" : 0.126,                  #Reference area
    "dCoP" : 1,                #Distance from CoM to Center of Pressure
    "rho" : 1.225,                 #Density of air
    "State0" : [0,0,0,
                0,0,0,
                0,0,0,
                0,0,0,0],
    "wind_speed" : 0.0,          #Steady horizontal wind magnitude
    "wind_direction" : 0.0,      #Wind direction from +x toward +y (radians)
    "gust_variance" : 0.0,       #Variance of each horizontal gust component
    "wind_seed" : None,
    "position_variance" : 1e-10,
    "velocity_variance" : 1e-5,
    "Q_kalman": np.diag([1e-10, 1e-10, 1e-5, 1e-5, 1e-10, 1e-7]),
}

np.random.seed(1)
sim = RocketSim(sim_params)
controls = Controls(control_params, sim_params)
runner = Runner(controls, sim) 
runner.clear_cache()
runner.solve()

t = np.ascontiguousarray(runner.timestamps_cache[0], dtype=np.float64)
x = np.ascontiguousarray(runner.states_cache[0], dtype = np.float64)
u = np.ascontiguousarray(runner.controls_cache[0], dtype = np.float64)

bundle = hashlib.sha256(t.tobytes() + x.tobytes() + u.tobytes()).hexdigest()

print(bundle)




