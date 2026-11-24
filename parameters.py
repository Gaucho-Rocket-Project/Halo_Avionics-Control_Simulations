## This is the source of truth for the parameters of the rocket, this file gets imported
## wherever simulations need to be run

import numpy as np

halo_params = {
    # control params
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

    # sim params
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
                0,0,0],
    "wind_speed" : 0.0,          #Steady horizontal wind magnitude
    "wind_direction" : 0.0,      #Wind direction from +x toward +y (radians)
    "gust_variance" : 0.0,       #Variance of each horizontal gust component
    "wind_seed" : None,
    "Q_kalman": np.diag([1e-10, 1e-10, 1e-5, 1e-5, 1e-10, 1e-7]),
}

halojr_params = {
    # control params
    "H": 10.0,
    "V": 4.0,
    "Ti": 4.60,
    "Th": 10.0,
    "Tf": 19.6,
    "Z": 2.30419829,
    "Q" : np.diag([4,53,4,5000,131,8.2]), 
    "R" : np.diag([200,1500]),
    "R_kalman": np.diag([1, 1, 1, 1, 1, 1]) * 1e-5,
    "control_frequency": 15, # control frequency, dt = 1 / frequency 

    # sim params
    "T" : 38,                   #Max thrust
    "P_max" : 0.9,               #Max throttle percentage
    "m0" : 2.18,                  #Initial mass
    "m_dot" : 0,               #Mass flow at max thrust
    "g" : 9.8,                     #Gravity
    "dNoz" : 0.32,                #Distance from CoM to Nozzle
    "t_final" : 20,             #Final simulation time
    "l" : 0.7,                   #Effective length
    "r" : 0.05,                   #Effective radius
    "Tau0" : 0.15,                #RCS torque magnitude
    "Cd0" : 0.8,                 #Coeff of drag NOTE: ADD THIS TO halo_control script (it's only here right now)

    "Cl0" : 6.28,                 #Coeff of lift
    "S0" : 0.0078,                  #Reference area
    "dCoP" : -0.03,                #Distance from CoM to Center of Pressure
    "rho" : 1.225,                 #Density of air
    "State0" : [0,0,0,             #the m0 gets appended in halo_runner code.
                0,0,0,
                0,0,0,
                0,0,0],
    "wind_speed" : 0.0,          #Steady horizontal wind magnitude
    "wind_direction" : 0.0,      #Wind direction from +x toward +y (radians)
    "gust_variance" : 0.0,       #Variance of each horizontal gust component
    "wind_seed" : None,
    "Q_kalman": np.diag([1e-10, 1e-10, 1e-5, 1e-5, 1e-10, 1e-7]),
}