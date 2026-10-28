These notebooks simulate the trajectory of the rocket (position and angle data) with given parameters.

-RocketSimPID.ipynb simulates a trajectory using PID control and functions well. I used an optimization algorithm to find optimal PID coefficients.
-RocketSim2DLQR(1,2 and 3).ipynb simulate a trajectory using LQR control (state space) for a 2D rocket model for ascent, hovering, and descent
-RocketSim3DLQR(1,2 and 3).ipynb simulate a trajectory using LQR control (state space) for a 3D rocket model for ascent, hovering, and descent

The methods of control I assume are the two TVC gimbal angles (which I denote gamma_x and gamma_y for angles about the x and y axis respectively) and the percentage of thrust that is throttled (P).

Data still needed:
-Mass of rocket and fuel
-Rate of mass flow
-Drag coefficient (Nose cone geometry)
-Cross sectional area (top down view)
-Inertia matrix
-Rate of Inertia matrix
-Distance between Center of Mass and Center of Pressure
-Distance between Center of Mass and Nozzle
-TVC angle range
-TVC angle maximum rate
-Thrust throttle range
-Thrust throttle maximum rate
-Thrust Curve (force vs. time)