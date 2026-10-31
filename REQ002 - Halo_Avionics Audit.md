## REQ002
Luca Romero
10/31/26

# subticket 002.1

AI: grok 4.7
Overview: 
In REQ001, the files from rocket-landing were ported into this Halo_GNC repo. This repository is meant to only focus on the simulation and controls software for Halo. Eveyrthing else will be left out, including reinforcement learning from rocket-landing. 

Halo_Avionics and rocket-landing are two directories that attempt to solve the same goals of simulations and creating the control software. However, I want to consolidate this into one repostiory (Halo_GNC) that can be worked off of for the forseeable future.

Instructions: Do an audit of Halo_Avionics and rocket-landing and write a report here about what functionality we should port from Halo_Avionics so that this repository contains the best of both directories (Halo_Avionics + rocket-landing). 

In the report, mention what files/code should be ported from Halo_Avionics and give an explanation for all your decisions. YOU ARE NOT TO EDIT ANY CODE. THIS IS PURELY A READ ONLY AUDIT. And you will write your report here in this subticket.

## REPORT:

Halo_GNC already has the rocket-landing flight model: air, wind, sensors, landing legs, and a flight computer that climbs, hovers, and lands with its built-in controller. That model should stay. Halo_Avionics should not be copied in as a second simulator. What it has that Halo_GNC does not have is a controller aimed at the Halo vehicle, and a set of Halo-sized guesses for mass and thrust.

Bring these pieces over, rewritten onto the flight computer that is already here.

The vehicle guesses live in `RocketSim.py`, in `RocketSim.__init__`. They are 285 kg, 3000 N of thrust, a throttle cap of 0.4, propellant use of 1.75 kg/s, nozzle 0.7 m from the center of mass, length 2.5 m, radius 0.2 m, and a roll torque of 3.23. `README.txt` still lists mass, drag, inertia, and thrust curve as data that has not been measured, so these numbers are the working guesses in that file, not hardware data. Put them in a new vehicle file under `configs/vehicles/`. Leave `configs/vehicles/electric_hopper.yaml` as it is. The flight model already burns propellant and moves the center of mass, so that part of `RocketSim.update` does not need to be copied.

The controller to bring over is `Controls.LQR`, the gain setup in `Controls.__init__` (the Q and R weights and the small model those weights were designed on), and the two-plane call inside `Controls.LQR_control_algo`. It flies by tilting the engine and setting the throttle, once for the forward plane and once for the side plane. Halo_GNC has no equivalent. It should become another choice next to the built-in controller in `control/`, and it should follow the height and speed reference the mission file already builds. Do not copy `Controls.z_ref`. That is a second flight plan, written as fixed times, and the mission file already does that job.

The roll law to bring over is `Controls.Roll_control`. It turns a fixed roll torque on and off. The source marks it as subject to change. The hopper uses a different roll actuator, so this law belongs with the Halo vehicle, not as a replacement for the hopper roll control.

Leave these where they are.

Do not copy `RocketSim.update` or `RocketSim.solve`. That is a separate flight model. It has no air, no wind, no landing legs, and no sensor models, and it steps time in a different way. The throttle in those equations also does not mean the same thing as throttle in Halo_GNC. Copying them would give this repo two simulators that do not agree.

Do not copy `Controls.Kalman_Filter` into the flight computer. It filters six numbers taken from the true simulated state and then adds noise. It never reads an IMU, a barometer, or a GPS. Halo_GNC already has an estimator that does read those sensors. The Kalman filter can stay in Halo_Avionics as the record of how the LQR was tested.

Do not copy the notebooks. `Old models/` holds the older two-dimensional runs and the separate ascent, hover, and descent studies, plus a PID tune on that older model. `RocketSim3DLQRKalman.ipynb` is the same controller as `RocketSim.py`, with plots. `RocketSimTest1.ipynb` only runs `RocketSim.py`. `README.txt` is the list of missing measurements. None of these should move into Halo_GNC.

grok 4.7

## Notes

I haven't reviewed this audit yet. Will do that later.