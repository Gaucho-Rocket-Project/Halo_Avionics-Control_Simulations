## REQ 001 
Luca Romero
10/31/26
Grok 4.7 agent executed this ticket

## subticket 001.1
Port files into this directory from the rocket-landing directory in this format:

simulation/simulation.py
simulation/modules/physics.py
simulation/modules/aero.py
simulation/modules/quaternion.py
simulation/modules/touchdown.py
simulation/modules/motors.py
simulation/modules/actuators.py
simulation/modules/sensors.py
simulation/modules/simconfig.py
simulation/modules/config.py
simulation/modules/servo_calibration.py
simulation/modules/dragdevice.py
simulation/modules/flightlog.py
simulation/modules/units.py
simulation/modules/yaml_section.py

control/computer.py
control/modules/mission.py
control/modules/vehicle.py
control/modules/estimator.py
control/modules/pid.py
control/modules/guidance.py
control/modules/commands.py

scripts/plot_flight.py
scripts/fly_mission.py

## subticket 001.2 
Fix anything that may have been broken during this port. 

List out the fixes that were made here:

> The copied files still pointed at the old rocket-landing folders. Those lines now point at the simulation and control folders in Halo_GNC. The flight math and the flight computer were not changed.
>
> Empty `__init__.py` files were added so Python treats simulation and control as folders it can load. `plot_flight.py` and `animate_flight.py` were given one extra line so they can find those folders. Plot and animation files were copied in, because `fly_mission.py` uses them and they were missing. The vehicle, mission, motor, sensor, and world settings were copied into `configs/`.
>
> `fly_mission.py` is the one file whose behavior changed. The helpers it used to borrow from the trainer now sit in the script itself. A normal flight still flies with the built-in controller. `--model` and `--compare` stop and say the trained model was not brought over.
>
> grok 4.7 (medium)


