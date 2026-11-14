# REQ001 — Split simulation, control, and the runner

Move `RocketSim.py` apart. Do not change the equations, gains, noise, or stop conditions. Essentially, the functionality of the code should not change, it should only be reorganized so that it can work in the format specified below. 

## Layout

- `halo_parameters.py` — shared numbers (`T`, `P_max`, `m0`, `m_dot`, `g`, `dNoz`, `l`, `r`, `Tau0`, `Q_kalman`, `t_final`)
- `simulation/halo_sim.py` — `RocketSim` and `update`. No control calls.
- `control/halo_controls.py` — `Controls`. No simulator import. Reads the shared numbers.
- `runner/halo_runner.py` — today's `solve` loop, logs, and plots.
- Delete `RocketSim.py`, `Old models/`
- Point `RocketSimTest1.ipynb` at the runner. Keep the same three sweeps.

`simulation` does not import `control`. `control` does not import `simulation`. The runner imports both.

## comments
Currently the code has limited comments explaining the flow and what the variables are. 
You need to add comments to the code to make it more understandable.

## Test

This check answers one question: did the move change the flight, or only the file layout?

Do it in the `Halo_Avionics-Control_Simulations` folder. The commands use `import RocketSim`, and that file is in this folder. Also use the `(rocket)` conda environment. That is the environment where `numpy`, `scipy`, `matplotlib`, and `control` are installed. 

### 1. Save a before-flight, before any files are moved

`RocketSim.py` adds random noise during a flight. Two runs of the same code normally produce two different trajectories. `np.random.seed(1)` fixes the sequence of those random numbers, so the same code produces the same flight again. That is what "seeded" means.

Run the current code once with that seed. Write the result to a file before editing. After `RocketSim.py` is split, this exact run cannot be repeated, so the file is the only record of the original flight.

The file holds three arrays from that one flight:

- `t` — the simulation times
- `x` — the vehicle state at those times
- `u` — the commands at those times

The saved flight stops at 2 seconds of simulation time, not the usual 65. `solve` also gives up after 10 seconds of real clock time. A 65-second flight can still be running when that clock runs out, and a slower run then saves fewer steps. Two seconds of simulation finishes before that clock, so the before-file and the after-file are the whole flight. Every other number stays at the default. `Tau0` stays `3.23`.

```bash
python -c "
import numpy as np, RocketSim
np.random.seed(1)
sim = RocketSim.RocketSim()
c = RocketSim.Controls()
sim.set_params(3000, 0.4, 285, 1.75, 0.7, 2.0, 2.5, 0.2, 3.23)
sim.with_control_algo(c.LQR_control_algo).solve()
np.savez('/tmp/rocketsim_before.npz', t=sim.timestamps_cache[0], x=sim.states_cache[0], u=np.array(sim.controls_cache[0]))
"
```

This is its own small program. It is not `RocketSimTest1.ipynb`. The notebook does not set a seed and does not save these arrays. Run it with the `(rocket)` environment Python.

### 2. Repeat that same flight after the move

Run one flight through the new runner with `np.random.seed(1)` again. Call `set_params(3000, 0.4, 285, 1.75, 0.7, 2.0, 2.5, 0.2, 3.23)` before the flight, same as the before-run. Save it the same way to `/tmp/rocketsim_after.npz`, with the same three arrays: `t`, `x`, and `u`.

Compare the two files:

```bash
python -c "
import numpy as np
b = np.load('/tmp/rocketsim_before.npz')
a = np.load('/tmp/rocketsim_after.npz')
for k in ('t','x','u'):
    assert np.allclose(a[k], b[k]), k
print('match')
"
```

`match` means the times, states, and commands are the same. The reorganization did not change the flight.

### 3. Run the notebook

Run All on `RocketSimTest1.ipynb`. The pitch sweep, yaw sweep, and mass sweep must finish and plot. The printed angles and times will not match an older run of the notebook. Each of those flights draws new noise, and the notebook never sets a seed. The file comparison above is the check that the results stayed the same.

### 4. Commit and Push

Once all checks have passed. Commit and push this as a draft PR named "REQ001_restructure". In the main comment area say " split RocketSim.py code into simulation, controls, and runner files. "

## Changes

- The sameness check now stops the flight at 2 seconds of simulation time. The saved command sets `t_final` to `2.0` and leaves the other vehicle numbers at their defaults, including `Tau0 = 3.23`. A 65-second flight can be cut off by the 10-second real-time stop, so two runs could save different lengths. The 2-second flight finishes first.
- Before any code was moved, that seeded flight was saved from `RocketSim.py`. It reached 2.0 seconds in 123 steps with status 0.
- `RocketSim.py` was split into `halo_parameters.py`, `simulation/halo_sim.py`, `control/halo_controls.py`, and `runner/halo_runner.py`. The derivative, the LQR and Kalman math, the noise, and both stop conditions were copied as they were. `State0.append` still stores `None`. `Controls.set_params` still does not store `H`.
- The folder `control/` has the same name as the installed python-control library. `control/__init__.py` loads that library first, then attaches `halo_controls`, so `lqr` still comes from the library.
- `RocketSimTest1.ipynb` now builds a `RocketSim`, a `Controls`, and a `Runner`. The three sweeps call `runner.solve`. The mass sweep passes `sim.params` into `controls.set_params`.
- `RocketSim.py` and `Old models/` were deleted. `RocketSim3DLQRKalman.ipynb`, `scripts/runner_template.py`, and the READMEs were left in place.
- The same seeded 2-second flight was run through the new runner. Times, states, and commands matched the before-file (`t`, `x`, and `u`).
- The notebook's code cells were run in the `(rocket)` environment. The pitch sweep, yaw sweep, and mass sweep all finished. The pitch sweep stopped at about 0.359 rad. The mass sweep printed 40 final altitudes and a mass-lost series. Plots were drawn. No plot window opened, because the run used a non-interactive Matplotlib backend.

