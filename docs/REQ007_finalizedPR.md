# REQ007 — Make the dashboard, notebooks, and Visualizer run

Status: executed

The plant, controls, and runner on this branch are the split from REQ006. This ticket makes the rest of the repo match that split.

Do not edit the plant equations, the gains, or `tests/hash_tests`. Do not run `record`.

Run Python from the rocket environment:

`/Users/lucaromero/anaconda3/envs/rocket/bin/python`

## 007.1 Point the dashboard at the runner

In `dashboard.py`, `run_simulation` imports `tests.hash_tests.Restructure_merge.RocketSim`. That module is not there.

Build one `RocketSim`, one `Controls`, and one `Runner` from the page's rocket, controller, wind, and initial state. Call `runner.solve` once. Keep the resampling and the browser page.

`docs/sim_notebooks_README.md` says to run `dashboard.py` from a `RocketSim` folder, and its image path is `RocketSim/dashboard/dashboard.png`. On this branch the script is at the repo root and the image is `dashboard/dashboard.png`. Update those two lines.

Done when Launch runs one flight and the page plays it.

## 007.2 Run RocketSimTest1.ipynb

This notebook already builds a `RocketSim`, a `Controls`, and a `Runner`. Run every code cell.

The pitch sweep, yaw sweep, and mass sweep must finish and plot. Fix a cell only when it throws.

Done when Run All finishes.

## 007.3 Run RocketSimTest2.ipynb

The first flight cells use the runner. Later cells call `RocketSim.RocketSim`, `with_control_algo`, and `sim.timestamps_cache`. Change those cells to build a `RocketSim`, a `Controls`, and a `Runner`, then call `runner.solve`.

The cost plot uses `-Cost`, which is the function. Plot `-Cost_array`.

Run every code cell. Fix a cell only when it throws.

Done when Run All finishes.

## 007.4 Run RocketSim3DLQRKalmanAero.ipynb

This notebook derives the equations. It does not import the simulator. Run every code cell. Edit a cell only when it throws.

Done when Run All finishes.

## 007.5 Run the Visualizer

`Visualizer/sensor_grapher.py` and `Visualizer/sensor_desktop_app.py` do not import the simulator. From `Visualizer/`, run:

```bash
/Users/lucaromero/anaconda3/envs/rocket/bin/python sensor_desktop_app.py
```

The window must open. Edit a file only when that command throws.

Done when the window opens.

## 007.6 Delete REQ004

Delete `docs/REQ004.md`. Delete the `REQ004 - proposed` line in `docs/LOG.md`. Its checks were written for the older plant.

Done when those two mentions are gone.

## 007.7 Remove the comparison copy

Delete `tests/Restructure_merge`. The three-flight comparison already passed, and the root files are those copies. `run.py check` is the flight check that stays.

Done when that folder is gone and `run.py check` still prints `True`.

## 007.8 Fix the readmes

`README.md` links rocket simulations to a `RocketSim` folder. Point that link at `docs/sim_notebooks_README.md` only.

In `docs/sim_notebooks_README.md`, replace the `RocketSim.py` section. Say the plant is `simulation/halo_sim.py`, the controller is `controls/halo_controls.py`, and the flight loop is `runner/halo_runner.py`.

Done when neither readme mentions a `RocketSim/` folder or a root `RocketSim.py`.

## 007.9 Compare the flights with main

Do this last. A byte match can be set up for one flight. Use the same parameters and the same seed on both sides. Hash the contiguous float64 bytes of `t`, then `x`, then `u`.

Run these flights on `origin/main` `RocketSim/RocketSim.py` and on this branch's runner:

- `dashboard.py` with the default rocket, controller, wind, and initial state
- each cell in `RocketSimTest2.ipynb` that calls `solve` once with fixed numbers

`RocketSim3DLQRKalmanAero.ipynb` derives the equations. It does not fly, so it has no `t`, `x`, and `u` to hash.

`RocketSimTest1.ipynb` on main is `RocketSim/Old models/RocketSimTest1.ipynb`. Hash a sweep flight only when that cell is the same setup on both sides.

Print the name and `True` or `False` for each flight.

Use a temporary worktree of `origin/main`. In that worktree only, change `RocketSim.solve` so the controller loop is `for i in range(6)`, and remove the real-time stop so the flight runs to `t_final`. In a second worktree of this branch, remove the real-time stop in `Runner.solve` the same way. Both flights then finish, so `RK45` takes the same steps. Set the same seed immediately before every `solve`. Do not commit either edit.

Write the printed lines into this ticket.

```
dashboard True
RocketSimTest2 halo True
RocketSimTest2 halo jr True
```

`RocketSimTest1.ipynb` on main builds `RocketSim()` with the class defaults. This branch's sweep uses the Halo numbers, so that sweep was not hashed.

Done when those lines are written here.
