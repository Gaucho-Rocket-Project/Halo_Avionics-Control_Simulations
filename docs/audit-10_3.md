# Code audit

10 Nov 2026. Code structure only. The plant equations, the gains, the noise, and the reference trajectory were not reviewed as physics and are not candidates for this cleanup.

The bar is the smallest code that still flies the notebook sweeps, plots them, and hashes one seeded flight. A repeated block is worth deleting. A new helper, class, or package is not. `Visualizer/` is out of this audit.

`tests/hash_tests/run.py check` is the gate. It hashes `t`, `x`, and `u` for the seeded 2 second flight. Anything that changes those arrays is a physics or numerics change and is out of scope, even when the algebra looks redundant.

## Runner

`runner/halo_runner.py` keeps six histories. Three of them are read:

- `timestamps_cache` and `states_cache` — the notebook and the hash test
- `controls_cache` — the hash test, and the previous command inside `solve`

The other three are write-only:

- `control_states_cache` is only read back as `[-1][-1]`, the previous Kalman state. The older entries are never used.
- `control_times_cache` is appended at the end of `solve` and never read. The loop only needs the previous control time, which is `control_times[-1]`.
- `status_cache` stores `0` or `2`. Nothing reads it. The ground stop is the `solver.status = "stopped"` line, and that one stays.

On top of the cache, `solve` keeps parallel lists (`kalman_states_x`, `kalman_states_y`, `kalman_roll`, `kalman_cov_x`, `kalman_cov_y`, `control_times`) and, after the loop, transposes the three Kalman logs and drops them. Those transposes are dead.

`get_cached_flights` has no caller.

`plot` and `plot_cache` are one job split in two. `plot_cache` takes a `labels` argument (default length 6). `plot` ignores it and writes its own 13 labels. For every flight except the last, those labels are empty strings, which still go into the legend. The time window is applied by converting seconds into a fraction of the step count of flight 0, then slicing flight `i` with flight 0's indexes. BDF steps are not uniform, and a later flight is not the same length as the first, so the window is not "0 to 10 seconds" and it is not that flight's own window. Only the height series is sliced. The other six axes ignore `interval`.

`matplotlib.pyplot` is imported at module import. `solve` does not plot, but `tests/hash_tests/run.py` still has to set `MPLBACKEND` before the import.

What `solve` actually needs across a step is the previous command, the previous control time, and the previous 5-tuple that `LQR_control_algo` already accepts. One variable each replaces the extra lists. The command log stays a list of flights, still starting with `[0, 0, 0, 0]`, because that is part of the hash.

## Controller

`controls/halo_controls.py` is the flight code. Most of it is load-bearing. A few spots are not.

In `LQR`, `bP` and `bgamma` are `1.0`, and the rate limit divides by them. The division does nothing.

In `Roll_control`, four comparisons use `self.dt * 1`. The on-state and the off-state then copy the same three-way `ta` / `tb` choice. The hysteresis (`e_on` when the last command was 0, `e_off` otherwise) is the real logic and has to stay. The copied choice does not.

`Kalman_Filter` builds `np.identity(6)` twice in one line.

`import scipy as sp` exists for one `sp.linalg.expm` call.

`set_params` stays. The mass sweep calls it. The x and y blocks in `LQR_control_algo` look repeated and should be left alone. Folding them into one axis helper is a new abstraction around sign flips that the hash will punish if any index is wrong.

## Plant

`simulation/halo_sim.py` is the derivative. Leave it. In particular, do not cancel `T` in the mass-flow line. `(-m_dot / T) * T * (1 - P)` is not the same float as `-m_dot * (1 - P)`, and the hash covers mass. Do not split or rewrite the acceleration lines for readability. Same reason.

The empty `simulation/__init__.py`, `controls/__init__.py`, and `runner/__init__.py` stay. They are what make the package imports work.

## Notebook

`RocketSimTest1.ipynb` is the driver. Two small code problems:

- The yaw sweep never calls `clear_cache`, so its `plot_cache` draws the pitch flights again under the yaw flights. The mass sweep does clear the cache. The yaw sweep should too.
- `theta_failure` and `psi_failure` start at `0`. `linspace` also starts at `0`, so a printed `0` means either "failed on the first point" or "never failed".

The last cell has a commented-out `plt.plot`. Delete that line.

The vehicle and control dicts in the notebook match the dicts in `tests/hash_tests/run.py` except `t_final` (65 in the notebook, 2.0 in the test). That copy is intentional. The test does not import the notebook, and `halo_parameters.py` was removed so the numbers would live in the notebook. Do not put the dicts back in a shared module.

## Hash test

`tests/hash_tests/run.py` is longer than the flight code because `check` has to write `mismatch.csv` in the shape REQ002 specified. That writer stays. So does the `spec_to_json` / `spec_from_json` pair: it is how a run is pinned to `versions.json`. Do not add a version, and do not edit `versions.json` or `golden/1.0.npz`.

The one line that exists only because the runner imports pyplot at import time is `os.environ.setdefault("MPLBACKEND", "Agg")`. It goes away once plotting imports pyplot itself. `import os` is then unused.

## Docs that describe code that is gone

`docs/sim_notebooks_README.md` lists `RocketSimPID.ipynb`, `RocketSim2DLQR*.ipynb`, and `RocketSim3DLQR*.ipynb`. None of those files are in the repo. The only notebook is `RocketSimTest1.ipynb`.

## Out of scope

- Any edit under `simulation/halo_sim.py`.
- Gains, noise draws, the reference breakpoints, the Kalman update, the LQR call, and the order of `np.random` calls. The runner draws process noise, then the controller draws measurement noise, from one global stream. Reordering those calls changes the hash.
- Merging the x and y control paths.
- Renaming `timestamps_cache`, `states_cache`, or `controls_cache`.
- `tests/hash_tests/versions.json` and `tests/hash_tests/golden/`.
- `Visualizer/`, `docs/data_visualizer_README.md`, and `.github/workflows/build-desktop.yml`.
