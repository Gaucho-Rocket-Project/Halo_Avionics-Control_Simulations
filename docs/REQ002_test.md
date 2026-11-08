# REQ002 — Tests that the restructure keeps the same flight

Add tests that prove the move in `REQ001_restructure.md` did not change the plant, the control tick, or a closed-loop flight. Do not change `RocketSim.py` in this ticket.

## Golden file

From the repo root, in the `(rocket)` environment, save `tests/golden_flight.npz` from the current `RocketSim.py` before that file is deleted.

Use this state and command for the first two checks:

```python
state = [1, -1, 10, 0.1, -0.1, 2, 0.05, 0.1, 0.2, 0.01, -0.02, 0.03, 285]
command = [0.2, 0.05, -0.05, 1.0]
```

```bash
python -c "
import numpy as np, RocketSim
state = [1, -1, 10, 0.1, -0.1, 2, 0.05, 0.1, 0.2, 0.01, -0.02, 0.03, 285]
command = [0.2, 0.05, -0.05, 1.0]

sim = RocketSim.RocketSim()
sim.controls_cache.append([command])
dx = np.array(sim.update(0.0, state), dtype=float)

np.random.seed(1)
c = RocketSim.Controls()
X, u_tick = c.LQR_control_algo(1.0, state, 0.0, [np.zeros(6), np.zeros(6), [0, 0], np.zeros((6, 6)), np.zeros((6, 6))], command)

np.random.seed(1)
sim = RocketSim.RocketSim()
c = RocketSim.Controls()
sim.set_params(3000, 0.4, 285, 1.75, 0.7, 2.0, 2.5, 0.2, 0.1)
sim.with_control_algo(c.LQR_control_algo).solve()
assert sim.status_cache[-1] == 0

np.savez('tests/golden_flight.npz',
         dx=dx,
         u_tick=np.array(u_tick, dtype=float),
         t=np.array(sim.timestamps_cache[0]),
         x=np.array(sim.states_cache[0]),
         u=np.array(sim.controls_cache[0], dtype=float))
print('saved')
"
```

`t_final` is 2 seconds so the wall-clock stop cannot cut the flight. Commit the `.npz`.

## Test file

Add `tests/test_flight_unchanged.py`. It loads the golden file and checks three things with `np.allclose`:

1. **Plant.** Set the command to `[0.2, 0.05, -0.05, 1.0]`, call `update(0.0, state)`, and compare the 13 derivatives to `dx`.
2. **One control tick.** `np.random.seed(1)`, then one `LQR_control_algo(1.0, state, 0.0, [zeros, zeros, [0, 0], zero covariance, zero covariance], command)`. Compare the returned command to `u_tick`.
3. **Short flight.** `np.random.seed(1)`, `set_params(3000, 0.4, 285, 1.75, 0.7, 2.0, 2.5, 0.2, 0.1)`, then the same `solve` path as above. Status must be `0`. Compare `t`, `x`, and `u`.

Run it from the repo root:

```bash
python tests/test_flight_unchanged.py
```

Before the move, the test calls `RocketSim.RocketSim` and `RocketSim.Controls`. After `REQ001_restructure.md`, change only those calls so the plant check hits the simulator, the tick check hits `Controls`, and the flight check hits the runner. Keep the seed, the state, the command, and `set_params` arguments the same. A mismatch means the move changed the flight.

Do not add the 40-case sweeps, plot checks, or a seed inside `RocketSim.py`.
