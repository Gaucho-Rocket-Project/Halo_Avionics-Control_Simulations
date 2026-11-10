## REQ002 byte-identical verification tests
Luca Romero
11/9/2026
execution status: planned, not executed

## Goal

Show that a code change reproduces the same closed-loop flight, byte for byte, or save a new baseline when the physics or the controller is supposed to change.

A refactor that leaves the equations, the gains, and the noise sequence alone must match the saved hash. A real physics or control change fails that check until someone records the next version on purpose.

## Hash test

`tests/hash_tests/run.py` runs one flight through `Runner.solve`. It uses the notebook's vehicle and control numbers with `t_final = 2.0`. It does not import the notebook and it does not plot.

Set `np.random.seed(1)` immediately before `solve`. Both the runner and the controller draw from that global NumPy stream. Start slightly off upright so pitch, yaw, and roll all leave zero:

`State0 = [0, 0, 0, 0, 0, 0, 0, 0.05, 0.05, 0, 0, 0, m0]`

Collect the three logs from that flight:

- `t` — `timestamps_cache[0]`. Accepted step times. This is the clock the integrator chose and the clock the controller reads.
- `x` — `states_cache[0]`. The 13 vehicle states: position, velocity, attitude, angular rate, mass.
- `u` — `controls_cache[0]` stacked as `[P, gx, gy, f]`. This is `sim.command`, the only input the plant takes from the controller.

Cast each array to contiguous float64 and SHA-256 the bytes. Save one bundle hash of `t`, `x`, and `u`, plus a hash of each state row and each command column. The bundle hash passes or fails the test. The channel hashes show what moved.

Write the same arrays to `tests/hash_tests/golden/<version>.npz`.

## versions.json

`tests/hash_tests/versions.json` stores every accepted baseline. `current` names the entry that `check` compares against.

Each entry stores:

- version name, git commit, date, and a one-line note
- numpy, scipy, and python-control versions
- the scenario spec: seed, params, control, State0, `t_final`
- the bundle hash and the per-channel hashes

`check` reads the current spec, runs it, and hashes the result. It compares that hash to the one stored for `current`. `check` leaves `versions.json` and the golden npz unchanged.

On a mismatch, `check` writes `tests/hash_tests/mismatch.csv` and prints that path. The file has the saved flight and the new flight side by side, then a third set of columns that subtracts them point by point: `saved - new`. One block is `t` and the 13 rows of `x`, one row per step. The next block is `u` as `[P, gx, gy, f]`, which is one entry longer because the log includes the initial command. If a pair of arrays has different lengths, that block still lists both flights and leaves the difference columns empty. A passing `check` deletes `mismatch.csv` if one is left over.

`record <version>` runs the flight, appends a new entry, points `current` at it, and writes that version's npz. It refuses to overwrite an existing version name.

The first run records version `1.0` from the current code. Later, `check` must pass against that entry.

When behavior is supposed to change, record the next version and commit the new entry, the new npz, and the code together. Old entries stay. A new control path keeps an old hash passing only when that saved scenario never executes the new path. A change inside the current `solve` loop fails `check` until it is recorded as the next version.

## Run

From this folder, with the rocket environment:

```bash
/Users/lucaromero/anaconda3/envs/rocket/bin/python tests/hash_tests/run.py record 1.0
/Users/lucaromero/anaconda3/envs/rocket/bin/python tests/hash_tests/run.py check
```

`record 1.0` flies the scenario, writes the hash into `versions.json`, and saves `golden/1.0.npz`. `check` flies it again and compares the new hash to that entry. Later versions use the same `record` command with a new name, such as `record 2.0`.

## End of ticket

Record version `1.0`, then run `check`. The ticket is done when `check` passes against that entry. 

Once done,
commit and push to the draft PR (REQ001-REQ003). Name the commit, REQ002 - verification tests. Do not add any comments to the commit, just write that.

Edit LOG.md so that it says executed for REQ002.