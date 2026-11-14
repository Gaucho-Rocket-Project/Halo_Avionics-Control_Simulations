"""Record the flight hash and its date, or check it and print True or False.

`record` replaces the saved hash only when the new flight hash is different.
"""

import hashlib
import json
import sys
from datetime import date
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controls.halo_controls import Controls
from runner.halo_runner import Runner
from simulation.halo_sim import RocketSim

HERE = Path(__file__).resolve().parent
VERSIONS_PATH = HERE / "versions.json"

BASE = {
    "H": 50.0,
    "V": 4.0,
    "Ti": 21.0,
    "Th": 10.0,
    "Tf": 54.0,
    "Z": 0.83,
    "Q": np.diag([0.1, 350, 0.1, 320, 20, 50]),
    "R": np.diag([2000, 1536]),
    "R_kalman": np.diag([1, 1, 1, 1, 1, 1]) * 1e-5,
    "control_frequency": 15,
    "T": 3000,
    "P_max": 0.4,
    "m0": 285,
    "m_dot": 1.75,
    "g": 9.8,
    "dNoz": 0.7,
    "l": 2.5,
    "r": 0.2,
    "Tau0": 3.23,
    "Cd0": 0.8,
    "Cl0": 6.28,
    "S0": 0.126,
    "dCoP": 1,
    "rho": 1.225,
    "State0": [0, 0, 0, 0, 0, 0, 0, 0.05, 0.05, 0, 0, 0, 285],
    "Q_kalman": np.diag([1e-10, 1e-10, 1e-5, 1e-5, 1e-10, 1e-7]),
    "seed": 1,
    "t_final": 2.0,
}

SCENARIOS = [
    {"wind_speed": 0.0, "wind_direction": 0.0, "gust_variance": 0.0, "wind_seed": 1},
    {"wind_speed": 5.0, "wind_direction": 0.5, "gust_variance": 0.0, "wind_seed": 1},
    {"wind_speed": 5.0, "wind_direction": 0.5, "gust_variance": 1.0, "wind_seed": 1},
]


def pack(array):
    return np.ascontiguousarray(array, dtype=np.float64)


def fly():
    blob = b""
    for overrides in SCENARIOS:
        params = dict(BASE)
        params.update(overrides)
        params["State0"] = list(BASE["State0"])
        sim = RocketSim(params)
        controls = Controls(params, params)
        runner = Runner(controls, sim)
        np.random.seed(params["seed"])
        runner.solve(State0=list(params["State0"]))
        t = pack(runner.timestamps_cache[0])
        x = pack(runner.states_cache[0])
        u = pack(runner.controls_cache[0])
        blob += t.tobytes() + x.tobytes() + u.tobytes()
    return hashlib.sha256(blob).hexdigest()


def load_versions():
    with VERSIONS_PATH.open() as handle:
        return json.load(handle)


def save_versions(document):
    with VERSIONS_PATH.open("w") as handle:
        json.dump(document, handle, indent=2)
        handle.write("\n")


def record():
    new_hash = fly()
    if VERSIONS_PATH.exists() and load_versions().get("hash") == new_hash:
        print("hash already equals current hash in versions.json")
        return 0
    save_versions({"hash": new_hash, "date": date.today().isoformat()})
    print(True)
    return 0


def check():
    if not VERSIONS_PATH.exists():
        print("no current baseline", file=sys.stderr)
        return 1
    same = fly() == load_versions()["hash"]
    print(same)
    return 0 if same else 1


def main(argv):
    if len(argv) == 2 and argv[1] == "check":
        return check()
    if len(argv) == 2 and argv[1] == "record":
        return record()
    print("usage: run.py check | run.py record", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
