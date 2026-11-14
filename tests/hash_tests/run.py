"""Record one flight hash, or check the current hash and print True or False."""

import hashlib
import json
import sys
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

PARAMETERS = {
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
    "wind_speed": 0.0,
    "wind_direction": 0.0,
    "gust_variance": 0.0,
    "wind_seed": None,
    "Q_kalman": np.diag([1e-10, 1e-10, 1e-5, 1e-5, 1e-10, 1e-7]),
    "seed": 1,
    "t_final": 2.0,
}


def pack(array):
    return np.ascontiguousarray(array, dtype=np.float64)


def fly():
    sim = RocketSim(PARAMETERS)
    controls = Controls(PARAMETERS, PARAMETERS)
    runner = Runner(controls, sim)
    np.random.seed(PARAMETERS["seed"])
    runner.solve(State0=list(PARAMETERS["State0"]))
    t = pack(runner.timestamps_cache[0])
    x = pack(runner.states_cache[0])
    u = pack(runner.controls_cache[0])
    return hashlib.sha256(t.tobytes() + x.tobytes() + u.tobytes()).hexdigest()


def load_versions():
    with VERSIONS_PATH.open() as handle:
        return json.load(handle)


def save_versions(document):
    with VERSIONS_PATH.open("w") as handle:
        json.dump(document, handle, indent=2)
        handle.write("\n")


def find_version(document, name):
    for entry in document["versions"]:
        if entry["version"] == name:
            return entry
    return None


def record(name):
    document = load_versions()
    if find_version(document, name) is not None:
        print(f"{name} already exists", file=sys.stderr)
        return 1
    entry = {"version": name, "hash": fly()}
    document["versions"].append(entry)
    document["current"] = name
    save_versions(document)
    print(entry["hash"])
    return 0


def check():
    document = load_versions()
    name = document.get("current")
    entry = find_version(document, name) if name else None
    if entry is None:
        print("no current baseline", file=sys.stderr)
        return 1
    saved = entry["hash"] if "hash" in entry else entry["bundle"]
    same = fly() == saved
    print(same)
    return 0 if same else 1


def main(argv):
    if len(argv) == 2 and argv[1] == "check":
        return check()
    if len(argv) == 3 and argv[1] == "record":
        return record(argv[2])
    print("usage: run.py check | run.py record <version>", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
