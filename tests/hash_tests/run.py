"""Hash one seeded closed-loop flight and compare it to a saved baseline.

`record <version>` flies the scenario, writes versions.json, and saves the
arrays under golden/. `check` flies the scenario stored for `current` and
compares the bundle hash. A mismatch writes mismatch.csv.
"""

import csv
import hashlib
import json
import os
import subprocess
import sys
from datetime import date
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

os.environ.setdefault("MPLBACKEND", "Agg")

from controls.halo_controls import Controls
from runner.halo_runner import Runner
from simulation.halo_sim import RocketSim

HERE = Path(__file__).resolve().parent
VERSIONS_PATH = HERE / "versions.json"
GOLDEN_DIR = HERE / "golden"
MISMATCH_PATH = HERE / "mismatch.csv"

STATE_NAMES = [
    "x", "y", "z", "dx", "dy", "dz",
    "phi", "theta", "psi", "dphi", "dtheta", "dpsi", "m",
]
COMMAND_NAMES = ["P", "gx", "gy", "f"]

PARAMS = {
    "T": 3000,
    "P_max": 0.4,
    "m0": 285,
    "m_dot": 1.75,
    "dNoz": 0.7,
    "t_final": 2.0,
    "l": 2.5,
    "r": 0.2,
    "Tau0": 3.23,
    "g": 9.8,
    "Q_kalman": np.diag([1e-10, 1e-10, 1e-5, 1e-5, 1e-10, 1e-7]),
}
CONTROL = {
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
}


def default_spec():
    params = dict(PARAMS)
    params["Q_kalman"] = np.array(PARAMS["Q_kalman"], dtype=np.float64, copy=True)
    control = dict(CONTROL)
    for key in ("Q", "R", "R_kalman"):
        control[key] = np.array(CONTROL[key], dtype=np.float64, copy=True)
    return {
        "seed": 1,
        "t_final": 2.0,
        "State0": [0, 0, 0, 0, 0, 0, 0, 0.05, 0.05, 0, 0, 0, params["m0"]],
        "params": params,
        "control": control,
    }


def spec_to_json(spec):
    params = dict(spec["params"])
    params["Q_kalman"] = np.asarray(params["Q_kalman"], dtype=np.float64).tolist()
    params["t_final"] = spec["t_final"]
    control = dict(spec["control"])
    for key in ("Q", "R", "R_kalman"):
        control[key] = np.asarray(control[key], dtype=np.float64).tolist()
    return {
        "seed": spec["seed"],
        "t_final": spec["t_final"],
        "State0": list(spec["State0"]),
        "params": params,
        "control": control,
    }


def spec_from_json(raw):
    params = dict(raw["params"])
    params["Q_kalman"] = np.array(params["Q_kalman"], dtype=np.float64)
    params["t_final"] = raw["t_final"]
    control = dict(raw["control"])
    for key in ("Q", "R", "R_kalman"):
        control[key] = np.array(control[key], dtype=np.float64)
    return {
        "seed": int(raw["seed"]),
        "t_final": float(raw["t_final"]),
        "State0": list(raw["State0"]),
        "params": params,
        "control": control,
    }


def pack(array):
    return np.ascontiguousarray(array, dtype=np.float64)


def sha256(array):
    return hashlib.sha256(pack(array).tobytes()).hexdigest()


def fingerprint(t, x, u):
    t = pack(t)
    x = pack(x)
    u = pack(u)
    bundle = hashlib.sha256(t.tobytes() + x.tobytes() + u.tobytes()).hexdigest()
    channels = {"t": sha256(t)}
    for i, name in enumerate(STATE_NAMES):
        channels[name] = sha256(x[i])
    for j, name in enumerate(COMMAND_NAMES):
        channels[name] = sha256(u[:, j])
    return bundle, channels


def fly(spec):
    spec = spec_from_json(spec_to_json(spec))
    params = spec["params"]
    sim = RocketSim(params)
    controls = Controls(params, spec["control"])
    runner = Runner(sim, controls)
    # Seed immediately before solve. The runner and the controller share this sequence.
    np.random.seed(spec["seed"])
    runner.solve(State0=list(spec["State0"]))
    t = pack(runner.timestamps_cache[0])
    x = pack(runner.states_cache[0])
    u = pack(runner.controls_cache[0])
    if x.shape[0] != len(STATE_NAMES) or u.ndim != 2 or u.shape[1] != len(COMMAND_NAMES):
        raise RuntimeError(f"unexpected log shapes t={t.shape} x={x.shape} u={u.shape}")
    return t, x, u


def load_versions():
    if not VERSIONS_PATH.exists():
        return {"current": None, "versions": []}
    with VERSIONS_PATH.open() as handle:
        return json.load(handle)


def save_versions(document):
    with VERSIONS_PATH.open("w") as handle:
        json.dump(document, handle, indent=2)
        handle.write("\n")


def git_head():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


def package_versions():
    import control
    import scipy
    return {
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "python-control": control.__version__,
    }


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
    spec = default_spec()
    t, x, u = fly(spec)
    bundle, channels = fingerprint(t, x, u)
    entry = {
        "version": name,
        "commit": git_head(),
        "date": date.today().isoformat(),
        "note": "seeded 2 second flight, theta0 = psi0 = 0.05",
        "packages": package_versions(),
        "spec": spec_to_json(spec),
        "bundle": bundle,
        "channels": channels,
    }
    document["versions"].append(entry)
    document["current"] = name
    save_versions(document)
    GOLDEN_DIR.mkdir(parents=True, exist_ok=True)
    np.savez(GOLDEN_DIR / f"{name}.npz", t=t, x=x, u=u)
    print(f"recorded {name}")
    print(bundle)
    return 0


def fmt(value):
    if value is None:
        return ""
    return format(float(value), ".16g")


def column_pairs(saved, new):
    same_length = saved.shape == new.shape
    rows = max(saved.shape[0], new.shape[0])
    paired = []
    for i in range(rows):
        s = saved[i] if i < saved.shape[0] else None
        n = new[i] if i < new.shape[0] else None
        diff = (s - n) if same_length else None
        paired.append((s, n, diff))
    return paired


def write_mismatch(saved, new):
    t_s, x_s, u_s = saved
    t_n, x_n, u_n = new
    x_s = pack(x_s)
    x_n = pack(x_n)
    u_s = pack(u_s)
    u_n = pack(u_n)
    with MISMATCH_PATH.open("w", newline="") as handle:
        writer = csv.writer(handle)
        state_header = ["step", "t_saved", "t_new", "t_diff"]
        for name in STATE_NAMES:
            state_header += [f"{name}_saved", f"{name}_new", f"{name}_diff"]
        writer.writerow(state_header)
        t_pairs = column_pairs(pack(t_s).reshape(-1), pack(t_n).reshape(-1))
        x_pairs = [column_pairs(x_s[i], x_n[i]) for i in range(len(STATE_NAMES))]
        steps = max([len(t_pairs)] + [len(pairs) for pairs in x_pairs])
        for i in range(steps):
            if i < len(t_pairs):
                t_pair = t_pairs[i]
                row = [i, fmt(t_pair[0]), fmt(t_pair[1]), fmt(t_pair[2])]
            else:
                row = [i, "", "", ""]
            for pairs in x_pairs:
                if i < len(pairs):
                    row += [fmt(pairs[i][0]), fmt(pairs[i][1]), fmt(pairs[i][2])]
                else:
                    row += ["", "", ""]
            writer.writerow(row)
        writer.writerow([])
        command_header = ["step"]
        for name in COMMAND_NAMES:
            command_header += [f"{name}_saved", f"{name}_new", f"{name}_diff"]
        writer.writerow(command_header)
        u_pairs = [column_pairs(u_s[:, j], u_n[:, j]) for j in range(len(COMMAND_NAMES))]
        steps = max(len(pairs) for pairs in u_pairs)
        for i in range(steps):
            row = [i]
            for pairs in u_pairs:
                if i < len(pairs):
                    row += [fmt(pairs[i][0]), fmt(pairs[i][1]), fmt(pairs[i][2])]
                else:
                    row += ["", "", ""]
            writer.writerow(row)


def check():
    document = load_versions()
    name = document.get("current")
    entry = find_version(document, name) if name else None
    if entry is None:
        print("no current baseline", file=sys.stderr)
        return 1
    spec = spec_from_json(entry["spec"])
    t, x, u = fly(spec)
    bundle, channels = fingerprint(t, x, u)
    if bundle == entry["bundle"]:
        if MISMATCH_PATH.exists():
            MISMATCH_PATH.unlink()
        print(f"pass {name}")
        return 0
    golden = np.load(GOLDEN_DIR / f"{name}.npz")
    write_mismatch((golden["t"], golden["x"], golden["u"]), (t, x, u))
    golden.close()
    moved = [key for key, value in channels.items() if value != entry["channels"].get(key)]
    print(MISMATCH_PATH)
    if moved:
        print("moved: " + ", ".join(moved))
    return 1


def main(argv):
    if len(argv) >= 2 and argv[1] == "check" and len(argv) == 2:
        return check()
    if len(argv) == 3 and argv[1] == "record":
        return record(argv[2])
    print("usage: run.py check | run.py record <version>", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
