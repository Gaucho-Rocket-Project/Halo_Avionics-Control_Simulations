"""Fly nominal, wind, and gust on the split tree and on RocketSim.py.

Prints whether each pair of flights hashes the same. Does not write a baseline.
"""

import hashlib
import sys

import numpy as np

from simulation.halo_sim import RocketSim
from controls.halo_controls import Controls
from runner.halo_runner import Runner
import RocketSim as OldRocketSim


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
    {"name": "nominal", "wind_speed": 0.0, "wind_direction": 0.0, "gust_variance": 0.0, "wind_seed": 1},
    {"name": "wind", "wind_speed": 5.0, "wind_direction": 0.5, "gust_variance": 0.0, "wind_seed": 1},
    {"name": "gust", "wind_speed": 5.0, "wind_direction": 0.5, "gust_variance": 1.0, "wind_seed": 1},
]


def scenario_params(overrides):
    params = dict(BASE)
    params.update(overrides)
    params["State0"] = list(BASE["State0"])
    return params


def pack(array):
    return np.ascontiguousarray(array, dtype=np.float64)


def flight_hash(t, x, u):
    t = pack(t)
    x = pack(x)
    u = pack(u)
    return hashlib.sha256(t.tobytes() + x.tobytes() + u.tobytes()).hexdigest()


def fly_split(params):
    sim = RocketSim(params)
    controls = Controls(params, params)
    runner = Runner(controls, sim)
    np.random.seed(params["seed"])
    runner.solve(State0=list(params["State0"]))
    return runner.timestamps_cache[0], runner.states_cache[0], runner.controls_cache[0]


def fly_old(params):
    sim = OldRocketSim.RocketSim(
        T=params["T"],
        P_max=params["P_max"],
        m0=params["m0"],
        m_dot=params["m_dot"],
        dNoz=params["dNoz"],
        t_final=params["t_final"],
        l=params["l"],
        r=params["r"],
        Tau0=params["Tau0"],
        Cd0=params["Cd0"],
        Cl0=params["Cl0"],
        S0=params["S0"],
        dCoP=params["dCoP"],
        rho=params["rho"],
        wind_speed=params["wind_speed"],
        wind_direction=params["wind_direction"],
        gust_variance=params["gust_variance"],
        wind_seed=params["wind_seed"],
    )
    controls = OldRocketSim.Controls(
        H=params["H"],
        V=params["V"],
        Ti=params["Ti"],
        Th=params["Th"],
        Tf=params["Tf"],
        Z=params["Z"],
        Q=params["Q"],
        R=params["R"],
        control_frequency=params["control_frequency"],
        rocket=sim,
    )
    np.random.seed(params["seed"])
    sim.with_control_algo(controls.LQR_control_algo).solve(State0=list(params["State0"]))
    return sim.timestamps_cache[0], sim.states_cache[0], sim.controls_cache[0]


def main():
    failed = False
    for overrides in SCENARIOS:
        params = scenario_params(overrides)
        new_hash = flight_hash(*fly_split(params))
        old_hash = flight_hash(*fly_old(params))
        same = new_hash == old_hash
        print(overrides["name"], same)
        if not same:
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
