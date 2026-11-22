"""Compare the C vehicle_step() with halo_controls.py LQR_control_algo().

Run from the repository root:
    uv run python controls/c-port/tests/compare_halo_controls.py

Both sides get the same measurements every tick, but each keeps its own
Kalman state and previous command, so float/double differences accumulate the
way they would in flight. The python's simulated sensor noise is turned off.
Uses the tests/hash_tests/run.py BASE parameters, matching the C configs.
"""

import ctypes as ct
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    import numpy as np
except ModuleNotFoundError as exc:
    raise SystemExit(
        f"Missing Python dependency '{exc.name}'. Install NumPy "
        "before running this comparison."
    ) from exc

try:
    from controls.halo_controls import Controls
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing Python dependency "
        f"'{exc.name}'. Install NumPy, SciPy, and python-control "
        "before running this comparison."
    ) from exc

PARAMS = {
    "H": 50.0, "V": 4.0, "Ti": 21.0, "Th": 10.0, "Tf": 54.0, "Z": 0.83,
    "Q": np.diag([0.1, 350, 0.1, 320, 20, 50]),
    "R": np.diag([2000, 1536]),
    "R_kalman": np.diag([1, 1, 1, 1, 1, 1]) * 1e-5,
    "control_frequency": 15,
    "T": 3000, "P_max": 0.4, "m0": 285, "g": 9.8, "dNoz": 0.7,
    "l": 2.5, "r": 0.2, "Tau0": 3.23,
    "Q_kalman": np.diag([1e-10, 1e-10, 1e-5, 1e-5, 1e-10, 1e-7]),
}

SOURCES = [
    "src/lqr/lqr.c", "src/lqr/lqr_config.c", "src/vehicle_controller.c",
    "src/roll_control/roll_control.c", "src/roll_control/roll_config.c",
    "src/kalman_filter/kalman.c", "src/kalman_filter/kalman_config.c",
    "src/trajectory/trajectory.c", "src/trajectory/trajectory_config.c",
]

XZ_SIGNS = np.array([1, 1, 1, 1, -1, -1])
YZ_SIGNS = np.array([-1, 1, -1, 1, -1, -1])

KfState = ct.c_float * 6
KfCovariance = (ct.c_float * 6) * 6


class State(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in (
        "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps",
        "theta_rad", "theta_rate_radps", "phi_rad", "phi_rate_radps",
        "psi_rad", "psi_rate_radps",
    )]


class Goal(ct.Structure):
    _fields_ = [("height_m", ct.c_float), ("vertical_velocity_mps", ct.c_float),
                ("shutdown", ct.c_bool)]


class Command(ct.Structure):
    _fields_ = [("gimbal_x_rad", ct.c_float), ("gimbal_y_rad", ct.c_float),
                ("thrust_reduction", ct.c_float), ("roll", ct.c_int)]


class Controller(ct.Structure):
    _fields_ = [("xz_estimate", KfState), ("xz_covariance", KfCovariance),
                ("yz_estimate", KfState), ("yz_covariance", KfCovariance),
                ("command", Command)]


def sim_order(s):
    """VehicleState -> python state [x y z dx dy dz phi theta psi dphi dtheta dpsi]."""
    return np.array([s.x_m, s.y_m, s.z_m, s.vx_mps, s.vy_mps, s.vz_mps,
                     s.phi_rad, s.theta_rad, s.psi_rad,
                     s.phi_rate_radps, s.theta_rate_radps, s.psi_rate_radps],
                    dtype=np.float64)


def build(folder):
    library = Path(folder) / "controller.so"
    subprocess.run([
        os.environ.get("CC", "cc"), "-std=c11", "-O2", "-Wall", "-Wextra",
        "-Wpedantic", "-shared", "-fPIC", "-I", str(ROOT / "include"),
        *[str(ROOT / s) for s in SOURCES], "-lm", "-o", str(library),
    ], check=True)
    lib = ct.CDLL(str(library))
    lib.vehicle_step.argtypes = [ct.POINTER(Controller), ct.POINTER(State),
                                 ct.POINTER(Goal)]
    lib.vehicle_step.restype = Command
    lib.vehicle_controller_init.argtypes = [ct.POINTER(Controller),
                                            ct.POINTER(State)]
    lib.trajectory_goal.argtypes = [ct.c_void_p, ct.c_float]
    lib.trajectory_goal.restype = Goal
    return lib


def check_trajectory(lib, py):
    config = ct.addressof(ct.c_char.in_dll(lib, "trajectory_config"))
    worst = np.zeros(2)
    for t in np.float32(np.linspace(-1, 60, 20001)):
        t = float(t)
        goal = lib.trajectory_goal(config, t)
        expected = np.array([py.z_ref(t), py.dz_ref(t)])
        actual = np.array([goal.height_m, goal.vertical_velocity_mps])
        # Python returns 0 exactly at breakpoints; skip those float hits.
        if any(abs(t - b) < 2e-3 for b in (py.x1, py.x2, py.Ti, py.x4, py.x5,
                                           py.x6, py.Tf)):
            continue
        diff = np.abs(actual - expected)
        assert np.all(diff < [1e-4, 1e-3]), (t, actual, expected)
        assert goal.shutdown == (t > py.Tf), t
        worst = np.maximum(worst, diff)
    print("trajectory: max |error| [z_ref m, dz_ref m/s] =", worst)


def check_closed_sequence(lib, py, seed, ticks):
    rng = np.random.default_rng(seed)
    dt = py.dt
    config = ct.addressof(ct.c_char.in_dll(lib, "trajectory_config"))

    def measurement(k):
        t = k * dt
        wobble = np.sin(0.4 * t + seed)
        return State(
            x_m=0.8 * wobble + rng.normal(0, 0.05),
            y_m=-0.5 * wobble + rng.normal(0, 0.05),
            z_m=py.z_ref(t) + rng.normal(0, 0.3),
            vx_mps=0.3 * np.cos(0.4 * t) + rng.normal(0, 0.05),
            vy_mps=rng.normal(0, 0.1),
            vz_mps=py.dz_ref(t) + rng.normal(0, 0.2),
            theta_rad=rng.normal(0, 0.03), theta_rate_radps=rng.normal(0, 0.05),
            phi_rad=rng.normal(0, 0.03), phi_rate_radps=rng.normal(0, 0.05),
            psi_rad=0.05 * np.sin(0.2 * t) + rng.normal(0, 0.005),
            psi_rate_radps=rng.normal(0, 0.02),
        )

    initial = measurement(0)
    controller = Controller()
    lib.vehicle_controller_init(ct.byref(controller), ct.byref(initial))

    s0 = sim_order(initial)
    # Stored the way LQR_control_algo stores it (multiplied back by the signs).
    kx = np.array([s0[i] for i in [0, 2, 3, 5, 7, 10]]) * XZ_SIGNS * XZ_SIGNS
    ky = np.array([s0[i] for i in [1, 2, 4, 5, 6, 9]]) * YZ_SIGNS * YZ_SIGNS
    control_state = [kx, ky, [0, 0], np.zeros((6, 6)), np.zeros((6, 6))]
    p_hover = 1 - PARAMS["m0"] * PARAMS["g"] / PARAMS["T"]
    u = [p_hover, 0, 0, 0]

    worst = np.zeros(4)
    worst_est = 0.0
    for k in range(1, ticks):
        t = float(np.float32(k * dt))
        meas = measurement(k)

        out, u = py.LQR_control_algo(t, sim_order(meas), -1e9,
                                     control_state, u)
        control_state = out[1:]
        expected = np.array([u[1], u[2], u[0], u[3]], dtype=np.float64)

        goal = lib.trajectory_goal(config, t)
        cmd = lib.vehicle_step(ct.byref(controller), ct.byref(meas),
                               ct.byref(goal))
        actual = np.array([cmd.gimbal_x_rad, cmd.gimbal_y_rad,
                           cmd.thrust_reduction, cmd.roll], dtype=np.float64)

        est_diff = max(
            np.max(np.abs(np.array(controller.xz_estimate)
                          - control_state[0] * XZ_SIGNS)),
            np.max(np.abs(np.array(controller.yz_estimate)
                          - control_state[1] * YZ_SIGNS)))
        diff = np.abs(actual - expected)
        if np.any(diff > [1e-4, 1e-4, 1e-4, 0]) or est_diff > 1e-3:
            raise AssertionError(
                f"seed {seed} tick {k} t={t:.3f}: C={actual} "
                f"python={expected} estimate error={est_diff}")
        worst = np.maximum(worst, diff)
        worst_est = max(worst_est, est_diff)

    print(f"sequence seed {seed}: {ticks - 1} ticks, max |error| "
          f"[gx, gy, P, roll] = {worst}, estimate {worst_est:.2e}")


def main():
    # The python adds simulated sensor noise inside LQR_control_algo.
    np.random.multivariate_normal = lambda mean, cov: np.zeros(len(mean))
    py = Controls(PARAMS, PARAMS)

    # Python z_ref returns 0 exactly at a breakpoint (strict < and > on both
    # sides), and 15 Hz ticks land exactly on Ti = 21 and Ti + Th = 31. The C
    # version closes that gap, so close it here too to compare everything else.
    original_z_ref = py.z_ref
    breaks = (py.x1, py.x2, py.Ti, py.x4, py.x5, py.x6)
    py.z_ref = lambda t: (original_z_ref(t + 1e-9) if t in breaks
                          else original_z_ref(t))

    with tempfile.TemporaryDirectory(prefix="halo-controls-check-") as folder:
        lib = build(folder)
        check_trajectory(lib, py)
        ticks = int(60 * PARAMS["control_frequency"])  # past Tf = 54 s
        for seed in range(5):
            check_closed_sequence(lib, py, seed, ticks)
    print("PASS")


if __name__ == "__main__":
    main()
