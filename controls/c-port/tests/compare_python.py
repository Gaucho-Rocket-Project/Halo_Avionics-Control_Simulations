"""Compare the compiled C controller with the original Python LQR method.

Run from flight-control/: python3 tests/compare_python.py
Requires NumPy and a C compiler. Extracts the actual method using Python's AST
so SciPy, python-control, and matplotlib are not needed. Uses previously
generated gains; does not independently validate gain generation, the Kalman
filter, scheduler, or flight dynamics.
"""

import ast
import ctypes as ct
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT.parent / "Halo_Avionics-Control_Simulations" / "RocketSim.py"


class State(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in (
        "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps",
        "theta_rad", "theta_rate_radps", "phi_rad", "phi_rate_radps",
        "psi_rad", "psi_rate_radps",
    )]


class Command(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in (
        "gimbal_x_rad", "gimbal_y_rad", "thrust_reduction",
    )]


class Goal(ct.Structure):
    _fields_ = [("height_m", ct.c_float), ("vertical_velocity_mps", ct.c_float)]


def load_reference(source):
    tree = ast.parse(source.read_text())
    controls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                    and n.name == "Controls")
    method = next(n for n in controls.body if isinstance(n, ast.FunctionDef)
                  and n.name == "LQR")
    namespace = {"np": np}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"),
         namespace)
    # Independent copy of the user's Python-generated matrix, in float64.
    reference = SimpleNamespace(
        K=np.array([
            [-7.0710678119e-03, -9.6187139127e-19, -2.3728626707e-02,
             6.2161582696e-18, 3.5552393520e-01, 2.6485428503e-01],
            [-5.5927676097e-19, -4.7735163489e-01, -1.0232161620e-18,
             -5.4683648741e-01, 8.9901224699e-18, -7.2299453409e-18],
        ]),
        dt=1.0 / 15.0,
        rocket_sim_instance=SimpleNamespace(P_max=0.4),
    )
    return namespace["LQR"], reference


def python_step(method, reference, state, previous, goal):
    reference.z_ref = lambda t: goal.height_m
    reference.dz_ref = lambda t: goal.vertical_velocity_mps
    # Original controller's signed coordinates. Inputs here are already
    # estimates, bypassing the original Kalman estimator and its storage logic.
    gy, px = method(reference, 0, [
        state.x_m, state.z_m, state.vx_mps, state.vz_mps,
        -state.theta_rad, -state.theta_rate_radps,
    ], [previous[1], previous[2]])
    gx, py = method(reference, 0, [
        -state.y_m, state.z_m, -state.vy_mps, state.vz_mps,
        -state.phi_rad, -state.phi_rate_radps,
    ], [previous[0], previous[2]])
    return np.array([gx, gy, (px + py) / 2])


def main():
    source = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_SOURCE
    method, reference = load_reference(source)
    rng = np.random.default_rng(753)
    worst = np.zeros(3)
    count = 0

    with tempfile.TemporaryDirectory(prefix="lqr-python-check-") as folder:
        library = Path(folder) / "controller.so"
        subprocess.run([
            os.environ.get("CC", "cc"), "-std=c11", "-O2", "-Wall", "-Wextra",
            "-Wpedantic", "-shared", "-fPIC", "-I", str(ROOT / "include"),
            str(ROOT / "src/lqr/lqr.c"),
            str(ROOT / "src/lqr/lqr_config.c"),
            str(ROOT / "src/vehicle_controller.c"), "-o", str(library),
        ], check=True)
        lib = ct.CDLL(str(library))
        step = lib.vehicle_control_step
        step.argtypes = [ct.POINTER(State), ct.POINTER(Command), ct.POINTER(Goal)]
        step.restype = Command

        def compare(state, goal, c_previous, py_previous, label):
            nonlocal count, worst
            result = step(ct.byref(state), ct.byref(c_previous), ct.byref(goal))
            actual = np.array([result.gimbal_x_rad, result.gimbal_y_rad,
                               result.thrust_reduction])
            expected = python_step(method, reference, state, py_previous, goal)
            difference = np.abs(actual - expected)
            if not np.all(np.isfinite(actual)) or np.any(difference > 1e-6):
                raise AssertionError(
                    f"{label}: C={actual}, Python={expected}, error={difference}")
            worst = np.maximum(worst, difference)
            count += 1
            return result, expected

        # One signed input at a time: axis isolation and sign conventions.
        for field, _ in State._fields_:
            for value in (-0.01, 0.01):
                compare(State(**{field: value}), Goal(), Command(),
                        np.zeros(3), f"isolated {field}={value}")

        # Broad independent states and histories, including saturating errors.
        scales = np.array([10, 10, 30, 5, 5, 10, .3, 1, .3, 1, .3, 1])
        for i in range(2000):
            state = State(*(rng.uniform(-1, 1, 12) * scales))
            goal = Goal(rng.uniform(0, 50), rng.uniform(-5, 5))
            previous = Command(rng.uniform(-.22, .22), rng.uniform(-.22, .22),
                               rng.uniform(0, .4))
            py_previous = np.array([previous.gimbal_x_rad, previous.gimbal_y_rad,
                                    previous.thrust_reduction])
            compare(state, goal, previous, py_previous, f"independent {i}")

        # Each implementation feeds back its OWN previous output. This checks
        # numerical differences over time rather than resetting them each tick.
        for mode in ("smooth", "reversing", "random"):
            c_previous = Command(.13, -.17, .2)
            py_previous = np.array([c_previous.gimbal_x_rad,
                                    c_previous.gimbal_y_rad,
                                    c_previous.thrust_reduction])
            for i in range(1000):
                if mode == "random":
                    values = rng.uniform(-1, 1, 12) * scales
                else:
                    amplitude = (np.sin(i * .03) if mode == "smooth"
                                 else (1 if (i // 40) % 2 else -1))
                    values = amplitude * np.array(
                        [2, -3, 11, .2, -.3, .5, .1, -.2, -.15, .3, 0, 0])
                state = State(*values)
                c_previous, py_previous = compare(
                    state, Goal(10, .1), c_previous, py_previous,
                    f"{mode} tick {i}")

    print(f"PASS: {count} two-plane comparisons, absolute tolerance 1e-6")
    print("Maximum absolute differences [gx rad, gy rad, P]:", worst)
    print("Compared original Controls.LQR() feedback and limits with supplied")
    print("references and signed two-plane mapping; no estimator or dynamics.")


if __name__ == "__main__":
    main()
