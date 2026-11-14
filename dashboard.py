"""Browser dashboard for the HALO RocketSim LQR trajectory simulator.

Run from this directory with::

    python dashboard.py

The simulation is solved once when Launch is pressed.  Its adaptive-timestep
output is resampled for smooth, deterministic playback in the browser.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import tempfile
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import unquote, urlparse
import webbrowser


ROOT = Path(__file__).resolve().parent
STATIC_ROOT = ROOT / "dashboard"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8051
MAX_REQUEST_BYTES = 1_000_000
PLAYBACK_HZ = 30.0
MAX_PLAYBACK_FRAMES = 9_000
RENDERED_ROCKET_HEIGHT_M = 1.0
RENDERED_LEG_FRACTION = 0.18

DEFAULT_CONFIG: dict[str, Any] = {
    "rocket": {
        "T": 38.0,
        "P_max": 0.9,
        "m0": 2.180,
        "m_dot": 0,
        "g": 9.8,
        "dNoz": 0.32,
        "t_final": 20.0,
        "l": 0.7,
        "r": 0.05,
        "Tau0": 0.15,
        "Cd0": 0.8,
        "Cl0": 6.28,
        "S0": 0.0078,
        "dCoP": -0.03,
        "rho": 1.225,
        "Q_kalman": [1e-10, 1e-10, 1e-5, 1e-5, 1e-10, 1e-7],
    },
    "controller": {
        "H": 10.0,
        "V": 4.0,
        "Ti": 4.60,
        "Th": 10.0,
        "Tf": 19.6,
        "Z": 2.30419829,
        "Q": [4,53,4,5000,131,8.2],
        "R": [200,1500],
        "control_frequency": 15.0,
    },
    "wind": {
        "speed": 0.0,
        "direction_deg": 0.0,
        "gust_variance": 0.0,
    },
    # x, y, z, dx, dy, dz, phi, theta, psi, dphi, dtheta, dpsi, mass
    "state": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.180],
    "seed": 0,
}

ROCKET_LIMITS = {
    "T": (1e-6, 1e7),
    "P_max": (0.0, 0.999999),
    "m0": (1e-6, 1e7),
    "m_dot": (0.0, 1e5),
    "g": (1e-6, 100.0),
    "dNoz": (1e-6, 1e4),
    "t_final": (0.01, 300.0),
    "l": (1e-4, 1e4),
    "r": (1e-4, 1e3),
    "Tau0": (1e-8, 1e8),
    "Cd0": (0.0, 100.0),
    "Cl0": (0.0, 100.0),
    "S0": (0.0, 1e5),
    "dCoP": (-1e4, 1e4),
    "rho": (0.0, 100.0),
}

CONTROLLER_LIMITS = {
    "H": (0.0, 1e5),
    "V": (1e-6, 1e4),
    "Ti": (1e-6, 300.0),
    "Th": (0.0, 300.0),
    "Tf": (1e-6, 300.0),
    "Z": (1e-6, 1e4),
    "control_frequency": (0.1, 500.0),
}

WIND_LIMITS = {
    "speed": (0.0, 20.0),
    "direction_deg": (-180.0, 180.0),
    "gust_variance": (0.0, 100.0),
}


class InputError(ValueError):
    """A dashboard request contains an invalid simulation parameter."""


def _number(value: Any, name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool):
        raise InputError(f"{name} must be a number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise InputError(f"{name} must be a number") from exc
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise InputError(f"{name} must be between {minimum:g} and {maximum:g}")
    return result


def _vector(value: Any, name: str, length: int, minimum: float, maximum: float) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise InputError(f"{name} must contain exactly {length} numbers")
    return [_number(item, f"{name}[{index}]", minimum, maximum) for index, item in enumerate(value)]


def validate_config(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise InputError("request body must be a JSON object")
    raw_rocket = payload.get("rocket")
    raw_controller = payload.get("controller")
    if not isinstance(raw_rocket, dict) or not isinstance(raw_controller, dict):
        raise InputError("rocket and controller must be objects")

    rocket = {
        key: _number(raw_rocket.get(key), f"rocket.{key}", *limits)
        for key, limits in ROCKET_LIMITS.items()
    }
    rocket["Q_kalman"] = _vector(raw_rocket.get("Q_kalman"), "rocket.Q_kalman", 6, 0.0, 1e6)

    controller = {
        key: _number(raw_controller.get(key), f"controller.{key}", *limits)
        for key, limits in CONTROLLER_LIMITS.items()
    }
    controller["Q"] = _vector(raw_controller.get("Q"), "controller.Q", 6, 1e-12, 1e12)
    controller["R"] = _vector(raw_controller.get("R"), "controller.R", 2, 1e-12, 1e12)
    raw_wind = payload.get("wind")
    if not isinstance(raw_wind, dict):
        raise InputError("wind must be an object")
    wind = {
        key: _number(raw_wind.get(key), f"wind.{key}", *limits)
        for key, limits in WIND_LIMITS.items()
    }
    if controller["Tf"] <= controller["Ti"] + controller["Th"]:
        raise InputError("controller.Tf must be later than Ti + Th")
    if rocket["t_final"] < controller["Tf"]:
        raise InputError("rocket.t_final must be at least controller.Tf")

    state = _vector(payload.get("state"), "state", 13, -1e7, 1e7)
    if state[12] <= 0:
        raise InputError("state mass must be greater than zero")
    seed = int(_number(payload.get("seed", 0), "seed", 0, 2_147_483_647))
    return {"rocket": rocket, "controller": controller, "wind": wind, "state": state, "seed": seed}


def vehicle_info(config: dict[str, Any]) -> dict[str, Any]:
    rocket = config["rocket"]
    controller = config["controller"]
    # Rendering is intentionally independent of the effective length used by
    # the inertia model. Scale the complete solid geometry (feet to nose) to
    # exactly one metre while preserving the configured radius/length ratio.
    airframe_length = RENDERED_ROCKET_HEIGHT_M / (1.0 + RENDERED_LEG_FRACTION)
    visual_scale = airframe_length / rocket["l"]
    leg_height = RENDERED_ROCKET_HEIGHT_M - airframe_length
    pad_cg_height = 0.5 * airframe_length + leg_height
    return {
        "vehicle": {
            "length": airframe_length,
            "diameter": 2.0 * rocket["r"] * visual_scale,
            "cg": 0.5 * airframe_length,
            "pivot": airframe_length,
            "leg_count": 4,
            "leg_span": max(6.0 * rocket["r"], 0.55 * rocket["l"]) * visual_scale,
            "leg_height": leg_height,
            "pad_cg_height": pad_cg_height,
            "rendered_height": RENDERED_ROCKET_HEIGHT_M,
            "max_thrust": rocket["T"],
        },
        "mission": {
            "target_altitude": controller["H"],
            "altitude_tolerance": max(0.5, 0.02 * max(controller["H"], 1.0)),
            "landing_radius": max(2.0, 4.0 * rocket["r"]),
        },
    }


def _quaternion_zyx(phi: float, theta: float, psi: float) -> list[float]:
    """Return [w, x, y, z] for Rz(psi) Ry(theta) Rx(phi)."""
    cr, sr = math.cos(phi / 2.0), math.sin(phi / 2.0)
    cp, sp = math.cos(theta / 2.0), math.sin(theta / 2.0)
    cy, sy = math.cos(psi / 2.0), math.sin(psi / 2.0)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def _phase(t: float, controller: dict[str, Any]) -> str:
    if t < controller["Ti"]:
        return "ASCENT"
    if t < controller["Ti"] + controller["Th"]:
        return "HOVER"
    if t < controller["Tf"]:
        return "DESCENT"
    return "LANDING"


def _rounded(values: Any, digits: int = 6) -> list[float]:
    return [round(float(value), digits) for value in values]


def run_simulation(config: dict[str, Any]) -> dict[str, Any]:
    """Run RocketSim and convert its adaptive cache to browser playback frames."""
    # RocketSim imports matplotlib even though this path does not plot.  Keep its
    # cache out of a potentially read-only home directory.
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "halo-matplotlib"))
    try:
        import numpy as np
        from controls.halo_controls import Controls
        from runner.halo_runner import Runner
        from simulation.halo_sim import RocketSim
    except ImportError as exc:
        raise RuntimeError(
            f"RocketSim dependency is missing ({exc.name}). Install numpy, scipy, matplotlib, and control in this Python environment."
        ) from exc

    rocket = config["rocket"]
    controller = config["controller"]
    wind = config["wind"]
    params = {
        "T": rocket["T"],
        "P_max": rocket["P_max"],
        "m0": rocket["m0"],
        "m_dot": rocket["m_dot"],
        "g": rocket["g"],
        "dNoz": rocket["dNoz"],
        "t_final": rocket["t_final"],
        "l": rocket["l"],
        "r": rocket["r"],
        "Tau0": rocket["Tau0"],
        "Cd0": rocket["Cd0"],
        "Cl0": rocket["Cl0"],
        "S0": rocket["S0"],
        "dCoP": rocket["dCoP"],
        "rho": rocket["rho"],
        "Q_kalman": np.diag(rocket["Q_kalman"]),
        "H": controller["H"],
        "V": controller["V"],
        "Ti": controller["Ti"],
        "Th": controller["Th"],
        "Tf": controller["Tf"],
        "Z": controller["Z"],
        "Q": np.diag(controller["Q"]),
        "R": np.diag(controller["R"]),
        "R_kalman": np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]) * 1e-5,
        "control_frequency": controller["control_frequency"],
        "wind_speed": wind["speed"],
        "wind_direction": math.radians(wind["direction_deg"]),
        "gust_variance": wind["gust_variance"],
        "wind_seed": config["seed"],
    }
    sim = RocketSim(params)
    controls = Controls(params, params)
    runner = Runner(controls, sim)

    np.random.seed(config["seed"])
    runner.solve(State0=list(config["state"]))
    if not runner.timestamps_cache or not runner.timestamps_cache[-1]:
        raise RuntimeError("RocketSim completed without producing trajectory samples")

    raw_t = np.asarray(runner.timestamps_cache[-1], dtype=float)
    raw_states = np.asarray(runner.states_cache[-1], dtype=float)
    # The runner stores each completed run as (P, gx, gy, f) rows. Column 0
    # is the initial command; the remaining columns align with raw_t.
    raw_controls = np.asarray(runner.controls_cache[-1], dtype=float)[:, 1:].T
    sample_count = min(len(raw_t), raw_states.shape[1], len(raw_controls))
    raw_t = raw_t[:sample_count]
    raw_states = raw_states[:, :sample_count]
    raw_controls = raw_controls[:sample_count]
    finite = np.isfinite(raw_t) & np.all(np.isfinite(raw_states), axis=0) & np.all(np.isfinite(raw_controls), axis=1)
    if not np.any(finite):
        raise RuntimeError("RocketSim produced no finite trajectory samples")
    raw_t, raw_states, raw_controls = raw_t[finite], raw_states[:, finite], raw_controls[finite]

    # Include the exact launch condition and initial control in the playback.
    if raw_t[0] > 1e-9:
        raw_t = np.concatenate(([0.0], raw_t))
        raw_states = np.column_stack((np.asarray(config["state"], dtype=float), raw_states))
        raw_controls = np.vstack((np.zeros(4), raw_controls))

    duration = float(raw_t[-1])
    frame_count = min(MAX_PLAYBACK_FRAMES, max(2, int(math.ceil(duration * PLAYBACK_HZ)) + 1))
    playback_t = np.linspace(0.0, duration, frame_count)
    states = np.vstack([np.interp(playback_t, raw_t, row) for row in raw_states])
    # Avoid drawing a full revolution backwards if an Euler angle crosses +/-pi.
    for index in (6, 7, 8):
        states[index] = np.interp(playback_t, raw_t, np.unwrap(raw_states[index]))
    commands = np.vstack([np.interp(playback_t, raw_t, raw_controls[:, index]) for index in range(4)])

    frames = []
    rad_to_deg = 180.0 / math.pi
    for index, t_value in enumerate(playback_t):
        state = states[:, index]
        p_cut, gx, gy, rcs = commands[:, index]
        reference = float(controls.z_ref(float(t_value)))
        frames.append({
            "t": round(float(t_value), 4),
            "position": _rounded(state[0:3]),
            "velocity": _rounded(state[3:6]),
            "quaternion": _rounded(_quaternion_zyx(*state[6:9])),
            "euler_deg": _rounded(state[6:9] * rad_to_deg, 4),
            "angular_rate_deg": _rounded(state[9:12] * rad_to_deg, 4),
            "mass": round(float(state[12]), 5),
            "throttle_pct": round(100.0 * (1.0 - float(p_cut)), 3),
            "thrust": round(rocket["T"] * (1.0 - float(p_cut)), 4),
            "gimbal_deg": _rounded([gx * rad_to_deg, gy * rad_to_deg], 4),
            "rcs": round(float(rcs), 4),
            "reference_altitude": round(reference, 5),
            "phase": _phase(float(t_value), controller),
            "wind": _rounded(sim.wind_at(float(t_value)), 4),
        })

    final = states[:, -1]
    horizontal_distance = math.hypot(float(final[0]), float(final[1]))
    speed = float(np.linalg.norm(final[3:6]))
    status_code = int(runner.status_cache[-1]) if runner.status_cache else 0
    status_messages = {
        0: "Reached the configured final simulation time",
        1: "RocketSim stopped after its 10 second solver-time limit",
        2: "Vehicle crossed ground level",
    }
    result = {
        "status": status_code,
        "message": status_messages.get(status_code, f"RocketSim status {status_code}"),
        "duration": round(duration, 4),
        "max_altitude": round(float(np.max(states[2])), 4),
        "final_altitude": round(float(final[2]), 4),
        "horizontal_distance": round(horizontal_distance, 4),
        "final_speed": round(speed, 4),
        "raw_samples": int(sample_count),
        "playback_frames": len(frames),
    }
    return {**vehicle_info(config), "frames": frames, "result": result}


def _safe_json(value: Any) -> bytes:
    return json.dumps(value, allow_nan=False, separators=(",", ":")).encode("utf-8")


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "HALORocketDashboard/1.0"

    def log_message(self, format_string: str, *args: Any) -> None:
        print(f"{self.client_address[0]} - {format_string % args}")

    def _json(self, status: int, value: Any) -> None:
        body = _safe_json(value)
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _serve_static(self, request_path: str) -> None:
        relative = "index.html" if request_path in ("", "/") else unquote(request_path.lstrip("/"))
        candidate = (STATIC_ROOT / relative).resolve()
        try:
            candidate.relative_to(STATIC_ROOT.resolve())
        except ValueError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if not candidate.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        mime = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".js": "text/javascript; charset=utf-8",
            ".svg": "image/svg+xml",
        }.get(candidate.suffix, "application/octet-stream")
        body = candidate.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler
        path = urlparse(self.path).path
        if path == "/api/defaults":
            self._json(HTTPStatus.OK, {"config": DEFAULT_CONFIG, **vehicle_info(DEFAULT_CONFIG)})
            return
        if path.startswith("/api/"):
            self._json(HTTPStatus.NOT_FOUND, {"error": "unknown API route"})
            return
        self._serve_static(path)

    def do_POST(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler
        if urlparse(self.path).path != "/api/simulate":
            self._json(HTTPStatus.NOT_FOUND, {"error": "unknown API route"})
            return
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0 or content_length > MAX_REQUEST_BYTES:
                raise InputError("request body is empty or too large")
            payload = json.loads(self.rfile.read(content_length))
            config = validate_config(payload)
            # RocketSim and NumPy's global random generator are not safe to run
            # concurrently. Other static/API requests remain threaded.
            with self.server.simulation_lock:  # type: ignore[attr-defined]
                answer = run_simulation(config)
            self._json(HTTPStatus.OK, answer)
        except (InputError, json.JSONDecodeError) as exc:
            self._json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
        except Exception as exc:  # keep numerical/configuration failures visible in the form
            self._json(HTTPStatus.UNPROCESSABLE_ENTITY, {"error": f"Simulation failed: {exc}"})


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int]):
        super().__init__(address, DashboardHandler)
        self.simulation_lock = threading.Lock()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=DEFAULT_HOST, help="listen address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"listen port (default: {DEFAULT_PORT})")
    parser.add_argument("--no-browser", action="store_true", help="do not open the dashboard in the default browser")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    server = DashboardServer((args.host, args.port))
    host, port = server.server_address[:2]
    browser_host = "127.0.0.1" if host in ("0.0.0.0", "::") else host
    url = f"http://{browser_host}:{port}/"
    print(f"HALO rocket dashboard running at {url}  (Ctrl+C to stop)", flush=True)
    if not args.no_browser:
        threading.Timer(0.35, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopping dashboard")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
