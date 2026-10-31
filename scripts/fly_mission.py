"""Fly the electric vehicle's mission (launch, climb, hover, descent, landing) with the PID or a trained model.

Usage: python scripts/fly_mission.py --model models/hop_v1 --flights 50 --compare --wind-mps 4 --animate
"""

import argparse
import subprocess
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from control.modules.guidance import PID
from control.modules.mission import MissionConfig, load_mission_config
from control.modules.plots import batch_figure, mission_figure
from control.modules.vehicle import HopVehicleConfig, load_vehicle_config
from simulation.modules.flightlog import read_flight_log
from simulation.modules.motors import MotorFileError
from simulation.modules.simconfig import SimConfig, WindConfig, load_sim_config
from simulation.modules.units import deg_to_rad
from simulation.modules.yaml_section import ConfigError, Section, read_yaml_mapping
from simulation.simulation import HopErrors, HopSimulation

DEFAULT_VEHICLE = "configs/vehicles/electric_hopper.yaml"
DEFAULT_MISSION = "configs/missions/hop_50m.yaml"
DEFAULT_WORLD = "configs/training/hop.yaml"
DEFAULT_OUT = "runs/flights"
DPI = 120
ANIMATION_SPEED = "3"
ERROR_SEED_OFFSET = 7919


@dataclass(frozen=True)
class HiddenErrorRanges:
    thrust_scale: tuple[float, float] = (1.0, 1.0)
    dry_mass_offset: tuple[float, float] = (0.0, 0.0)


def load_hidden_errors(path: str) -> HiddenErrorRanges:
    """The hidden-error ranges in a world file. Missing section means no error."""
    data, source = read_yaml_mapping(path)
    root = Section(data, source)
    if not root.has("hidden_errors"):
        return HiddenErrorRanges()
    section = root.sub("hidden_errors")
    section.only_keys("thrust_scale", "dry_mass_g")
    return HiddenErrorRanges(
        thrust_scale=section.pair("thrust_scale") if section.has("thrust_scale") else (1.0, 1.0),
        dry_mass_offset=section.pair("dry_mass_g") if section.has("dry_mass_g") else (0.0, 0.0),
    )


def scale_range(low_high: tuple[float, float], factor: float) -> tuple[float, float]:
    centre = 0.5 * (low_high[0] + low_high[1])
    half_width = 0.5 * (low_high[1] - low_high[0]) * factor
    return centre - half_width, centre + half_width


def draw_errors(ranges: HiddenErrorRanges, seed: int, factor: float = 1.0) -> HopErrors:
    rng = np.random.default_rng(seed + ERROR_SEED_OFFSET)
    return HopErrors(
        thrust_scale=float(rng.uniform(*scale_range(ranges.thrust_scale, factor))),
        dry_mass_offset=float(rng.uniform(*scale_range(ranges.dry_mass_offset, factor))),
    )


def with_mission_wind(sim: SimConfig, speed: float | None, direction: float | None, gust: float | None) -> SimConfig:
    wind: WindConfig = sim.wind
    if speed is not None:
        wind = replace(wind, steady=speed)
    if direction is not None:
        wind = replace(wind, steady_direction=direction)
    if gust is not None:
        wind = replace(wind, gust_std=gust)
    return replace(sim, wind=wind)


def load_world(vehicle_path: Path, mission_path: Path) -> tuple[HopVehicleConfig, MissionConfig]:
    return load_vehicle_config(vehicle_path), load_mission_config(mission_path)


def flight_info(sim: HopSimulation) -> dict[str, Any]:
    result = sim.result
    return {
        "landed": result.landed, "success": result.success,
        "timeout": sim.flight.touchdown is None and not result.aborted,
        "aborted": result.aborted, "max_height": result.max_height, "hover_held": result.hover_held,
        "miss": result.miss_distance, "vertical_speed": result.touchdown_speed, "flight_time": sim.t,
        "policy_fallbacks": sim.computer.policy_fallbacks, "target_altitude": sim.mission.target_altitude,
    }


def fly(
    vehicle: HopVehicleConfig, mission: MissionConfig, sim: SimConfig, seed: int,
    errors: HopErrors | None = None, log_path: str | Path | None = None,
) -> tuple[HopSimulation, dict[str, Any]]:
    """One mission flight with the PID."""
    vehicle = replace(vehicle, controllers=replace(vehicle.controllers, steering=PID, throttle=PID))
    simulation = HopSimulation(vehicle, mission, sim, seed=seed, log_path=log_path)
    simulation.reset(seed, errors=errors)
    while not simulation.done:
        simulation.control_step(None)
    simulation.close()
    return simulation, flight_info(simulation)


def summarize(infos: list[dict[str, Any]]) -> dict[str, float]:
    touched = [i for i in infos if np.isfinite(i["vertical_speed"])]
    return {
        "flights": len(infos),
        "success": sum(i["success"] for i in infos),
        "landed": sum(i["landed"] for i in infos),
        "aborted": sum(i["aborted"] for i in infos),
        "max_height": float(np.mean([i["max_height"] for i in infos])),
        "hover_held": float(np.mean([i["hover_held"] for i in infos])),
        "miss": float(np.mean([i["miss"] for i in infos])),
        "touchdown_speed": float(np.mean([i["vertical_speed"] for i in touched])) if touched else float("nan"),
    }


def summary_table(rows: dict[str, dict[str, float]]) -> str:
    lines = [f"{'controller':<14}{'mission ok':>12}{'landed':>9}{'aborted':>9}{'max h':>8}{'hover':>8}{'miss':>8}{'v down':>8}"]
    for name, s in rows.items():
        n = int(s["flights"])
        lines.append(
            f"{name:<14}{int(s['success']):>8}/{n:<3}{int(s['landed']):>6}/{n:<2}{int(s['aborted']):>7}  "
            f"{s['max_height']:>6.1f}{s['hover_held']:>8.1f}{s['miss']:>8.1f}{s['touchdown_speed']:>8.2f}"
        )
    lines.append("max h in m, hover = longest hold inside the altitude band in s, miss = distance from the pad in m")
    return "\n".join(lines)


def parse() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--vehicle", default=DEFAULT_VEHICLE)
    parser.add_argument("--mission", default=DEFAULT_MISSION)
    parser.add_argument("--world", default=DEFAULT_WORLD, help="training YAML that gives the simulation settings and the hidden error ranges")
    parser.add_argument("--model", help="model folder (models/<name>) for the network; without it the PID flies")
    parser.add_argument("--seed", type=int, default=100)
    parser.add_argument("--flights", type=int, default=1)
    parser.add_argument("--compare", action="store_true", help="also fly the PID on the same seeds (with --model)")
    parser.add_argument("--wind-mps", type=float, help="steady wind speed")
    parser.add_argument("--wind-direction-deg", type=float, help="direction the wind blows toward")
    parser.add_argument("--gust-mps", type=float, help="gust standard deviation")
    parser.add_argument("--hidden-errors", action="store_true", help="draw thrust and mass errors from the world file's ranges")
    parser.add_argument("--out", default=DEFAULT_OUT, help="folder for logs, plots and animations")
    parser.add_argument("--plot", action=argparse.BooleanOptionalAction, default=True, help="write plots (on by default)")
    parser.add_argument("--animate", action="store_true", help="write a 3D GIF of the (first) flight")
    args = parser.parse_args()
    if args.flights < 1:
        parser.error("--flights must be at least 1")
    return args


def main() -> None:
    args = parse()
    if args.model or args.compare:
        raise SystemExit("trained models were not part of this port; run without --model")
    try:
        vehicle, mission = load_world(Path(args.vehicle), Path(args.mission))
        sim = load_sim_config(args.world)
        direction = deg_to_rad(args.wind_direction_deg) if args.wind_direction_deg is not None else None
        sim = with_mission_wind(sim, args.wind_mps, direction, args.gust_mps)
        ranges = load_hidden_errors(args.world) if args.hidden_errors else HiddenErrorRanges()
    except (ConfigError, MotorFileError, FileNotFoundError) as error:
        raise SystemExit(str(error)) from error
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    controllers = ["pid"]
    rows = {}
    for name in controllers:
        infos, logs = [], []
        for seed in range(args.seed, args.seed + args.flights):
            log_path = out / f"{name}_seed{seed}.csv"
            _, info = fly(vehicle, mission, sim, seed, draw_errors(ranges, seed), log_path)
            infos.append(info)
            logs.append(log_path)
            if args.flights == 1 or not info["success"]:
                print(f"{name} seed {seed}: {describe(info)}")
        rows[name] = summarize(infos)
        if args.plot:
            if args.flights == 1:
                figure = mission_figure(read_flight_log(logs[0]), mission, f"{name}, seed {args.seed}: {describe(infos[0])}")
                figure.savefig(logs[0].with_suffix(".png"), dpi=DPI)
                print(f"wrote {logs[0].with_suffix('.png')}")
            else:
                figure = batch_figure([read_flight_log(p) for p in logs], [i["success"] for i in infos], mission, name)
                figure.savefig(out / f"{name}_flights.png", dpi=DPI)
                print(f"wrote {out / f'{name}_flights.png'}")
        if args.animate:
            gif = logs[0].with_suffix(".gif")
            subprocess.run([sys.executable, str(Path(__file__).parent / "animate_flight.py"), str(logs[0]), "--rocket",
                            args.vehicle, "--out", str(gif), "--follow", "--speed", ANIMATION_SPEED], check=True)
    if args.flights > 1 or len(rows) > 1:
        print(summary_table(rows))
    print(f"logs in {out}")


def describe(info: dict[str, Any]) -> str:
    verdict = "MISSION OK" if info["success"] else "mission failed"
    if info["aborted"]:
        verdict += " (aborted)"
    return (
        f"{verdict}; max height {info['max_height']:.1f} m, hover held {info['hover_held']:.1f} s, "
        f"{'landed' if info['landed'] else 'hard landing'} at {info['vertical_speed']:.2f} m/s, {info['miss']:.1f} m from the pad"
    )


if __name__ == "__main__":
    main()
