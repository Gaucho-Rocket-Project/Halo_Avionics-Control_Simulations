#!/usr/bin/env python3
"""
sensor_grapher.py
=================
Core sensor data parser, timestamp converter, AST-safe calibration engine,
and Matplotlib graph generator designed to connect seamlessly to a native
Python desktop software application (sensor_desktop_app.py) or run headlessly
from another Python script / CLI (no web UI).

Expected CSV header format:
    timestamp_ms,tempK0_C,intK0_C,tempK1_C,intK1_C,tempK2_C,intK2_C,
    tempK3_C,intK3_C,psi_500,psi_1000,psi_pin9,mA_500,mA_1000,mA_pin9
"""

from __future__ import annotations

import ast
import csv
import io
import json
import math
import operator
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple


SAMPLE_CSV_DATA = """timestamp_ms,tempK0_C,intK0_C,tempK1_C,intK1_C,tempK2_C,intK2_C,tempK3_C,intK3_C,psi_500,psi_1000,psi_pin9,mA_500,mA_1000,mA_pin9
2399,nan,nan,16.50,15.94,16.25,16.19,15.50,15.44,0.00,0.51,0.00,3.931,4.008,1.182
2499,nan,nan,16.50,15.94,16.50,16.25,15.50,15.50,0.00,0.00,0.00,3.947,3.925,3.865
2599,nan,nan,16.50,15.88,16.50,16.25,15.50,15.38,0.51,0.00,0.00,4.016,3.904,3.984
2699,nan,nan,16.50,15.94,16.50,16.25,15.50,15.56,0.76,0.00,0.00,4.024,3.880,3.906
2799,nan,nan,16.50,16.06,16.25,16.25,15.50,15.62,0.00,2.93,0.00,3.920,4.047,3.836
2899,nan,nan,16.50,15.88,16.50,16.31,15.75,15.62,0.64,0.00,0.00,4.021,3.907,3.908
"""

TIME_MODE_ABSOLUTE_S = "absolute_s"
TIME_MODE_RELATIVE_S = "relative_s"
TIME_MODE_RAW_MS = "raw_ms"

ALL_TEMP_TARGET = "__ALL_TEMP__"
ALL_PSI_TARGET = "__ALL_PSI__"


class FormulaError(ValueError):
    """Raised when a calibration formula cannot be parsed or evaluated safely."""


_SAFE_BINOPS: Dict[type, Callable[[float, float], float]] = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.FloorDiv: operator.floordiv,
    ast.Mod: operator.mod,
    ast.Pow: operator.pow,
}

_SAFE_UNARYOPS: Dict[type, Callable[[float], float]] = {
    ast.UAdd: operator.pos,
    ast.USub: operator.neg,
}

_SAFE_FUNCTIONS: Dict[str, Callable[..., float]] = {
    "abs": abs,
    "min": min,
    "max": max,
    "round": round,
    "sqrt": math.sqrt,
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "log": math.log,
    "log10": math.log10,
    "exp": math.exp,
    "pow": pow,
}

_SAFE_CONSTANTS: Dict[str, float] = {
    "pi": math.pi,
    "e": math.e,
    "nan": math.nan,
}


def normalize_formula_expression(raw_text: str, fallback_name: Optional[str] = None) -> Tuple[Optional[str], str]:
    """
    Accept either 'expr' (e.g. '0.528 * tempC + 0.517')
    or 'name = expr' (e.g. 'tempCalibrated_engine1 = 0.528 * tempC + 0.517').
    Returns (parsed_name_or_fallback, rhs_expression).
    """
    text = raw_text.strip()
    if not text:
        raise FormulaError("Formula expression cannot be empty.")

    if "=" in text and not any(op in text for op in ("==", "<=", ">=", "!=")):
        lhs, rhs = text.split("=", 1)
        lhs_clean = lhs.strip()
        rhs_clean = rhs.strip()
        if not rhs_clean:
            raise FormulaError("Missing right-hand expression after '='.")
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", lhs_clean):
            return (lhs_clean, rhs_clean)
        raise FormulaError(f"Invalid formula variable name on left side of '=': '{lhs_clean}'")

    return (fallback_name, text)


class SafeFormulaEvaluator:
    """
    AST-based mathematical expression evaluator.
    Prevents arbitrary code execution while allowing rich calibration math
    with variables like tempC, psi, x, val, raw, and dataset column names.
    """

    def __init__(self, expression: str) -> None:
        _, expr = normalize_formula_expression(expression)
        self.expression = expr
        try:
            self._tree = ast.parse(expr, mode="eval")
        except SyntaxError as exc:
            raise FormulaError(f"Syntax error in formula '{expr}': {exc.msg}") from exc
        self._validate_node(self._tree)

    def _validate_node(self, node: ast.AST) -> None:
        if isinstance(node, ast.Expression):
            self._validate_node(node.body)
        elif isinstance(node, ast.BinOp):
            if type(node.op) not in _SAFE_BINOPS:
                raise FormulaError(f"Unsupported binary operator: {type(node.op).__name__}")
            self._validate_node(node.left)
            self._validate_node(node.right)
        elif isinstance(node, ast.UnaryOp):
            if type(node.op) not in _SAFE_UNARYOPS:
                raise FormulaError(f"Unsupported unary operator: {type(node.op).__name__}")
            self._validate_node(node.operand)
        elif isinstance(node, ast.Constant):
            if not isinstance(node.value, (int, float)):
                raise FormulaError(f"Only numeric constants are allowed, got {type(node.value).__name__}")
        elif isinstance(node, ast.Name):
            if node.id.startswith("__"):
                raise FormulaError("Dunder identifiers are not permitted.")
        elif isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name) or node.func.id not in _SAFE_FUNCTIONS:
                func_name = getattr(node.func, "id", type(node.func).__name__)
                raise FormulaError(
                    f"Unsupported function '{func_name}'. Allowed: {', '.join(sorted(_SAFE_FUNCTIONS))}"
                )
            if node.keywords:
                raise FormulaError("Keyword arguments are not supported in calibration formulas.")
            for arg in node.args:
                self._validate_node(arg)
        else:
            raise FormulaError(f"Unsupported syntax element: {type(node).__name__}")

    def referenced_names(self) -> List[str]:
        names: List[str] = []
        for node in ast.walk(self._tree):
            if isinstance(node, ast.Name) and node.id not in _SAFE_CONSTANTS and node.id not in _SAFE_FUNCTIONS:
                if node.id not in names:
                    names.append(node.id)
        return names

    def evaluate(self, variables: Dict[str, float]) -> float:
        return self._eval_node(self._tree.body, variables)

    def _eval_node(self, node: ast.AST, variables: Dict[str, float]) -> float:
        if isinstance(node, ast.Constant):
            return float(node.value)
        if isinstance(node, ast.Name):
            if node.id in variables:
                return float(variables[node.id])
            if node.id in _SAFE_CONSTANTS:
                return float(_SAFE_CONSTANTS[node.id])
            raise FormulaError(f"Unknown variable '{node.id}' in formula '{self.expression}'.")
        if isinstance(node, ast.UnaryOp):
            operand = self._eval_node(node.operand, variables)
            return float(_SAFE_UNARYOPS[type(node.op)](operand))
        if isinstance(node, ast.BinOp):
            left = self._eval_node(node.left, variables)
            right = self._eval_node(node.right, variables)
            if math.isnan(left) or math.isnan(right):
                return math.nan
            try:
                return float(_SAFE_BINOPS[type(node.op)](left, right))
            except ZeroDivisionError:
                return math.nan
            except ValueError:
                return math.nan
        if isinstance(node, ast.Call):
            assert isinstance(node.func, ast.Name)
            fn = _SAFE_FUNCTIONS[node.func.id]
            args = [self._eval_node(arg, variables) for arg in node.args]
            if any(math.isnan(a) for a in args):
                return math.nan
            try:
                return float(fn(*args))
            except (ValueError, ZeroDivisionError, OverflowError):
                return math.nan
        raise FormulaError(f"Cannot evaluate node type {type(node).__name__}")


@dataclass
class CalibrationFormula:
    """Represents a user-configurable calibration formula for temp or psi data."""

    name: str
    expression: str
    domain: str = "temp"  # "temp" | "psi"
    target_column: str = "tempK1_C"  # specific column name or ALL_TEMP_TARGET / ALL_PSI_TARGET
    enabled: bool = True
    unit: str = ""
    notes: str = ""

    def __post_init__(self) -> None:
        self.domain = self.domain.lower().strip()
        if self.domain not in ("temp", "psi"):
            raise FormulaError(f"Calibration domain must be 'temp' or 'psi', got '{self.domain}'.")
        parsed_name, rhs = normalize_formula_expression(self.expression, fallback_name=self.name)
        self.name = (self.name or parsed_name or "calibrated").strip()
        self.expression = rhs
        # Validate expression immediately
        SafeFormulaEvaluator(self.expression)
        if not self.unit:
            self.unit = "°C (cal)" if self.domain == "temp" else "PSI (cal)"

    def apply_value(self, raw_val: float, row_context: Optional[Dict[str, float]] = None) -> float:
        if math.isnan(raw_val):
            return math.nan
        evaluator = SafeFormulaEvaluator(self.expression)
        env: Dict[str, float] = {}
        if row_context:
            env.update(row_context)
        env.update(
            {
                "x": raw_val,
                "val": raw_val,
                "raw": raw_val,
                "tempC": raw_val,
                "psi": raw_val,
                "T": raw_val,
                "P": raw_val,
            }
        )
        return evaluator.evaluate(env)

    def format_equation(self) -> str:
        return f"{self.name} = {self.expression}"


def default_calibrations() -> List[CalibrationFormula]:
    """Default calibration formulas required by the specification."""
    return [
        CalibrationFormula(
            name="tempCalibrated_engine1",
            expression="0.528 * tempC + 0.517",
            domain="temp",
            target_column="tempK1_C",
            enabled=True,
            unit="°C (cal)",
            notes="Default Engine 1 thermocouple calibration",
        ),
        CalibrationFormula(
            name="tempCalibrated_engine2",
            expression="0.395 * tempC + 19.3",
            domain="temp",
            target_column="tempK2_C",
            enabled=True,
            unit="°C (cal)",
            notes="Default Engine 2 thermocouple calibration",
        ),
    ]


def calibrations_to_json(formulas: Sequence[CalibrationFormula]) -> str:
    return json.dumps([asdict(f) for f in formulas], indent=2)


def calibrations_from_json(raw_json: str) -> List[CalibrationFormula]:
    payload = json.loads(raw_json)
    if not isinstance(payload, list):
        raise FormulaError("Calibration JSON must be a list of calibration objects.")
    return [CalibrationFormula(**item) for item in payload]


def _parse_float(raw_value: Any) -> float:
    if raw_value is None:
        return math.nan
    s = str(raw_value).strip()
    if not s or s.lower() in ("nan", "na", "null", "none", "-"):
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def infer_column_domain(column_name: str) -> str:
    low = column_name.lower()
    if "timestamp" in low:
        return "time"
    if "temp" in low or "intk" in low:
        return "temp"
    if "psi" in low:
        return "psi"
    if "ma" in low:
        return "current"
    return "other"


def infer_column_unit(column_name: str) -> str:
    low = column_name.lower()
    if column_name == "timestamp_s" or column_name == "timestamp_rel_s":
        return "s"
    if column_name == "timestamp_ms":
        return "ms"
    if "temp" in low or "intk" in low:
        return "°C"
    if "psi" in low:
        return "PSI"
    if low.startswith("ma") or "_ma" in low:
        return "mA"
    return ""


@dataclass
class ChannelStats:
    channel: str
    valid_count: int
    nan_count: int
    min_val: float
    max_val: float
    mean_val: float
    std_val: float


class SensorDataset:
    """
    Stores parsed sensor CSV rows, converts timestamps between ms and seconds,
    evaluates calibration formulas, and slices data by timestamp intervals.
    """

    def __init__(self, headers: List[str], rows: List[Dict[str, float]]) -> None:
        if not headers or not rows:
            raise ValueError("Sensor dataset cannot be empty.")
        self.raw_headers = list(headers)
        self.rows = rows

        # Determine base timestamp in ms for relative-seconds calculation
        valid_ms = [r["timestamp_ms"] for r in rows if "timestamp_ms" in r and not math.isnan(r["timestamp_ms"])]
        self.t0_ms: float = valid_ms[0] if valid_ms else 0.0

        for row in self.rows:
            ms = row.get("timestamp_ms", math.nan)
            row["timestamp_s"] = ms / 1000.0 if not math.isnan(ms) else math.nan
            row["timestamp_rel_s"] = (ms - self.t0_ms) / 1000.0 if not math.isnan(ms) else math.nan

    @classmethod
    def from_csv_text(cls, csv_text: str) -> "SensorDataset":
        lines = [line for line in csv_text.splitlines() if line.strip()]
        if len(lines) < 2:
            raise ValueError("CSV must contain a header row and at least one data row.")
        reader = csv.DictReader(io.StringIO("\n".join(lines)))
        if not reader.fieldnames:
            raise ValueError("Missing CSV header row.")
        headers = [h.strip() for h in reader.fieldnames if h and h.strip()]
        if "timestamp_ms" not in headers:
            raise ValueError("CSV must include a 'timestamp_ms' column.")

        rows: List[Dict[str, float]] = []
        for raw_row in reader:
            parsed_row: Dict[str, float] = {}
            for h in headers:
                parsed_row[h] = _parse_float(raw_row.get(h))
            rows.append(parsed_row)

        if not rows:
            raise ValueError("No valid rows parsed from CSV.")
        return cls(headers=headers, rows=rows)

    @classmethod
    def from_csv_file(cls, path: str | Path) -> "SensorDataset":
        content = Path(path).read_text(encoding="utf-8")
        return cls.from_csv_text(content)

    @classmethod
    def sample(cls) -> "SensorDataset":
        return cls.from_csv_text(SAMPLE_CSV_DATA)

    @property
    def temp_columns(self) -> List[str]:
        return [h for h in self.raw_headers if infer_column_domain(h) == "temp"]

    @property
    def psi_columns(self) -> List[str]:
        return [h for h in self.raw_headers if infer_column_domain(h) == "psi"]

    @property
    def current_columns(self) -> List[str]:
        return [h for h in self.raw_headers if infer_column_domain(h) == "current"]

    def time_bounds(self, time_mode: str = TIME_MODE_ABSOLUTE_S) -> Tuple[float, float]:
        key = self.time_column_for_mode(time_mode)
        vals = [r[key] for r in self.rows if not math.isnan(r.get(key, math.nan))]
        if not vals:
            return (0.0, 0.0)
        return (min(vals), max(vals))

    @staticmethod
    def time_column_for_mode(time_mode: str) -> str:
        if time_mode == TIME_MODE_RELATIVE_S:
            return "timestamp_rel_s"
        if time_mode == TIME_MODE_RAW_MS:
            return "timestamp_ms"
        return "timestamp_s"

    def build_calibrated_channels(
        self, calibrations: Sequence[CalibrationFormula]
    ) -> Tuple[List[str], Dict[str, List[float]], Dict[str, str]]:
        """
        Compute derived calibrated series for all enabled CalibrationFormula items.
        Returns:
            calibrated_names: ordered list of derived channel identifiers
            values_by_channel: channel_name -> list[float] (same length as self.rows)
            units_by_channel: channel_name -> unit string
        """
        calibrated_names: List[str] = []
        values_by_channel: Dict[str, List[float]] = {}
        units_by_channel: Dict[str, str] = {}

        for cal in calibrations:
            if not cal.enabled:
                continue

            if cal.target_column == ALL_TEMP_TARGET:
                targets = self.temp_columns
                multi_target = True
            elif cal.target_column == ALL_PSI_TARGET:
                targets = self.psi_columns
                multi_target = True
            elif cal.target_column in self.raw_headers:
                targets = [cal.target_column]
                multi_target = False
            else:
                # Target column not in current CSV; skip gracefully
                continue

            for target in targets:
                channel_name = f"{cal.name} ({target})" if multi_target else cal.name
                # Avoid accidental name collision if user created two formulas with same name
                if channel_name in values_by_channel:
                    channel_name = f"{channel_name} [{target}]"
                series = [cal.apply_value(row.get(target, math.nan), row_context=row) for row in self.rows]
                calibrated_names.append(channel_name)
                values_by_channel[channel_name] = series
                units_by_channel[channel_name] = cal.unit

        return calibrated_names, values_by_channel, units_by_channel

    def available_channels(self, calibrations: Sequence[CalibrationFormula]) -> List[str]:
        cal_names, _, _ = self.build_calibrated_channels(calibrations)
        base = ["timestamp_s", "timestamp_rel_s", "timestamp_ms"]
        sensors = [h for h in self.raw_headers if h != "timestamp_ms"]
        return base + sensors + cal_names

    def filter_indices(
        self,
        time_mode: str = TIME_MODE_ABSOLUTE_S,
        t_start: Optional[float] = None,
        t_end: Optional[float] = None,
    ) -> List[int]:
        time_col = self.time_column_for_mode(time_mode)
        indices: List[int] = []
        for idx, row in enumerate(self.rows):
            t_val = row.get(time_col, math.nan)
            if math.isnan(t_val):
                continue
            if t_start is not None and t_val < t_start - 1e-9:
                continue
            if t_end is not None and t_val > t_end + 1e-9:
                continue
            indices.append(idx)
        return indices

    def get_channel_series(
        self,
        channel: str,
        calibrations: Sequence[CalibrationFormula],
        indices: Optional[Sequence[int]] = None,
        axis_calibration: Optional[CalibrationFormula] = None,
    ) -> Tuple[List[float], str]:
        """
        Retrieve values and unit for any raw, time, or calibrated channel,
        optionally applying an axis-level calibration transform on the fly.
        """
        _, cal_values, cal_units = self.build_calibrated_channels(calibrations)
        idx_list = list(range(len(self.rows))) if indices is None else list(indices)

        if channel in cal_values:
            raw_series = [cal_values[channel][i] for i in idx_list]
            unit = cal_units.get(channel, "")
        else:
            raw_series = [self.rows[i].get(channel, math.nan) for i in idx_list]
            unit = infer_column_unit(channel)

        if axis_calibration is not None:
            domain = infer_column_domain(channel)
            if domain == axis_calibration.domain:
                transformed = [
                    axis_calibration.apply_value(raw_series[pos], row_context=self.rows[idx_list[pos]])
                    for pos in range(len(idx_list))
                ]
                return transformed, f"{axis_calibration.unit} [{axis_calibration.name}]"

        return raw_series, unit

    def compute_stats(
        self,
        channels: Sequence[str],
        calibrations: Sequence[CalibrationFormula],
        indices: Optional[Sequence[int]] = None,
    ) -> List[ChannelStats]:
        results: List[ChannelStats] = []
        for ch in channels:
            vals, _ = self.get_channel_series(ch, calibrations, indices=indices)
            valid = [v for v in vals if not math.isnan(v)]
            nan_count = len(vals) - len(valid)
            if valid:
                mean_v = sum(valid) / len(valid)
                var_v = sum((v - mean_v) ** 2 for v in valid) / len(valid)
                results.append(
                    ChannelStats(
                        channel=ch,
                        valid_count=len(valid),
                        nan_count=nan_count,
                        min_val=min(valid),
                        max_val=max(valid),
                        mean_val=mean_v,
                        std_val=math.sqrt(var_v),
                    )
                )
            else:
                results.append(
                    ChannelStats(
                        channel=ch,
                        valid_count=0,
                        nan_count=nan_count,
                        min_val=math.nan,
                        max_val=math.nan,
                        mean_val=math.nan,
                        std_val=math.nan,
                    )
                )
        return results

    def export_csv(
        self,
        path: str | Path,
        calibrations: Sequence[CalibrationFormula],
        indices: Optional[Sequence[int]] = None,
    ) -> None:
        cal_names, cal_values, _ = self.build_calibrated_channels(calibrations)
        idx_list = list(range(len(self.rows))) if indices is None else list(indices)
        fieldnames = ["timestamp_ms", "timestamp_s", "timestamp_rel_s"] + [
            h for h in self.raw_headers if h != "timestamp_ms"
        ] + cal_names

        with open(path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            for i in idx_list:
                out_row: Dict[str, Any] = {}
                for k in fieldnames:
                    if k in cal_values:
                        v = cal_values[k][i]
                    else:
                        v = self.rows[i].get(k, math.nan)
                    out_row[k] = "nan" if math.isnan(v) else f"{v:.6g}"
                writer.writerow(out_row)


@dataclass
class PlotConfig:
    x_channel: str = "timestamp_s"
    y_left_channels: List[str] = field(
        default_factory=lambda: ["tempK1_C", "tempCalibrated_engine1", "tempCalibrated_engine2"]
    )
    y_right_channels: List[str] = field(default_factory=list)
    time_mode: str = TIME_MODE_ABSOLUTE_S
    t_start: Optional[float] = None
    t_end: Optional[float] = None
    plot_style: str = "line_markers"  # "line_markers" | "line" | "scatter"
    sort_by_x: bool = False
    show_grid: bool = True
    show_legend: bool = True
    title: str = "Sensor Telemetry & Calibration Analysis"
    x_axis_calibration: Optional[CalibrationFormula] = None
    y_left_axis_calibration: Optional[CalibrationFormula] = None
    y_right_axis_calibration: Optional[CalibrationFormula] = None


_PALETTE = [
    "#0f766e",  # teal
    "#2563eb",  # cobalt blue
    "#dc2626",  # crimson
    "#d97706",  # amber
    "#7c3aed",  # violet
    "#059669",  # emerald
    "#db2777",  # magenta
    "#475569",  # slate
    "#0284c7",  # sky
    "#ea580c",  # orange
]


class SensorGrapher:
    """
    Generates high-precision Matplotlib figures from SensorDataset and PlotConfig.
    Connects directly to the Tkinter desktop app (via FigureCanvasTkAgg) or can
    export figures headlessly to PNG/SVG/PDF.
    """

    def render(
        self,
        figure: Any,
        dataset: SensorDataset,
        calibrations: Sequence[CalibrationFormula],
        config: PlotConfig,
    ) -> Tuple[Any, Optional[Any], List[int]]:
        """
        Draw the configured graph onto `figure` (matplotlib.figure.Figure).
        Returns (ax_left, ax_right_or_None, filtered_indices).
        """
        figure.clear()
        ax_left = figure.add_subplot(111)
        ax_right = None

        indices = dataset.filter_indices(
            time_mode=config.time_mode,
            t_start=config.t_start,
            t_end=config.t_end,
        )

        if not indices:
            ax_left.text(
                0.5,
                0.5,
                "No samples in selected timestamp range.",
                ha="center",
                va="center",
                transform=ax_left.transAxes,
                fontsize=11,
                color="#64748b",
            )
            ax_left.set_title(config.title)
            figure.tight_layout()
            return ax_left, None, indices

        x_vals, x_unit = dataset.get_channel_series(
            config.x_channel,
            calibrations,
            indices=indices,
            axis_calibration=config.x_axis_calibration,
        )

        # Style setup
        if config.plot_style == "scatter":
            linestyle = "None"
            marker = "o"
        elif config.plot_style == "line":
            linestyle = "-"
            marker = None
        else:
            linestyle = "-"
            marker = "o"

        handles = []
        labels = []
        color_idx = 0
        left_units: List[str] = []

        for ch in config.y_left_channels:
            y_vals, y_unit = dataset.get_channel_series(
                ch,
                calibrations,
                indices=indices,
                axis_calibration=config.y_left_axis_calibration,
            )
            if y_unit and y_unit not in left_units:
                left_units.append(y_unit)
            pairs = [
                (xv, yv)
                for xv, yv in zip(x_vals, y_vals)
                if not math.isnan(xv) and not math.isnan(yv)
            ]
            color = _PALETTE[color_idx % len(_PALETTE)]
            color_idx += 1

            if not pairs:
                line, = ax_left.plot([], [], color=color, linestyle=linestyle, marker=marker, label=f"{ch} (all NaN)")
                handles.append(line)
                labels.append(f"{ch} (all NaN)")
                continue

            if config.sort_by_x:
                pairs.sort(key=lambda pt: pt[0])

            xs = [p[0] for p in pairs]
            ys = [p[1] for p in pairs]
            lbl = f"{ch} [{y_unit}]" if y_unit else ch
            line, = ax_left.plot(
                xs,
                ys,
                color=color,
                linestyle=linestyle,
                marker=marker,
                markersize=4.5,
                linewidth=1.8,
                label=lbl,
            )
            handles.append(line)
            labels.append(lbl)

        right_units: List[str] = []
        if config.y_right_channels:
            ax_right = ax_left.twinx()
            for ch in config.y_right_channels:
                y_vals, y_unit = dataset.get_channel_series(
                    ch,
                    calibrations,
                    indices=indices,
                    axis_calibration=config.y_right_axis_calibration,
                )
                if y_unit and y_unit not in right_units:
                    right_units.append(y_unit)
                pairs = [
                    (xv, yv)
                    for xv, yv in zip(x_vals, y_vals)
                    if not math.isnan(xv) and not math.isnan(yv)
                ]
                color = _PALETTE[color_idx % len(_PALETTE)]
                color_idx += 1

                if not pairs:
                    line, = ax_right.plot(
                        [], [], color=color, linestyle="--", marker="s", label=f"{ch} (R, all NaN)"
                    )
                    handles.append(line)
                    labels.append(f"{ch} (R, all NaN)")
                    continue

                if config.sort_by_x:
                    pairs.sort(key=lambda pt: pt[0])

                xs = [p[0] for p in pairs]
                ys = [p[1] for p in pairs]
                lbl = f"{ch} (Right) [{y_unit}]" if y_unit else f"{ch} (Right)"
                line, = ax_right.plot(
                    xs,
                    ys,
                    color=color,
                    linestyle="--" if linestyle != "None" else "None",
                    marker="s" if marker else None,
                    markersize=4.0,
                    linewidth=1.7,
                    label=lbl,
                )
                handles.append(line)
                labels.append(lbl)

        x_label = f"{config.x_channel} [{x_unit}]" if x_unit else config.x_channel
        if config.x_channel == "timestamp_s":
            x_label = "Timestamp (seconds)"
        elif config.x_channel == "timestamp_rel_s":
            x_label = "Elapsed Time (seconds from t0)"
        elif config.x_channel == "timestamp_ms":
            x_label = "Timestamp (ms)"
        ax_left.set_xlabel(x_label, fontsize=10, fontweight="bold")

        if config.y_left_channels:
            left_unit_txt = ", ".join(left_units) if left_units else "Value"
            ax_left.set_ylabel(left_unit_txt, fontsize=10, fontweight="bold")
        if ax_right is not None:
            right_unit_txt = ", ".join(right_units) if right_units else "Secondary Value"
            ax_right.set_ylabel(right_unit_txt, fontsize=10, fontweight="bold", color="#334155")

        ax_left.set_title(config.title, fontsize=12, fontweight="bold", pad=10)
        if config.show_grid:
            ax_left.grid(True, linestyle=":", linewidth=0.7, alpha=0.65)
        ax_left.spines["top"].set_visible(False)
        if ax_right is not None:
            ax_right.spines["top"].set_visible(False)

        if config.show_legend and handles:
            ax_left.legend(handles, labels, loc="best", fontsize=8.5, frameon=True, framealpha=0.92)

        figure.tight_layout()
        return ax_left, ax_right, indices

    def save_graph(
        self,
        output_path: str | Path,
        dataset: SensorDataset,
        calibrations: Sequence[CalibrationFormula],
        config: PlotConfig,
        dpi: int = 150,
        figsize: Tuple[float, float] = (10.5, 5.8),
    ) -> Path:
        """Render and save a graph directly to PNG/SVG/PDF without launching any UI."""
        from matplotlib.figure import Figure

        fig = Figure(figsize=figsize, dpi=dpi)
        self.render(fig, dataset, calibrations, config)
        out = Path(output_path)
        fig.savefig(out, dpi=dpi, bbox_inches="tight")
        return out


def _cli_main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate sensor telemetry graphs from CSV data with calibrations."
    )
    parser.add_argument("csv_file", nargs="?", help="Path to CSV sensor log (defaults to embedded sample).")
    parser.add_argument("--x", default="timestamp_s", help="X-axis column (default: timestamp_s).")
    parser.add_argument(
        "--y",
        nargs="+",
        default=["tempK1_C", "tempCalibrated_engine1", "tempCalibrated_engine2"],
        help="Left Y-axis channels to compare.",
    )
    parser.add_argument("--y-right", nargs="*", default=[], help="Optional Right Y-axis channels.")
    parser.add_argument("--time-mode", choices=[TIME_MODE_ABSOLUTE_S, TIME_MODE_RELATIVE_S, TIME_MODE_RAW_MS], default=TIME_MODE_ABSOLUTE_S)
    parser.add_argument("--t-start", type=float, default=None, help="Start timestamp in selected time mode.")
    parser.add_argument("--t-end", type=float, default=None, help="End timestamp in selected time mode.")
    parser.add_argument("--style", choices=["line_markers", "line", "scatter"], default="line_markers")
    parser.add_argument("--cal", action="append", default=[], help="Extra calibration as 'name=expr@target_col' (e.g. 'psiCal=1.02*psi-0.4@psi_500').")
    parser.add_argument("--no-defaults", action="store_true", help="Do not load default engine1/engine2 calibrations.")
    parser.add_argument("--out", default="sensor_graph.png", help="Output image path (.png/.svg/.pdf).")
    args = parser.parse_args(argv)

    dataset = SensorDataset.from_csv_file(args.csv_file) if args.csv_file else SensorDataset.sample()
    cals = [] if args.no_defaults else default_calibrations()

    for spec in args.cal:
        if "@" in spec:
            formula_part, target = spec.rsplit("@", 1)
            target = target.strip()
        else:
            formula_part, target = spec, ALL_TEMP_TARGET
        name, expr = normalize_formula_expression(formula_part)
        domain = "psi" if "psi" in target.lower() or "psi" in expr.lower() else "temp"
        cals.append(
            CalibrationFormula(
                name=name or f"cal_{len(cals)+1}",
                expression=expr,
                domain=domain,
                target_column=target,
            )
        )

    cfg = PlotConfig(
        x_channel=args.x,
        y_left_channels=args.y,
        y_right_channels=args.y_right,
        time_mode=args.time_mode,
        t_start=args.t_start,
        t_end=args.t_end,
        plot_style=args.style,
    )
    out_path = SensorGrapher().save_graph(args.out, dataset, cals, cfg)
    print(f"Saved graph to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_cli_main())
