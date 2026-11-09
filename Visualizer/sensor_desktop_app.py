#!/usr/bin/env python3
"""
sensor_desktop_app.py
=====================
Native Python Desktop Software Application (Tkinter + Matplotlib, NO Web UI)
connected to `sensor_grapher.py` for analyzing and graphing sensor CSV logs.

Features:
- Pure OS native GUI window via tkinter / ttk + Matplotlib FigureCanvasTkAgg
- Automatic conversion of timestamp_ms -> seconds (absolute and relative t0=0s)
- Timestamp interval selection + sync from interactive zoom bounds
- Switchable X and Y axes + one-click Swap X <-> Y + Secondary Right Y-Axis
- Interactive zooming: Box Zoom, X-Time Span Zoom (auto-fits Y), Pan,
  Mouse Scroll Wheel zoom centered on cursor, Double-click reset, & Toolbar
- Add / Edit / Toggle / Remove calibration formulas for temp & psi channels
  (preloaded with tempCalibrated_engine1 and tempCalibrated_engine2)
- Export filtered + calibrated CSV and high-DPI PNG/SVG/PDF graphs
"""

from __future__ import annotations

import io
import json
import math
import os
import platform
import queue
import subprocess
import sys
import tempfile
import threading
import traceback
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List, Optional, Tuple

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.widgets import RectangleSelector, SpanSelector

from sensor_grapher import (
    ALL_PSI_TARGET,
    ALL_TEMP_TARGET,
    TIME_MODE_ABSOLUTE_S,
    TIME_MODE_RAW_MS,
    TIME_MODE_RELATIVE_S,
    CalibrationFormula,
    FormulaError,
    PlotConfig,
    SensorDataset,
    SensorGrapher,
    calibrations_from_json,
    calibrations_to_json,
    default_calibrations,
    infer_column_domain,
)


class CalibrationDialog(tk.Toplevel):
    """Modal dialog for adding or modifying a temp/psi calibration formula."""

    def __init__(
        self,
        parent: tk.Misc,
        dataset: SensorDataset,
        existing: Optional[CalibrationFormula] = None,
    ) -> None:
        super().__init__(parent)
        self.withdraw()  # Hide until centered and populated to prevent Linux WM blank window artifacts
        self.title("Edit Calibration Formula" if existing else "Add Calibration Formula")
        if parent is not None and getattr(parent, "winfo_viewable", lambda: False)():
            self.transient(parent)
        self.resizable(False, False)
        self.result: Optional[CalibrationFormula] = None
        self.dataset = dataset

        pad = {"padx": 10, "pady": 6}
        frame = ttk.Frame(self, padding=14)
        frame.pack(fill="both", expand=True)

        ttk.Label(frame, text="Calibration Name:", font=("TkDefaultFont", 9, "bold")).grid(
            row=0, column=0, sticky="w", **pad
        )
        self.name_var = tk.StringVar(value=existing.name if existing else "")
        ttk.Entry(frame, textvariable=self.name_var, width=36).grid(row=0, column=1, sticky="ew", **pad)

        ttk.Label(frame, text="Sensor Domain:", font=("TkDefaultFont", 9, "bold")).grid(
            row=1, column=0, sticky="w", **pad
        )
        self.domain_var = tk.StringVar(value=existing.domain if existing else "temp")
        domain_box = ttk.Combobox(
            frame, textvariable=self.domain_var, values=["temp", "psi"], state="readonly", width=33
        )
        domain_box.grid(row=1, column=1, sticky="ew", **pad)
        domain_box.bind("<<ComboboxSelected>>", lambda _e: self._refresh_targets())

        ttk.Label(frame, text="Target Column:", font=("TkDefaultFont", 9, "bold")).grid(
            row=2, column=0, sticky="w", **pad
        )
        self.target_var = tk.StringVar(value=existing.target_column if existing else ALL_TEMP_TARGET)
        self.target_box = ttk.Combobox(frame, textvariable=self.target_var, state="readonly", width=33)
        self.target_box.grid(row=2, column=1, sticky="ew", **pad)

        ttk.Label(frame, text="Formula Expression:", font=("TkDefaultFont", 9, "bold")).grid(
            row=3, column=0, sticky="w", **pad
        )
        self.expr_var = tk.StringVar(
            value=existing.expression if existing else "0.528 * tempC + 0.517"
        )
        ttk.Entry(frame, textvariable=self.expr_var, width=36).grid(row=3, column=1, sticky="ew", **pad)

        hint_text = (
            "Variables: tempC (for temp), psi (for pressure), x, raw, or column names (e.g. mA_500).\n"
            "Functions: abs, min, max, sqrt, log, log10, exp, pow, sin, cos."
        )
        ttk.Label(frame, text=hint_text, foreground="#475569", justify="left").grid(
            row=4, column=0, columnspan=2, sticky="w", padx=10, pady=(0, 6)
        )

        ttk.Label(frame, text="Output Unit:", font=("TkDefaultFont", 9, "bold")).grid(
            row=5, column=0, sticky="w", **pad
        )
        self.unit_var = tk.StringVar(value=existing.unit if existing else "°C (cal)")
        ttk.Entry(frame, textvariable=self.unit_var, width=36).grid(row=5, column=1, sticky="ew", **pad)

        self.enabled_var = tk.BooleanVar(value=existing.enabled if existing else True)
        ttk.Checkbutton(frame, text="Enabled (compute derived channel)", variable=self.enabled_var).grid(
            row=6, column=1, sticky="w", **pad
        )

        # Live test row
        test_frame = ttk.LabelFrame(frame, text="Quick Test Evaluator", padding=8)
        test_frame.grid(row=7, column=0, columnspan=2, sticky="ew", padx=10, pady=8)
        ttk.Label(test_frame, text="Sample Input:").pack(side="left", padx=(0, 6))
        self.test_in_var = tk.StringVar(value="16.50")
        ttk.Entry(test_frame, textvariable=self.test_in_var, width=10).pack(side="left", padx=(0, 8))
        ttk.Button(test_frame, text="Test", command=self._run_test).pack(side="left", padx=(0, 8))
        self.test_out_lbl = ttk.Label(test_frame, text="Result: —", foreground="#0f766e")
        self.test_out_lbl.pack(side="left")

        btn_row = ttk.Frame(frame)
        btn_row.grid(row=8, column=0, columnspan=2, sticky="e", padx=10, pady=(8, 0))
        ttk.Button(btn_row, text="Cancel", command=self.destroy).pack(side="right", padx=(6, 0))
        ttk.Button(btn_row, text="Save Formula", command=self._on_save).pack(side="right")

        self._refresh_targets(preserve_current=existing is not None)
        self._run_test()

        # Center dialog over parent window, map it, and establish grab
        self.update_idletasks()
        try:
            req_w = self.winfo_reqwidth()
            req_h = self.winfo_reqheight()
            p_w = parent.winfo_width()
            p_h = parent.winfo_height()
            p_x = parent.winfo_rootx()
            p_y = parent.winfo_rooty()
            pos_x = max(0, p_x + (p_w - req_w) // 2)
            pos_y = max(0, p_y + (p_h - req_h) // 2)
            self.geometry(f"+{pos_x}+{pos_y}")
        except Exception:
            pass

        self.deiconify()
        self.lift()
        try:
            self.grab_set()
        except Exception:
            pass

    def _refresh_targets(self, preserve_current: bool = False) -> None:
        domain = self.domain_var.get()
        if domain == "temp":
            cols = [ALL_TEMP_TARGET] + self.dataset.temp_columns
            if not preserve_current:
                self.unit_var.set("°C (cal)")
                if not self.expr_var.get().strip() or "psi" in self.expr_var.get():
                    self.expr_var.set("1.0 * tempC + 0.0")
        else:
            cols = [ALL_PSI_TARGET] + self.dataset.psi_columns
            if not preserve_current:
                self.unit_var.set("PSI (cal)")
                if not self.expr_var.get().strip() or "tempC" in self.expr_var.get():
                    self.expr_var.set("1.0 * psi + 0.0")

        self.target_box["values"] = cols
        if self.target_var.get() not in cols and cols:
            self.target_var.set(cols[0])

    def _build_formula(self) -> CalibrationFormula:
        return CalibrationFormula(
            name=self.name_var.get().strip(),
            expression=self.expr_var.get().strip(),
            domain=self.domain_var.get(),
            target_column=self.target_var.get(),
            enabled=self.enabled_var.get(),
            unit=self.unit_var.get().strip(),
        )

    def _run_test(self) -> None:
        try:
            cal = self._build_formula()
            sample_val = float(self.test_in_var.get().strip())
            out_val = cal.apply_value(sample_val)
            self.test_out_lbl.config(text=f"Result: {out_val:.5g}", foreground="#0f766e")
        except Exception as exc:
            self.test_out_lbl.config(text=f"Error: {exc}", foreground="#dc2626")

    def _on_save(self) -> None:
        try:
            cal = self._build_formula()
            if not cal.name:
                raise FormulaError("Please enter a calibration name.")
            self.result = cal
            self.destroy()
        except Exception as exc:
            messagebox.showerror("Invalid Formula", str(exc), parent=self)


class ScrollableFrame(ttk.Frame):
    """Reusable vertically scrollable container for the left control sidebar."""

    def __init__(self, parent: tk.Misc, width: int = 385) -> None:
        super().__init__(parent)
        self.canvas = tk.Canvas(self, width=width, highlightthickness=0, borderwidth=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.inner = ttk.Frame(self.canvas)

        self.inner.bind(
            "<Configure>",
            lambda _e: self.canvas.configure(scrollregion=self.canvas.bbox("all")),
        )
        self._window_id = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.bind(
            "<Configure>",
            lambda e: self.canvas.itemconfigure(self._window_id, width=e.width),
        )

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")


class BuildApplicationDialog(tk.Toplevel):
    """Run the native build in a worker, keeping the desktop application responsive."""

    def __init__(self, parent: tk.Misc, calibrations: List[CalibrationFormula]) -> None:
        super().__init__(parent)
        self.title("Build Executable Application")
        self.geometry("760x540")
        self.transient(parent)
        self._running = False
        self._last_output: Optional[str] = None
        self._messages: queue.Queue = queue.Queue()
        self._calibration_json = calibrations_to_json(calibrations)
        self._builder = Path(__file__).resolve().with_name("build_app.py")
        self._can_build = not getattr(sys, "frozen", False) and self._builder.is_file()
        self._target = {"Windows": "windows", "Darwin": "macos", "Linux": "linux"}.get(platform.system(), "")
        self._output = tk.StringVar(value=str(Path.home() / "SensorTelemetry-builds"))
        self._mode = tk.StringVar(value="onedir")

        frame = ttk.Frame(self, padding=16)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="Build a shareable desktop application", font=("TkDefaultFont", 13, "bold")).pack(anchor="w")
        ttk.Label(
            frame,
            text=f"Target: {platform.system()} / {platform.machine()}. Recipients do not need Python.\n"
                 "Build on each target OS, or use the included GitHub Actions workflow for all platforms.",
            wraplength=700, justify="left",
        ).pack(anchor="w", pady=(6, 12))

        mode_row = ttk.Frame(frame)
        mode_row.pack(fill="x")
        ttk.Label(mode_row, text="Layout:").pack(side="left", padx=(0, 10))
        ttk.Radiobutton(mode_row, text="Application folder (recommended)", variable=self._mode, value="onedir").pack(side="left")
        ttk.Radiobutton(
            mode_row, text="Single executable", variable=self._mode, value="onefile",
            state="disabled" if self._target == "macos" else "normal",
        ).pack(side="left", padx=10)

        path_row = ttk.Frame(frame)
        path_row.pack(fill="x", pady=12)
        ttk.Label(path_row, text="Output folder:").pack(side="left", padx=(0, 8))
        ttk.Entry(path_row, textvariable=self._output).pack(side="left", fill="x", expand=True)
        ttk.Button(path_row, text="Browse...", command=self._browse).pack(side="left", padx=(8, 0))

        notice = (
            "First install the build tools using this Python environment:\n"
            "python -m pip install -r requirements-build.txt\n"
            "Current calibration formulas will be included. Build output is unsigned unless you sign it separately."
            if self._can_build else
            "Building requires the source project and a Python build environment. Open the downloaded build kit\n"
            "and run build-windows.bat, build-macos.command, or build-linux.sh on the matching OS.\n"
            "A packaged application cannot compile other applications using its bundled Python runtime."
        )
        ttk.Label(frame, text=notice, wraplength=700, justify="left", foreground="#475569").pack(anchor="w", pady=(0, 10))

        log_frame = ttk.Frame(frame)
        log_frame.pack(fill="both", expand=True)
        self._log = tk.Text(log_frame, height=12, wrap="word", state="disabled", font=("TkFixedFont", 9))
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self._log.yview)
        self._log.configure(yscrollcommand=scroll.set)
        self._log.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        buttons = ttk.Frame(frame)
        buttons.pack(fill="x", pady=(12, 0))
        self._status = ttk.Label(buttons, text="Ready" if self._can_build else "Source build environment required")
        self._status.pack(side="left")
        ttk.Button(buttons, text="Close", command=self._close).pack(side="right", padx=(8, 0))
        self._build_button = ttk.Button(buttons, text="Build Application", command=self._start, state="normal" if self._can_build else "disabled")
        self._build_button.pack(side="right")
        self._open_button = ttk.Button(buttons, text="Open Output Folder", command=self._open_output, state="disabled")
        self._open_button.pack(side="right", padx=(0, 8))
        self.protocol("WM_DELETE_WINDOW", self._close)
        self._poll_id = self.after(150, self._poll)

    def _browse(self) -> None:
        selected = filedialog.askdirectory(parent=self, title="Choose build output folder")
        if selected:
            self._output.set(selected)

    def _start(self) -> None:
        if self._running or not self._can_build:
            return
        if not self._output.get().strip():
            messagebox.showerror("Output Folder", "Choose an output folder first.", parent=self)
            return
        output = str(Path(self._output.get()).expanduser().resolve())
        self._last_output = output
        self._running = True
        self._build_button.configure(state="disabled")
        self._open_button.configure(state="disabled")
        self._status.configure(text="Building and testing. This can take several minutes.")
        threading.Thread(target=self._worker, args=(output, self._mode.get()), daemon=True).start()

    def _worker(self, output: str, mode: str) -> None:
        try:
            with tempfile.TemporaryDirectory(prefix="sensor-calibrations-") as temp:
                profile = Path(temp) / "calibrations.json"
                profile.write_text(self._calibration_json, encoding="utf-8")
                command = [
                    sys.executable, "-u", str(self._builder), "--target", self._target,
                    "--mode", mode, "--output", output, "--entry", str(Path(__file__).resolve()),
                    "--calibrations", str(profile),
                ]
                with subprocess.Popen(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding="utf-8", errors="replace",
                    creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0,
                ) as process:
                    assert process.stdout is not None
                    for line in process.stdout:
                        self._messages.put(("log", line))
                    code = process.wait()
            self._messages.put(("done", code))
        except Exception as exc:
            self._messages.put(("log", f"Build failed: {exc}\n"))
            self._messages.put(("done", 1))

    def _poll(self) -> None:
        try:
            while True:
                kind, value = self._messages.get_nowait()
                if kind == "log":
                    self._log.configure(state="normal")
                    self._log.insert("end", value)
                    self._log.see("end")
                    self._log.configure(state="disabled")
                else:
                    self._running = False
                    self._build_button.configure(state="normal")
                    self._open_button.configure(state="normal" if value == 0 else "disabled")
                    self._status.configure(text="Build and smoke test passed. Share the release archive." if value == 0 else "Build failed. See the log above.")
        except queue.Empty:
            pass
        self._poll_id = self.after(150, self._poll)

    def _open_output(self) -> None:
        path = self._last_output or str(Path(self._output.get()).expanduser().resolve())
        try:
            if sys.platform == "win32":
                os.startfile(path)
            else:
                subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", path])
        except OSError as exc:
            messagebox.showerror("Open Folder", str(exc), parent=self)

    def _close(self) -> None:
        if self._running:
            messagebox.showinfo("Build Running", "Wait for the build to finish before closing this dialog.", parent=self)
            return
        self.after_cancel(self._poll_id)
        self.destroy()


def startup_calibrations() -> List[CalibrationFormula]:
    profile = Path(__file__).resolve().with_name("default_calibrations.json")
    if profile.is_file():
        return calibrations_from_json(profile.read_text(encoding="utf-8"))
    return default_calibrations()


class SensorDesktopApp(tk.Tk):
    """Main Tkinter Desktop Software Window connected to SensorGrapher."""

    def __init__(self, initial_csv: Optional[Path] = None) -> None:
        super().__init__()
        self.title("Sensor Telemetry & Calibration Desktop Suite")
        self.geometry("1420x880")
        self.minsize(1120, 700)

        self.grapher = SensorGrapher()
        self.dataset: SensorDataset = (
            SensorDataset.from_csv_file(initial_csv) if initial_csv else SensorDataset.sample()
        )
        self.current_file_label = str(initial_csv) if initial_csv else "Embedded Sample Sensor CSV"
        self.calibrations: List[CalibrationFormula] = startup_calibrations()

        # Plot state & zoom tracking
        self._ax_left = None
        self._ax_right = None
        self._filtered_indices: List[int] = []
        self._home_limits: Optional[Tuple[Tuple[float, float], Tuple[float, float], Optional[Tuple[float, float]]]] = None
        self._rect_selector: Optional[RectangleSelector] = None
        self._span_selector: Optional[SpanSelector] = None
        self._pan_active = False
        self._pan_start_px: Optional[Tuple[int, int]] = None
        self._pan_start_xlim: Optional[Tuple[float, float]] = None
        self._pan_start_ylim_l: Optional[Tuple[float, float]] = None
        self._pan_start_ylim_r: Optional[Tuple[float, float]] = None

        # Tk Variables
        self.time_mode_var = tk.StringVar(value=TIME_MODE_ABSOLUTE_S)
        t_min, t_max = self.dataset.time_bounds(TIME_MODE_ABSOLUTE_S)
        self.t_start_var = tk.StringVar(value=f"{t_min:.3f}")
        self.t_end_var = tk.StringVar(value=f"{t_max:.3f}")

        self.x_channel_var = tk.StringVar(value="timestamp_s")
        self.x_cal_var = tk.StringVar(value="None (Raw)")
        self.y_left_cal_var = tk.StringVar(value="None (Use Channel)")
        self.y_right_cal_var = tk.StringVar(value="None (Use Channel)")

        self.plot_style_var = tk.StringVar(value="line_markers")
        self.sort_by_x_var = tk.BooleanVar(value=False)
        self.show_grid_var = tk.BooleanVar(value=True)
        self.show_legend_var = tk.BooleanVar(value=True)
        self.interaction_mode_var = tk.StringVar(value="box_zoom")

        self.y_left_vars: Dict[str, tk.BooleanVar] = {}
        self.y_right_vars: Dict[str, tk.BooleanVar] = {}

        self._build_menu()
        self._build_ui()
        self._sync_channel_controls(preserve_selection=False)
        self._refresh_calibration_tree()
        self.replot()

    # ------------------------------------------------------------------ UI setup
    def _build_menu(self) -> None:
        menubar = tk.Menu(self)

        file_menu = tk.Menu(menubar, tearoff=False)
        file_menu.add_command(label="Open Sensor CSV...", command=self.load_csv_dialog, accelerator="Ctrl+O")
        file_menu.add_command(label="Load Embedded Sample CSV", command=self.load_sample_data)
        file_menu.add_separator()
        file_menu.add_command(label="Export Filtered & Calibrated CSV...", command=self.export_csv_dialog)
        file_menu.add_command(label="Save Graph Image (PNG/SVG/PDF)...", command=self.save_graph_dialog)
        file_menu.add_separator()
        file_menu.add_command(label="Build Executable Application...", command=self.build_application_dialog)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.destroy)
        menubar.add_cascade(label="File", menu=file_menu)

        cal_menu = tk.Menu(menubar, tearoff=False)
        cal_menu.add_command(label="Add Calibration Formula...", command=self.add_calibration_dialog)
        cal_menu.add_command(label="Save Calibrations JSON...", command=self.save_calibrations_dialog)
        cal_menu.add_command(label="Load Calibrations JSON...", command=self.load_calibrations_dialog)
        cal_menu.add_separator()
        cal_menu.add_command(label="Restore Default Engine1 / Engine2 Formulas", command=self.restore_default_calibrations)
        menubar.add_cascade(label="Calibration", menu=cal_menu)

        view_menu = tk.Menu(menubar, tearoff=False)
        view_menu.add_command(label="Reset Zoom (Full Window)", command=self.reset_zoom)
        view_menu.add_command(label="Reset Timestamp Filter to Full Dataset", command=self.reset_time_filter)
        view_menu.add_command(label="Swap X <-> Primary Y Axis", command=self.swap_x_y)
        menubar.add_cascade(label="View", menu=view_menu)

        self.config(menu=menubar)
        self.bind("<Control-o>", lambda _e: self.load_csv_dialog())

    def _build_ui(self) -> None:
        # Top status / action bar
        top_bar = ttk.Frame(self, padding=(10, 6))
        top_bar.pack(fill="x", side="top")

        ttk.Button(top_bar, text="Open CSV...", command=self.load_csv_dialog).pack(side="left", padx=(0, 6))
        ttk.Button(top_bar, text="Sample Data", command=self.load_sample_data).pack(side="left", padx=(0, 6))
        ttk.Button(top_bar, text="Export CSV", command=self.export_csv_dialog).pack(side="left", padx=(0, 6))
        ttk.Button(top_bar, text="Save Plot Image", command=self.save_graph_dialog).pack(side="left", padx=(0, 12))
        ttk.Button(top_bar, text="Build Executable...", command=self.build_application_dialog).pack(side="left", padx=(0, 12))

        self.file_status_lbl = ttk.Label(
            top_bar,
            text=f"Source: {self.current_file_label} ({len(self.dataset.rows)} rows)",
            foreground="#334155",
        )
        self.file_status_lbl.pack(side="left", fill="x", expand=True)

        # Main horizontal PanedWindow
        paned = ttk.PanedWindow(self, orient="horizontal")
        paned.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        # Left Sidebar
        sidebar_container = ScrollableFrame(paned, width=395)
        paned.add(sidebar_container, weight=0)
        sb = sidebar_container.inner

        # 1. Timestamp Frame
        time_lf = ttk.LabelFrame(sb, text="1. Timestamp Window (Seconds)", padding=8)
        time_lf.pack(fill="x", padx=6, pady=6)

        ttk.Label(time_lf, text="Time Scale:").grid(row=0, column=0, sticky="w", pady=3)
        mode_combo = ttk.Combobox(
            time_lf,
            textvariable=self.time_mode_var,
            values=[TIME_MODE_ABSOLUTE_S, TIME_MODE_RELATIVE_S, TIME_MODE_RAW_MS],
            state="readonly",
            width=24,
        )
        mode_combo.grid(row=0, column=1, columnspan=2, sticky="ew", pady=3)
        mode_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_time_mode_changed())

        ttk.Label(time_lf, text="Start:").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(time_lf, textvariable=self.t_start_var, width=12).grid(row=1, column=1, sticky="ew", padx=(0, 4))
        ttk.Label(time_lf, text="End:").grid(row=2, column=0, sticky="w", pady=3)
        ttk.Entry(time_lf, textvariable=self.t_end_var, width=12).grid(row=2, column=1, sticky="ew", padx=(0, 4))

        time_btns = ttk.Frame(time_lf)
        time_btns.grid(row=1, column=2, rowspan=2, sticky="ns", padx=(4, 0))
        ttk.Button(time_btns, text="Apply Filter", command=self.replot).pack(fill="x", pady=1)
        ttk.Button(time_btns, text="Full Range", command=self.reset_time_filter).pack(fill="x", pady=1)

        ttk.Button(
            time_lf,
            text="Lock Time Filter to Current Zoom",
            command=self.lock_time_to_zoom,
        ).grid(row=3, column=0, columnspan=3, sticky="ew", pady=(6, 2))
        time_lf.columnconfigure(1, weight=1)

        # 2. Axis & Comparison Frame
        axis_lf = ttk.LabelFrame(sb, text="2. Compare Data & Switch X / Y Axes", padding=8)
        axis_lf.pack(fill="x", padx=6, pady=6)

        x_row = ttk.Frame(axis_lf)
        x_row.pack(fill="x", pady=2)
        ttk.Label(x_row, text="X Axis:", width=8).pack(side="left")
        self.x_combo = ttk.Combobox(x_row, textvariable=self.x_channel_var, state="readonly")
        self.x_combo.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.x_combo.bind("<<ComboboxSelected>>", lambda _e: self.replot())
        ttk.Button(x_row, text="Swap X<->Y", width=10, command=self.swap_x_y).pack(side="right")

        x_cal_row = ttk.Frame(axis_lf)
        x_cal_row.pack(fill="x", pady=2)
        ttk.Label(x_cal_row, text="X Cal:", width=8).pack(side="left")
        self.x_cal_combo = ttk.Combobox(x_cal_row, textvariable=self.x_cal_var, state="readonly")
        self.x_cal_combo.pack(side="left", fill="x", expand=True)
        self.x_cal_combo.bind("<<ComboboxSelected>>", lambda _e: self.replot())

        style_row = ttk.Frame(axis_lf)
        style_row.pack(fill="x", pady=(4, 4))
        ttk.Label(style_row, text="Style:", width=8).pack(side="left")
        style_combo = ttk.Combobox(
            style_row,
            textvariable=self.plot_style_var,
            values=["line_markers", "line", "scatter"],
            state="readonly",
            width=14,
        )
        style_combo.pack(side="left", padx=(0, 6))
        style_combo.bind("<<ComboboxSelected>>", lambda _e: self.replot())
        ttk.Checkbutton(style_row, text="Sort X", variable=self.sort_by_x_var, command=self.replot).pack(side="left")
        ttk.Checkbutton(style_row, text="Grid", variable=self.show_grid_var, command=self.replot).pack(side="left")
        ttk.Checkbutton(style_row, text="Legend", variable=self.show_legend_var, command=self.replot).pack(side="left")

        # Y-axis selection tabs (Left Y vs Right Secondary Y)
        y_notebook = ttk.Notebook(axis_lf)
        y_notebook.pack(fill="both", expand=True, pady=(4, 0))

        self.y_left_tab = ttk.Frame(y_notebook, padding=6)
        self.y_right_tab = ttk.Frame(y_notebook, padding=6)
        y_notebook.add(self.y_left_tab, text="Left Y Channels")
        y_notebook.add(self.y_right_tab, text="Right Y (Secondary)")

        ly_cal_row = ttk.Frame(self.y_left_tab)
        ly_cal_row.pack(fill="x", pady=(0, 4))
        ttk.Label(ly_cal_row, text="Axis Transform:").pack(side="left", padx=(0, 4))
        self.y_left_cal_combo = ttk.Combobox(ly_cal_row, textvariable=self.y_left_cal_var, state="readonly")
        self.y_left_cal_combo.pack(side="left", fill="x", expand=True)
        self.y_left_cal_combo.bind("<<ComboboxSelected>>", lambda _e: self.replot())

        self.y_left_checks_frame = ttk.Frame(self.y_left_tab)
        self.y_left_checks_frame.pack(fill="both", expand=True)

        ry_cal_row = ttk.Frame(self.y_right_tab)
        ry_cal_row.pack(fill="x", pady=(0, 4))
        ttk.Label(ry_cal_row, text="Axis Transform:").pack(side="left", padx=(0, 4))
        self.y_right_cal_combo = ttk.Combobox(ry_cal_row, textvariable=self.y_right_cal_var, state="readonly")
        self.y_right_cal_combo.pack(side="left", fill="x", expand=True)
        self.y_right_cal_combo.bind("<<ComboboxSelected>>", lambda _e: self.replot())

        self.y_right_checks_frame = ttk.Frame(self.y_right_tab)
        self.y_right_checks_frame.pack(fill="both", expand=True)

        # 3. Calibration Formulas Frame
        cal_lf = ttk.LabelFrame(sb, text="3. Calibration Formulas (Temp / PSI)", padding=8)
        cal_lf.pack(fill="x", padx=6, pady=6)

        cols = ("on", "name", "target", "expr")
        self.cal_tree = ttk.Treeview(cal_lf, columns=cols, show="headings", height=6, selectmode="browse")
        self.cal_tree.heading("on", text="On")
        self.cal_tree.heading("name", text="Name")
        self.cal_tree.heading("target", text="Target")
        self.cal_tree.heading("expr", text="Formula")
        self.cal_tree.column("on", width=34, anchor="center")
        self.cal_tree.column("name", width=118)
        self.cal_tree.column("target", width=75)
        self.cal_tree.column("expr", width=130)
        self.cal_tree.pack(fill="x", expand=True)
        self.cal_tree.bind("<Double-1>", self._on_cal_tree_double_click)

        cal_btns = ttk.Frame(cal_lf)
        cal_btns.pack(fill="x", pady=(6, 0))
        ttk.Button(cal_btns, text="+ Add", width=7, command=self.add_calibration_dialog).pack(side="left", padx=(0, 3))
        ttk.Button(cal_btns, text="Edit", width=6, command=self.edit_calibration_dialog).pack(side="left", padx=(0, 3))
        ttk.Button(cal_btns, text="Toggle", width=7, command=self.toggle_selected_calibration).pack(side="left", padx=(0, 3))
        ttk.Button(cal_btns, text="Remove", width=8, command=self.remove_selected_calibration).pack(side="left", padx=(0, 3))
        ttk.Button(cal_btns, text="Defaults", width=8, command=self.restore_default_calibrations).pack(side="right")

        # Right Main Workspace
        right_pane = ttk.Frame(paned)
        paned.add(right_pane, weight=1)

        # Interactive Zoom & Tool Mode Strip
        tool_strip = ttk.Frame(right_pane, padding=(6, 4))
        tool_strip.pack(fill="x")

        ttk.Label(tool_strip, text="Graph Tool:", font=("TkDefaultFont", 9, "bold")).pack(side="left", padx=(0, 6))
        for label, mode in [
            ("Box Zoom", "box_zoom"),
            ("X-Time Zoom", "span_zoom"),
            ("Pan", "pan"),
            ("Inspect", "inspect"),
        ]:
            ttk.Radiobutton(
                tool_strip,
                text=label,
                variable=self.interaction_mode_var,
                value=mode,
                command=self._apply_interaction_mode,
            ).pack(side="left", padx=4)

        ttk.Separator(tool_strip, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Button(tool_strip, text="Zoom +", width=8, command=lambda: self._zoom_step(0.75)).pack(side="left", padx=2)
        ttk.Button(tool_strip, text="Zoom -", width=8, command=lambda: self._zoom_step(1.33)).pack(side="left", padx=2)
        ttk.Button(tool_strip, text="Reset Zoom", command=self.reset_zoom).pack(side="left", padx=4)

        self.zoom_bounds_lbl = ttk.Label(tool_strip, text="", foreground="#475569")
        self.zoom_bounds_lbl.pack(side="right", padx=6)

        # Vertical split inside right pane: Chart top, Stats/Table bottom
        v_split = ttk.PanedWindow(right_pane, orient="vertical")
        v_split.pack(fill="both", expand=True)

        chart_frame = ttk.Frame(v_split)
        v_split.add(chart_frame, weight=4)

        self.figure = Figure(figsize=(8.6, 5.2), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.figure, master=chart_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill="both", expand=True)

        self.toolbar = NavigationToolbar2Tk(self.canvas, chart_frame, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(fill="x", side="bottom")

        self.cursor_lbl = ttk.Label(
            chart_frame,
            text="Tip: Drag to zoom a section, scroll wheel to zoom at cursor, double-click to reset.",
            padding=(6, 3),
            foreground="#0f172a",
        )
        self.cursor_lbl.pack(fill="x", side="bottom")

        # Canvas mouse events
        self.canvas.mpl_connect("scroll_event", self._on_scroll_zoom)
        self.canvas.mpl_connect("motion_notify_event", self._on_mouse_move)
        self.canvas.mpl_connect("button_press_event", self._on_mouse_press)
        self.canvas.mpl_connect("button_release_event", self._on_mouse_release)

        # Bottom tabs: Statistics & Data Table
        bottom_tabs = ttk.Notebook(v_split)
        v_split.add(bottom_tabs, weight=1)

        stats_tab = ttk.Frame(bottom_tabs)
        data_tab = ttk.Frame(bottom_tabs)
        bottom_tabs.add(stats_tab, text="Channel Statistics (Selected Window)")
        bottom_tabs.add(data_tab, text="Data Table (ms & seconds)")

        self.stats_tree = ttk.Treeview(
            stats_tab,
            columns=("channel", "valid", "nan", "min", "max", "mean", "std"),
            show="headings",
            height=6,
        )
        for col_id, title, w in [
            ("channel", "Channel", 220),
            ("valid", "Valid", 65),
            ("nan", "NaN", 60),
            ("min", "Min", 110),
            ("max", "Max", 110),
            ("mean", "Mean", 110),
            ("std", "Std Dev", 110),
        ]:
            self.stats_tree.heading(col_id, text=title)
            self.stats_tree.column(col_id, width=w, anchor="w" if col_id == "channel" else "e")
        stats_scroll = ttk.Scrollbar(stats_tab, orient="vertical", command=self.stats_tree.yview)
        self.stats_tree.configure(yscrollcommand=stats_scroll.set)
        self.stats_tree.pack(side="left", fill="both", expand=True)
        stats_scroll.pack(side="right", fill="y")

        self.data_tree = ttk.Treeview(data_tab, show="headings", height=6)
        data_yscroll = ttk.Scrollbar(data_tab, orient="vertical", command=self.data_tree.yview)
        data_xscroll = ttk.Scrollbar(data_tab, orient="horizontal", command=self.data_tree.xview)
        self.data_tree.configure(yscrollcommand=data_yscroll.set, xscrollcommand=data_xscroll.set)
        self.data_tree.grid(row=0, column=0, sticky="nsew")
        data_yscroll.grid(row=0, column=1, sticky="ns")
        data_xscroll.grid(row=1, column=0, sticky="ew")
        data_tab.rowconfigure(0, weight=1)
        data_tab.columnconfigure(0, weight=1)

    def build_application_dialog(self) -> None:
        BuildApplicationDialog(self, self.calibrations)

    # ------------------------------------------------------------ Channel & Cal UI
    def _sync_channel_controls(self, preserve_selection: bool = True) -> None:
        all_channels = self.dataset.available_channels(self.calibrations)
        self.x_combo["values"] = all_channels
        if self.x_channel_var.get() not in all_channels:
            self.x_channel_var.set("timestamp_s")

        cal_options = ["None (Raw)"] + [c.name for c in self.calibrations if c.enabled]
        self.x_cal_combo["values"] = cal_options
        if self.x_cal_var.get() not in cal_options:
            self.x_cal_var.set("None (Raw)")

        axis_cal_options = ["None (Use Channel)"] + [c.name for c in self.calibrations if c.enabled]
        self.y_left_cal_combo["values"] = axis_cal_options
        self.y_right_cal_combo["values"] = axis_cal_options
        if self.y_left_cal_var.get() not in axis_cal_options:
            self.y_left_cal_var.set("None (Use Channel)")
        if self.y_right_cal_var.get() not in axis_cal_options:
            self.y_right_cal_var.set("None (Use Channel)")

        prev_left = {k for k, v in self.y_left_vars.items() if v.get()} if preserve_selection else {
            "tempK1_C",
            "tempCalibrated_engine1",
            "tempCalibrated_engine2",
        }
        prev_right = {k for k, v in self.y_right_vars.items() if v.get()} if preserve_selection else set()

        for frame in (self.y_left_checks_frame, self.y_right_checks_frame):
            for child in frame.winfo_children():
                child.destroy()

        self.y_left_vars.clear()
        self.y_right_vars.clear()

        sensor_candidates = [c for c in all_channels if c not in ("timestamp_s", "timestamp_rel_s", "timestamp_ms")]
        plot_candidates = sensor_candidates + ["timestamp_s", "timestamp_rel_s", "timestamp_ms"]
        for idx, ch in enumerate(plot_candidates):
            lv = tk.BooleanVar(value=ch in prev_left)
            rv = tk.BooleanVar(value=ch in prev_right)
            self.y_left_vars[ch] = lv
            self.y_right_vars[ch] = rv

            r, c = divmod(idx, 2)
            ttk.Checkbutton(self.y_left_checks_frame, text=ch, variable=lv, command=self.replot).grid(
                row=r, column=c, sticky="w", padx=4, pady=1
            )
            ttk.Checkbutton(self.y_right_checks_frame, text=ch, variable=rv, command=self.replot).grid(
                row=r, column=c, sticky="w", padx=4, pady=1
            )

        # Ensure at least one series is checked initially
        if not any(v.get() for v in self.y_left_vars.values()) and not any(v.get() for v in self.y_right_vars.values()):
            for fallback in ("tempK1_C", plot_candidates[0] if plot_candidates else None):
                if fallback and fallback in self.y_left_vars:
                    self.y_left_vars[fallback].set(True)
                    break

    def _refresh_calibration_tree(self) -> None:
        for item in self.cal_tree.get_children():
            self.cal_tree.delete(item)
        for idx, cal in enumerate(self.calibrations):
            self.cal_tree.insert(
                "",
                "end",
                iid=str(idx),
                values=(
                    "Yes" if cal.enabled else "No",
                    cal.name,
                    cal.target_column,
                    cal.expression,
                ),
            )

    def _find_calibration(self, label: str) -> Optional[CalibrationFormula]:
        for cal in self.calibrations:
            if cal.name == label and cal.enabled:
                return cal
        return None

    # ---------------------------------------------------------- Calibration CRUD
    def add_calibration_dialog(self) -> None:
        dlg = CalibrationDialog(self, self.dataset, existing=None)
        self.wait_window(dlg)
        if dlg.result is not None:
            self.calibrations.append(dlg.result)
            self._refresh_calibration_tree()
            self._sync_channel_controls(preserve_selection=True)
            # Automatically select newly added derived channel on Left Y if single target
            if dlg.result.name in self.y_left_vars:
                self.y_left_vars[dlg.result.name].set(True)
            self.replot()

    def _on_cal_tree_double_click(self, event: Optional[tk.Event] = None) -> None:
        if event is not None:
            region = self.cal_tree.identify_region(event.x, event.y)
            if region not in ("tree", "cell"):
                return
            row_id = self.cal_tree.identify_row(event.y)
            if row_id:
                self.cal_tree.selection_set(row_id)
            else:
                return
        self.edit_calibration_dialog()

    def edit_calibration_dialog(self) -> None:
        sel = self.cal_tree.selection()
        if not sel:
            messagebox.showinfo("Select Formula", "Select a calibration formula row to edit.", parent=self)
            return
        try:
            idx = int(sel[0])
            if idx < 0 or idx >= len(self.calibrations):
                return
        except (ValueError, IndexError):
            return
        old_name = self.calibrations[idx].name
        dlg = CalibrationDialog(self, self.dataset, existing=self.calibrations[idx])
        self.wait_window(dlg)
        if dlg.result is not None:
            was_checked = self.y_left_vars.get(old_name, tk.BooleanVar(value=False)).get()
            self.calibrations[idx] = dlg.result
            self._refresh_calibration_tree()
            self._sync_channel_controls(preserve_selection=True)
            if was_checked and dlg.result.name in self.y_left_vars:
                self.y_left_vars[dlg.result.name].set(True)
            self.replot()

    def toggle_selected_calibration(self) -> None:
        sel = self.cal_tree.selection()
        if not sel:
            return
        idx = int(sel[0])
        self.calibrations[idx].enabled = not self.calibrations[idx].enabled
        self._refresh_calibration_tree()
        self._sync_channel_controls(preserve_selection=True)
        self.replot()

    def remove_selected_calibration(self) -> None:
        sel = self.cal_tree.selection()
        if not sel:
            return
        idx = int(sel[0])
        del self.calibrations[idx]
        self._refresh_calibration_tree()
        self._sync_channel_controls(preserve_selection=True)
        self.replot()

    def restore_default_calibrations(self) -> None:
        self.calibrations = default_calibrations()
        self._refresh_calibration_tree()
        self._sync_channel_controls(preserve_selection=True)
        for name in ("tempCalibrated_engine1", "tempCalibrated_engine2"):
            if name in self.y_left_vars:
                self.y_left_vars[name].set(True)
        self.replot()

    def save_calibrations_dialog(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save Calibration Profile",
            defaultextension=".json",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")],
        )
        if path:
            Path(path).write_text(calibrations_to_json(self.calibrations), encoding="utf-8")

    def load_calibrations_dialog(self) -> None:
        path = filedialog.askopenfilename(
            title="Load Calibration Profile",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")],
        )
        if not path:
            return
        try:
            loaded = calibrations_from_json(Path(path).read_text(encoding="utf-8"))
            self.calibrations = loaded
            self._refresh_calibration_tree()
            self._sync_channel_controls(preserve_selection=True)
            self.replot()
        except Exception as exc:
            messagebox.showerror("Load Error", str(exc), parent=self)

    # ------------------------------------------------------------ Data & Time ops
    def load_csv_dialog(self) -> None:
        path = filedialog.askopenfilename(
            title="Open Sensor CSV File",
            filetypes=[("CSV Files", "*.csv"), ("Text / Log Files", "*.txt;*.log"), ("All Files", "*.*")],
        )
        if not path:
            return
        try:
            self.dataset = SensorDataset.from_csv_file(path)
            self.current_file_label = Path(path).name
            self.file_status_lbl.config(
                text=f"Source: {self.current_file_label} ({len(self.dataset.rows)} rows)"
            )
            self.reset_time_filter(replot_now=False)
            self._sync_channel_controls(preserve_selection=True)
            self.replot()
        except Exception as exc:
            messagebox.showerror("CSV Load Error", str(exc), parent=self)

    def load_sample_data(self) -> None:
        self.dataset = SensorDataset.sample()
        self.current_file_label = "Embedded Sample Sensor CSV"
        self.file_status_lbl.config(
            text=f"Source: {self.current_file_label} ({len(self.dataset.rows)} rows)"
        )
        self.reset_time_filter(replot_now=False)
        self._sync_channel_controls(preserve_selection=True)
        self.replot()

    def export_csv_dialog(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Export Filtered & Calibrated CSV",
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv")],
        )
        if not path:
            return
        self.dataset.export_csv(path, self.calibrations, indices=self._filtered_indices)
        messagebox.showinfo("Exported", f"Saved {len(self._filtered_indices)} rows to {path}", parent=self)

    def save_graph_dialog(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save Graph Image",
            defaultextension=".png",
            filetypes=[("PNG Image", "*.png"), ("SVG Vector", "*.svg"), ("PDF Document", "*.pdf")],
        )
        if not path:
            return
        self.figure.savefig(path, dpi=200, bbox_inches="tight")
        messagebox.showinfo("Saved", f"Saved graph to {path}", parent=self)

    def _on_time_mode_changed(self) -> None:
        mode = self.time_mode_var.get()
        if self.x_channel_var.get() in ("timestamp_s", "timestamp_rel_s", "timestamp_ms"):
            self.x_channel_var.set(SensorDataset.time_column_for_mode(mode))
        self.reset_time_filter(replot_now=True)

    def reset_time_filter(self, replot_now: bool = True) -> None:
        mode = self.time_mode_var.get()
        t_min, t_max = self.dataset.time_bounds(mode)
        fmt = "{:.0f}" if mode == TIME_MODE_RAW_MS else "{:.3f}"
        self.t_start_var.set(fmt.format(t_min))
        self.t_end_var.set(fmt.format(t_max))
        if replot_now:
            self.replot()

    def lock_time_to_zoom(self) -> None:
        if self._ax_left is None:
            return
        if self.x_channel_var.get() not in ("timestamp_s", "timestamp_rel_s", "timestamp_ms"):
            messagebox.showinfo(
                "X-Axis Not Time",
                "Switch X Axis to a timestamp column (timestamp_s / timestamp_rel_s / timestamp_ms) to crop timestamps to zoom.",
                parent=self,
            )
            return
        x0, x1 = sorted(self._ax_left.get_xlim())
        mode = self.time_mode_var.get()
        fmt = "{:.0f}" if mode == TIME_MODE_RAW_MS else "{:.3f}"
        self.t_start_var.set(fmt.format(x0))
        self.t_end_var.set(fmt.format(x1))
        self.replot()

    def swap_x_y(self) -> None:
        left_sel = [k for k, v in self.y_left_vars.items() if v.get()]
        right_sel = [k for k, v in self.y_right_vars.items() if v.get()]
        using_left = bool(left_sel)
        target_y = left_sel[0] if using_left else (right_sel[0] if right_sel else None)
        if not target_y:
            return
        old_x = self.x_channel_var.get()
        self.x_channel_var.set(target_y)
        target_map = self.y_left_vars if using_left else self.y_right_vars
        if old_x in target_map:
            for k in target_map:
                target_map[k].set(k == old_x)
        self.replot()

    # --------------------------------------------------------- Plotting & Tables
    def _parse_time_bound(self, text: str) -> Optional[float]:
        s = text.strip()
        if not s:
            return None
        return float(s)

    def replot(self) -> None:
        try:
            t_start = self._parse_time_bound(self.t_start_var.get())
            t_end = self._parse_time_bound(self.t_end_var.get())
        except ValueError:
            t_start, t_end = None, None

        y_left = [k for k, v in self.y_left_vars.items() if v.get()]
        y_right = [k for k, v in self.y_right_vars.items() if v.get()]

        # If user swapped a timestamp onto Y, ensure it gets plotted if selected
        cfg = PlotConfig(
            x_channel=self.x_channel_var.get(),
            y_left_channels=y_left,
            y_right_channels=y_right,
            time_mode=self.time_mode_var.get(),
            t_start=t_start,
            t_end=t_end,
            plot_style=self.plot_style_var.get(),
            sort_by_x=self.sort_by_x_var.get(),
            show_grid=self.show_grid_var.get(),
            show_legend=self.show_legend_var.get(),
            title=f"{self.current_file_label} — {', '.join(y_left + y_right) or 'No Y Series'}",
            x_axis_calibration=self._find_calibration(self.x_cal_var.get()),
            y_left_axis_calibration=self._find_calibration(self.y_left_cal_var.get()),
            y_right_axis_calibration=self._find_calibration(self.y_right_cal_var.get()),
        )

        ax_l, ax_r, indices = self.grapher.render(self.figure, self.dataset, self.calibrations, cfg)
        self._ax_left = ax_l
        self._ax_right = ax_r
        self._filtered_indices = indices
        self._home_limits = (
            ax_l.get_xlim(),
            ax_l.get_ylim(),
            ax_r.get_ylim() if ax_r is not None else None,
        )

        self._update_zoom_label()
        self._apply_interaction_mode()
        self.canvas.draw_idle()
        self._populate_stats_and_table(y_left + y_right)

    def _populate_stats_and_table(self, active_channels: List[str]) -> None:
        # Statistics table
        for item in self.stats_tree.get_children():
            self.stats_tree.delete(item)

        stats = self.dataset.compute_stats(active_channels, self.calibrations, indices=self._filtered_indices)
        for st in stats:
            fmt = lambda v: "NaN" if math.isnan(v) else f"{v:.4f}"
            self.stats_tree.insert(
                "",
                "end",
                values=(st.channel, st.valid_count, st.nan_count, fmt(st.min_val), fmt(st.max_val), fmt(st.mean_val), fmt(st.std_val)),
            )

        # Data Preview table
        for item in self.data_tree.get_children():
            self.data_tree.delete(item)

        cal_names, cal_values, _ = self.dataset.build_calibrated_channels(self.calibrations)
        cols = ["timestamp_ms", "timestamp_s", "timestamp_rel_s"] + [
            h for h in self.dataset.raw_headers if h != "timestamp_ms"
        ] + cal_names
        self.data_tree["columns"] = cols
        for c in cols:
            self.data_tree.heading(c, text=c)
            self.data_tree.column(c, width=108, anchor="e")

        for idx in self._filtered_indices[:500]:
            row = self.dataset.rows[idx]
            vals = []
            for c in cols:
                v = cal_values[c][idx] if c in cal_values else row.get(c, math.nan)
                if math.isnan(v):
                    vals.append("nan")
                elif c == "timestamp_ms":
                    vals.append(f"{v:.0f}")
                elif c in ("timestamp_s", "timestamp_rel_s"):
                    vals.append(f"{v:.3f}")
                else:
                    vals.append(f"{v:.4g}")
            self.data_tree.insert("", "end", values=vals)

    # ---------------------------------------------------- Interactive Zoom & Pan
    def _clear_selectors(self) -> None:
        if self._rect_selector is not None:
            self._rect_selector.set_active(False)
            self._rect_selector = None
        if self._span_selector is not None:
            self._span_selector.set_active(False)
            self._span_selector = None

    def _apply_interaction_mode(self) -> None:
        self._clear_selectors()
        if self._ax_left is None:
            return
        top_ax = self._ax_right if self._ax_right is not None else self._ax_left
        mode = self.interaction_mode_var.get()

        if mode == "box_zoom":
            self._rect_selector = RectangleSelector(
                top_ax,
                self._on_box_select,
                useblit=True,
                button=[1],
                minspanx=5,
                minspany=5,
                spancoords="pixels",
                interactive=False,
            )
        elif mode == "span_zoom":
            self._span_selector = SpanSelector(
                top_ax,
                self._on_span_select,
                "horizontal",
                useblit=True,
                props={"alpha": 0.22, "facecolor": "#2563eb"},
                interactive=False,
            )

    def _on_box_select(self, eclick, erelease) -> None:
        if self._ax_left is None or eclick.xdata is None or erelease.xdata is None:
            return
        x0, x1 = sorted([eclick.xdata, erelease.xdata])
        if abs(x1 - x0) <= 1e-12:
            return

        # Convert release Y fraction relative to top_ax so both left and right axes zoom cleanly
        top_ax = self._ax_right if self._ax_right is not None else self._ax_left
        ty0, ty1 = top_ax.get_ylim()
        if ty1 != ty0 and eclick.ydata is not None and erelease.ydata is not None:
            sy0, sy1 = sorted([eclick.ydata, erelease.ydata])
            f0 = (sy0 - ty0) / (ty1 - ty0)
            f1 = (sy1 - ty0) / (ty1 - ty0)
            ly0, ly1 = self._ax_left.get_ylim()
            self._ax_left.set_ylim(ly0 + f0 * (ly1 - ly0), ly0 + f1 * (ly1 - ly0))
            if self._ax_right is not None:
                self._ax_right.set_ylim(ty0 + f0 * (ty1 - ty0), ty0 + f1 * (ty1 - ty0))

        self._ax_left.set_xlim(x0, x1)
        self._update_zoom_label()
        self.canvas.draw_idle()

    def _on_span_select(self, xmin: float, xmax: float) -> None:
        if self._ax_left is None or abs(xmax - xmin) <= 1e-12:
            return
        x0, x1 = sorted([xmin, xmax])
        self._ax_left.set_xlim(x0, x1)

        # Auto-fit Y axes to data inside [x0, x1]
        for ax in (self._ax_left, self._ax_right):
            if ax is None:
                continue
            ys_visible: List[float] = []
            for line in ax.get_lines():
                xdata = line.get_xdata()
                ydata = line.get_ydata()
                for xv, yv in zip(xdata, ydata):
                    if x0 <= xv <= x1 and not math.isnan(yv):
                        ys_visible.append(yv)
            if ys_visible:
                ymin, ymax = min(ys_visible), max(ys_visible)
                pad = (ymax - ymin) * 0.1 if ymax > ymin else max(abs(ymin) * 0.1, 0.5)
                ax.set_ylim(ymin - pad, ymax + pad)

        self._update_zoom_label()
        self.canvas.draw_idle()

    def _zoom_step(self, scale: float) -> None:
        if self._ax_left is None:
            return
        x0, x1 = self._ax_left.get_xlim()
        xc = 0.5 * (x0 + x1)
        half_w = 0.5 * (x1 - x0) * scale
        self._ax_left.set_xlim(xc - half_w, xc + half_w)
        for ax in (self._ax_left, self._ax_right):
            if ax is not None:
                y0, y1 = ax.get_ylim()
                yc = 0.5 * (y0 + y1)
                half_h = 0.5 * (y1 - y0) * scale
                ax.set_ylim(yc - half_h, yc + half_h)
        self._update_zoom_label()
        self.canvas.draw_idle()

    def reset_zoom(self) -> None:
        if self._ax_left is None or self._home_limits is None:
            return
        xlim, ylim_l, ylim_r = self._home_limits
        self._ax_left.set_xlim(*xlim)
        self._ax_left.set_ylim(*ylim_l)
        if self._ax_right is not None and ylim_r is not None:
            self._ax_right.set_ylim(*ylim_r)
        self._update_zoom_label()
        self.canvas.draw_idle()

    def _on_scroll_zoom(self, event) -> None:
        if self._ax_left is None or event.inaxes not in (self._ax_left, self._ax_right):
            return
        if event.xdata is None:
            return
        scale = 0.82 if event.button == "up" else 1.22
        x0, x1 = self._ax_left.get_xlim()
        # Map xdata cleanly even if event fired on twin axis
        inv = self._ax_left.transData.inverted()
        xdata_l, ydata_l = inv.transform((event.x, event.y))
        new_x0 = xdata_l - (xdata_l - x0) * scale
        new_x1 = xdata_l + (x1 - xdata_l) * scale
        self._ax_left.set_xlim(new_x0, new_x1)

        ly0, ly1 = self._ax_left.get_ylim()
        self._ax_left.set_ylim(ydata_l - (ydata_l - ly0) * scale, ydata_l + (ly1 - ydata_l) * scale)

        if self._ax_right is not None:
            inv_r = self._ax_right.transData.inverted()
            _, ydata_r = inv_r.transform((event.x, event.y))
            ry0, ry1 = self._ax_right.get_ylim()
            self._ax_right.set_ylim(ydata_r - (ydata_r - ry0) * scale, ydata_r + (ry1 - ydata_r) * scale)

        self._update_zoom_label()
        self.canvas.draw_idle()

    def _on_mouse_press(self, event) -> None:
        if event.dblclick and event.inaxes in (self._ax_left, self._ax_right):
            self.reset_zoom()
            return
        if (
            self.interaction_mode_var.get() == "pan"
            and event.button == 1
            and event.inaxes in (self._ax_left, self._ax_right)
            and self._ax_left is not None
        ):
            self._pan_active = True
            self._pan_start_px = (event.x, event.y)
            self._pan_start_xlim = self._ax_left.get_xlim()
            self._pan_start_ylim_l = self._ax_left.get_ylim()
            self._pan_start_ylim_r = self._ax_right.get_ylim() if self._ax_right is not None else None

    def _on_mouse_release(self, _event) -> None:
        self._pan_active = False
        self._pan_start_px = None

    def _on_mouse_move(self, event) -> None:
        if self._ax_left is None:
            return
        if self._pan_active and self._pan_start_px and self._pan_start_xlim and self._pan_start_ylim_l:
            dx_px = event.x - self._pan_start_px[0]
            dy_px = event.y - self._pan_start_px[1]
            bbox = self._ax_left.bbox
            if bbox.width > 0 and bbox.height > 0:
                dx = -dx_px / bbox.width * (self._pan_start_xlim[1] - self._pan_start_xlim[0])
                dy_l = -dy_px / bbox.height * (self._pan_start_ylim_l[1] - self._pan_start_ylim_l[0])
                self._ax_left.set_xlim(self._pan_start_xlim[0] + dx, self._pan_start_xlim[1] + dx)
                self._ax_left.set_ylim(self._pan_start_ylim_l[0] + dy_l, self._pan_start_ylim_l[1] + dy_l)
                if self._ax_right is not None and self._pan_start_ylim_r is not None:
                    dy_r = -dy_px / bbox.height * (self._pan_start_ylim_r[1] - self._pan_start_ylim_r[0])
                    self._ax_right.set_ylim(self._pan_start_ylim_r[0] + dy_r, self._pan_start_ylim_r[1] + dy_r)
                self._update_zoom_label()
                self.canvas.draw_idle()
            return

        if event.inaxes in (self._ax_left, self._ax_right) and event.x is not None and event.y is not None:
            x_l, y_l = self._ax_left.transData.inverted().transform((event.x, event.y))
            msg = f"Cursor -> X ({self.x_channel_var.get()}): {x_l:.4f} | Left Y: {y_l:.4f}"
            if self._ax_right is not None:
                _, y_r = self._ax_right.transData.inverted().transform((event.x, event.y))
                msg += f" | Right Y: {y_r:.4f}"
            self.cursor_lbl.config(text=msg)

    def _update_zoom_label(self) -> None:
        if self._ax_left is None:
            return
        x0, x1 = self._ax_left.get_xlim()
        y0, y1 = self._ax_left.get_ylim()
        self.zoom_bounds_lbl.config(text=f"View X:[{x0:.3f}, {x1:.3f}]  Y:[{y0:.3f}, {y1:.3f}]")


def main(argv: Optional[List[str]] = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Native Sensor Telemetry desktop application")
    parser.add_argument("csv_file", nargs="?", type=Path)
    parser.add_argument("--smoke-test", action="store_true", help="Validate the packaged GUI, exports, and calibrations, then exit.")
    parser.add_argument("--smoke-report", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args([arg for arg in (sys.argv[1:] if argv is None else argv) if not arg.startswith("-psn_")])
    if args.smoke_test:
        app = None
        try:
            app = SensorDesktopApp(initial_csv=args.csv_file)
            app.withdraw()
            app.update_idletasks()
            app.canvas.draw()
            for image_format in ("png", "pdf", "svg"):
                image = io.BytesIO()
                app.figure.savefig(image, format=image_format)
                if image.getbuffer().nbytes == 0:
                    raise RuntimeError(f"Empty {image_format} graph export")
            test_cal = CalibrationFormula(name="smoke", expression="0.528 * tempC + 0.517")
            if not math.isclose(test_cal.apply_value(16.5), 9.229):
                raise RuntimeError("Calibration formula smoke test failed")
            report = {"status": "passed", "tk": tk.TkVersion, "samples": len(app.dataset.rows), "calibrations": [cal.name for cal in app.calibrations], "exports": ["png", "pdf", "svg"]}
            code = 0
        except Exception:
            report = {"status": "failed", "error": traceback.format_exc()}
            code = 1
        finally:
            if app is not None:
                app.destroy()
        if args.smoke_report:
            args.smoke_report.write_text(json.dumps(report, indent=2), encoding="utf-8")
        if sys.stdout is not None:
            print(json.dumps(report))
        return code
    app = SensorDesktopApp(initial_csv=args.csv_file)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
