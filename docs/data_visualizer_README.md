# Sensor Telemetry & Calibration Software

## Quick Start

```bash
pip install -r requirements.txt
python sensor_desktop_app.py
```

Or pass a CSV file directly on launch:

```bash
python sensor_desktop_app.py path/to/your_sensor_log.csv
```

## Headless / Programmatic Usage (`sensor_grapher.py`)

```python
from sensor_grapher import SensorDataset, SensorGrapher, PlotConfig, default_calibrations

dataset = SensorDataset.from_csv_file("sample_sensor_data.csv")
calibrations = default_calibrations()  # includes engine1 & engine2 defaults

config = PlotConfig(
    x_channel="timestamp_s",
    y_left_channels=["tempK1_C", "tempCalibrated_engine1", "tempCalibrated_engine2"],
    y_right_channels=["psi_1000"],
    t_start=2.40,
    t_end=2.85,
)
SensorGrapher().save_graph("engine_plot.png", dataset, calibrations, config)
```

## Default Calibration Formulas
- `tempCalibrated_engine1 = 0.528 * tempC + 0.517`
- `tempCalibrated_engine2 = 0.395 * tempC + 19.3`

Both defaults can be modified, toggled, reassigned to any `temp*` column, or removed in the desktop app, and custom `temp` or `psi` formulas can be added at runtime.

## Architecture
- `sensor_grapher.py`: Core CSV parser, millisecond-to-second timestamp converter, AST-safe calibration formula evaluator, channel statistics calculator, and Matplotlib `SensorGrapher` class (can be imported by any Python desktop app or run via CLI).
- `sensor_desktop_app.py`: Pure native desktop GUI (`tkinter` + `FigureCanvasTkAgg`) connected to `sensor_grapher.py` with zero web UI / zero browser dependencies.
- `sample_sensor_data.csv`: Sample sensor CSV log in the exact specified format.
- `test_sensor_grapher.py`: Standard-library unit tests covering CSV parsing, `nan` handling, seconds conversion, timestamp slicing, and calibration formula CRUD.