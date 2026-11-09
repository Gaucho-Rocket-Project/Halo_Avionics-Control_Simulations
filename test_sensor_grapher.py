#!/usr/bin/env python3
"""Unit tests for sensor_grapher.py (standard library only)."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sensor_grapher import (
    ALL_PSI_TARGET,
    ALL_TEMP_TARGET,
    TIME_MODE_ABSOLUTE_S,
    TIME_MODE_RELATIVE_S,
    CalibrationFormula,
    FormulaError,
    SafeFormulaEvaluator,
    SensorDataset,
    default_calibrations,
)


class TestSensorGrapherCore(unittest.TestCase):
    def setUp(self) -> None:
        self.dataset = SensorDataset.sample()

    def test_sample_parsing_and_nan_handling(self) -> None:
        self.assertEqual(len(self.dataset.rows), 6)
        self.assertTrue(math.isnan(self.dataset.rows[0]["tempK0_C"]))
        self.assertTrue(math.isnan(self.dataset.rows[0]["intK0_C"]))
        self.assertAlmostEqual(self.dataset.rows[0]["tempK1_C"], 16.50)
        self.assertAlmostEqual(self.dataset.rows[4]["psi_1000"], 2.93)
        self.assertAlmostEqual(self.dataset.rows[0]["mA_pin9"], 1.182)

    def test_timestamp_conversion_to_seconds(self) -> None:
        self.assertAlmostEqual(self.dataset.rows[0]["timestamp_s"], 2.399)
        self.assertAlmostEqual(self.dataset.rows[5]["timestamp_s"], 2.899)
        self.assertAlmostEqual(self.dataset.rows[0]["timestamp_rel_s"], 0.000)
        self.assertAlmostEqual(self.dataset.rows[5]["timestamp_rel_s"], 0.500)

        abs_indices = self.dataset.filter_indices(TIME_MODE_ABSOLUTE_S, t_start=2.49, t_end=2.80)
        self.assertEqual(abs_indices, [1, 2, 3, 4])

        rel_indices = self.dataset.filter_indices(TIME_MODE_RELATIVE_S, t_start=0.10, t_end=0.30)
        self.assertEqual(rel_indices, [1, 2, 3])

    def test_default_calibrations(self) -> None:
        cals = default_calibrations()
        self.assertEqual(len(cals), 2)
        names, vals, _ = self.dataset.build_calibrated_channels(cals)
        self.assertEqual(names, ["tempCalibrated_engine1", "tempCalibrated_engine2"])

        # Row 0: tempK1_C = 16.50, tempK2_C = 16.25
        expected_e1 = 0.528 * 16.50 + 0.517
        expected_e2 = 0.395 * 16.25 + 19.3
        self.assertAlmostEqual(vals["tempCalibrated_engine1"][0], expected_e1, places=6)
        self.assertAlmostEqual(vals["tempCalibrated_engine2"][0], expected_e2, places=6)

    def test_add_modify_remove_psi_and_temp_calibrations(self) -> None:
        cals = default_calibrations()
        # Add custom PSI calibration
        psi_cal = CalibrationFormula(
            name="psiCal_500",
            expression="1.02 * psi + 0.5",
            domain="psi",
            target_column="psi_500",
        )
        cals.append(psi_cal)

        # Modify engine1 formula and retarget all temp columns
        cals[0] = CalibrationFormula(
            name="tempCalibrated_engine1",
            expression="tempCalibrated_engine1 = 0.55 * tempC + 1.0",
            domain="temp",
            target_column=ALL_TEMP_TARGET,
        )
        # Remove engine2
        cals.pop(1)

        names, vals, _ = self.dataset.build_calibrated_channels(cals)
        self.assertIn("psiCal_500", names)
        self.assertIn("tempCalibrated_engine1 (tempK1_C)", names)
        self.assertNotIn("tempCalibrated_engine2", names)

        # Row 2: psi_500 = 0.51 -> 1.02 * 0.51 + 0.5 = 1.0202
        self.assertAlmostEqual(vals["psiCal_500"][2], 1.0202, places=6)
        # Row 0: tempK0_C is nan -> calibrated value must remain nan
        self.assertTrue(math.isnan(vals["tempCalibrated_engine1 (tempK0_C)"][0]))

    def test_axis_level_calibration_override(self) -> None:
        cals = default_calibrations()
        psi_axis_cal = CalibrationFormula(
            name="psi_offset",
            expression="psi * 2.0 + 1.0",
            domain="psi",
            target_column=ALL_PSI_TARGET,
        )
        series, unit = self.dataset.get_channel_series("psi_1000", cals, axis_calibration=psi_axis_cal)
        self.assertIn("psi_offset", unit)
        self.assertAlmostEqual(series[4], 2.93 * 2.0 + 1.0, places=6)

    def test_safe_evaluator_blocks_unsafe_code(self) -> None:
        with self.assertRaises(FormulaError):
            SafeFormulaEvaluator("__import__('os').system('echo pwned')")
        with self.assertRaises(FormulaError):
            SafeFormulaEvaluator("[x for x in range(10)]")


if __name__ == "__main__":
    unittest.main()
