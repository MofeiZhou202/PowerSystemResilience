import unittest

import numpy as np

import climate_data as cd
import climate_signal_fidelity as signal
import future_weather_batch as future


class ClimateSignalFidelityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.definition = cd.load_json(future.DEFAULT_DEFINITION)

    def test_maximum_run(self):
        values = np.asarray([False, True, True, False, True, True, True])
        self.assertEqual(signal.maximum_run(values), 3)

    def test_metrics_use_complete_years_and_historical_thresholds(self):
        days = 365 * 2
        tasmax = np.full(days, 25.0)
        tasmax[10:15] = 36.0
        tasmax[380:383] = 37.0
        precipitation = np.full(days, 2.0)
        precipitation[10:15] = 0.0
        precipitation[380:383] = 0.0
        precipitation[100:103] = (10.0, 20.0, 30.0)
        precipitation[500:503] = (20.0, 30.0, 40.0)
        arrays = {
            "tasmax": tasmax,
            "tasmin": tasmax - 8.0,
            "hurs": np.linspace(40.0, 80.0, days),
            "sfcWind": np.full(days, 4.0),
            "rsds": np.linspace(100.0, 300.0, days),
            "pr": precipitation,
        }
        metrics = signal.metric_values(
            arrays,
            hot_threshold=35.0,
            low_wind_threshold=5.0,
            dry_threshold=1.0,
            definition=self.definition,
        )
        self.assertEqual(set(metrics), set(signal.METRIC_LABELS))
        self.assertAlmostEqual(metrics["hot_days_per_year"], 4.0)
        self.assertAlmostEqual(metrics["hot_dry_low_wind_days_per_year"], 4.0)
        self.assertAlmostEqual(metrics["mean_annual_rx3day_mm"], 75.0)
        self.assertAlmostEqual(metrics["mean_annual_max_hot_spell_days"], 4.0)

    def test_incomplete_year_is_rejected(self):
        with self.assertRaises(cd.ContractError):
            list(signal.annual_slices(364))

    def test_member_rows_aggregate_to_method_unit(self):
        metric = "tasmax_mean_degC"
        rows = []
        for member, generated in enumerate((22.0, 24.0)):
            rows.append(
                {
                    "model": "M",
                    "experiment": "ssp245",
                    "site": "xian",
                    "method": "seasonal_var_block_ecdf_v2",
                    "member": member,
                    f"historical_{metric}": 20.0,
                    f"source_future_{metric}": 23.0,
                    f"generated_{metric}": generated,
                }
            )
        aggregated = signal.aggregate_units(rows, [metric])
        self.assertEqual(len(aggregated), 1)
        self.assertEqual(aggregated[0]["member_count"], 2)
        self.assertAlmostEqual(aggregated[0][f"generated_mean_{metric}"], 23.0)
        self.assertAlmostEqual(aggregated[0][f"source_signal_{metric}"], 3.0)
        self.assertAlmostEqual(aggregated[0][f"signal_error_{metric}"], 0.0)


if __name__ == "__main__":
    unittest.main()
