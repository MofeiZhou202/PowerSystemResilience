import unittest

import numpy as np

import climate_data as cd
import future_weather_batch as future
import weather_to_grid as mapping


class WeatherToGridTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.definition = cd.load_json(future.DEFAULT_DEFINITION)

    def test_event_selection_is_deterministic_and_uses_complete_window(self):
        stress = np.asarray([0.1, 0.2, 0.3, 0.9, 0.8, 0.7, 0.1])
        start, rolling = mapping.event_window_start(stress, 3)
        self.assertEqual(start, 3)
        self.assertAlmostEqual(rolling[start], 0.8)

    def test_fixed_mapping_produces_physical_72_hour_profiles(self):
        days = 8
        arrays = {
            "tasmax": np.linspace(30.0, 40.0, days),
            "tasmin": np.linspace(18.0, 24.0, days),
            "hurs": np.linspace(50.0, 90.0, days),
            "sfcWind": np.linspace(2.0, 15.0, days),
            "rsds": np.full(days, 220.0),
            "pr": np.linspace(0.0, 60.0, days),
        }
        profiles = mapping.reconstruct_event_profiles(
            arrays, 3, self.definition
        )
        for name in (
            "load_multiplier",
            "pv_capacity_factor",
            "wind_capacity_factor",
        ):
            self.assertEqual(profiles[name].shape, (72,))
            self.assertTrue(np.all(np.isfinite(profiles[name])))
        self.assertTrue(np.all(profiles["load_multiplier"] >= 0.6))
        self.assertTrue(np.all(profiles["load_multiplier"] <= 1.8))
        self.assertTrue(np.all(profiles["pv_capacity_factor"] >= 0.0))
        self.assertTrue(np.all(profiles["pv_capacity_factor"] <= 1.0))
        self.assertTrue(np.all(profiles["wind_capacity_factor"] >= 0.0))
        self.assertTrue(np.all(profiles["wind_capacity_factor"] <= 1.0))

    def test_compound_stress_is_bounded(self):
        values = np.asarray([-100.0, 0.0, 100.0])
        stress = mapping.compound_stress(
            values, values, values, values, self.definition
        )
        self.assertTrue(np.all(stress >= 0.0))
        self.assertTrue(np.all(stress <= 1.0))


if __name__ == "__main__":
    unittest.main()
