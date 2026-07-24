import tempfile
import unittest
from pathlib import Path

import numpy as np

import weather_generator as wg
import weather_generator_knn as knn


class WeatherGeneratorKnnTest(unittest.TestCase):
    def setUp(self):
        self.definition = knn.cd.load_json(knn.DEFAULT_DEFINITION)

    def synthetic(self):
        _, months = wg.noleap_dates(2001, 2003)
        rng = np.random.default_rng(15)
        tasmax = rng.normal(20.0, 5.0, months.size)
        values = np.column_stack(
            (
                tasmax,
                4.0 + rng.random(months.size),
                30.0 + 60.0 * rng.random(months.size),
                4.0 * rng.random(months.size),
                300.0 * rng.random(months.size),
                np.where(
                    rng.random(months.size) < 0.7,
                    0.0,
                    rng.exponential(3.0, months.size),
                ),
            )
        )
        return values, months

    def test_fit_excludes_masked_transition(self):
        values, months = self.synthetic()
        valid = np.ones(values.shape[0] - 1, dtype=bool)
        valid[300] = False
        fitted = knn.fit_generator(values, months, valid)
        self.assertNotIn(301, fitted.transition_source_indices)
        self.assertEqual(fitted.predecessors.shape[0], values.shape[0] - 2)

    def test_sampling_is_reproducible_physical_and_season_conditioned(self):
        values, months = self.synthetic()
        fitted = knn.fit_generator(values, months)
        trees = knn.build_season_trees(fitted)
        first, selected_first = knn.sample_generator(
            fitted, months, self.definition, 9, trees
        )
        second, selected_second = knn.sample_generator(
            fitted, months, self.definition, 9, trees
        )
        for name in wg.OUTPUT_VARIABLES:
            np.testing.assert_array_equal(first[name], second[name])
        np.testing.assert_array_equal(selected_first, selected_second)
        self.assertTrue(all(wg.output_diagnostics(first)["checks"].values()))
        for source_month, target_month in zip(
            fitted.successor_months[selected_first[1:]], months[1:]
        ):
            self.assertEqual(wg.season(int(source_month)), wg.season(int(target_month)))

    def test_parameter_archive_is_deterministic(self):
        values, months = self.synthetic()
        fitted = knn.fit_generator(values, months)
        with tempfile.TemporaryDirectory() as directory:
            first = Path(directory) / "first.npz"
            second = Path(directory) / "second.npz"
            knn.save_fit(first, fitted)
            knn.save_fit(second, fitted)
            self.assertEqual(knn.cd.sha256_file(first), knn.cd.sha256_file(second))
            loaded = knn.load_fit(first)
            np.testing.assert_array_equal(loaded.successors, fitted.successors)

    def test_rank_probabilities_decrease_and_sum_to_one(self):
        probabilities = knn.rank_probabilities(5, 1.0)
        self.assertAlmostEqual(float(np.sum(probabilities)), 1.0)
        self.assertTrue(np.all(np.diff(probabilities) < 0.0))


if __name__ == "__main__":
    unittest.main()
