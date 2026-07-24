import copy
import tempfile
import unittest
from pathlib import Path

import numpy as np

import weather_generator as wg


class WeatherGeneratorTest(unittest.TestCase):
    def setUp(self):
        self.definition = wg.cd.load_json(wg.DEFAULT_DEFINITION)

    def test_definition_rejects_cross_gcm_pooling(self):
        invalid = copy.deepcopy(self.definition)
        invalid["conditioning"]["pool_gcms_or_ssps"] = True
        with self.assertRaises(wg.cd.ContractError):
            wg.validate_definition(invalid)

    def test_midrank_transform_and_step_inverse_preserve_zero_mass(self):
        months = np.repeat(np.arange(1, 13), 4)
        base = np.tile(np.asarray([0.0, 0.0, 1.0, 3.0]), 12)
        values = np.column_stack([base + column for column in range(6)])
        values[:, 5] = base
        gaussian, sorted_values, counts = wg.fit_monthly_marginals(values, months)
        restored = wg.inverse_monthly_marginals(gaussian, months, sorted_values, counts)
        np.testing.assert_array_equal(restored, values)
        self.assertEqual(np.count_nonzero(restored[:, 5] == 0.0), 24)

    def test_kmeans_is_deterministic_and_canonically_labelled(self):
        values = np.asarray([[-3.0, -2.0], [-2.9, -2.1], [2.9, 2.0], [3.0, 2.1]])
        first = wg.deterministic_kmeans(values, 2, 50, 1e-12)
        second = wg.deterministic_kmeans(values, 2, 50, 1e-12)
        np.testing.assert_array_equal(first[0], second[0])
        np.testing.assert_allclose(first[1], second[1])
        self.assertLess(first[1][0, 0], first[1][1, 0])

    def test_month_conditioned_probabilities_are_smoothed_and_normalized(self):
        labels = np.tile(np.asarray([0, 1, 0, 1]), 12)
        months = np.repeat(np.arange(1, 13), 4)
        initial, transitions = wg.fit_state_probabilities(labels, months, 2, 1.0)
        np.testing.assert_allclose(initial.sum(axis=1), 1.0)
        np.testing.assert_allclose(transitions.sum(axis=2), 1.0)
        self.assertTrue(np.all(transitions > 0.0))

    def test_var_stabilization_respects_radius(self):
        rng = np.random.default_rng(4)
        gaussian = np.cumsum(rng.normal(size=(500, 2)), axis=0)
        labels = np.arange(500) % 2
        coefficients, residuals, _, effective = wg.fit_state_var(
            gaussian, labels, 2, 0.01, 0.75
        )
        self.assertEqual(coefficients.shape, (4, 2))
        self.assertEqual(residuals.shape, (499, 2))
        self.assertLessEqual(effective, 0.7500001)

    def test_diagonal_var_removes_cross_lag_coefficients(self):
        rng = np.random.default_rng(5)
        gaussian = rng.normal(size=(400, 3))
        labels = np.arange(400) % 2
        coefficients, _, _, _ = wg.fit_state_var(
            gaussian, labels, 2, 0.1, 0.98, diagonal_autoregression=True
        )
        autoregression = coefficients[:3]
        np.testing.assert_array_equal(
            autoregression - np.diag(np.diag(autoregression)), np.zeros((3, 3))
        )

    def test_seed_is_stable_and_member_specific(self):
        first = wg.derive_member_seed(7, "m", "g", "e", "s", 0)
        self.assertEqual(first, wg.derive_member_seed(7, "m", "g", "e", "s", 0))
        self.assertNotEqual(first, wg.derive_member_seed(7, "m", "g", "e", "s", 1))

    def test_end_to_end_sample_is_reproducible_and_physical(self):
        _, months = wg.noleap_dates(2001, 2003)
        index = np.arange(months.size, dtype=float)
        rng = np.random.default_rng(9)
        tasmax = (
            18.0
            + 12.0 * np.sin(2.0 * np.pi * index / 365.0)
            + rng.normal(size=index.size)
        )
        values = np.column_stack(
            (
                tasmax,
                7.0 + rng.random(index.size),
                30.0 + 50.0 * rng.random(index.size),
                1.0 + 3.0 * rng.random(index.size),
                50.0 + 250.0 * rng.random(index.size),
                np.where(
                    rng.random(index.size) < 0.7, 0.0, rng.exponential(5.0, index.size)
                ),
            )
        )
        fitted = wg.fit_generator(values, months, self.definition)
        with tempfile.TemporaryDirectory() as directory:
            first_archive = Path(directory) / "first.npz"
            second_archive = Path(directory) / "second.npz"
            wg.save_fit(first_archive, fitted)
            wg.save_fit(second_archive, fitted)
            self.assertEqual(
                wg.cd.sha256_file(first_archive), wg.cd.sha256_file(second_archive)
            )
            loaded = wg.load_fit(first_archive)
            np.testing.assert_array_equal(loaded.coefficients, fitted.coefficients)
        first, states_first = wg.sample_generator(fitted, months, self.definition, 42)
        second, states_second = wg.sample_generator(fitted, months, self.definition, 42)
        for name in wg.OUTPUT_VARIABLES:
            np.testing.assert_array_equal(first[name], second[name])
        np.testing.assert_array_equal(states_first, states_second)
        diagnostics = wg.output_diagnostics(first)
        self.assertTrue(all(diagnostics["checks"].values()))


if __name__ == "__main__":
    unittest.main()
