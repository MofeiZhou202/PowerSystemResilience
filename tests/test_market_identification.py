"""Preflight inputs and independent mathematical identities; no market optimizer mocks."""
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools/market_intelligence'))
from run_identification_oracle import build_design, validate_input, FACTORS
from analyze_identification import representations, grouped_folds, ridge_predict, pair_floor


class IdentificationDesignTests(unittest.TestCase):
    def setUp(self):
        self.design = build_design({'operation': {}, 'marginals': [{'factor': f} for f in FACTORS]})

    def test_crossed_inputs_and_repeats(self):
        specs = self.design['specs']
        self.assertEqual(len(specs), 50)
        for i in range(12):
            base, action, reverse, reverse_action = specs[4*i:4*i+4]
            for first, second in ((base, action), (reverse, reverse_action)):
                for a, b in zip(first['expected_days'], second['expected_days']):
                    for f in FACTORS[:6]:self.assertEqual(a[f], b[f])
                    self.assertAlmostEqual(b['line_limit_scale']/a['line_limit_scale'], .9)
            self.assertEqual(base['expected_days'][:7][::-1], reverse['expected_days'][:7])
            self.assertEqual(base['expected_days'][7], reverse['expected_days'][7])
        for r in specs[48:]:
            original = specs[r['metadata']['repeat_of']]
            self.assertEqual(r['expected_days'], original['expected_days'])
            self.assertEqual(r['config'], original['config'])

    def test_reversal_preserves_statistics_but_changes_order(self):
        specs = self.design['specs']
        for i in range(12):
            a = np.array([[d[f] for f in FACTORS] for d in specs[4*i]['expected_days'][:7]])
            b = np.array([[d[f] for f in FACTORS] for d in specs[4*i+2]['expected_days'][:7]])
            for fn in (np.mean, np.std, np.min, np.max):
                np.testing.assert_allclose(fn(a, axis=0), fn(b, axis=0), atol=1e-12, rtol=0)
            np.testing.assert_allclose(abs(np.diff(a,axis=0)).max(axis=0),
                                       abs(np.diff(b,axis=0)).max(axis=0), atol=1e-12, rtol=0)
            self.assertFalse(np.array_equal(a, b))

    def test_generated_input_mutations_rejected(self):
        spec = self.design['specs'][0]
        days = [{**d, 'bid_scale': 1., 'generator_outages': [], 'branch_outages': [],
                 'first_slot': 0, 'last_slot': 95} for d in spec['expected_days']]
        job = {'scenarios': [{'config': {'days': days, 'reference_days': copy.deepcopy(days)}}]}
        validate_input(job, spec)
        for mutate in (lambda c:c['days'][0].update(load_scale=9),
                       lambda c:c['reference_days'][0].update(load_scale=9),
                       lambda c:c['days'][0].update(bid_scale=2),
                       lambda c:c['days'][0].update(branch_outages=[1]),
                       lambda c:c['days'][0].update(last_slot=90)):
            bad = copy.deepcopy(job)
            mutate(bad['scenarios'][0]['config'])
            with self.assertRaises(ValueError):validate_input(bad, spec)

    def test_coordinates_detect_order_without_lookahead_leak(self):
        specs = self.design['specs']
        rows = [{'features': {f'd{d}.{f}': day[f] for d, day in enumerate(spec['expected_days'])
                              for f in FACTORS}} for spec in (specs[0], specs[2])]
        x = representations(rows)
        self.assertEqual(x['low5'].shape, (2, 5))
        self.assertEqual(x['temporal17'].shape, (2, 17))
        self.assertEqual(x['full56'].shape, (2, 56))
        np.testing.assert_array_equal(x['low5'][0], x['low5'][1])
        np.testing.assert_allclose(x['orderless42'][0], x['orderless42'][1], atol=1e-12, rtol=0)
        self.assertGreater(np.max(abs(x['temporal17'][0]-x['temporal17'][1])), .01)
        rows[0]['price'] = {'mean': 999999}
        np.testing.assert_array_equal(representations(rows)['temporal17'], x['temporal17'])

    def test_family_nested_splits(self):
        covered = []
        for order, valid in grouped_folds():
            self.assertEqual(len(order), 8)
            self.assertEqual(len(valid), 4)
            self.assertFalse(set(order) & set(valid))
            self.assertTrue(set(order[:4]) <= set(order[:6]) <= set(order[:8]))
            covered.extend(valid)
        self.assertEqual(sorted(covered), list(range(12)))

    def test_compression_lower_bound(self):
        a, b = np.array([1., 5., -7.]), np.array([5., 5., 3.])
        mae, mse = pair_floor(a, b)
        np.testing.assert_array_equal(mae, [2., 0., 5.])
        np.testing.assert_array_equal(mse, [4., 0., 25.])
        for estimate in (a, b, (a+b)/2, np.zeros(3)):
            self.assertTrue(np.all((abs(a-estimate)+abs(b-estimate))/2 >= mae))
            self.assertTrue(np.all(((a-estimate)**2+(b-estimate)**2)/2 >= mse))

    def test_ridge_matches_closed_form_shrinkage(self):
        x = np.array([[-1.], [0.], [1.]])
        pred = ridge_predict(x, x[:, 0], np.array([[-2.], [2.]]))
        np.testing.assert_allclose(pred, [-6/13, 6/13], atol=1e-12, rtol=0)
        a = ridge_predict(x, x[:, 0]+3, np.array([[2.]]))
        b = ridge_predict(x, x[:, 0]+3, np.array([[2.], [10000.]]))
        np.testing.assert_allclose(a, b[:1], atol=1e-12, rtol=0)


if __name__ == '__main__':
    unittest.main()
