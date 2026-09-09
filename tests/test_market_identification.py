"""Preflight inputs and independent mathematical identities; no market optimizer mocks."""
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools/market_intelligence'))
from run_identification_oracle import build_design, validate_input, FACTORS


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


if __name__ == '__main__':
    unittest.main()
