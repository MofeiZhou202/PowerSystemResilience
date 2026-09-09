"""Analytic, mutation and identity checks for the weekly label contract."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/market_intelligence'))
from build_market_label_dataset import extract, price_summary, FACTORS


def analytic_week():
    prices = [10.]*94+[100., 200.]
    stage = {'max_residual': 0., 'mip_gap': 0., 'requested_mip_gap': .01,
             'objective': 123., 'optimality_proven': True, 'price_consistency': {'passed': True}}
    loss = [0.]*94+[2., 2.]
    day = {'day': 0, 'valid': True, 'diagnostic_prices_valid': True, 'runtime_sec': 1.,
           'state_start': {'storage': [1.]}, 'state_end': {'storage': [1.]},
           'stages': {k: copy.deepcopy(stage) for k in ('scuc', 'sced', 'lmp')},
           'nodes': [{'id': 3, 'deficit_mw': loss, 'lmp_per_mwh': prices}],
           'lines': [{'id': 6, 'power_mw': [0.]*96, 'min_mw': [0.]*96, 'max_mw': [0.]*96,
                      'available': [1]*96, 'overload_mw': [0.]*96}],
           'resources': {'generators': [{'id': 9, 'kind': 'wind', 'power_mw': [8.]*96,
                                          'renewable_available_mw': [10.]*96, 'curtailment_mw': [2.]*96}]},
           'deficit_mwh': 1., 'overload_mwh': 0.}
    days = [copy.deepcopy(day) for _ in range(7)]
    for i, d in enumerate(days):
        d['day'] = i
    config = {'days': [{f: 1. for f in FACTORS} for _ in range(8)],
              'start_date': '2026-09-10', 'penalty_per_mwh': 100000}
    return {'base': {'periods': [{'duration_hr': .25}]*98,
                      'buses': [{'id': 3}], 'branches': [{'id': 6}], 'generators': [{'id': 9}]},
            'config': {'seed': 1}, 'scenarios': [{'id': 0, 'model_scope': 'analytic-diagnostic',
              'days': days, 'config': config, 'completed_days': 7, 'status': 'completed'}]}


class LabelTests(unittest.TestCase):
    def label(self, job):
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp) / 'job.json'
            file.write_text(json.dumps(job))
            return extract(file, 100.)[0]

    def test_analytic_energy_and_price(self):
        r = self.label(analytic_week())
        self.assertTrue(r['train_eligible'])
        self.assertEqual(r['price']['count'], 672)
        self.assertEqual(r['price']['p95'], 10.)
        self.assertEqual(r['price']['p99'], 200.)
        self.assertEqual(r['price']['cvar99'], 200.)
        self.assertAlmostEqual(r['price']['spike_fraction'], 1/96)
        self.assertEqual(r['price']['spike_duration_hr'], 1.75)
        self.assertEqual(r['metrics']['load_loss_mwh'], 7.)
        self.assertEqual(r['metrics']['renewable_available_mwh'], 1680.)
        self.assertEqual(r['metrics']['renewable_used_mwh'], 1344.)
        self.assertEqual(r['metrics']['renewable_utilization'], .8)
        self.assertNotIn('weekly_cost', r['metrics'])

    def test_cvar_fractional_atom(self):
        # Five observations: the worst 1% consists of fractional mass at 5, not a rounded tail.
        self.assertEqual(price_summary([1, 2, 3, 4, 5])['cvar99'], 5)

    def test_failure_and_missing_price_are_not_zero(self):
        mutations = [lambda d: d.update(valid=False),
                     lambda d: d['nodes'][0].update(lmp_per_mwh=None),
                     lambda d: d['nodes'][0].update(lmp_per_mwh=[float('nan')]*96),
                     lambda d: d['stages']['lmp']['price_consistency'].update(passed=False),
                     lambda d: d['stages']['lmp'].update(optimality_proven=False),
                     lambda d: d['stages']['scuc'].update(mip_gap=.1),
                     lambda d: d['state_start'].update(storage=[2.]),
                     lambda d: d['nodes'].append(copy.deepcopy(d['nodes'][0]))]
        for mutate in mutations:
            j = analytic_week()
            mutate(j['scenarios'][0]['days'][1])
            r = self.label(j)
            self.assertFalse(r['train_eligible'])
            self.assertIsNone(r['price'])
            self.assertEqual(r['metrics'], {})

    def test_partial_week_rejected(self):
        j = analytic_week()
        j['scenarios'][0]['days'].pop()
        self.assertFalse(self.label(j)['train_eligible'])

    def test_physical_identity_not_filename_or_execution(self):
        j = analytic_week()
        first = self.label(j)['sample_id']
        self.assertEqual(first, self.label(j)['sample_id'])
        j['base']['execution'] = {'threads': 2}
        # Canonical base includes an execution object; compare like schemas.
        second = self.label(j)['sample_id']
        j['base']['execution']['threads'] = 8
        self.assertEqual(second, self.label(j)['sample_id'])
        j['scenarios'][0]['config']['days'][7]['load_scale'] = 1.1
        self.assertNotEqual(second, self.label(j)['sample_id'])

    def test_directional_line_recompute(self):
        j = analytic_week()
        for d in j['scenarios'][0]['days']:
            d['lines'][0].update(power_mw=[-5.]*96, min_mw=[-4.]*96,
                                 max_mw=[10.]*96, overload_mw=[1.]*96)
            d['overload_mwh'] = 24.
        r = self.label(j)
        self.assertTrue(r['train_eligible'])
        self.assertEqual(r['metrics']['line_overload_integral_mwh'], 168.)
        j['scenarios'][0]['days'][0]['lines'][0]['overload_mw'][0] = 0.
        self.assertFalse(self.label(j)['train_eligible'])

    def test_old_export_without_base_rejected(self):
        j = analytic_week()
        del j['base']
        with self.assertRaises(ValueError):
            self.label(j)


if __name__ == '__main__':
    unittest.main()
