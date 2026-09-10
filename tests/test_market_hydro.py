"""Hydro conservation identities and negative audits; frozen design §8."""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools/market_intelligence'))
from hydro_allocation import (synthetic_mapping, validate_mapping, terminal_contract,
                              compile_action, audit_hydrology, sample_identity)


def fixture():
    r = {'id': 7, 'generators': [31], 'source': 'synthetic unit test', 'upstream': -1,
         'lag_slots': 1, 'area_m2': 90000., 'water_m3_mwh': 3600., 'initial_level_m': 200.,
         'inflow_m3_s': [90.]*98, 'min_level_m': [0.]*98, 'max_level_m': [300.]*98,
         'min_mwh': 0., 'max_mwh': 3000., 'release_history_m3_s': [100.], 'initial_release_m3_s': 100.}
    b = {'reservoirs': [r], 'generators': [{'id': 31, 'kind': 'hydro'}],
         'periods': [{'duration_hr': .25}]*98}
    mapping = synthetic_mapping(b)
    external = [{'inflow_scale': 1.} for _ in range(8)]
    quotas, weekly = {'7': [2400.]*7}, {'7': 16800.}
    terminal = terminal_contract(b, mapping, external, quotas)
    days = compile_action(b, mapping, external, quotas, weekly, terminal)
    cfg = {'days': days, 'reference_days': external}
    spec = {'config': cfg, 'quotas': quotas, 'weekly': weekly, 'terminal': terminal}
    state = {'reservoirs': [{k: r[k] for k in ('id', 'initial_level_m', 'initial_release_m3_s', 'release_history_m3_s')}]}
    executed = []
    for d in range(7):
        start = copy.deepcopy(state)
        level = [200.-.1*(96*d+t+1) for t in range(96)]
        state = {'reservoirs': [{'id': 7, 'initial_level_m': level[-1],
                                 'initial_release_m3_s': 100., 'release_history_m3_s': [100.]*96}]}
        executed.append({'valid': True, 'day': d, 'state_start': start, 'state_end': copy.deepcopy(state),
                         'resources': {'generators': [{'id': 31, 'power_mw': [100.]*96}],
                                       'reservoirs': [{'id': 7, 'level_m': level, 'spill_m3_s': [0.]*96,
                                                       'release_m3_s': [100.]*96}]}})
    job = {'base': b, 'scenarios': [{'status': 'completed', 'completed_days': 7, 'config': cfg, 'days': executed}]}
    return b, mapping, spec, job


class HydroAuditTests(unittest.TestCase):
    def test_si_identity_and_executed_energy(self):
        b, mapping, spec, job = fixture()
        audit = audit_hydrology(job, spec, mapping)
        self.assertTrue(audit['passed'])
        self.assertAlmostEqual(spec['terminal']['7']['level_m'], 132.8)
        self.assertEqual(audit['station_weekly_mwh']['7'], 16800.)

    def test_quota_drift_and_hidden_release_detected(self):
        _, mapping, spec, job = fixture()
        job['scenarios'][0]['days'][2]['resources']['generators'][0]['power_mw'][0] += 1
        audit = audit_hydrology(job, spec, mapping)
        self.assertFalse(audit['passed'])
        self.assertEqual(audit['max_errors']['daily_energy_mwh'], .25)
        self.assertEqual(audit['max_errors']['release_definition_m3_s'], 1.)

    def test_terminal_in_transit_difference_detected(self):
        _, mapping, spec, job = fixture()
        day = job['scenarios'][0]['days'][-1]
        day['resources']['reservoirs'][0]['release_m3_s'][-1] += 1
        day['state_end']['reservoirs'][0]['release_history_m3_s'][-1] += 1
        day['state_end']['reservoirs'][0]['initial_release_m3_s'] += 1
        self.assertFalse(audit_hydrology(job, spec, mapping)['passed'])

    def test_missing_and_nonfinite_not_zero(self):
        for value in (None, float('nan')):
            _, mapping, spec, job = fixture()
            job['scenarios'][0]['days'][0]['resources']['reservoirs'][0]['spill_m3_s'][0] = value
            with self.assertRaises(ValueError):
                audit_hydrology(job, spec, mapping)

    def test_mapping_duplicate_unknown_and_nonhydro(self):
        for kind in ('duplicate', 'unknown', 'thermal'):
            b, mapping, _, _ = fixture()
            if kind == 'duplicate':
                mapping[0]['generator_ids'].append(31)
            elif kind == 'unknown':
                mapping[0]['generator_ids'] = [1]
            else:
                b['generators'][0]['kind'] = kind
            with self.assertRaises(ValueError):
                validate_mapping(b, mapping)

    def test_zero_sum_transfer_and_no_day8_quota(self):
        b, mapping, spec, _ = fixture()
        quotas = copy.deepcopy(spec['quotas'])
        quotas['7'][1] -= 150
        quotas['7'][4] += 150
        days = compile_action(b, mapping, spec['config']['reference_days'], quotas, spec['weekly'], spec['terminal'])
        self.assertNotIn('boundary_overrides', days[7])
        self.assertEqual(days[1]['boundary_overrides'][0]['value'], 2250)
        quotas['7'][4] += 1
        with self.assertRaises(ValueError):
            compile_action(b, mapping, spec['config']['reference_days'], quotas, spec['weekly'], spec['terminal'])

    def test_reference_identity_and_reject_wrong_reference(self):
        b, mapping, spec, job = fixture()
        old = sample_identity(b, spec['config'], mapping, spec['quotas'], spec['terminal'], 'binary')
        altered = copy.deepcopy(spec['config'])
        altered['reference_days'][7]['inflow_scale'] = .5
        new = sample_identity(b, altered, mapping, spec['quotas'], spec['terminal'], 'binary')
        self.assertNotEqual(old, new)
        job['scenarios'][0]['config'] = altered
        with self.assertRaises(ValueError):
            audit_hydrology(job, spec, mapping)

    def test_lag_mass_budget_uses_upstream_history(self):
        b, mapping, spec, _ = fixture()
        r = copy.deepcopy(b['reservoirs'][0])
        r.update(id=19, generators=[45], upstream=7, lag_slots=1)
        r['inflow_m3_s'] = [0.]*98
        b['reservoirs'].append(r)
        b['generators'].append({'id': 45, 'kind': 'hydro'})
        b['reservoirs'][0]['release_history_m3_s'] = [50.]
        mapping = synthetic_mapping(b)
        quotas = {'7': [2400.]*7, '19': [2400.]*7}
        target = terminal_contract(b, mapping, spec['config']['reference_days'], quotas)
        self.assertAlmostEqual(target['19']['level_m'], 199.5)

class HydroLearningTests(unittest.TestCase):
    def test_grouped_learning_never_trains_on_held_out_family(self):
        from analyze_hydro import learning_curves
        specs, pairs, manifests = [], [], []
        for family in range(6):
            for delta in (0., -300., -150., 150., 300.):
                index = len(specs)
                specs.append({'delta_mwh': delta, 'external_days': [
                    {'load_scale': .5+family*.05, 'wind_scale': 1., 'solar_scale': 1.,
                     'inflow_scale': 1., 'line_limit_scale': 1.} for _ in range(8)]})
                pairs.append({'family': family, 'index': index, 'delta_mwh': delta, 'gain_mwh': delta*.2})
                manifests.append({'family': family, 'wall_sec': 100.})
        curves = learning_curves(pairs, specs, manifests)
        self.assertEqual(len(curves['predictions']), 216)
        for row in curves['predictions']:
            self.assertNotIn(row['family'], row['train_families'])
            self.assertIn(row['family'], row['validation_families'])
            self.assertNotEqual(specs[row['index']]['delta_mwh'], 0)
        for row in curves['aggregate']:
            self.assertEqual(row['n_pairs'], 24)
            self.assertEqual(row['worker_sec'], row['n_families']*500)
        zero = next(r for r in curves['aggregate'] if r['model'] == 'zero_gain')
        self.assertEqual(zero['mae_mwh'], 45.)

    def test_missing_action_does_not_silently_replace_family(self):
        from analyze_hydro import learning_curves
        pairs = [{'family': family} for family in range(6) for _ in range(5)]
        result = learning_curves(pairs[:-1], [], [])
        self.assertEqual(result['status'], 'insufficient_complete_families')
        self.assertEqual(result['complete_families'], [0, 1, 2, 3, 4])


if __name__ == '__main__':
    unittest.main()
