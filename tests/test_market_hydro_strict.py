"""Prospective splits, gain targeting and stability gates; hydro theory §14."""
import copy
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools/market_intelligence'))
from test_market_hydro_temporal import fixture
from run_hydro_temporal import build_design as old_design
from run_hydro_strict import build_design, precision_design, stability_report, frozen_save, verify_model_freeze
from hydro_physics import feature, physics_gain
from hydro_complete_context import complete_feature, recover_factors, FACTORS


class StrictProtocolTests(unittest.TestCase):
    def test_representatives_span_all_roots_shapes_conditions_without_old_test(self):
        d = precision_design(old_design(fixture(), {'solver_options': {}}, 'test'))
        active = [s for s in d['specs'] if s['delta_mwh']]
        self.assertEqual({s['root_family'] for s in active}, {0, 1, 2, 3})
        self.assertEqual({(s['shape'], s['condition']) for s in active}, {(0, 0), (0, 1), (1, 0), (1, 1)})
        self.assertTrue(all(s['split'] == 'train' for s in d['specs']))

    def test_high_benefits_preserve_water_energy_and_keep_siblings_together(self):
        d = build_design(fixture(), {'solver_options': {}}, 'test')
        self.assertEqual(len(d['specs']), 48)
        for root in range(6):
            group = [s for s in d['specs'] if s['root_family'] == root]
            self.assertEqual(len(group), 8)
            self.assertEqual(len({s['split'] for s in group}), 1)
            for s in group:
                self.assertEqual(s['terminal'], group[0]['terminal'])
                self.assertEqual(sum(s['quotas']['1']), 21000)
                b = d['boundaries'][s['boundary_key']]
                if root in (0, 1, 4) and s['delta_mwh']:
                    self.assertAlmostEqual(physics_gain(feature(b, s))[0], 150, places=8)
                self.assertTrue(all(.8 <= day['line_limit_scale'] <= 1 for day in s['external_days']))
                x = complete_feature(b, s)
                np.testing.assert_allclose(recover_factors(x), [[day[k] for k in FACTORS] for day in s['external_days']])

    def test_new_exogenous_roots_differ_from_old_and_each_other(self):
        b = fixture()
        old = old_design(b, {'solver_options': {}}, 'test')
        new = build_design(b, {'solver_options': {}}, 'test')
        trajectories = {str(s['external_days']) for s in new['specs']}
        self.assertEqual(len(trajectories), 6)
        self.assertFalse(trajectories & {str(s['external_days']) for s in old['specs']})

    def test_pair_stability_rejects_cancelling_label_drift_and_unsafe_baseline(self):
        base = dict(index=0, family=0, root_family=0, split='train', delta_mwh=0,
                    gain_mwh=0, curtailment_mwh=200, diagnostic_pass=False)
        action = dict(base, index=1, delta_mwh=150, gain_mwh=150, curtailment_mwh=50, diagnostic_pass=True)
        coarse = {'rows': [base, action]}
        fine = copy.deepcopy(coarse)
        for r in fine['rows']:
            r['curtailment_mwh'] += 10
        result = stability_report(coarse, fine)
        self.assertFalse(result['all_stable'])
        self.assertEqual(result['maximum_gain_change_mwh'], 0)
        self.assertFalse(result['rows'][1]['pair_diagnostic_pass'])
        self.assertTrue(stability_report(coarse, coarse)['all_stable'])

    def test_freeze_rejects_mutation_and_test_before_training(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'design.json'
            frozen_save(p, {'a': 1})
            frozen_save(p, {'a': 1})
            with self.assertRaises(ValueError):
                frozen_save(p, {'a': 2})
            with self.assertRaises(ValueError):
                verify_model_freeze(Path(tmp), {'a': 1})

    def test_candidate_rejects_hidden_quota_gap_and_domain_changes(self):
        from hydro_strict_model import validate_candidate
        d = build_design(fixture(), {'solver_options': {}}, 'test')
        original = d['specs'][1]
        b = d['boundaries'][original['boundary_key']]
        validate_candidate(b, original, d)
        for mutation in ('quota', 'gap', 'line', 'terminal'):
            s = copy.deepcopy(original)
            if mutation == 'quota': s['quotas']['1'][0] += 1
            elif mutation == 'gap': s['config']['solver_options']['mip_gap'] = .01
            elif mutation == 'line': s['external_days'][7]['line_limit_scale'] = .5
            else: s['terminal']['1']['level_m'] += 1
            with self.assertRaises(ValueError): validate_candidate(b, s, d)

    def test_selection_does_not_invent_safe_fallback_or_choose_with_hindsight(self):
        from analyze_hydro_strict import selection
        rows = [dict(family=0,delta_mwh=0,gain_mwh=0,pair_diagnostic_pass=False),
                dict(family=0,delta_mwh=150,gain_mwh=150,pair_diagnostic_pass=False),
                dict(family=1,delta_mwh=0,gain_mwh=0,pair_diagnostic_pass=True),
                dict(family=1,delta_mwh=150,gain_mwh=150,pair_diagnostic_pass=True)]
        choices = selection(rows, [0, 200, 0, -1])
        self.assertIsNone(choices[0]['regret_mwh'])
        self.assertEqual(choices[1]['selected_delta_mwh'], 0)
        self.assertEqual(choices[1]['regret_mwh'], 150)

    def test_training_refuses_a_prospective_test_directory(self):
        from analyze_hydro_strict import train
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'gap-1e-6/week-032').mkdir(parents=True)
            with self.assertRaisesRegex(ValueError, 'Held-out Oracle started'):
                train(root, {'specs': []})

    def test_switching_counts_include_initial_boundary_and_exclude_other_assets(self):
        from audit_hydro_strict_switching import trace
        job = {'base': {'generators': [{'id': 7, 'kind': 'thermal', 'initial_on': 0},
                                      {'id': 9, 'kind': 'wind', 'initial_on': 1}]},
               'scenarios': [{'days': [{'resources': {'generators': [
                   {'id': 7, 'online': [1] * 48 + [0] * 48},
                   {'id': 9, 'online': [1] * 96}]}} for _ in range(7)]}]}
        ids, values, counts = trace(job)
        self.assertEqual(ids, [7])
        self.assertEqual(values.shape, (1, 672))
        self.assertEqual(counts, {'startups': 7, 'shutdowns': 7, 'on_unit_slots': 336})
        job['base']['generators'][0]['initial_on'] = 1
        self.assertEqual(trace(job)[2]['startups'], 6)

    def test_switching_rejects_fractional_commitment(self):
        from audit_hydro_strict_switching import trace
        job = {'base': {'generators': [{'id': 7, 'kind': 'thermal', 'initial_on': 0}]},
               'scenarios': [{'days': [{'resources': {'generators': [
                   {'id': 7, 'online': [.25] * 96}]}} for _ in range(7)]}]}
        with self.assertRaisesRegex(ValueError, 'Nonbinary'):
            trace(job)

    def test_diagnostic_integral_handles_directional_and_disabled_limits(self):
        from audit_hydro_strict_switching import diagnostic_energy
        day = {'nodes': [{'deficit_mw': [.5] * 96}], 'lines': [
            {'power_mw': [-5] * 96, 'min_mw': [-4] * 96, 'max_mw': [10] * 96, 'available': [1] * 96},
            {'power_mw': [2] * 96, 'min_mw': [-10] * 96, 'max_mw': [10] * 96, 'available': [0] * 96}]}
        result = diagnostic_energy({'scenarios': [{'days': [day] * 7}]})
        self.assertEqual(result, {'deficit_mwh': 84, 'overload_mwh': 504})
        day['lines'][0]['power_mw'][0] = float('nan')
        with self.assertRaisesRegex(ValueError, 'Invalid line'):
            diagnostic_energy({'scenarios': [{'days': [day]}]})

    def test_challenge_forces_nonconstant_hinge_responses_without_changing_parent(self):
        from run_hydro_strict_challenge import build_challenge
        parent=build_design(fixture(), {'solver_options': {}}, 'test')
        original=copy.deepcopy(parent)
        d=build_challenge(parent)
        self.assertEqual(parent,original)
        self.assertEqual(len(d['specs']),24)
        values=[]
        for s in d['specs']:
            self.assertEqual(s['split'],'test')
            self.assertIn(s['root_family'],(6,7,8))
            if s['delta_mwh']:
                b=d['boundaries'][s['boundary_key']]
                values.append(round(float(physics_gain(feature(b,s))[0]),6))
        self.assertEqual(sorted(set(values)),[-150.,90.,110.])
        self.assertEqual(len(values),12)


if __name__ == '__main__':
    unittest.main()
