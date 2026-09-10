"""Frozen strict-label inference contract; hydro allocation theory §14.3."""
import json
from pathlib import Path
import time

import numpy as np

from hydro_allocation import terminal_contract, compile_action
from hydro_physics import feature, daily_surplus
from hydro_temporal import temporal_feature, network_feature, TemporalGP
from hydro_complete_context import complete_feature
from run_hydro_strict import verify_model_freeze
from run_price_oracle import digest

FEATURES = {'daily_gp': feature, 'temporal_gp': temporal_feature,
            'network_gp': network_feature, 'complete_gp': complete_feature}


def validate_candidate(boundary, spec, design):
    """Reject unsupported authored changes; finite training boxes do not certify safety."""
    if design['protocol'] != 'hydro-strict-v4':
        raise ValueError('Unsupported strict research design')
    key = spec['boundary_key']
    if key not in design['boundaries'] or digest(boundary) != digest(design['boundaries'][key]):
        raise ValueError('Boundary outside the four frozen shape/condition variants')
    if spec['boundary_sha256'] != digest(boundary):
        raise ValueError('Candidate base hash mismatch')
    delta = spec['delta_mwh']
    if not np.isfinite(delta) or not 0 <= delta <= 150:
        raise ValueError('Only station1 day2 to day5 moves in [0,150] MWh supported')
    external = spec['external_days']
    if len(external) != 8:
        raise ValueError('All eight authored days are required')
    fields = {'load_scale','wind_scale','solar_scale','inflow_scale','line_limit_scale','generator_bid_scale',
              'load_bid_scale','bid_scale','first_slot','last_slot','generator_outages','branch_outages'}
    for d, day in enumerate(external):
        if set(day) != fields or day['first_slot'] != 0 or day['last_slot'] != 95 or day['generator_outages'] or day['branch_outages']:
            raise ValueError('Unsupported exogenous override or outage')
        for field, lower, upper in [('wind_scale',1,1.8),('solar_scale',1,1.8),('inflow_scale',.6,1),('line_limit_scale',.8,1)]:
            if not np.isfinite(day[field]) or not lower <= day[field] <= upper:
                raise ValueError('Exogenous factor outside v4 support: ' + field)
        if not np.isfinite(day['load_scale']) or day['load_scale'] <= 0:
            raise ValueError('Finite positive demand required')
        if d not in (1, 4) and not .6 <= day['load_scale'] <= .85:
            raise ValueError('Unaffected-day demand outside support')
        if any(day[k] != 1 for k in ('generator_bid_scale','load_bid_scale','bid_scale')):
            raise ValueError('Bid variation not supported')
    baseline = {str(s['id']): [3000.] * 7 for s in design['mapping']}
    weekly = {sid: 21000. for sid in baseline}
    if spec['weekly'] != weekly:
        raise ValueError('Weekly station energy changed')
    quotas = {sid: q.copy() for sid, q in baseline.items()}
    quotas['1'][1] -= delta
    quotas['1'][4] += delta
    if spec['quotas'] != quotas:
        raise ValueError('Hydro quotas do not execute the declared move')
    if np.max(np.abs(daily_surplus(boundary, spec)[[1,4]])) > 360+1e-6:
        raise ValueError('Affected-day surplus outside [-360,360] MWh support')
    terminal = terminal_contract(boundary, design['mapping'], external, baseline)
    if spec['terminal'] != terminal:
        raise ValueError('Terminal water comparison changed')
    contract = {k:v for k,v in design['specs'][0]['config'].items() if k not in ('days','reference_days')}
    config = spec['config']
    if {k:v for k,v in config.items() if k not in ('days','reference_days')} != contract:
        raise ValueError('Operation or precision contract changed')
    if config['reference_days'] != external or config['days'] != compile_action(boundary, design['mapping'], external, quotas, weekly, terminal):
        raise ValueError('Compiled daily action or reference trajectory differs')


class StrictPredictor:
    def __init__(self, root, model='network_gp', n_families=4):
        self.root = Path(root)
        self.design = json.loads((self.root/'design.json').read_text())
        self.frozen = verify_model_freeze(self.root, self.design)
        if model not in FEATURES or n_families not in (2, 4):
            raise ValueError('Unsupported frozen model or training scale')
        self.name = model
        self.model = TemporalGP.load(self.root/f'analysis/{model}-{n_families}.npz')

    def predict(self, boundary, spec):
        start = time.perf_counter_ns()
        validate_candidate(boundary, spec, self.design)
        x = FEATURES[self.name](boundary, spec)
        estimate = float(self.model.predict(x)[0])
        outside = bool(np.any((x < self.model.x.min(axis=0)-1e-10) | (x > self.model.x.max(axis=0)+1e-10)))
        return dict(model=self.name, predicted_gain_mwh=estimate, outside_training_box=outside,
                    requires_oracle_confirmation=True, production_admitted=False,
                    scope='fixed 12-station synthetic base; station1 day2/day5 move; no AC/N-1 or accuracy certificate',
                    full_inference_ms=(time.perf_counter_ns()-start)/1e6)
