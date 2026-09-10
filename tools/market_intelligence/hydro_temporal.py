"""Input-only intraday/UC features and residual GP; hydro allocation theory §12."""
import copy

import numpy as np
from scipy.linalg import cho_factor, cho_solve

from hydro_physics import feature, physics_gain, kernel


def variant_boundary(boundary, shape, condition):
    if shape not in (0, 1) or condition not in (0, 1):
        raise ValueError('Only the two frozen shape/condition levels are supported')
    b = copy.deepcopy(boundary)
    # §12.1: apply the SAME permutation to area, nodal P and Q. Renewable
    # forecasts stay authored, so physical availability after clipping is exact.
    if shape:
        for table, fields in (('areas', ('load_mw',)), ('buses', ('load_mw', 'q_load_mvar'))):
            for row in b[table]:
                for field in fields:
                    row[field][:96] = np.roll(row[field][:96], 24).tolist()
    if condition:
        for g in b['generators']:
            if g['kind'] != 'thermal':
                continue
            g['pmin_mw'] = [.2*p for p in g['pmax_mw']]
            g.update(initial_on=1, initial_power_mw=.2*g['pmax_mw'][0],
                     initial_state_minutes=15, min_up_minutes=240, min_down_minutes=240)
            for field in ('ramp_up_mw_min', 'ramp_down_mw_min'):
                g[field] *= .25
    validate_boundary(b)
    return b


def validate_boundary(b):
    """Check authored coherence, not a claim of optimization feasibility."""
    if b['external_schedules'] or b['trades']:
        raise ValueError('Temporal research scope excludes external schedules and trades')
    for area in b['areas']:
        nodal = np.sum([r['load_mw'][:96] for r in b['buses'] if r['area'] == area['id']], axis=0)
        if np.max(np.abs(nodal-np.asarray(area['load_mw'][:96]))) > 1e-6:
            raise ValueError('Area and node demand disagree')
    for g in b['generators']:
        if g['kind'] != 'thermal':
            continue
        cap, low = np.asarray(g['pmax_mw']), np.asarray(g['pmin_mw'])
        if not np.isfinite(cap).all() or not np.isfinite(low).all() or np.any(cap <= 0) or np.any(low < 0) or np.any(low > cap):
            raise ValueError('Invalid thermal output envelope')
        on, power = g['initial_on'], g['initial_power_mw']
        numeric = [power, g['initial_state_minutes'], g['min_up_minutes'], g['min_down_minutes'],
                   g['ramp_up_mw_min'], g['ramp_down_mw_min']]
        if not np.isfinite(numeric).all() or min(numeric) < 0 or on not in (0, 1):
            raise ValueError('Invalid initial state or running conditions')
        if (not on and power != 0) or (on and not low[0] <= power <= cap[0]):
            raise ValueError('Initial power inconsistent with commitment/envelope')


def net_load(boundary, spec):
    # Only authored base shape + daily factors are admitted. No silent ignoring
    # of daily overrides/outages; hydro overrides live in config, not external.
    validate_boundary(boundary)
    if any(d.get('boundary_overrides') or d.get('generator_outages') or d.get('branch_outages') for d in spec['external_days']):
        raise ValueError('Temporal feature scope excludes external overrides and outages')
    demand = np.sum([a['load_mw'][:96] for a in boundary['areas']], axis=0)
    curves = []
    for d in spec['external_days'][:7]:
        available = np.zeros(96)
        for g in boundary['generators']:
            if g['kind'] in ('wind', 'solar', 'renewable'):
                scale = d.get(g['kind']+'_scale', 1.)
                available += np.minimum(g['pmax_mw'][:96], np.asarray(g['forecast_mw'][:96])*scale)*np.asarray(g['available'][:96])*(1-np.asarray(g['must_off'][:96]))
        curves.append(demand*d['load_scale']-available)
    result = np.asarray(curves)
    if result.shape != (7, 96) or not np.isfinite(result).all():
        raise ValueError('Finite seven-day 15-minute net load required')
    return result


def temporal_feature(boundary, spec):
    n = net_load(boundary, spec)
    hourly = n.reshape(7, 24, 4).mean(axis=2).ravel()/1000
    summaries = np.c_[np.quantile(n, [.1, .5, .9], axis=1).T,
                      np.max(np.abs(np.diff(n, axis=1)), axis=1)].ravel()/1000
    state = []
    for g in sorted(boundary['generators'], key=lambda r:r['id']):
        if g['kind'] != 'thermal':
            continue
        cap, on = g['pmax_mw'][0], g['initial_on']
        state.extend([on, g['initial_power_mw']/cap, g['pmin_mw'][0]/cap,
            on*max(g['min_up_minutes']-g['initial_state_minutes'], 0)/240,
            (1-on)*max(g['min_down_minutes']-g['initial_state_minutes'], 0)/240,
            15*g['ramp_up_mw_min']/cap, 15*g['ramp_down_mw_min']/cap,
            g['min_up_minutes']/240, g['min_down_minutes']/240])
    if not state:
        raise ValueError('Frozen temporal model requires thermal units')
    # §12.2: block RMS scaling prevents feature-count-driven covariance decay.
    return np.r_[feature(boundary, spec), hourly/np.sqrt(len(hourly)),
                 summaries/np.sqrt(len(summaries)), np.asarray(state)/np.sqrt(len(state))]


class TemporalGP:
    """Fixed SE residual regression, RW2006 §2.2 / hydro theory §12.2."""
    def fit(self, x, y):
        self.x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        if self.x.ndim != 2 or self.x.shape[1] < 5 or y.shape != (len(self.x),) or not np.isfinite(self.x).all() or not np.isfinite(y).all():
            raise ValueError('Invalid residual GP training input')
        rhs = y-physics_gain(self.x[:, :5])
        cov = kernel(self.x, self.x)+25*np.eye(len(self.x))
        self.alpha = cho_solve(cho_factor(cov, lower=True), rhs)
        err = np.sum(cov*self.alpha[None, :], axis=1)-rhs
        if np.max(np.abs(err)) > 1e-8*max(1., np.max(np.abs(rhs))):
            raise ValueError('Residual GP solve check failed')
        return self

    def predict(self, x):
        x = np.atleast_2d(np.asarray(x, dtype=float))
        if x.shape[1] != self.x.shape[1] or not np.isfinite(x).all():
            raise ValueError('Nonfinite or incompatible temporal features')
        result = physics_gain(x[:, :5])+np.sum(kernel(x, self.x)*self.alpha[None, :], axis=1)
        result[x[:, 2] == 0] = 0.
        return result

    def save(self, path):
        np.savez(path, x=self.x, alpha=self.alpha, schema=np.array('hydro-temporal-gp-v3'))

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            if str(data['schema']) != 'hydro-temporal-gp-v3':
                raise ValueError('Incompatible temporal GP artifact')
            obj = cls()
            obj.x, obj.alpha = data['x'], data['alpha']
        return obj


def local_thermal_floor(boundary,spec):
    """Input-only disjoint singleton cut lower bounds, hydro theory §12.4."""
    validate_boundary(boundary)
    if boundary.get('dc_links') or boundary.get('dc_hubs'):
        raise ValueError('Singleton cut comparator excludes DC links/hubs')
    excluded={g['bus'] for g in boundary['generators'] if g['kind']=='hydro'}
    excluded.update(s['bus'] for s in boundary.get('storage',[]))
    excluded.update(s['bus'] for s in boundary.get('controllable_loads',[]))
    eligible={g['bus'] for g in boundary['generators'] if g['kind']=='thermal'}-excluded
    local={i:np.zeros(96) for i in eligible};demand={n['id']:np.asarray(n['load_mw'][:96]) for n in boundary['buses']}
    for branch in boundary['branches']:
        lo,hi=np.asarray(branch['min_mw'][:96]),np.asarray(branch['max_mw'][:96])
        available=np.asarray(branch['available'][:96])
        if np.any(lo>0) or np.any(hi<0):
            raise ValueError('Singleton cut requires directional limits containing zero')
        if branch['from_bus'] in local:local[branch['from_bus']]+=-lo*available
        if branch['to_bus'] in local:local[branch['to_bus']]+=hi*available
    result=[]
    for day in spec['external_days'][:7]:
        if day.get('generator_outages') or day.get('branch_outages') or day.get('boundary_overrides'):
            raise ValueError('Singleton cut excludes daily overrides and outages')
        floor=0.
        for bus in sorted(eligible):
            av=np.zeros(96)
            for g in boundary['generators']:
                if g['bus']==bus and g['kind'] in ('wind','solar','renewable'):
                    av+=np.minimum(g['pmax_mw'][:96],np.asarray(g['forecast_mw'][:96])*day.get(g['kind']+'_scale',1.))*np.asarray(g['available'][:96])*(1-np.asarray(g['must_off'][:96]))
            floor+=np.maximum(demand[bus]*day['load_scale']-av-local[bus]*day['line_limit_scale'],0).sum()*.25
        result.append(floor)
    result=np.asarray(result)
    if result.shape!=(7,) or not np.isfinite(result).all():raise ValueError('Invalid local thermal floor')
    return result


def network_feature(boundary,spec):
    x=temporal_feature(boundary,spec)
    # §12.4: shifted hinge is an approximation, not a certified curtailment bound.
    x[:2]+=local_thermal_floor(boundary,spec)[[1,4]]/300
    return x
