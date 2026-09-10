"""Energy-hinge surrogate and fixed-kernel GP; hydro_renewable_allocation.md §10."""
import numpy as np
from scipy.linalg import cho_factor, cho_solve, solve_triangular


def renewable_energy(boundary, day):
    """Physical availability is capped per executed 15-minute slot, design §10.1."""
    total = 0.
    for g in boundary['generators']:
        if g['kind'] not in ('wind', 'solar', 'renewable'):
            continue
        scale = day.get(g['kind']+'_scale', 1.)
        if not np.isfinite(scale) or scale < 0:
            raise ValueError('Finite nonnegative renewable scale required')
        for key in ('pmax_mw', 'forecast_mw', 'available', 'must_off'):
            values = np.asarray(g[key], dtype=float)
            if len(values) < 96 or not np.isfinite(values[:96]).all():
                raise ValueError('Incomplete or nonfinite renewable availability input')
        for t in range(96):
            total += min(g['pmax_mw'][t], g['forecast_mw'][t]*scale)*g['available'][t]*(1-g['must_off'][t])*.25
    return total


def daily_surplus(boundary, spec):
    if boundary['external_schedules'] or boundary['trades']:
        raise ValueError('Energy-hinge scope excludes external schedules and trades')
    if any(day.get('generator_outages') or day.get('branch_outages') for day in spec['external_days']):
        raise ValueError('Energy-hinge scope excludes outages')
    load = sum(sum(a['load_mw'][:96])*.25 for a in boundary['areas'])
    if load <= 0:
        raise ValueError('Positive baseline daily demand required')
    # Baseline H is known authored action, not an Oracle output.
    hydro = sum(spec['weekly'].values())/7
    values = [hydro+renewable_energy(boundary, day)-load*day['load_scale'] for day in spec['external_days'][:7]]
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite energy context')
    return np.asarray(values)


def feature(boundary, spec):
    r = daily_surplus(boundary, spec)
    return np.array([r[1]/300, r[4]/300, spec['delta_mwh']/300,
                     spec['external_days'][1]['line_limit_scale'], spec['external_days'][4]['line_limit_scale']])


def physics_gain(x):
    """Difference of positive parts from the aggregate relaxation, §10.1."""
    x = np.atleast_2d(np.asarray(x, dtype=float))
    if x.shape[1] != 5 or not np.isfinite(x).all():
        raise ValueError('Finite five-coordinate input required')
    r2, r5, delta = x[:, 0]*300, x[:, 1]*300, x[:, 2]*300
    return np.maximum(r2, 0)+np.maximum(r5, 0)-np.maximum(r2-delta, 0)-np.maximum(r5+delta, 0)


def kernel(a, b):
    # RW2006 §2.2, fixed squared-exponential covariance, frozen amplitudes §10.2.
    difference = np.asarray(a)[:, None, :]-np.asarray(b)[None, :, :]
    return 2500.*np.exp(-.5*np.sum(difference*difference, axis=2))


class PhysicsGP:
    def fit(self, x, y):
        self.x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        if self.x.ndim != 2 or self.x.shape[1] != 5 or len(y) != len(self.x) or not np.isfinite(y).all():
            raise ValueError('Invalid GP training arrays')
        residual = y-physics_gain(self.x)
        cov = kernel(self.x, self.x)+25.*np.eye(len(self.x))
        self.cholesky = cho_factor(cov, lower=True)
        self.alpha = cho_solve(self.cholesky, residual)
        # Independent sum-form solve residual (no normal-equation inverse).
        error = np.sum(cov*self.alpha[None, :], axis=1)-residual
        if np.max(np.abs(error)) > 1e-8*max(1., np.max(np.abs(residual))):
            raise ValueError('GP covariance solve residual failed')
        return self

    def predict(self, x, return_std=False):
        x = np.atleast_2d(np.asarray(x, dtype=float))
        prior = physics_gain(x)
        cross = kernel(x, self.x)
        mean = prior + np.sum(cross*self.alpha[None, :], axis=1)
        mean[x[:, 2] == 0] = 0.  # Exact identical-action gain invariant, design §10.2.
        if not return_std:
            return mean
        projected = solve_triangular(self.cholesky[0], cross.T, lower=True)
        variance = 2500.-np.sum(projected*projected, axis=0)
        if np.min(variance) < -1e-6:
            raise ValueError('Negative GP posterior variance beyond roundoff')
        std = np.sqrt(np.maximum(variance, 0.))
        std[x[:, 2] == 0] = 0.
        return mean, std

    def save(self, path):
        np.savez(path, x=self.x, alpha=self.alpha, cholesky=np.tril(self.cholesky[0]),
                 lower=np.array(True), schema=np.array('hydro-physics-gp-v2'))

    @classmethod
    def load(cls, path):
        data = np.load(path, allow_pickle=False)
        if str(data['schema']) != 'hydro-physics-gp-v2':
            raise ValueError('Unsupported GP artifact')
        obj = cls()
        obj.x, obj.alpha = data['x'], data['alpha']
        obj.cholesky = (data['cholesky'], True)
        return obj
