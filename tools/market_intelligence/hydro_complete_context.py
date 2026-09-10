"""Complete authored-input comparator for the fixed v3 design; theory §12.7."""
import numpy as np

from hydro_temporal import network_feature

FACTORS=('load_scale','wind_scale','solar_scale','inflow_scale','line_limit_scale')
SCALES=np.array([.25,.8,.8,.4,.6])


def complete_feature(boundary,spec):
    if len(spec['external_days'])!=8:
        raise ValueError('Complete context requires seven days plus authored day eight')
    factors=np.array([[day[k] for k in FACTORS] for day in spec['external_days']])
    shape=np.sum([a['load_mw'][:96] for a in boundary['areas']],axis=0)
    # §12.7: retain every sampled exogenous degree of freedom, including the
    # lookahead day; RMS block scaling, fixed independently of target labels.
    x=np.r_[network_feature(boundary,spec),(factors/SCALES).ravel()/np.sqrt(40),shape/1000/np.sqrt(96)]
    if not np.isfinite(x).all():raise ValueError('Nonfinite complete authored context')
    return x


def recover_factors(x):
    return np.asarray(x)[-136:-96].reshape(8,5)*np.sqrt(40)*SCALES
