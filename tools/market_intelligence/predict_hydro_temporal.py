#!/usr/bin/env python3
"""Validated local research inference, with mandatory Oracle confirmation (§12)."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from hydro_allocation import terminal_contract, compile_action
from hydro_physics import daily_surplus
from hydro_temporal import TemporalGP, temporal_feature, network_feature
from hydro_complete_context import complete_feature
from run_price_oracle import ROOT, digest, save


def validate_candidate(boundary,spec,design,frozen):
    if digest(boundary) not in frozen['boundary_hashes'].values():
        raise ValueError('Boundary shape/condition outside frozen four-base support')
    if not np.isfinite(spec['delta_mwh']) or abs(spec['delta_mwh'])>300:
        raise ValueError('Only station1 day2/day5 actions within ±300 MWh are supported')
    external=spec['external_days']
    if len(external)!=8:
        raise ValueError('Seven executed days plus authored lookahead day required')
    fields={'load_scale','wind_scale','solar_scale','inflow_scale','line_limit_scale','generator_bid_scale',
            'load_bid_scale','bid_scale','first_slot','last_slot','generator_outages','branch_outages'}
    for d,day in enumerate(external):
        if set(day)!=fields or day['first_slot']!=0 or day['last_slot']!=95 or day['generator_outages'] or day['branch_outages']:
            raise ValueError('Unsupported exogenous intervention')
        for field,bounds in (('wind_scale',(1,1.8)),('solar_scale',(1,1.8)),('inflow_scale',(.6,1)),('line_limit_scale',(.4,1))):
            if not np.isfinite(day[field]) or not bounds[0]<=day[field]<=bounds[1]:
                raise ValueError('Exogenous factor outside authored support: '+field)
        if not np.isfinite(day['load_scale']) or day['load_scale']<=0:
            raise ValueError('Invalid load scale')
        if d not in (1,4) and not .6<=day['load_scale']<=.85:
            raise ValueError('Unaffected-day load scale outside authored support')
        if any(day[k]!=1 for k in ('generator_bid_scale','load_bid_scale','bid_scale')):
            raise ValueError('Bid changes are outside the frozen surrogate support')
    baseline={str(s['id']):[3000.]*7 for s in design['mapping']}
    weekly={sid:21000. for sid in baseline}
    if spec['weekly']!=weekly:
        raise ValueError('Weekly station energy changed')
    quotas={sid:values.copy() for sid,values in baseline.items()}
    quotas['1'][1]-=spec['delta_mwh'];quotas['1'][4]+=spec['delta_mwh']
    if quotas!=spec['quotas']:
        raise ValueError('Candidate quotas do not implement the declared action')
    if np.max(np.abs(daily_surplus(boundary,spec)[[1,4]]))>180+1e-6:
        raise ValueError('Transition coordinates outside authored support')
    terminal=terminal_contract(boundary,design['mapping'],external,baseline)
    if terminal!=spec['terminal']:
        raise ValueError('Water terminal comparison contract changed')
    config=spec['config']
    if {k:v for k,v in config.items() if k not in ('days','reference_days')}!=frozen['operation_contract']:
        raise ValueError('Operation/solver contract differs from training labels')
    if config['reference_days']!=external or config['days']!=compile_action(boundary,design['mapping'],external,quotas,weekly,terminal):
        raise ValueError('Executed action/reference differs from declared candidate')


class ResearchPredictor:
    def __init__(self,model_dir,design_path,model_name='temporal_gp'):
        if model_name not in ('temporal_gp','network_gp','complete_gp'):
            raise ValueError('Choose the frozen primary or secondary research model')
        self.model_name=model_name
        self.design=json.loads(Path(design_path).read_text())
        self.frozen=json.loads((Path(model_dir)/'model-freeze.json').read_text())
        if digest(self.design)!=self.frozen['design_sha256']:
            raise ValueError('Design differs from saved model protocol')
        path=Path(model_dir)/f'{model_name}-4.npz'
        fit=next(f for f in self.frozen['fits'] if f['n_families']==4)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=fit['artifact_hashes'][model_name]:
            raise ValueError('Saved model hash mismatch')
        self.model=TemporalGP.load(path)

    def predict(self,boundary,spec):
        start=time.perf_counter_ns()
        validate_candidate(boundary,spec,self.design,self.frozen)
        compute={'temporal_gp':temporal_feature,'network_gp':network_feature,'complete_gp':complete_feature}[self.model_name]
        x=compute(boundary,spec);predicted=float(self.model.predict(x)[0])
        lo=self.model.x.min(axis=0);hi=self.model.x.max(axis=0)
        outside=bool(np.any((x<lo-1e-10)|(x>hi+1e-10)))
        return {'model':self.model_name,'predicted_gain_mwh':predicted,'outside_training_box':outside,
            'requires_oracle_confirmation':True,'production_admitted':False,
            'scope':'local frozen-base research estimate; does not certify gain accuracy or safety',
            'validated_inference_ms':(time.perf_counter_ns()-start)/1e6}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True,help='Saved Oracle input.json containing base and spec')
    p.add_argument('--model',type=Path,default=ROOT/'output/market-intelligence/hydro-temporal-analysis-v3')
    p.add_argument('--design',type=Path,default=ROOT/'output/market-intelligence/hydro-temporal-v3/design.json')
    p.add_argument('--output',type=Path)
    p.add_argument('--model-kind',choices=('temporal_gp','network_gp','complete_gp'),default='temporal_gp')
    args=p.parse_args();candidate=json.loads(args.candidate.read_text())
    if candidate.get('config')!=candidate['spec']['config']:
        raise ValueError('Candidate wrapper/config mismatch')
    result=ResearchPredictor(args.model,args.design,args.model_kind).predict(candidate['base'],candidate['spec'])
    if args.output:save(args.output,result)
    print(json.dumps(result))


if __name__=='__main__':main()
