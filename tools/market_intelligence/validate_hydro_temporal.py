#!/usr/bin/env python3
"""Independent raw conservation, UC execution and held-out audit; hydro theory §12."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from build_market_label_dataset import read_json
from run_price_oracle import ROOT, digest, save


def close(a,b,tolerance,name):
    a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
    if a.shape!=b.shape or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError(name+': incompatible or nonfinite audit values')
    error=float(np.max(np.abs(a-b)))
    if error>tolerance:
        raise ValueError(f'{name}: maximum difference {error}, tolerance {tolerance}')
    return error


def audit_raw(root,design):
    errors={k:0. for k in ('load_mw','renewable_mw','energy_mwh','water_m3','thermal_mw','storage_mwh')}
    curtailment={};daily_energy={};price_days=0;gaps=[];limit_stages=0;state_links=0
    original=design['boundaries']['0-0']
    for key,b in design['boundaries'].items():
        for old,new in zip(original['buses'],b['buses']):
            if old['id']!=new['id']:
                raise ValueError('Stable node ID mapping changed')
            close(sum(old['load_mw'][:96]),sum(new['load_mw'][:96]),1e-6,'node daily energy preserved')
        for old,new in zip(original['generators'],b['generators']):
            if old['kind'] in ('wind','solar','renewable') and old!=new:
                raise ValueError('Renewable input changed under shape/condition intervention')
    for i,spec in enumerate(design['specs']):
        job=read_json(root/f'week-{i:03d}/job.json.gz');b=job['base'];scenario=job['scenarios'][0]
        if digest(b)!=spec['boundary_sha256'] or digest(b)!=digest(design['boundaries'][spec['boundary_key']]):
            raise ValueError('Normalized executed base differs from frozen shape/state input')
        if scenario['config']!=spec['config'] or scenario['completed_days']!=7 or scenario['status']!='completed':
            raise ValueError('Incomplete or changed operation contract')
        days=scenario['days'];thermal={g['id']:g for g in b['generators'] if g['kind']=='thermal'}
        previous={g['id']:{k:g[k] for k in ('initial_on','initial_power_mw','initial_state_minutes')} for g in b['generators']}
        previous_storage={s['id']:s['initial_mwh'] for s in b['storage']}
        records=[]
        for d,day in enumerate(days):
            if not day['valid'] or not day['diagnostic_prices_valid'] or not day['stages']['lmp']['price_consistency']['passed']:
                raise ValueError('Invalid schedule or main LMP chain')
            price_days+=1
            for stage in day['stages'].values():
                gaps.append(stage['mip_gap']);limit_stages+=int(bool(stage['limit_reached']))
            factors=spec['external_days'][d]
            demand=0.
            for node in day['nodes']:
                authored=next(n for n in b['buses'] if n['id']==node['id'])
                expected=np.array(authored['load_mw'][:96])*factors['load_scale']
                errors['load_mw']=max(errors['load_mw'],close(node['load_mw'],expected,1e-6,'executed node load'))
                demand+=sum(node['load_mw'])*.25
            start={g['id']:g for g in day['state_start']['generators']}
            for gid,expected in previous.items():
                for field,value in expected.items():
                    close(start[gid][field],value,1e-6,'chronological generator state')
                state_links+=1
            av=cur=used=hydro=therm=0.
            for g in day['resources']['generators']:
                authored=next(n for n in b['generators'] if n['id']==g['id'])
                if g['kind'] in ('wind','solar','renewable'):
                    expected=[]
                    for t in range(96):
                        scale=factors.get(g['kind']+'_scale',1.)
                        expected.append(min(authored['pmax_mw'][t],authored['forecast_mw'][t]*scale)*authored['available'][t]*(1-authored['must_off'][t]))
                    errors['renewable_mw']=max(errors['renewable_mw'],close(g['renewable_available_mw'],expected,1e-6,'executed renewable availability'))
                    av+=sum(expected)*.25;cur+=sum(g['curtailment_mw'])*.25;used+=sum(g['power_mw'])*.25
                elif g['kind']=='hydro':hydro+=sum(g['power_mw'])*.25
                elif g['kind']=='thermal':therm+=sum(g['power_mw'])*.25
                online=[int(v>.5) for v in g['online']];powers=g['power_mw']
                if g['id'] in thermal:
                    # Independent execution check of output bounds, ramps and
                    # residual minimum-time obligations, §12 / rules 2.6.3.8/11/12.
                    prior_on=start[g['id']]['initial_on'];prior_p=start[g['id']]['initial_power_mw']
                    duration=start[g['id']]['initial_state_minutes']
                    for t,(on,power) in enumerate(zip(online,powers)):
                        low=authored['pmin_mw'][t]*on;high=authored['pmax_mw'][t]*on
                        violation=max(low-power,power-high,0.)
                        errors['thermal_mw']=max(errors['thermal_mw'],violation)
                        if violation>1e-5:raise ValueError('Thermal operating bounds not executed')
                        if on!=prior_on:
                            required=authored['min_up_minutes'] if prior_on else authored['min_down_minutes']
                            if duration<required:raise ValueError('Minimum run/down obligation violated')
                            duration=15
                        else:
                            # Empty startup/shutdown curves; transitions have
                            # explicit max-power allowance in the solver model.
                            ramp=max(power-prior_p-15*authored['ramp_up_mw_min'],prior_p-power-15*authored['ramp_down_mw_min'],0.)
                            if ramp>1e-5:raise ValueError('Thermal stable ramp violated')
                            duration+=15
                        prior_on,prior_p=on,power
                count=0
                for on in reversed(online):
                    if on!=online[-1]:break
                    count+=1
                expected_minutes=count*15+(start[g['id']]['initial_state_minutes'] if count==96 and start[g['id']]['initial_on']==online[-1] else 0)
                previous[g['id']]={'initial_on':online[-1],'initial_power_mw':max(0.,min(powers[-1],authored['pmax_mw'][95])) if online[-1] else 0.,'initial_state_minutes':expected_minutes}
            errors['energy_mwh']=max(errors['energy_mwh'],close(av-used,cur,1e-6,'renewable energy identity'),
                close(hydro,sum(spec['quotas'][str(s['id'])][d] for s in design['mapping']),1e-3,'daily realized hydro'))
            # §12.3: charge is a NEGATIVE injection in the raw market API.
            storage=-sum((sum(s['charge_mw'])+sum(s['discharge_mw']))*.25 for s in day['resources']['storage'])
            storage_start={s['id']:s['initial_mwh'] for s in day['state_start']['storage']}
            for resource in day['resources']['storage']:
                sid=resource['id'];authored=next(s for s in b['storage'] if s['id']==sid)
                close(storage_start[sid],previous_storage[sid],1e-6,'storage state recurrence')
                energy=np.asarray(resource['energy_mwh']);eta=np.sqrt(authored['roundtrip_efficiency'])
                expected=np.r_[previous_storage[sid],energy[:-1]]-.25*(eta*np.asarray(resource['charge_mw'])+np.asarray(resource['discharge_mw'])/eta)
                errors['storage_mwh']=max(errors['storage_mwh'],close(energy,expected,1e-6,'signed storage inventory'),
                    close(energy[-1],authored['terminal_mwh'],1e-6,'common daily terminal storage'))
                previous_storage[sid]=energy[-1]
            demand_response=sum(sum(s['reduction_mw'])*.25 for s in day['resources']['controllable_loads'])
            balance=hydro+av+therm-demand-storage+demand_response+day['deficit_mwh']-day['surplus_mwh']
            errors['energy_mwh']=max(errors['energy_mwh'],close(cur,balance,1e-3,'full daily energy balance'))
            records.append({'load_mwh':demand,'available_mwh':av,'curtailment_mwh':cur,'thermal_mwh':therm,'hydro_mwh':hydro})
        daily_energy[i]=records;curtailment[i]=sum(r['curtailment_mwh'] for r in records)
        water={r['id']:[next(x for x in day['resources']['reservoirs'] if x['id']==r['id']) for day in days] for r in b['reservoirs']}
        releases={rid:[q for day in values for q in day['release_m3_s']] for rid,values in water.items()}
        for r in b['reservoirs']:
            rid,parent,lag=r['id'],r['upstream'],r['lag_slots']
            energy=sum(sum(g['power_mw'])*.25 for day in days for g in day['resources']['generators'] if g['id'] in r['generators'])
            errors['energy_mwh']=max(errors['energy_mwh'],close(energy,spec['weekly'][str(rid)],1e-3,'weekly station hydro'))
            own=sum(sum(r['inflow_m3_s'][:96])*d['inflow_scale']*900 for d in spec['external_days'][:7]);arrival=0.
            if parent>=0:
                parent_row=next(x for x in b['reservoirs'] if x['id']==parent)
                upstream=parent_row['release_history_m3_s'][-lag:]+releases[parent][:-lag] if lag else releases[parent]
                if len(upstream)!=672:raise ValueError('Invalid water lag history')
                arrival=sum(upstream)*900
            final=water[rid][-1]['level_m'][-1]
            errors['water_m3']=max(errors['water_m3'],close((final-r['initial_level_m'])*r['area_m2'],own+arrival-sum(releases[rid])*900,1.,'inventory telescoping'))
            spill=sum(q for day in water[rid] for q in day['spill_m3_s'])*900
            errors['water_m3']=max(errors['water_m3'],close(sum(releases[rid])*900,energy*r['water_m3_mwh']+spill,1.,'SI turbine water'))
            close(spill,spec['terminal'][str(rid)]['spill_volume_m3'],1.,'common weekly spill budget')
            close(final,spec['terminal'][str(rid)]['level_m'],1/r['area_m2'],'terminal reservoir level')
            tail=spec['terminal'][str(rid)]['tail_slots']
            if tail<1:raise ValueError('Missing terminal release-tail contract')
            close(releases[rid][-tail:], [spec['terminal'][str(rid)]['release_m3_s']]*tail,1e-6,'common terminal water in transit')
    for i,spec in enumerate(design['specs']):
        common=next(j for j,s in enumerate(design['specs']) if s['root_family']==spec['root_family'] and s['delta_mwh']==spec['delta_mwh'])
        for d in range(7):
            for field in ('load_mwh','available_mwh','hydro_mwh'):
                close(daily_energy[i][d][field],daily_energy[common][d][field],1e-3,'executed same daily energy across variants')
    return {'raw_weeks':len(curtailment),'valid_main_price_days':price_days,'state_links':state_links,
        'max_errors':errors,'max_reported_gap':max(gaps),'limit_reached_stages':limit_stages,
        'daily_energy':daily_energy},curtailment


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-intelligence/hydro-temporal-v3')
    p.add_argument('--report',type=Path,default=ROOT/'output/market-intelligence/hydro-temporal-analysis-v3')
    args=p.parse_args();design=json.loads((args.input/'design.json').read_text())
    report=json.loads((args.report/'report.json').read_text());frozen=json.loads((args.report/'model-freeze.json').read_text())
    if digest(design)!=report['design_sha256'] or digest(design)!=frozen['design_sha256']:
        raise ValueError('Design identity mismatch')
    for fit in frozen['fits']:
        for name,value in fit['artifact_hashes'].items():
            if hashlib.sha256((args.report/f"{name}-{fit['n_families']}.npz").read_bytes()).hexdigest()!=value:
                raise ValueError('Model changed after freezing')
    raw,curtailment=audit_raw(args.input,design);specs=design['specs'];max_metric=0.
    for result in report['results']:
        rows=[r for r in report['predictions'] if r['model']==result['model'] and r['n_families']==result['n_families']]
        if len(rows)!=24 or {r['root_family'] for r in rows}!={4,5}:raise ValueError('Wrong held-out cohort')
        errors=[];positive=[]
        for r in rows:
            s=specs[r['index']]
            if s['split']!='test' or s['root_family'] in r['train_root_families']:raise ValueError('Sibling leakage')
            j=next(i for i,s in enumerate(specs) if s['family']==r['family'] and s['delta_mwh']==0)
            close(r['actual_gain_mwh'],curtailment[j]-curtailment[r['index']],1e-8,'raw gain')
            if r['delta_mwh']!=0:
                errors.append(abs(r['actual_gain_mwh']-r['predicted_gain_mwh']))
                if r['actual_gain_mwh']>=100:positive.append(r)
        max_metric=max(max_metric,close(sum(errors)/16,result['mae_mwh'],1e-8,'MAE'),close(max(errors),result['max_error_mwh'],1e-8,'max error'))
        if len(positive)!=result['positive_cases'] or sum(r['predicted_gain_mwh']<100 for r in positive)!=result['positive_misses']:
            raise ValueError('Positive benefit detection mismatch')
        for choice in result['selection']:
            options=[r for r in rows if r['family']==choice['family']]
            selected=max(options,key=lambda r:(r['predicted_gain_mwh'],r['delta_mwh']==0))
            if selected['delta_mwh']!=choice['selected_delta_mwh']:raise ValueError('Selection used hindsight')
            close(max(r['actual_gain_mwh'] for r in options)-selected['actual_gain_mwh'],choice['regret_mwh'],1e-8,'candidate regret')
    save(args.report/'raw-execution-audit.json',raw)
    result={k:v for k,v in raw.items() if k!='daily_energy'}
    result.update(passed=True,max_metric_reconstruction_error=max_metric,model_hashes_verified=True,
        prediction_rows=len(report['predictions']),test_root_families=[4,5])
    save(args.report/'independent-validation.json',result);print(json.dumps(result))


if __name__=='__main__':main()
