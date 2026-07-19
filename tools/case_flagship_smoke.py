#!/usr/bin/env python3
"""旗舰案例冒烟矩阵：每个旗舰案例跑其主打模块，验证"一键出有意义结果"。"""
import json, time, urllib.request, sys

BASE = 'http://127.0.0.1:18120'

def post(path, payload=None, timeout=300):
    req = urllib.request.Request(BASE + path, data=json.dumps(payload or {}).encode(),
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())

def load(case):
    d = post('/api/session/load_builtin', {'case': case})
    assert 'error' not in d, f"{case}: {d['error']}"
    return d

def numeric_series(o, path=''):
    """收集 JSON 里所有长度>10 的数值数组。"""
    out = []
    if isinstance(o, dict):
        for k, v in o.items():
            out += numeric_series(v, f'{path}.{k}')
    elif isinstance(o, list):
        if len(o) > 10 and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in o):
            out.append((path, o))
        else:
            for i, v in enumerate(o[:50]):
                out += numeric_series(v, f'{path}[{i}]')
    return out

fails = []
def check(name, ok, detail=''):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}  {detail}")
    if not ok:
        fails.append(name)

# 1. dist33 可靠性 + 弹性
load('dist33_microgrid_der')
rel = post('/api/session/run_reliability', {})
metrics = rel.get('metrics', {})
saidi = metrics.get('saidi')
asai = metrics.get('asai')
check('dist33 可靠性 SAIDI/ASAI', saidi is not None and saidi > 0 and asai is not None and asai < 1.0,
      f"SAIDI={saidi} SAIFI={metrics.get('saifi')} ASAI={asai} EENS={metrics.get('eens_mwh_yr')}")

res = post('/api/session/run_distribution_resilience', {})
res_txt = json.dumps(res, ensure_ascii=False)
has_restore = ('restor' in res_txt or '恢复' in res_txt or 'energized' in res_txt.lower()) and 'error' not in res
check('dist33 弹性恢复', has_restore, f'keys={list(res)[:8]}')

# 2. comprehensive 承载力
load('comprehensive_hybrid_acdc')
host = post('/api/session/run_hosting_capacity', {})
host_txt = json.dumps(host, ensure_ascii=False)
has_assess = ('error' not in host) and any(k in host_txt for k in ['hosting', 'capacity', '承载', 'assessment', 'transformer'])
check('comprehensive 承载力评估', has_assess, f'keys={list(host)[:8]}')

# 3-4. 两个微网暂态 + 发电机跳闸
def gen_trip_smoke(case, tag):
    d = load(case)
    raw = json.loads(d['_raw_json'])
    gens = [g for g in raw['ac']['generators'] if not g.get('is_slack') and g.get('in_service', True)]
    check(f'{tag} 含非 slack 机组', len(gens) > 0, f'{len(gens)} 台')
    if not gens: return
    target = gens[0]['index']
    tr = post('/api/session/run_transient', {
        'solver_type': 'heun', 't_end_s': 0.6, 'dt_s': 0.005,
        'run_power_flow_initialization': True,
        'events': [{'time_s': 0.1, 'type': 'GeneratorTrip', 'component_index': target,
                    'label': f'{tag} G{target} trip'}]})
    ok = tr.get('success', True) and 'error' not in tr
    series = numeric_series(tr)
    moving = [(p, s) for p, s in series if max(s) - min(s) > 1e-6]
    check(f'{tag} 暂态跳闸', ok and len(moving) > 0,
          f'success={tr.get("success")} 非平线轨迹={len(moving)} 示例={moving[0][0] if moving else "无"}')

gen_trip_smoke('hybrid_acdc_microgrid_island', '孤岛微网')
gen_trip_smoke('networked_microgrids_islanding', '联网微网群')

# 5. market_3bus 出清 LMP 分裂（市场模块为 AC-only；5bus 混合资产按设计被拒绝）
load('market_3bus_toy')
mk = post('/api/session/run_market_clearing', {
    'num_steps': 24, 'offer_segments': 8, 'reserve_fraction': 0.05,
    'network_constraints': True, 'run_ac_validation': False})
peak_split = []
for p in mk.get('pricing', []):
    lmp = p.get('lmp_per_mwh', [])
    if len(set(round(x, 3) for x in lmp)) > 1:
        peak_split.append((p['period'], lmp))
check('market_3bus 出清收敛', mk.get('status') == 'converged' and mk.get('feasible') is True,
      f"status={mk.get('status')} periods={mk.get('num_periods')}")
check('market_3bus 峰时 LMP 分裂', len(peak_split) > 0,
      f'分裂时段={len(peak_split)} 示例 period {peak_split[0][0]}: {peak_split[0][1]}' if peak_split else '无分裂')
load('market_5bus_acdc_toy')
mk5 = post('/api/session/run_market_clearing', {
    'num_steps': 24, 'offer_segments': 8, 'reserve_fraction': 0.05,
    'network_constraints': True, 'run_ac_validation': False})
check('market_5bus 混合资产诚实拒绝', mk5.get('status') == 'unsupported_hybrid_market_assets',
      f"status={mk5.get('status')}")

# 6. urban_lvn 三相潮流
load('urban_lvn_primary_secondary')
tp = post('/api/session/pf', {'method': 'three_phase', 'options': {
    'max_iter': 200, 'tol': 1e-8,
    'three_phase': {'algorithm': 'compact', 'scope': 'ac_only', 'include_shunts': True}}})
check('urban_lvn 三相潮流', tp.get('converged') is True, f"converged={tp.get('converged')} iter={tp.get('iterations')}")

# 7. five_province FDPF
load('five_province_acdc')
fp = post('/api/session/pf', {'method': 'fdpf'})
vm = fp.get('vm') or []
vm_ok = vm and 0.9 <= min(vm) and max(vm) <= 1.1
check('five_province FDPF', fp.get('converged') is True and vm_ok,
      f"converged={fp.get('converged')} vm=[{min(vm):.4f},{max(vm):.4f}]" if vm else f"converged={fp.get('converged')}")

# 8. case2000 性能旗舰纯 AC NR 计时（混合 NR 对 MTDC 下垂敏感，属既有案例数据限制）
load('case2000_acdc')
t0 = time.time()
c2k = post('/api/session/pf', {'method': 'pure_ac', 'options': {'max_iter': 200}}, timeout=600)
dt = time.time() - t0
check('case2000 纯AC NR 性能', c2k.get('converged') is True,
      f"converged={c2k.get('converged')} iter={c2k.get('iterations')} 墙钟={dt:.2f}s 服务端={c2k.get('execution_time_sec')}s")

print()
print(f'{len(fails)} 项失败' if fails else '全部通过')
sys.exit(1 if fails else 0)
