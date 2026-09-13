"""Windows LP timing/stability protocol, Windows remediation R6 (stdlib only)."""
import argparse
import ctypes
from ctypes import wintypes
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import statistics
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
CASES = ('dfl001', 'greenbea', 'maros-r7')
DIMS = {(6071, 12230): 'dfl001', (2392, 5405): 'greenbea', (3136, 9408): 'maros-r7'}

class ProcessMemory(ctypes.Structure):
    _fields_ = [('cb', wintypes.DWORD), ('PageFaultCount', wintypes.DWORD)] + [
        (n, ctypes.c_size_t) for n in ('PeakWorkingSetSize', 'WorkingSetSize',
        'QuotaPeakPagedPoolUsage', 'QuotaPagedPoolUsage', 'QuotaPeakNonPagedPoolUsage',
        'QuotaNonPagedPoolUsage', 'PagefileUsage', 'PeakPagefileUsage')]


def read(path):
    return strict_json(path.read_text(encoding='utf-8-sig'))


def strict_json(text):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result
    return json.loads(text, object_pairs_hook=pairs)


def write(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')


def distribution(values):
    values = sorted(values)
    q = statistics.quantiles(values, n=4, method='inclusive') if len(values) > 1 else [values[0]] * 3
    median = statistics.median(values)
    return dict(n=len(values), median=median, p95=values[math.ceil(.95*len(values))-1],
                maximum=max(values), iqr_over_median=(q[2]-q[0])/median if median else 0)


def correlation(xs, ys):
    if len(xs) != len(ys) or len(xs) < 2:
        return None
    pairs=[(x,y) for x,y in zip(xs,ys)
           if isinstance(x,(int,float)) and math.isfinite(x) and
              isinstance(y,(int,float)) and math.isfinite(y)]
    if len(pairs) < 2:
        return None
    xs=[x for x,_ in pairs]; ys=[y for _,y in pairs]
    mean_x=statistics.mean(xs); mean_y=statistics.mean(ys)
    dx=[x-mean_x for x in xs]; dy=[y-mean_y for y in ys]
    denom=math.sqrt(sum(x*x for x in dx)*sum(y*y for y in dy))
    return sum(x*y for x,y in zip(dx,dy))/denom if denom else None


def parse_telemetry(log, threads, timing=True):
    name=str(log)
    records=[]
    recoveries=[]
    for line in log.read_text(encoding='utf-8', errors='strict').splitlines():
        if line.startswith('LP-FACTOR '):
            record=strict_json(line[len('LP-FACTOR '):])
            if type(record.get('schema_version')) is not int or record['schema_version'] != 2:
                raise RuntimeError(f'{name}: unsupported telemetry schema')
            if record.get('entry_reason') not in {
                    'primary', 'normal_stalled', 'normal_rejected',
                    'factorization_failed'}:
                raise RuntimeError(f'{name}: invalid factor entry reason')
            if record.get('formulation') not in {
                    'banded', 'dense', 'normal', 'augmented', 'unselected'}:
                raise RuntimeError(f'{name}: invalid factor formulation')
            if not isinstance(record.get('adaptive_backend_sequence'), str) or not re.fullmatch(r'[LU]{0,16}',
                                record['adaptive_backend_sequence']):
                raise RuntimeError(f'{name}: invalid adaptive backend sequence')
            if (type(record.get('source_iterations')) is not int or
                    record['source_iterations'] < 0):
                raise RuntimeError(f'{name}: invalid factor transition iteration')
            if record.get('adaptive_backend_overflow') is not False:
                raise RuntimeError(f'{name}: incomplete backend sequence')
            for key in ('assembly_ms','symbolic_ms','numeric_ms','other_ms','retry_ms','variant_ms'):
                if type(record.get(key)) not in (int, float) or not math.isfinite(record[key]):
                    raise RuntimeError(f'{name}: non-finite factor timing {key}')
            if min(record[k] for k in ('assembly_ms','symbolic_ms','numeric_ms','other_ms','retry_ms')) < -1e-3:
                raise RuntimeError(f'{name}: invalid exclusive timing interval')
            if abs(sum(record[k] for k in ('assembly_ms','symbolic_ms','numeric_ms','other_ms'))-record['variant_ms'])>1e-3:
                raise RuntimeError(f'{name}: timing partition does not close')
            record['case']=DIMS.get((record['rows'],record['cols']))
            records.append(record)
        elif line.startswith('LP-RECOVERY '):
            record=strict_json(line[len('LP-RECOVERY '):])
            record['case']=DIMS.get((record.get('rows'),record.get('cols')))
            if record['case'] is None:
                raise RuntimeError(f'{name}: unknown recovery dimensions')
            if type(record.get('schema_version')) is not int or record['schema_version'] != 2:
                raise RuntimeError(f'{name}: unsupported telemetry schema')
            if record.get('entry_reason') not in {
                    'normal_stalled', 'normal_rejected',
                    'factorization_failed'}:
                raise RuntimeError(f'{name}: invalid recovery entry reason')
            for key in ('requested_mkl_threads', 'recovery_mkl_threads',
                        'source_iterations', 'recovery_iterations', 'seed_size'):
                if type(record.get(key)) is not int or record[key] < 0:
                    raise RuntimeError(f'{name}: invalid recovery {key}')
            if (record['requested_mkl_threads'] != threads or
                    record['recovery_mkl_threads'] > threads):
                raise RuntimeError(f'{name}: invalid recovery thread scope')
            if record.get('seed_present') is not True or record['seed_size'] <= 0:
                raise RuntimeError(f'{name}: missing retained recovery seed')
            if not re.fullmatch(r'[0-9a-f]{16}', record.get('seed_hash', '')):
                raise RuntimeError(f'{name}: invalid recovery seed hash')
            for key in ('source_runtime_ms', 'seed_norm_inf', 'seed_norm_2',
                        'seed_objective', 'seed_capture_ms',
                        'record_prepare_ms', 'recovery_runtime_ms'):
                if type(record.get(key)) not in (int, float) or not math.isfinite(record[key]):
                    raise RuntimeError(f'{name}: invalid recovery {key}')
            metric_keys = (
                'source_primal_feas', 'source_dual_feas',
                'source_complementarity', 'source_relative_primal_residual',
                'source_relative_dual_residual', 'source_relative_gap',
                'recovery_primal_feas', 'recovery_dual_feas',
                'recovery_complementarity',
                'recovery_relative_primal_residual',
                'recovery_relative_dual_residual', 'recovery_relative_gap')
            if any(key not in record or (record[key] is not None and
                   (type(record[key]) not in (int, float) or
                    not math.isfinite(record[key]))) for key in metric_keys):
                raise RuntimeError(f'{name}: invalid recovery state metric')
            if not isinstance(record.get('recovery_success'), bool):
                raise RuntimeError(f'{name}: invalid recovery result')
            if any(record[key] < 0 for key in ('source_runtime_ms', 'recovery_runtime_ms', 'seed_norm_inf', 'seed_norm_2', 'seed_capture_ms', 'record_prepare_ms')):
                raise RuntimeError(f'{name}: negative recovery measurement')
            if record['recovery_mkl_threads'] < 1:
                raise RuntimeError(f'{name}: invalid recovery thread budget')
            recoveries.append(record)
    if timing and not records:
        raise RuntimeError(f'{name}: no factor timing records')
    if not timing and records:
        raise RuntimeError(f'{name}: timing disabled but produced records')
    factor_transitions=sorted(
        (r['case'], r['entry_reason'], r['source_iterations'])
        for r in records if r['entry_reason'] != 'primary')
    recovery_transitions=sorted(
        (r['case'], r['entry_reason'], r['source_iterations'])
        for r in recoveries)
    if timing and factor_transitions != recovery_transitions:
        raise RuntimeError(f'{name}: recovery/factor transition mismatch')
    transition_backends={
        (r['case'], r['entry_reason'], r['source_iterations']):
            r['adaptive_backend_sequence']
        for r in records if r['entry_reason'] != 'primary'}
    for recovery in recoveries:
        key=(recovery['case'], recovery['entry_reason'],
             recovery['source_iterations'])
        recovery['adaptive_backend_sequence']=transition_backends[key]
    if not timing and recoveries:
        raise RuntimeError(f'{name}: timing disabled but produced recoveries')
    return records, recoveries


def execute(out, name, threads, binary_dir, timing=True, repeats=1,
            all_cases=False, solvers='native-ipm-direct', nlp=False,
            cases=CASES):
    env = os.environ.copy()
    for key in list(env):
        if key.startswith('MIPSOLVERS_') or key == 'MKL_CBWR':
            env.pop(key)
    env.update(OMP_NUM_THREADS='1', MKL_NUM_THREADS=str(threads), MKL_DYNAMIC='FALSE',
               MIPSOLVERS_LP_FACTOR_TIMING='1' if timing else '0')
    env.pop('MIPSOLVERS_IPM_VERBOSE', None)
    env.pop('MIPSOLVERS_LP_PIVOT_TRACE', None)
    env.pop('MIPSOLVERS_EXPERIMENTAL_LP_RECOVERY_CAP2', None)
    env['MIPSOLVERS_BENCH_GIT_COMMIT'] = subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip()
    exe = binary_dir/('nlp_open_benchmark.exe' if nlp else 'netlib_solver_benchmark.exe')
    command = [str(exe), '5', '1e-7'] if nlp else [str(exe), '--data-dir', 'tests/data',
        '--solvers', solvers, '--repeat', str(repeats), '--time-limit','15', '--max-iterations','100000',
        '--json', str(out/f'{name}.json')]
    if not nlp and not all_cases:
        command += ['--cases', ','.join(cases)]
    metadata = dict(binary_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(), started_ns=time.time_ns(), command=command, threads=threads, timing=timing, started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        environment={k:env.get(k) for k in ('OMP_NUM_THREADS','MKL_NUM_THREADS','MKL_DYNAMIC','MKL_CBWR','MIPSOLVERS_LP_FACTOR_TIMING','MIPSOLVERS_BENCH_GIT_COMMIT')})
    get_memory = ctypes.WinDLL('psapi').GetProcessMemoryInfo
    get_memory.argtypes = [wintypes.HANDLE, ctypes.POINTER(ProcessMemory), wintypes.DWORD]
    get_memory.restype = wintypes.BOOL
    peak = 0
    start = time.perf_counter()
    with (out/f'{name}.stdout.log').open('wb') as stdout, (out/f'{name}.stderr.log').open('wb') as stderr:
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stdout, stderr=stderr,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
        while True:
            memory = ProcessMemory()
            memory.cb = ctypes.sizeof(memory)
            if get_memory(wintypes.HANDLE(process._handle), ctypes.byref(memory), memory.cb):
                peak = max(peak, memory.PeakWorkingSetSize)
            if process.poll() is not None:
                break
            if time.perf_counter()-start > (1800 if all_cases else 180):
                process.kill()
                process.wait()
                raise RuntimeError(f'{name}: process watchdog expired')
            time.sleep(.05)
    metadata.update(pid=process.pid, finished_ns=time.time_ns(), wall_ms=(time.perf_counter()-start)*1000, peak_working_set_bytes=peak, exit_code=process.returncode)
    metadata['artifact_sha256'] = {
        suffix: hashlib.sha256((out/f'{name}{suffix}').read_bytes()).hexdigest()
        for suffix in ('.json', '.stdout.log', '.stderr.log')
        if (out/f'{name}{suffix}').is_file()}
    write(out/f'{name}.process.json', metadata)
    if process.returncode:
        raise RuntimeError(f'{name}: exit {process.returncode}; raw logs retained')
    if nlp:
        lines=(out/f'{name}.stdout.log').read_text(errors='replace').splitlines()
        if sum('accurate=1' in line for line in lines)!=10:
            raise RuntimeError(f'{name}: NLP accuracy gate failed')
        print(name, '10/10 accurate', flush=True)
        return
    data=read(out/f'{name}.json')
    if data['configuration']['mkl_max_threads']!=threads:
        raise RuntimeError(f'{name}: MKL thread setting not honored')
    if not data['runs'] or any(not r['accurate'] for r in data['runs']):
        raise RuntimeError(f'{name}: original-model accuracy gate failed')
    records, recoveries = parse_telemetry(out/f'{name}.stderr.log', threads, timing)
    write(out/f'{name}.factors.json',records)
    write(out/f'{name}.recoveries.json',recoveries)
    print(name, f"{len(data['runs'])} accurate, sum={sum(r['runtime_ms'] for r in data['runs']):.1f} ms", flush=True)


def summarize(out):
    result={}
    for threads in (2,4):
        files=sorted(out.glob(f'block-*-t{threads}.json'))
        files=[f for f in files if len(f.name.split('.'))==2]
        if not files:
            continue
        rows=[r for f in files for r in read(f)['runs']]
        totals=[sum(r['runtime_ms'] for r in read(f)['runs']) for f in files]
        records=[r for f in files for r in read(f.with_suffix('.factors.json'))]
        recoveries=[r for f in files for r in read(
            f.with_suffix('.recoveries.json'))]
        profiles={}
        for case in CASES:
            per_run=[]
            for f in files:
                factors=[r for r in read(f.with_suffix('.factors.json')) if r['case']==case]
                if not factors:
                    raise RuntimeError(f'{f}: missing profile for {case}')
                per_run.append({k:sum(r[k] for r in factors) for k in
                    ('assembly_ms','symbolic_ms','numeric_ms','other_ms','variant_ms','retry_ms','numeric_calls','symbolic_calls','retries')})
            profiles[case]={k:distribution([r[k] for r in per_run]) for k in per_run[0]}
        transition_profiles={}
        for case in CASES:
            transitions=[r for r in recoveries if r['case']==case]
            if not transitions:
                continue
            transition_profiles[case]=dict(
                n=len(transitions),
                entry_reasons=sorted(set(r['entry_reason'] for r in transitions)),
                source_iterations=sorted(set(r['source_iterations'] for r in transitions)),
                unique_seed_hashes=sorted(set(r['seed_hash'] for r in transitions)),
                adaptive_backend_sequences=sorted(set(
                    r['adaptive_backend_sequence'] for r in transitions)),
                source_runtime_ms=distribution([r['source_runtime_ms'] for r in transitions]),
                recovery_runtime_ms=distribution([r['recovery_runtime_ms'] for r in transitions]),
                recovery_iterations=distribution([r['recovery_iterations'] for r in transitions]),
                seed_norm_inf=distribution([r['seed_norm_inf'] for r in transitions]),
                seed_norm_2=distribution([r['seed_norm_2'] for r in transitions]),
                seed_objective=distribution([r['seed_objective'] for r in transitions]),
                telemetry_ms=distribution([
                    r['seed_capture_ms']+r['record_prepare_ms']
                    for r in transitions]),
                recovery_iteration_correlations=dict(
                    source_primal_feas=correlation(
                        [r['source_primal_feas'] for r in transitions],
                        [r['recovery_iterations'] for r in transitions]),
                    source_relative_primal_residual=correlation(
                        [r['source_relative_primal_residual'] for r in transitions],
                        [r['recovery_iterations'] for r in transitions]),
                    seed_norm_inf=correlation(
                        [r['seed_norm_inf'] for r in transitions],
                        [r['recovery_iterations'] for r in transitions]),
                    seed_objective=correlation(
                        [r['seed_objective'] for r in transitions],
                        [r['recovery_iterations'] for r in transitions])))
        result[str(threads)]=dict(cases={c:distribution([r['runtime_ms'] for r in rows if r['case']==c]) for c in CASES},
            aggregate=distribution(totals), accurate=sum(r['accurate'] for r in rows), runs=len(rows),
            peak_working_set_mib=distribution([read(f.with_suffix('.process.json'))['peak_working_set_bytes']/2**20 for f in files]),
            wall_ms=distribution([read(f.with_suffix('.process.json'))['wall_ms'] for f in files]), factors=profiles,
            recovery_transitions=transition_profiles,
            iterations={c:sorted(set(r['iterations'] for r in rows if r['case']==c)) for c in CASES})
    if set(result)=={'2','4'}:
        eligible=all(result['4']['cases'][c]['p95']<=1.1*result['2']['cases'][c]['p95'] for c in CASES)
        gain=1-result['4']['aggregate']['median']/result['2']['aggregate']['median']
        result['decision']=dict(recommended_threads=4 if eligible and gain>=.05 else 2,
            four_thread_p95_gate_passed=eligible, four_vs_two_median_reduction=gain)
    write(out/'summary.json',result)
    print(json.dumps(result.get('decision', {})),flush=True)


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--stage',choices=['overhead','stability','gates','summary'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--binary-dir',type=Path,default=Path('tests/Release'))
    args=parser.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if os.name!='nt':
        raise RuntimeError('This measurement protocol requires Windows')
    binary_dir=(args.binary_dir if args.binary_dir.is_absolute()
                else ROOT/args.binary_dir).resolve()
    required=['netlib_solver_benchmark.exe']
    if args.stage=='gates':
        required.append('nlp_open_benchmark.exe')
    missing=[str(binary_dir/name) for name in required if not (binary_dir/name).is_file()]
    if missing:
        raise FileNotFoundError('missing benchmark executable(s): '+', '.join(missing))
    binaries={name:hashlib.sha256((binary_dir/name).read_bytes()).hexdigest()
              for name in required}
    provenance=dict(platform=platform.platform(),binary_sha256=binaries['netlib_solver_benchmark.exe'],
        binary_sha256s=binaries,binary_dir=str(binary_dir),
        command=__import__('sys').argv,commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip())
    write(out/f'{args.stage}.provenance.json',provenance)
    if args.stage=='overhead':
        for threads in (2,4):
            execute(out,f'overhead-warm-t{threads}',threads,binary_dir,False)
            for pair in range(1,4):
                for timing in ((False,True) if pair%2 else (True,False)):
                    execute(out,f'overhead-{pair}-t{threads}-{int(timing)}',threads,binary_dir,timing)
    elif args.stage=='stability':
        for threads in (2,4):
            execute(out,f'warm-t{threads}',threads,binary_dir)
        for block in range(1,21):
            for threads in ((2,4) if block%2 else (4,2)):
                execute(out,f'block-{block:02}-t{threads}',threads,binary_dir)
        summarize(out)
    elif args.stage=='gates':
        for threads in (2,4):
            execute(out,f'full-t{threads}',threads,binary_dir,False,3,True,'native-ipm-direct,native-auto')
            execute(out,f'nlp-t{threads}',threads,binary_dir,False,nlp=True)
    else:
        summarize(out)

if __name__=='__main__':
    main()
