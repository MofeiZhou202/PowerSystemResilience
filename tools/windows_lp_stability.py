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
    return json.loads(path.read_text(encoding='utf-8-sig'))


def write(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')


def distribution(values):
    values = sorted(values)
    q = statistics.quantiles(values, n=4, method='inclusive') if len(values) > 1 else [values[0]] * 3
    median = statistics.median(values)
    return dict(n=len(values), median=median, p95=values[math.ceil(.95*len(values))-1],
                maximum=max(values), iqr_over_median=(q[2]-q[0])/median if median else 0)


def execute(out, name, threads, binary_dir, timing=True, repeats=1,
            all_cases=False, solvers='native-ipm-direct', nlp=False):
    env = os.environ.copy()
    env.update(OMP_NUM_THREADS='1', MKL_NUM_THREADS=str(threads), MKL_DYNAMIC='FALSE',
               MIPSOLVERS_LP_FACTOR_TIMING='1' if timing else '0')
    env.pop('MIPSOLVERS_IPM_VERBOSE', None)
    env['MIPSOLVERS_BENCH_GIT_COMMIT'] = subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip()
    exe = binary_dir/('nlp_open_benchmark.exe' if nlp else 'netlib_solver_benchmark.exe')
    command = [str(exe), '5', '1e-7'] if nlp else [str(exe), '--data-dir', 'tests/data',
        '--solvers', solvers, '--repeat', str(repeats), '--time-limit','15', '--max-iterations','100000',
        '--json', str(out/f'{name}.json')]
    if not nlp and not all_cases:
        command += ['--cases', ','.join(CASES)]
    metadata = dict(command=command, threads=threads, timing=timing, started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
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
    metadata.update(wall_ms=(time.perf_counter()-start)*1000, peak_working_set_bytes=peak, exit_code=process.returncode)
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
    records=[]
    for line in (out/f'{name}.stderr.log').read_text(errors='replace').splitlines():
        if line.startswith('LP-FACTOR '):
            record=json.loads(line[len('LP-FACTOR '):])
            if min(record[k] for k in ('assembly_ms','symbolic_ms','numeric_ms','other_ms','retry_ms')) < -1e-3:
                raise RuntimeError(f'{name}: invalid exclusive timing interval')
            if abs(sum(record[k] for k in ('assembly_ms','symbolic_ms','numeric_ms','other_ms'))-record['variant_ms'])>1e-3:
                raise RuntimeError(f'{name}: timing partition does not close')
            record['case']=DIMS.get((record['rows'],record['cols']))
            records.append(record)
    if timing and not records:
        raise RuntimeError(f'{name}: no factor timing records')
    if not timing and records:
        raise RuntimeError(f'{name}: timing disabled but produced records')
    write(out/f'{name}.factors.json',records)
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
        result[str(threads)]=dict(cases={c:distribution([r['runtime_ms'] for r in rows if r['case']==c]) for c in CASES},
            aggregate=distribution(totals), accurate=sum(r['accurate'] for r in rows), runs=len(rows),
            peak_working_set_mib=distribution([read(f.with_suffix('.process.json'))['peak_working_set_bytes']/2**20 for f in files]),
            wall_ms=distribution([read(f.with_suffix('.process.json'))['wall_ms'] for f in files]), factors=profiles,
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
