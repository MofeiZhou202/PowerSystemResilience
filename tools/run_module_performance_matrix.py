"""Sequential, resumable Windows module audit; build Release before invoking.

Records process status separately from numerical validity in underlying reports.
Existing output entries are reused only with the same executable/source manifest.
No running user server or model is reused. See docs/testing/performance_tooling.md.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

from module_performance_audit import execute


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--dependency', required=True)
    parser.add_argument('--phase', choices=['all', 'cpp', 'extended', 'registered', 'native'], default='all')
    parser.add_argument('--registered-timeout', type=float, default=600)
    args = parser.parse_args()
    if args.registered_timeout <= 0:
        parser.error('--registered-timeout must be positive')
    root = Path(__file__).resolve().parents[1]
    build, out, dependency = map(lambda x: Path(x).resolve(), (args.build_dir, args.output, args.dependency))
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PYTHONUTF8='1', PYTHONDONTWRITEBYTECODE='1')
    def git(*argv, cwd=root):
        return subprocess.check_output(['git', *argv], cwd=cwd).decode('utf-8', errors='replace').strip()
    executables = sorted(set((build/'tests/Release').glob('*.exe')) | set((build/'Release').glob('*.exe')))
    source_files = [p for folder in ['src', 'include', 'tests', 'tools', 'web', 'cmake']
                    for p in (root/folder).rglob('*') if p.is_file() and p.suffix in
                    {'.cpp', '.hpp', '.h', '.inc', '.cmake', '.py', '.mjs', '.js', '.css', '.html', '.json'}]
    source_files += [root/'CMakeLists.txt', root/'tests/CMakeLists.txt']
    dependency_status = git('status', '--porcelain', cwd=dependency)
    if dependency_status:
        raise RuntimeError('Dependency worktree must be clean for reproducible evidence.')
    manifest = {'dependency_head': git('rev-parse', 'HEAD', cwd=dependency),
                'dependency_status': dependency_status,
                'project_head': git('rev-parse', 'HEAD'),
                'executables': {str(p.relative_to(build)): digest(p) for p in executables},
                'source_sha256': {str(p.relative_to(root)): digest(p) for p in sorted(set(source_files))}}
    manifest_file = out/'manifest.json'
    if manifest_file.exists() and json.loads(manifest_file.read_text()) != manifest:
        raise RuntimeError('Build/source/dependency changed; use a new output directory.')
    manifest_file.write_text(json.dumps(manifest, indent=2)+'\n')
    (out/'environment.json').write_text(json.dumps({
        'platform': platform.platform(), 'python': sys.version, 'build_dir': str(build),
        'dependency': str(dependency), 'registered_outer_limit_s': args.registered_timeout,
        'solver_environment': {k:v for k,v in env.items() if k.startswith(('MIPSOLVERS_', 'HACDCPF_', 'OMP_', 'MKL_'))},
        'scope': 'Serial process wall time, including setup and assertions. Numerical validity stays in each underlying result.'}, indent=2)+'\n')
    report_file = out/'execution.json'
    rows = json.loads(report_file.read_text()) if report_file.exists() else []
    def run(name, argv, timeout=300, cwd=root, skip_code=None, declared_timeout=None):
        if any(x['name'] == name for x in rows):
            return
        argv = list(map(str, argv))
        row = execute(argv, cwd, out/(name+'.log'), timeout, env)
        row.update(name=name, timeout_s=timeout, declared_timeout_s=declared_timeout,
                   executable_sha256=digest(argv[0]) if Path(argv[0]).is_file() else None)
        if skip_code is not None and row['exit_code'] == skip_code:
            row['status'] = 'skipped'
        rows.append(row)
        report_file.write_text(json.dumps(rows, indent=2)+'\n')
        print(name, row['status'], round(row['wall_seconds'], 3), flush=True)

    def record_unavailable(name, reason):
        if any(x['name'] == name for x in rows):
            return
        rows.append({'name': name, 'status': 'unavailable', 'reason': reason})
        report_file.write_text(json.dumps(rows, indent=2)+'\n')
        print(name, 'unavailable', reason, flush=True)

    if args.phase in ['all', 'cpp']:
        run('cpp-suites', [sys.executable, root/'tools/module_performance_audit.py',
            '--build-dir', build, '--output', out/'cpp', '--repeats', 3,
            '--timeout', 300, '--resume'], 7200)
    if args.phase in ['all', 'extended']:
        exe = build/'Release'
        test = build/'tests/Release'
        for repeat in range(3):
            run(f'dc-short-{repeat}', [exe/'dc_short_circuit_benchmark.exe'])
        run('sppt', [exe/'sppt_benchmark.exe', out/'sppt', 5, root/'external_data/matpower'])
        run('fmea', [exe/'pf_doc_benchmark.exe', 'relfmea',
            *[root/f'external_data/matpower/case{x}.m' for x in ['33bw', '69', '85', '141']]])
        # These historical programs expect ./data/case*.m; reuse the verified junction.
        pf_cwd = out/'pf-doc-cwd'
        if not (pf_cwd/'data/case9.m').is_file():
            raise RuntimeError('pf_doc_benchmark requires a data junction to external_data/matpower')
        for mode in ['dyn', 'dyn2']:
            run('dynamics-'+mode, [exe/'pf_doc_benchmark.exe', mode], cwd=pf_cwd)
        for name, target, filter_, timeout in [
            ('ipopt-2000-bounded', 'test_opf_solver_backends', 'case2000 AC/DC Ipopt bounded performance benchmark', 300),
            ('ipopt-parameter-matrix', 'test_opf_solver_backends', 'Ipopt OPF parameter stability matrix', 600),
            ('rpo-300-seeded', 'test_reactive_power_opt', 'case300 RPO returns complete display vectors from a seeded solve', 300)]:
            run(name, [test/(target+'.exe'), filter_, '-s'], timeout)
        for case in ['case9', 'case30', 'case118', 'case300', 'case2869pegase']:
            for mode in ['power-flow', 'prepared-power-flow', 'dc-opf-highs']:
                run(f'opf-{case}-{mode}', [exe/'opf_numerical_benchmark.exe', root/f'external_data/matpower/{case}.m',
                    '--mode', mode, '--warmups', 1, '--repeats', 3, '--max-iterations', 300])
        for case in ['example', '118', '2000']:
            run('market-'+case, [test/'run_southern_market_benchmark.exe', case,
                out/f'market-{case}.json', 30, 'compact', 'auto', 'highs'], 180)
        for name, script in [('harmonics-matrix', 'run_cross_engine_matrix.py'),
                             ('harmonics-device', 'compare_device_opendss.py')]:
            run(name, [sys.executable, root/'tools/harmonics_validation'/script,
                '--cpp-bin', test/'validate_harmonics_xref.exe', '--out', out/(name+'.json')])
        run('time-series-external', [sys.executable, root/'tools/time_series_validation/run_cross_engine_matrix.py',
            '--cpp-bin', test/'validate_time_series_xref.exe', '--out', out/'time-series.json',
            '--markdown', out/'time-series.md'], 600)
        run('api-profile', [sys.executable, root/'tools/module_api_performance.py',
            '--server', test/'run_gui_server.exe', '--output', out/'api'], 600)
        opf_driver = root/'tools/opf_api_performance.py'
        if opf_driver.is_file():
            run('opf-api-matrix', [sys.executable, opf_driver,
                '--server', test/'run_gui_server.exe', '--output', out/'opf-api'], 1800)
        else:
            record_unavailable(
                'opf-api-matrix',
                'tools/opf_api_performance.py is not installed in this checkout',
            )
        run('pf-api-matrix', [sys.executable, root/'tools/power_flow_performance.py',
            '--server', test/'run_gui_server.exe', '--case', 'case2000_acdc', '--case', 'case_SyntheticUSA.m',
            '--backend', 'eigen', '--backend', 'klu', '--repeat', 3, '--require-profile',
            '--output', out/'pf-api'], 600)
    if args.phase in ['all', 'registered']:
        inventory = json.loads(subprocess.check_output(['ctest', '--test-dir', str(build),
            '-C', 'Release', '--show-only=json-v1'], encoding='utf-8'))
        (out/'ctest-inventory.json').write_text(json.dumps(inventory, indent=2)+'\n')
        for item in inventory['tests']:
            argv = item.get('command', [])[:]
            if not any(str(x).endswith(('.py', '.mjs')) for x in argv):
                continue
            # CTest may cache the system Python without oracle dependencies;
            # use the explicitly selected audit interpreter for Python entries.
            if any(str(x).endswith('.py') for x in argv):
                argv[0] = sys.executable
            props = {x['name']:x['value'] for x in item.get('properties', [])}
            name = item['name']
            # Preserve prior oracle evidence when registered commands use fixed outputs.
            for flag, suffix in [('--out', '.json'), ('--markdown', '.md')]:
                if flag in argv:
                    argv[argv.index(flag)+1] = str(out/(name+suffix))
            run('registered-'+name, argv, args.registered_timeout,
                cwd=props.get('WORKING_DIRECTORY', root), skip_code=props.get('SKIP_RETURN_CODE'),
                declared_timeout=props.get('TIMEOUT'))

    if args.phase in ['all', 'native']:
        inventory = json.loads(subprocess.check_output(['ctest', '--test-dir', str(build),
            '-C', 'Release', '--show-only=json-v1'], encoding='utf-8'))
        suite_executables = {p.resolve() for p in (build/'tests/Release').glob('*test*.exe')}
        for item in inventory['tests']:
            argv = item.get('command', [])[:]
            if not argv or any(str(x).endswith(('.py', '.mjs')) for x in argv):
                continue
            if Path(argv[0]).resolve() in suite_executables:
                continue
            name = item['name']
            props = {x['name']:x['value'] for x in item.get('properties', [])}
            for flag, suffix in [('--output', '.json'), ('--csv', '.csv')]:
                if flag in argv:
                    argv[argv.index(flag)+1] = str(out/(name+suffix))
            if name == 'transient_native_disturbance_matrix':
                argv[1] = str(out/(name+'.json'))
            run('native-'+name, argv, args.registered_timeout,
                cwd=props.get('WORKING_DIRECTORY', root), skip_code=props.get('SKIP_RETURN_CODE'),
                declared_timeout=props.get('TIMEOUT'))


if __name__ == '__main__':
    main()
