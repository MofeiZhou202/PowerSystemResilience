"""Run module suites sequentially, recording process time and peak working set.

Suite time includes assertions and setup; never label it pure solver time.
Timeouts, skipped cases, and unavailable executables are retained as evidence.
psutil enables process-tree memory sampling but is not required for execution.
Does not alter live GUI sessions.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import signal
import statistics
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

try:
    import psutil
except ImportError:
    psutil = None


REPRESENTATIVES = {
    'analysis': 'test_hosting_capacity test_multidimensional_weak_link test_counterfactual_planning',
    'api': 'test_solver_capabilities test_verified_certificate',
    'carbon_analysis': 'test_carbonflow_tracing test_carbonflow_dynamic_storage test_carbonflow_case_validation',
    'dynamics': 'test_transient_dynamics test_dynamic_model_catalog',
    'ev_power_traffic': 'test_ev_power_traffic test_ctm_ltm_comparison test_ev_power_traffic_formulations_e_f_g_h',
    'graph': 'test_graph test_graph_kron test_graph_roundtrip test_topology_crossval',
    'harmonics_power_flow': 'test_harmonics_power_flow',
    'integrated_energy': 'test_integrated_energy_campus',
    'io': 'test_io_json test_io_matpower test_io_cim test_io_etap test_bpa_dsp_compare',
    'market': 'hacdcpf_test_market_simulation test_southern_market test_market_forecast',
    'model': 'test_model_semantics_contract test_result_attribution test_component_models_math_audit',
    'network_reconfiguration': 'test_reconfig_options test_distribution_pipeline test_device_flows',
    'optimal_power_flow': 'test_acopf_dcopf_crossval test_opf_solver_backends test_reactive_power_opt test_three_phase_hybrid_opf',
    'power_flow': 'test_hacdcpf test_advanced_pf test_three_phase_hybrid_pf',
    'power_models': 'test_scaling_regression',
    'reliability': 'test_reliability_resolver test_three_stage_reliability test_intelligent_cyber_physical_reliability',
    'resilience': 'test_resilience_assessment',
    'scenario_generation': 'test_scenario_generation test_typhoon_traffic_impact test_scenario_bundle_schema',
    'server': 'test_edition_profile',
    'short_circuit': 'test_short_circuit_crossval test_dc_short_circuit test_short_circuit_iec60909_4',
    'sppt': 'test_sppt_benchmark test_sppt_metamorphic test_sppt_guard',
    'time_series': 'test_multiscale_comprehensive test_uc_storage_efficiency test_uc_energy_router',
    'validation': 'test_validation',
}


def document_inventory(root):
    modules = {}
    for directory in sorted((root / 'docs/modules').iterdir()):
        if not directory.is_dir():
            continue
        refs, timing = set(), []
        for path in sorted(directory.rglob('*')):
            if path.suffix not in ('.md', '.tex'):
                continue
            for number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
                plain = line.replace('\\_', '_')
                refs.update(re.findall(r'\b(?:test_[a-z0-9_]+|[a-z0-9_]+_cross_validation)\b', plain))
                if re.search(r'(?:[0-9].*(?:\\mathrm\{(?:ms|s)\}|\bms\b|\bseconds\b)|耗时|墙钟|wall.clock|wall time)', plain, re.I):
                    timing.append({'file': path.relative_to(root).as_posix(), 'line': number, 'text': line})
        modules[directory.name] = {'document_test_references': sorted(refs), 'timing_references': timing,
                                   'representative_suites': REPRESENTATIVES.get(directory.name, '').split()}
    return modules


def test_executable(build, name):
    """Resolve a test in CMake single- or multi-configuration layouts."""
    suffix = '.exe' if os.name == 'nt' else ''
    candidates = [
        build / 'tests' / 'Release' / (name + suffix),
        build / 'tests' / (name + suffix),
        build / 'Release' / (name + suffix),
        build / (name + suffix),
    ]
    return next((path for path in candidates if path.is_file()), candidates[0])


def discover_test_suites(build):
    suffix = '*.exe' if os.name == 'nt' else '*'
    directories = [build / 'tests' / 'Release', build / 'tests']
    return {
        path.stem
        for directory in directories
        for path in directory.glob(suffix)
        if path.is_file()
        and 'test' in path.stem
        and (os.name == 'nt' or os.access(path, os.X_OK))
    }


def _kill_process_tree(process):
    if process.poll() is not None:
        return
    if os.name == 'nt':
        subprocess.run(
            ['taskkill', '/PID', str(process.pid), '/T', '/F'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if process.poll() is None:
            process.kill()
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def execute(argv, cwd, output, timeout, env):
    output.parent.mkdir(parents=True, exist_ok=True)
    start, peak = time.perf_counter(), 0
    with output.open('w', encoding='utf-8') as log:
        popen_options = {
            'cwd': cwd,
            'env': env,
            'stdout': log,
            'stderr': subprocess.STDOUT,
        }
        if os.name == 'nt':
            popen_options['creationflags'] = (
                subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
            )
        else:
            popen_options['start_new_session'] = True
        process = subprocess.Popen(argv, **popen_options)
        watched = None
        if psutil is not None:
            try:
                watched = psutil.Process(process.pid)
            except psutil.Error:
                pass
        stop = threading.Event()

        def sample_memory():
            nonlocal peak
            while not stop.is_set():
                try:
                    members = [watched, *watched.children(recursive=True)]
                    resident = 0
                    for member in members:
                        try:
                            resident += member.memory_info().rss
                        except psutil.Error:
                            pass
                    peak = max(peak, resident)
                except psutil.Error:
                    pass
                stop.wait(.02)
        monitor = None
        if watched is not None:
            monitor = threading.Thread(target=sample_memory, daemon=True)
            monitor.start()
        timed_out = False
        try:
            code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            _kill_process_tree(process)
            code = process.wait()
        finally:
            elapsed = time.perf_counter() - start
            stop.set()
            if monitor is not None:
                monitor.join()
    return {'argv': argv, 'wall_seconds': elapsed, 'exit_code': code,
            'status': 'timeout' if timed_out else 'passed' if code == 0 else 'failed',
            'observed_peak_working_set_bytes': peak or None,
            'memory_sampling': 'psutil_process_tree' if watched is not None else 'unavailable',
            'stdout': str(output)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', required=True)
    parser.add_argument('--output', default='build/module-performance/audit')
    parser.add_argument('--inventory-only', action='store_true')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=180)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--suite', action='append', help='Limit execution to named suites (repeatable)')
    args = parser.parse_args()
    if args.repeats < 1 or args.timeout <= 0:
        parser.error('--repeats and --timeout must be positive')
    root = Path(__file__).resolve().parents[1]
    build, output = Path(args.build_dir).resolve(), Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    modules = document_inventory(root)
    (output / 'document-inventory.json').write_text(json.dumps(modules, ensure_ascii=False, indent=2), encoding='utf-8')
    if args.inventory_only:
        print(f'{len(modules)} modules inventoried')
        return
    report_path = output / 'report.json'
    report = json.loads(report_path.read_text(encoding='utf-8')) if args.resume and report_path.exists() else {
        'platform': platform.platform(), 'cpu': platform.processor(), 'build_dir': str(build),
        'scope': 'Sequential suite process wall time includes setup/assertions; sampled process tree working set, not solver-only latency.',
        'modules': modules, 'runs': []}
    representatives = {name for item in modules.values() for name in item['representative_suites']}
    plain_suites = set(re.findall(r'hacdcpf_add_plain_test\((\w+)',
        (root / 'tests/CMakeLists.txt').read_text(encoding='utf-8')))
    all_suites = sorted(discover_test_suites(build) | representatives)
    if args.suite:
        all_suites = sorted(set(args.suite))
    env = os.environ.copy()
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    for name in all_suites:
        exe = test_executable(build, name)
        if not exe.exists():
            if not any(x.get('suite') == name for x in report['runs']):
                report['runs'].append({'suite': name, 'status': 'unavailable', 'reason': str(exe)})
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
            continue
        digest = hashlib.sha256(exe.read_bytes()).hexdigest()
        for repeat in range(args.repeats if name in representatives else 1):
            existing = [x for x in report['runs'] if x.get('suite') == name and x.get('repeat') == repeat]
            if existing:
                if existing[0].get('executable_sha256') != digest:
                    raise RuntimeError(f'Cannot resume changed executable: {name}')
                if existing[0]['status'] != 'passed':
                    break
                continue
            xml = output / f'{name}-{repeat}.xml'
            command = [str(exe)] if name in plain_suites else [str(exe), '--reporter', 'junit', '--out', str(xml), '--durations', 'yes']
            row = execute(command, root,
                          output / f'{name}-{repeat}.log', args.timeout, env)
            row.update(suite=name, repeat=repeat, executable_sha256=digest, junit=str(xml))
            if xml.exists() and row['status'] != 'timeout':
                try:
                    tree = ET.parse(xml)
                    row['cases'] = len(tree.findall('.//testcase'))
                    row['skipped'] = len(tree.findall('.//skipped'))
                    row['failure_count'] = len(tree.findall('.//failure')) + len(tree.findall('.//error'))
                    row['test_case_seconds'] = sum(float(case.get('time', 0)) for case in tree.findall('.//testcase'))
                    if row['cases'] and row['skipped'] == row['cases'] and row['failure_count'] == 0:
                        row['status'] = 'skipped'
                    elif row['failure_count'] and row['status'] == 'passed':
                        row['status'] = 'failed'
                except ET.ParseError:
                    row['status'] = 'invalid_test_report'
            elif name not in plain_suites and row['status'] == 'passed':
                row['status'] = 'missing_test_report'
            report['runs'].append(row)
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
            print(name, repeat, row['status'], round(row['wall_seconds'], 3), flush=True)
            if row['status'] != 'passed':
                break
    summary = {}
    for name in all_suites:
        rows = [x for x in report['runs'] if x.get('suite') == name]
        good = [x['wall_seconds'] for x in rows if x['status'] == 'passed']
        summary[name] = {'statuses': [x['status'] for x in rows], 'samples': len(good),
                         'median_seconds': statistics.median(good) if good else None}
    report['summary'] = summary
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
