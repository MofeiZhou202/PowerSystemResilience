"""Fail-closed R3 evidence gate; see the fixed 2026-09-13 derivation."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from windows_lp_stability import CASES, distribution, parse_telemetry


def load(path):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result
    return json.loads(Path(path).read_text(encoding='utf-8-sig'),
                      object_pairs_hook=pairs)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def accuracy(data, cases, threads, solver):
    require(data['provenance']['compiler'] == 'msvc 1944' and
            data['provenance']['build_type'] == 'Release/NDEBUG', 'wrong measured build profile')
    cfg = data['configuration']
    require(cfg['mkl_max_threads'] == threads and cfg['repeats'] == 1,
            'thread/repeat configuration mismatch')
    require(cfg['normalized_feasibility_tolerance'] == 1e-7 and
            cfg['objective_tolerance'] == 1e-5, 'accuracy tolerances changed')
    require(cfg['max_iterations'] == 100000, 'iteration budget changed')
    require(cfg['time_limit_sec'] == (15.0 if solver.startswith('Native-IPM') else 1e9),
            'process protocol time budget changed')
    rows = data['runs']
    require(len(rows) == len(cases) and {r['case'] for r in rows} == set(cases),
            'missing or duplicate original-model runs')
    for row in rows:
        require(row['accurate'] is True and row['success'] is True and
                row['available'] is True and row['repeat'] == 1,
                f"{row['case']}: inaccurate original model")
        require(row['solver'] == solver, 'unexpected solver')
        require(number(row['runtime_ms']) and row['runtime_ms'] > 0,
                'invalid run time')
        require(type(row['iterations']) is int and row['iterations'] >= 0,
                'invalid iteration count')
        require(number(row['normalized_primal_violation']) and
                0 <= row['normalized_primal_violation'] <= 1e-7 and
                number(row['objective_rel_error']) and
                0 <= row['objective_rel_error'] <= 1e-5, 'original-model audit failed')
        if solver.startswith('Native-IPM'):
            for key in ('relative_primal_residual', 'relative_dual_residual', 'relative_gap'):
                require(number(row[key]) and 0 <= row[key] <= 1e-7,
                        f'original KKT audit failed: {key}')
        require(number(row['objective']) and number(row['reference_objective']),
                'missing original objective')
    return {r['case']: r for r in rows}


def process_record(path, binary_hash, threads, seen):
    record = load(path)
    require(record['exit_code'] == 0 and record['threads'] == threads,
            'failed process or wrong budget')
    require(record['binary_sha256'] == binary_hash, 'mixed binaries in evidence')
    require(sha(record['command'][0]) == binary_hash, 'recorded command ran another binary')
    stem = Path(str(path).removesuffix('.process.json'))
    for suffix in ('.json', '.stdout.log', '.stderr.log'):
        require(sha(stem.with_suffix(suffix)) == record['artifact_sha256'][suffix],
                'raw evidence digest mismatch')
    require(type(record['pid']) is int and record['pid'] > 0 and
            type(record['started_ns']) is int and type(record['finished_ns']) is int and
            record['finished_ns'] > record['started_ns'], 'invalid process identity/interval')
    identity = (record['pid'], record['started_ns'])
    require(identity not in seen, 'reused process evidence')
    seen.add(identity)
    env = record['environment']
    require(env['MKL_NUM_THREADS'] == str(threads) and env['OMP_NUM_THREADS'] == '1'
            and env['MKL_DYNAMIC'] == 'FALSE' and env.get('MKL_CBWR') is None,
            'noncanonical resource environment')
    return record


def build_profile(folder):
    cache = (folder/'CMakeCache.txt').read_text(encoding='utf-8')
    for key, value in dict(MIPSOLVERS_MKL_THREADING='INTEL', MIPSOLVERS_ENABLE_IPO='OFF',
                           MIPSOLVERS_ENABLE_NATIVE_ARCH='OFF', MIPSOLVERS_PGO_MODE='OFF',
                           MIPSOLVERS_USE_MKL='ON').items():
        require(re.search(rf'^{key}:[^=]+={value}$', cache, re.M), f'wrong build flag {key}')
    compilers = list((folder/'CMakeFiles').glob('*/CMakeCXXCompiler.cmake'))
    require(len(compilers) == 1, 'ambiguous compiler profile')
    compiler = compilers[0].read_text()
    require('set(CMAKE_CXX_COMPILER_ID "MSVC")' in compiler and
            'set(CMAKE_CXX_COMPILER_VERSION "19.44.' in compiler, 'requires MSVC 19.44')
    ns = {'m': 'http://schemas.microsoft.com/developer/msbuild/2003'}
    project = ET.parse(folder/'mipsolvers.vcxproj')
    release = [g for g in project.findall('m:ItemDefinitionGroup', ns)
               if "'Release|x64'" in g.attrib.get('Condition', '')]
    require(len(release) == 1, 'missing Release/x64 compiler options')
    for key, value in dict(Optimization='MaxSpeed', InlineFunctionExpansion='AnySuitable',
                           IntrinsicFunctions='true', FloatingPointModel='Fast').items():
        require(release[0].findtext(f'm:ClCompile/m:{key}', namespaces=ns) == value,
                f'requires /O2 /Ob2 /Oi /fp:fast: {key}')
    source = re.search(r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$', cache, re.M)
    require(source is not None, 'missing source checkout identity')
    return Path(source.group(1))


def baseline_probe(source, contract):
    # Source lineage only; actual cross-binary trace comparison remains mandatory.
    def git(*args):
        return subprocess.check_output(['git', '-C', str(source), *args]).decode('utf-8').replace('\r\n', '\n')
    require(git('rev-parse', 'HEAD').strip() == contract['code_baseline_commit'], 'wrong actual baseline checkout')
    prefix = 'src/engine/kernel/lp_kernel/native_dual/'
    allowed = {prefix+'solver.cpp', prefix+'primal.cpp'}
    require(set(git('diff', '--name-only', 'HEAD').splitlines()) <= allowed,
            'baseline contains changes beyond the common trace probe')
    for file in ('solver.cpp', 'primal.cpp'):
        original = git('show', 'HEAD:'+prefix+file)
        expected = original.replace('#include "primal.hpp"', '#include "primal.hpp"\n#include "pivot_trace.hpp"')
        if file == 'solver.cpp':
            expected = expected.replace('    ++statistics.iterations;\n    if (state.phase == Phase::DualOne)',
                '    ++statistics.iterations;\n'
                '    emit_pivot_trace(static_cast<int>(state.phase), leaving.row, entering.col,\n'
                '                     leaving_col, column_pivot, primal_step, dual_step);\n'
                '    if (state.phase == Phase::DualOne)')
        else:
            expected = expected.replace('      ++statistics.bound_flips;\n',
                '      ++statistics.bound_flips;\n'
                '      emit_pivot_trace(static_cast<int>(state.phase), -1, entering.col,\n'
                '                       entering.col, 0.0, primal_delta, 0.0);\n')
            expected = expected.replace('    state.objective = candidate_objective;\n',
                '    state.objective = candidate_objective;\n'
                '    emit_pivot_trace(static_cast<int>(state.phase), leaving.row, entering.col,\n'
                '                     leaving_col, pivot, leaving.step, dual_step);\n')
        require((source/prefix/file).read_text(encoding='utf-8') == expected,
                'baseline probe transcription differs from fixed observer')
    for folder in (source, ROOT):
        require(sha(folder/prefix/'pivot_trace.hpp') == contract['pivot_probe_sha256'],
                'pivot probe identity changed')


def ordered(intervals):
    for before, after in zip(intervals, intervals[1:]):
        require(before['finished_ns'] <= after['started_ns'],
                'blocks not interleaved in independent nonoverlapping processes')


def telemetry(log, rows, threads, experimental):
    factors, recoveries = parse_telemetry(log, threads)
    require(all(f['case'] in CASES for f in factors), 'unknown factor case')
    transitions = [(f['case'], f['entry_reason'], f['source_iterations'])
                   for f in factors if f['entry_reason'] != 'primary']
    require(len(set(transitions)) == len(transitions), 'ambiguous recovery/factor join')
    for case in CASES:
        local = [f for f in factors if f['case'] == case]
        require(local and local[0]['entry_reason'] == 'primary', 'missing primary factor')
        total = sum(f['variant_ms'] for f in local)
        # Nested timers differ by logging/outer audit; containment, not equality.
        # Exclusive partition tolerance is the historical 0.001 ms print bound.
        require(0 < total <= rows[case]['runtime_ms'] + .001,
                'factor intervals exceed enclosing solve')
        for f in local:
            require(0 <= f['retry_ms'] <= f['variant_ms'] + .001,
                    'retry is not an enclosing-variant subset')
            for key in ('assembly_calls', 'symbolic_calls', 'numeric_calls', 'retries'):
                require(type(f[key]) is int and f[key] >= 0, 'invalid factor call count')
    for rec in recoveries:
        local = [f for f in factors if f['case'] == rec['case']]
        match = [f for f in local if f['entry_reason'] == rec['entry_reason'] and
                 f['source_iterations'] == rec['source_iterations']]
        require(len(match) == 1 and local.index(match[0]) > 0, 'missing recovery predecessor')
        source = local[local.index(match[0]) - 1]
        require(rec['source_runtime_ms'] <= source['variant_ms'] + .001 and
                rec['recovery_runtime_ms'] <= match[0]['variant_ms'] + .001,
                'transition timing exceeds factor intervals')
        require(rec['seed_size'] == rec['cols'] and rec['recovery_success'] is True,
                'invalid seed/result')
        require(rec['recovery_mkl_threads'] == (min(threads, 2) if experimental else threads),
                'unapproved resource scope on measured path')
    return recoveries


def stability(folder, binary_hash, contract, experimental, seen):
    samples = {str(t): {c: [] for c in CASES} for t in (2, 4)}
    transitions = {str(t): [] for t in (2, 4)}
    intervals = []
    expected = {f'block-{b:02}-t{t}.json' for b in range(1, 21) for t in (2, 4)}
    actual = {p.name for p in folder.glob('block-*-t*.json') if p.name.count('.') == 1}
    require(actual == expected, 'R3 requires exactly 20 blocks per 2T/4T arm')
    for block in range(1, 21):
        for t in ((2, 4) if block % 2 else (4, 2)):
            stem = folder / f'block-{block:02}-t{t}'
            meta = process_record(stem.with_suffix('.process.json'), binary_hash, t, seen)
            require(meta['timing'] is True, 'R3 timing diagnostics disabled')
            require(meta['environment']['MIPSOLVERS_LP_FACTOR_TIMING'] == '1', 'factor timing environment mismatch')
            intervals.append(meta)
            rows = accuracy(load(stem.with_suffix('.json')), CASES, t,
                            'Native-IPM[centrality-step,direct]')
            for c, count in (('dfl001', 44), ('maros-r7', 21)):
                require(rows[c]['iterations'] == count, f'{c}: changed pivot trajectory')
            for c in CASES:
                samples[str(t)][c].append(rows[c]['runtime_ms'])
            transitions[str(t)].extend(telemetry(stem.with_suffix('.stderr.log'), rows, t, experimental))
    ordered(intervals)
    return samples, transitions


def compare(samples, references, contract, failures, measurements, prefix):
    for t, cases in samples.items():
        groups = {c: distribution(v) for c, v in cases.items()}
        groups['aggregate'] = distribution([sum(cases[c][i] for c in cases)
                                             for i in range(20)])
        for c, actual in groups.items():
            limits = contract['aggregate_limits' if c == 'aggregate' else 'case_limits']
            ref = references[t]['aggregate'] if c == 'aggregate' else references[t]['cases'][c]
            for metric in ('median', 'p95'):
                ceiling = ref[metric] * limits[metric + '_ratio']
                label = f'{prefix}.{t}.{c}.{metric}'
                measurements[label] = dict(measured=actual[metric], ceiling=ceiling,
                                           reference=ref[metric])
                if actual[metric] > ceiling:
                    failures.append(f'{label}: {actual[metric]:.3f} > {ceiling:.3f} ms')


def greenbea(samples, transitions, reference, contract, failures, measurements):
    limits = contract['greenbea']
    two, four = samples['2']['greenbea'], samples['4']['greenbea']
    paired = statistics.median([b-a for a, b in zip(two, four)])
    m2, m4 = statistics.median(two), statistics.median(four)
    measurements['greenbea.paired_delta_ms'] = paired
    low, high = limits['median_4t_interval_ms']
    if paired >= 0 or m4 >= m2:
        failures.append('greenbea paired or marginal median has wrong direction')
    if not low <= m4 <= high:
        failures.append(f'greenbea 4T median {m4:.3f} outside fixed [{low},{high}] ms')
    if distribution(four)['p95'] > limits['p95_4t_over_2t'] * distribution(two)['p95']:
        failures.append('greenbea 4T P95 regression')
    for t in ('2', '4'):
        records = [r for r in transitions[t] if r['case'] == 'greenbea']
        require(len(records) == 20, 'missing greenbea recovery state')
        states = {(r['source_iterations'], r['seed_hash'], r['seed_norm_inf'],
                   r['seed_norm_2'], r['seed_objective'], r['adaptive_backend_sequence'])
                  for r in records}
        if len(states) != 1:
            failures.append(f'{t}T: source/seed/backend state varies within arm')
        for metric in ('recovery_runtime_ms', 'recovery_iterations'):
            measured = distribution([r[metric] for r in records])['p95']
            ceiling = (reference[t]['recovery_transitions']['greenbea'][metric]['p95'] *
                       limits['recovery_p95_ratio'])
            measurements[f'{t}.greenbea.{metric}.p95'] = dict(measured=measured, ceiling=ceiling)
            if measured > ceiling:
                failures.append(f'{t}T: {metric} P95 {measured:.3f} exceeds {ceiling:.3f}')
        cost = statistics.median([r['seed_capture_ms'] + r['record_prepare_ms'] for r in records])
        if cost >= limits['telemetry_median_upper_ms']:
            failures.append(f'{t}T: telemetry preparation budget failed')


def broad(folder, hashes, contract, seen):
    corpus = list(contract['corpus'])
    samples = {arm: {str(t): {c: [] for c in corpus} for t in (2, 4)}
               for arm in ('baseline', 'candidate')}
    intervals = []
    expected = {f'broad-{b:02}-t{t}-{arm}.json' for b in range(1, 21)
                for t in (2, 4) for arm in ('baseline', 'candidate')}
    for b in range(1, 21):
        for t in ((2, 4) if b % 2 else (4, 2)):
            for arm in (('baseline', 'candidate') if b % 2 else ('candidate', 'baseline')):
                stem = folder / f'broad-{b:02}-t{t}-{arm}'
                meta = process_record(stem.with_suffix('.process.json'), hashes[arm], t, seen)
                require(meta['timing'] is False, 'broad corpus must disable timing diagnostics')
                require(meta['environment']['MIPSOLVERS_LP_FACTOR_TIMING'] == '0', 'broad diagnostic environment mismatch')
                parse_telemetry(stem.with_suffix('.stderr.log'), t, False)
                intervals.append(meta)
                rows = accuracy(load(stem.with_suffix('.json')), corpus, t,
                                'Native-IPM[centrality-step,direct]')
                for case, count in (('dfl001', 44), ('maros-r7', 21)):
                    require(rows[case]['iterations'] == count, 'broad corpus fixed-iteration failure')
                for c in corpus:
                    samples[arm][str(t)][c].append(rows[c]['runtime_ms'])
    require({p.name for p in folder.glob('broad-*.json') if p.name.count('.') == 1} == expected,
            'broad corpus requires exactly 20 paired blocks per thread budget')
    ordered(intervals)
    references = {t: dict(cases={c: distribution(v) for c, v in cases.items()},
                           aggregate=distribution([sum(cases[c][i] for c in cases) for i in range(20)]))
                  for t, cases in samples['baseline'].items()}
    return samples['candidate'], references


def traces(folder, hashes, contract, seen):
    counts = {}
    pattern = re.compile(rb'LP-PIVOT 2 -?\d+ -?\d+ \d+ \d+ [0-9a-f]{16} [0-9a-f]{16} [0-9a-f]{16}')
    for t in (2, 4):
        for case in contract['corpus']:
            payloads = []
            for arm in ('baseline', 'candidate'):
                stem = folder / f'trace-t{t}-{case}-{arm}'
                meta = process_record(stem.with_suffix('.process.json'), hashes[arm], t, seen)
                require(meta['environment']['MIPSOLVERS_LP_PIVOT_TRACE'] == '1',
                        'trace not generated by binary')
                accuracy(load(stem.with_suffix('.json')), [case], t, 'Native-DualSimplex[direct]')
                lines = [line for line in stem.with_suffix('.stderr.log').read_bytes().splitlines(keepends=True)
                         if line.startswith(b'LP-PIVOT')]
                require(lines and all(pattern.fullmatch(line.rstrip(b'\r\n')) for line in lines),
                        f'{arm}/{case}: empty or malformed actual pivot trace')
                payloads.append(b''.join(lines))
            require(payloads[0] == payloads[1], f'{case}/{t}T: actual pivot bytes differ')
            counts[f'{case}/{t}T'] = dict(records=payloads[0].count(b'\n'),
                                        sha256=hashlib.sha256(payloads[0]).hexdigest())
    return counts


def validation(manifest):
    evidence = manifest['validation']
    require(evidence['candidate_sha256'] == manifest['binary_sha256s']['candidate'],
            'tests belong to another candidate')
    for name in ('ctest', 'test_dual_simplex', 'test_engine_api', 'test_milp_solver',
                 'native_kernel_comparison', 'py_compile', 'gate_tests', 'doc_anchors',
                 'diff_check', 'added_markers'):
        rec = evidence['checks'][name]
        require(rec['exit_code'] == 0 and rec['command'] and
                sha(rec['log']) == rec['log_sha256'], f'{name}: missing/failed validation')
        if name == 'ctest':
            log = Path(rec['log']).read_text(encoding='utf-8', errors='replace')
            require(re.search(r'100% tests passed', log) and
                    '-j 1' in rec['command'] and '-C Release' in rec['command'] and
                    ' -R ' not in rec['command'], 'full serial Release CTest required')


def check(result_path, contract_path):
    failures, measurements = [], {}
    def section(name, action):
        try:
            return action()
        except (OSError, ValueError, KeyError, TypeError, IndexError, RuntimeError,
                subprocess.SubprocessError, ET.ParseError) as error:
            failures.append(f'{name}: {error}')
            return None

    def validate():
        contract = load(contract_path)
        require(contract == load(ROOT / 'benchmark/windows_lp_r3_contract.json'),
                'R3 contract differs from fixed repository contract')
        manifest = load(result_path)
        require(isinstance(manifest, dict) and manifest.get('schema_version') == 2,
                'summary-only/schema-v1 evidence cannot approve R3')
        require(manifest['protocol'] == contract['protocol'], 'wrong protocol')
        require(manifest['code_baseline_commit'] == contract['code_baseline_commit'],
                'code baseline replaced')
        require(type(manifest['experimental_cap']) is bool, 'missing explicit experimental-policy identity')
        for case, digest in contract['corpus'].items():
            require(sha(ROOT / 'tests/data/netlib' / f'{case}.mps') == digest, 'fixed corpus changed')
        references = {}
        for key, ref in contract['references'].items():
            path = ROOT / ref['path']
            require(sha(path) == ref['sha256'], f'{key} baseline replaced')
            references[key] = load(path)
        hashes = manifest['binary_sha256s']
        require(hashes['baseline'] != hashes['candidate'], 'same binary supplied for both arms')
        for arm in ('baseline', 'candidate'):
            require(sha(manifest['binaries'][arm]) == hashes[arm], 'binary provenance mismatch')
            source = build_profile(Path(manifest['build_dirs'][arm]))
            if arm == 'baseline':
                baseline_probe(source, contract)
        section('minimum validation', lambda: validation(manifest))
        seen = set()
        measured = section('stability', lambda: stability(Path(manifest['stability_dir']),
                           hashes['candidate'], contract, manifest['experimental_cap'], seen))
        if measured:
            samples, transitions = measured
            count = sum(len(values) for cases in samples.values() for values in cases.values())
            measurements['stability_accuracy'] = dict(accurate=count, runs=count)
            measurements['fixed_iterations'] = dict(dfl001=44, **{'maros-r7': 21})
            compare(samples, references['performance'], contract, failures, measurements, 'historical')
            section('greenbea', lambda: greenbea(samples, transitions, references['recovery'],
                                                contract, failures, measurements))
        measured_broad = section('broad corpus', lambda: broad(Path(manifest['broad_dir']), hashes, contract, seen))
        if measured_broad:
            count = 2 * sum(len(values) for cases in measured_broad[0].values() for values in cases.values())
            measurements['broad_accuracy'] = dict(accurate=count, runs=count,
                                                   cases=len(contract['corpus']))
            compare(*measured_broad, contract, failures, measurements, 'broad')
        measurements['pivot_traces'] = section('pivot traces', lambda: traces(
            Path(manifest['trace_dir']), hashes, contract, seen))

    section('schema-v2 evidence', validate)
    report = dict(schema_version=2, r3_release_approved=not failures,
                  failures=failures, measured_vs_contract=measurements)
    output = Path(result_path).with_suffix('.gate.json')
    output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return int(bool(failures))
