"""Profile a generated PF server copy using the retained incremental Release build.

See docs/testing/module_code_audit.md, PF presentation profiling protocol.
Production sources and dependency guards are not modified.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import re
import shlex
import shutil
import socket
import statistics
import subprocess
import time
import urllib.request


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def instrument(source):
    start = source.index('  svr.Post("/api/session/pf",')
    end = source.index('  // ---- Session: topology analysis', start)
    route = source[start:end]

    def replace(old, new):
        nonlocal route
        if route.count(old) != 1:
            raise ValueError(f"Expected one PF anchor: {old!r}, got {route.count(old)}")
        route = route.replace(old, new)

    replace('      double presentation_ms = 0.0;', '''      double presentation_ms = 0.0;
      double geo_lifetime_ms = 0.0;
      size_t matched_attribution_rows = 0;
      size_t discarded_attribution_metrics = 0;
      struct RequestLifetime {
        httplib::Response& response;
        std::chrono::steady_clock::time_point started;
        std::chrono::steady_clock::time_point content_ready;
        bool ready{false};
        ~RequestLifetime() {
          if (!ready) return;
          const auto finished = std::chrono::steady_clock::now();
          response.set_header("X-HySim-Handler-Lifetime-Ms", std::to_string(
              std::chrono::duration<double, std::milli>(finished - started).count()));
          response.set_header("X-HySim-Handler-Cleanup-Ms", std::to_string(
              std::chrono::duration<double, std::milli>(finished - content_ready).count()));
        }
      } request_lifetime{res, request_started, request_started};
      json profile_stages = json::array();
      auto profile_tick = std::chrono::steady_clock::now();
      const auto profile_mark = [&](const char* name) {
        const auto now = std::chrono::steady_clock::now();
        profile_stages.push_back(json{{"name", name}, {"ms",
            std::chrono::duration<double, std::milli>(now - profile_tick).count()}});
        profile_tick = std::chrono::steady_clock::now();
      };
      struct ScanMetric { double ms{0.0}; size_t calls{0}; };
      std::array<ScanMetric, 3> scan_metrics{};
      const bool profile_scans = req.get_header_value("X-HySim-Profile-Scans") == "1";
      struct ScanTimer {
        ScanMetric* metric;
        std::chrono::steady_clock::time_point started;
        explicit ScanTimer(ScanMetric* value) : metric(value) {
          if (metric) started = std::chrono::steady_clock::now();
        }
        ~ScanTimer() {
          if (metric) {
            metric->ms += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
            ++metric->calls;
          }
        }
      };''')
    replace('        const auto presentation_started = std::chrono::steady_clock::now();',
            '''        struct GeoLifetime {
          double& elapsed;
          std::chrono::steady_clock::time_point started{std::chrono::steady_clock::now()};
          ~GeoLifetime() {
            elapsed += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
          }
        } geo_lifetime{geo_lifetime_ms};
        const auto presentation_started = std::chrono::steady_clock::now();
'''
            '        profile_tick = presentation_started;')
    anchors = [
        ('        // ── Compute solved per-generator P/Q', 'initial_aggregation_3w_recovery'),
        ('        // Build AC bus coordinate lookup', 'generator_balance_2w_recovery'),
        ('        out["geo_buses"] = geo_buses;\n        out["geo_ac_branches"]', 'geo_json_build'),
        ('        // Full post-power-flow component result list', 'geo_json_copy'),
        ('        // Reconstruct per-bus AC flow', 'component_helpers_voltage_indexes'),
        ('        // AC/DC bus rows.', 'device_injection_reassembly'),
        ('        std::unordered_map<int, double> ac_terminal_export_p =', 'terminal_flow_recovery'),
        ('        auto metric_detail =', 'source_balance_refresh'),
        ('        json balance_diag;', 'diagnostic_group_build'),
        ('        for (const auto& [root, members] : ac_balance_groups)', 'diagnostic_json_setup'),
        ('        balance_diag["ordinary_bad_count"]', 'diagnostic_scan_and_rows'),
        ('        out["power_balance_diagnostics"] = balance_diag;', 'diagnostic_finalize'),
        ('\t        const auto projection =', 'component_json_build'),
        ('\t        const auto attribution =', 'attribution_projection'),
        ('\t        const json attributed_rows =', 'attribution_apply'),
        ('\t        std::unordered_map<std::string, std::size_t> component_row_by_key;', 'attribution_json_build'),
        ('\t        out["component_results"] = std::move(component_results);', 'component_json_merge'),
        ('\t        presentation_ms +=', 'attribution_metadata')]
    for anchor, name in anchors:
        replace(anchor, f'        profile_mark("{name}");\n' + anchor)
    replace('        out["power_balance_diagnostics"] = balance_diag;',
            '        out["power_balance_diagnostics"] = balance_diag;\n'
            '        profile_mark("diagnostic_json_copy");')
    for index, anchor in enumerate([
            '        auto ac_bus_by_id = [&](int bus) -> const hacdcpf::ACBus* {',
            '        auto ac_is_slack_bus = [&](int bus) {',
            '        auto ac_bus_is_slack_source = [&](int bus) {']):
        replace(anchor, anchor + f'\n          ScanTimer scan_timer(profile_scans ? &scan_metrics[{index}] : nullptr);')
    anchor = '      res.set_header("X-HySim-Response-Bytes", std::to_string(payload.size()));'
    replace(anchor, '''      json scans = json::array();
      for (size_t index = 0; index < scan_metrics.size(); ++index) {
        scans.push_back(json{{"ms", scan_metrics[index].ms}, {"calls", scan_metrics[index].calls}});
      }
      res.set_header("X-HySim-Presentation-Profile",
          json{{"stages", profile_stages}, {"scans_enabled", profile_scans},
               {"scans", scans}, {"geo_lifetime_ms", geo_lifetime_ms},
               {"geo_cleanup_ms", geo_lifetime_ms - presentation_ms},
               {"matched_attribution_rows", matched_attribution_rows},
               {"discarded_attribution_metrics", discarded_attribution_metrics}}.dump());
''' + anchor)
    replace('\t          auto& row = component_results[existing->second];',
            '\t          ++matched_attribution_rows;\n'
            '\t          discarded_attribution_metrics += attributed["metrics"].size();\n'
            '\t          auto& row = component_results[existing->second];')
    replace('      res.set_content(std::move(payload), "application/json");',
            '      res.set_content(std::move(payload), "application/json");\n'
            '      request_lifetime.content_ready = std::chrono::steady_clock::now();\n'
            '      request_lifetime.ready = true;')
    return source[:start] + route + source[end:]


def build_profile(root, out, baseline):
    source = root / 'tests/run_gui_server.cpp'
    generated = out / 'run_gui_server-profile.cpp'
    generated.write_text(instrument(source.read_text()))
    database = json.loads((root / 'build/macos-release/compile_commands.json').read_text())
    entry = next(row for row in database if Path(row['file']) == source)
    command = shlex.split(entry['command'])
    obj = out / 'run_gui_server-profile.o'
    command[command.index('-c') + 1] = str(generated)
    command[command.index('-o') + 1] = str(obj)
    command += ['-I' + str(root / 'tests')]
    parent = root / 'output/attribution-performance/server-build.json'
    record = json.loads(parent.read_text())['commands'][0]
    link = list(record['argv'])
    position = next(index for index, value in enumerate(link) if value.endswith('/run_gui_server.cpp.o'))
    link[position] = str(obj)
    binary = out / 'run_gui_server'
    link[link.index('-o') + 1] = str(binary)
    provenance = {'source_sha256': digest(source), 'generated_sha256': digest(generated),
                  'tool_sha256': digest(Path(__file__)),
                  'repository_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                  'dependency_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root / '../MIPSolvers', text=True).strip(),
                  'attribution_sha256': digest(root / 'src/model/result_attribution.cpp'),
                  'baseline_sha256': digest(baseline), 'parent': str(parent),
                  'scope': 'Generated server only; current retained Release overlays/archives; no dependency guard override.',
                  'commands': [{'argv': command, 'cwd': entry['directory']},
                               {'argv': link, 'cwd': record['cwd']}]}
    (out / 'build.json').write_text(json.dumps(provenance, indent=2) + '\n')
    for item in provenance['commands']:
        subprocess.run(item['argv'], cwd=item['cwd'], check=True)
    for library in baseline.parent.glob('*.dylib'):
        shutil.copy2(library, out / library.name)
    provenance['binary_sha256'] = digest(binary)
    (out / 'build.json').write_text(json.dumps(provenance, indent=2) + '\n')


def request(base, path, payload=None, scans=False):
    req = urllib.request.Request(base + path,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={'Content-Type': 'application/json', 'X-HySim-Profile-Scans': '1' if scans else '0'})
    started = time.perf_counter()
    with urllib.request.urlopen(req, timeout=120) as response:
        headers_ready = time.perf_counter()
        raw = response.read()
        finished = time.perf_counter()
        return raw, dict(response.headers), (headers_ready - started) * 1000, (finished - headers_ready) * 1000


def field_sizes(raw):
    text = raw.decode()
    decoder = json.JSONDecoder()
    position = 1
    sizes = {}
    while text[position] != '}':
        key, position = decoder.raw_decode(text, position)
        if text[position] != ':':
            raise ValueError('Expected compact JSON object')
        begin = position + 1
        _, position = decoder.raw_decode(text, begin)
        sizes[key] = len(text[begin:position].encode())
        if text[position] == ',':
            position += 1
    return sizes


def stable_body(value):
    if isinstance(value, dict):
        return {key: stable_body(child) for key, child in value.items()
                if key not in ('timing', 'execution_time_sec', 'profiling')}
    if isinstance(value, list):
        return [stable_body(child) for child in value]
    return value


def run_profile(root, out, baseline, repeats):
    provenance = json.loads((out / 'build.json').read_text())
    assert provenance['source_sha256'] == digest(root / 'tests/run_gui_server.cpp')
    assert provenance['binary_sha256'] == digest(out / 'run_gui_server')
    assert provenance['baseline_sha256'] == digest(baseline)
    report = {'platform': platform.platform(), 'tool_sha256': digest(Path(__file__)),
              'repository_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
              'runs': [], 'summary': [],
              'scope': 'Full PF response, fresh server per sample, paired alternating order; no browser rendering.',
              'scan_names': ['ac_bus_by_id', 'ac_is_slack_bus', 'ac_bus_is_slack_source']}
    references = {}
    fixtures = ['case14.m', 'case118.m', 'case_ACTIVSg2000.m', 'case9241pegase.m']
    for fixture in fixtures:
        for repeat in range(repeats):
            modes = ['baseline', 'coarse', 'scans'] if repeat % 2 == 0 else ['scans', 'coarse', 'baseline']
            for mode in modes:
                binary = baseline if mode == 'baseline' else out / 'run_gui_server'
                with socket.socket() as sock:
                    sock.bind(('127.0.0.1', 0))
                    port = sock.getsockname()[1]
                base = f'http://127.0.0.1:{port}'
                with (out / f'{fixture}-{repeat}-{mode}.log').open('w') as log:
                    process = subprocess.Popen([str(binary), '--host', '127.0.0.1', '--port', str(port),
                        '--data-dir', str(root / 'data'), '--matpower-dir', str(root / 'external_data/matpower')],
                        cwd=root, stdout=log, stderr=subprocess.STDOUT)
                    try:
                        for attempt in range(100):
                            try:
                                request(base, '/api/session/status')
                                break
                            except OSError:
                                time.sleep(.1)
                        else:
                            raise RuntimeError('Server failed to start')
                        request(base, '/api/session/load_matpower', {'filename': fixture})
                        raw, headers, first_ms, read_ms = request(base, '/api/session/pf',
                            {'method': 'ac_newton', 'response_detail': 'full'}, mode == 'scans')
                        body = json.loads(raw)
                        assert body['converged'], (fixture, mode)
                        normalized = json.dumps(stable_body(body), sort_keys=True, separators=(',', ':')).encode()
                        result_hash = hashlib.sha256(normalized).hexdigest()
                        if fixture not in references:
                            references[fixture] = result_hash
                        assert result_hash == references[fixture], (fixture, mode, 'output mismatch')
                        server_timing = {key: float(value) for key, value in re.findall(
                            r'([a-z_]+);dur=([0-9.]+)', headers['Server-Timing'])}
                        row = {'fixture': fixture, 'repeat': repeat, 'mode': mode,
                               'wall_ms': first_ms + read_ms, 'headers_ms': first_ms, 'read_ms': read_ms,
                               'bytes': len(raw), 'body_sha256': result_hash, 'binary_sha256': digest(binary),
                               'timing': body['timing'], 'server_timing': server_timing,
                               'field_bytes': field_sizes(raw)}
                        if mode != 'baseline':
                            row['profile'] = json.loads(headers['X-HySim-Presentation-Profile'])
                            stage_sum = sum(stage['ms'] for stage in row['profile']['stages'])
                            row['stage_gap_ms'] = body['timing']['presentation_ms'] - stage_sum
                            assert abs(row['stage_gap_ms']) < max(1, body['timing']['presentation_ms'] * .01)
                            row['handler_lifetime_ms'] = float(headers['X-HySim-Handler-Lifetime-Ms'])
                            row['handler_cleanup_ms'] = float(headers['X-HySim-Handler-Cleanup-Ms'])
                            row['outside_handler_ms'] = row['wall_ms'] - row['handler_lifetime_ms']
                        row['unassigned_wall_ms'] = row['wall_ms'] - body['timing']['total_before_serialize_ms'] - server_timing['serialize']
                        report['runs'].append(row)
                        print(f'{fixture} {repeat} {mode}: wall={row["wall_ms"]:.3f} ms presentation={body["timing"]["presentation_ms"]:.3f} ms', flush=True)
                        (out / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
                    finally:
                        process.terminate()
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
        for mode in ['baseline', 'coarse', 'scans']:
            rows = [row for row in report['runs'] if row['fixture'] == fixture and row['mode'] == mode]
            summary = {'fixture': fixture, 'mode': mode}
            for key in ['wall_ms', 'headers_ms', 'read_ms', 'unassigned_wall_ms']:
                summary[key] = statistics.median(row[key] for row in rows)
            summary['presentation_ms'] = statistics.median(row['timing']['presentation_ms'] for row in rows)
            summary['serialize_ms'] = statistics.median(row['server_timing']['serialize'] for row in rows)
            if mode != 'baseline':
                for key in ['handler_lifetime_ms', 'handler_cleanup_ms', 'outside_handler_ms']:
                    summary[key] = statistics.median(row[key] for row in rows)
                for key in ['geo_lifetime_ms', 'geo_cleanup_ms', 'matched_attribution_rows', 'discarded_attribution_metrics']:
                    summary[key] = statistics.median(row['profile'][key] for row in rows)
                names = [stage['name'] for stage in rows[0]['profile']['stages']]
                summary['stages'] = {name: statistics.median(next(stage['ms'] for stage in row['profile']['stages']
                    if stage['name'] == name) for row in rows) for name in names}
                summary['scans'] = {name: {'ms': statistics.median(row['profile']['scans'][index]['ms'] for row in rows),
                    'calls': rows[0]['profile']['scans'][index]['calls']} for index, name in enumerate(report['scan_names'])}
            report['summary'].append(summary)
    report['acceptance'] = {'all_outputs_equal': True,
        'max_absolute_stage_gap_ms': max(abs(row.get('stage_gap_ms', 0)) for row in report['runs']),
        'large_case_instrumentation': []}
    for fixture in fixtures[2:]:
        summaries = {row['mode']: row for row in report['summary'] if row['fixture'] == fixture}
        for mode in ['coarse', 'scans']:
            overhead = 100 * (summaries[mode]['wall_ms'] / summaries['baseline']['wall_ms'] - 1)
            report['acceptance']['large_case_instrumentation'].append({
                'fixture': fixture, 'mode': mode, 'median_wall_overhead_pct': overhead,
                'within_five_percent': overhead <= 5})
    (out / 'results.json').write_text(json.dumps(report, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', action='store_true')
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--output', default='output/pf-presentation-profile')
    parser.add_argument('--baseline', default='output/attribution-performance/run_gui_server')
    args = parser.parse_args()
    if not (args.build or args.run) or args.repeats < 1:
        parser.error('Select --build and/or --run; repeats must be positive')
    root = Path(__file__).resolve().parents[1]
    out = (root / args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    baseline = (root / args.baseline).resolve()
    if args.build:
        build_profile(root, out, baseline)
    if args.run:
        run_profile(root, out, baseline, args.repeats)


if __name__ == '__main__':
    main()
