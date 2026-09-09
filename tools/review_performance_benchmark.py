"""Sequential baseline/candidate benchmark for AUD-095/096.

Uses a configured Release compile database and retained archives; does not
bypass dependency guards or certify a clean build. Generated sources add only
measurement hooks. Annual samples reuse one real verified carbon snapshot.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shlex
import shutil
import statistics
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", default="build/macos-release")
    parser.add_argument("--baseline", default="48e0cf62")
    parser.add_argument("--output", default="output/code-optimization/performance")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = (root / args.build_dir).resolve()
    out = (root / args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    database = json.loads((build / "compile_commands.json").read_text())
    units = {"carbon": "src/carbon_analysis/annual_carbon_analysis.cpp",
             "slack": "src/power_flow/distributed_slack_solver.cpp"}
    report = {"platform": platform.platform(), "baseline": args.baseline,
              "scope": "Incremental Release; frozen verified carbon aggregation and frozen PF mismatch only",
              "commands": [], "sources": {}, "runs": [], "summary": []}
    link_template = shlex.split((build / "tests/CMakeFiles/test_graph.dir/link.txt").read_text())
    for library in link_template:
        if Path(library).name == "libdss_capi.dylib":
            for dependency in Path(library).parent.glob("*.dylib"):
                shutil.copy2(dependency, out / dependency.name)
    for kind, source in units.items():
        for mode in ("baseline", "candidate"):
            code = (subprocess.check_output(["git", "show", f"{args.baseline}:{source}"], cwd=root).decode()
                    if mode == "baseline" else (root / source).read_text())
            report["sources"][f"{kind}-{mode}"] = hashlib.sha256(code.encode()).hexdigest()
            if kind == "carbon":
                call = "compute_carbon_analysis(system_at(t), pf_results[t], options.carbon_options)"
                timer = "void extend_hourly_width(std::vector<std::vector<double>>& rows, size_t width, double fill) {"
                assert code.count(call) == code.count(timer) == 1
                code = code.replace(call, "benchmark_snapshot").replace(timer, timer + "\n  PaddingTimer padding_timer;")
            else:
                hook = "return (pcalc - psch).sum();"
                assert code.count(hook) == 1
                code = code.replace(hook, "captured_pcalc = pcalc;\n  " + hook)
            generated = out / f"{kind}-{mode}.cpp"
            generated.write_text(code)
            entry = next(x for x in database if Path(x["file"]) == root / source)
            command = shlex.split(entry["command"])
            obj = out / f"{kind}-{mode}.o"
            command[command.index("-o") + 1] = str(obj)
            command[command.index("-c") + 1] = str(root / "tools/review_performance_benchmark.cpp")
            command += [f'-DBENCH_SOURCE="{generated}"']
            if kind == "carbon":
                command.append("-DBENCH_CARBON")
            report["commands"].append({"argv": command, "cwd": entry["directory"]})
            subprocess.run(command, cwd=entry["directory"], check=True)
            command = shlex.split((build / "tests/CMakeFiles/test_graph.dir/link.txt").read_text())
            index = next(i for i, item in enumerate(command) if item.endswith("test_graph.cpp.o"))
            command[index] = str(obj)
            command[command.index("-o") + 1] = str(out / f"{kind}-{mode}")
            report["commands"].append({"argv": command, "cwd": str(build / "tests")})
            subprocess.run(command, cwd=build / "tests", check=True)
    (out / "build.json").write_text(json.dumps(report, indent=2) + "\n")
    for kind in units:
        fixtures = ([(b, l, t) for b, l in [(1, 128), (200, 200)] for t in (1000, 2000, 4000)]
                    if kind == "carbon" else [(n, hybrid, 1) for n in (1000, 2000, 10000) for hybrid in (0, 1)] + [(3, -1, 1), (3, -2, 1)])
        for fixture in fixtures:
            groups = {}
            reference = None
            for mode in ("baseline", "candidate"):
                rows = []
                for repeat in range(3):
                    evidence = out / "values.txt"
                    command = [str(out / f"{kind}-{mode}"), *map(str, fixture), str(evidence)]
                    result = subprocess.run(command, cwd=root, text=True, capture_output=True)
                    if result.returncode:
                        raise RuntimeError(f"{command}: exit {result.returncode}\n{result.stdout}\n{result.stderr}")
                    row = json.loads(result.stdout)
                    digest = hashlib.sha256(evidence.read_bytes()).hexdigest()
                    if reference is None:
                        reference = digest
                    assert reference == digest, (kind, fixture, mode, "output differs")
                    row.update(kind=kind, fixture=fixture, mode=mode, repeat=repeat,
                               output_sha256=digest, argv=command)
                    rows.append(row)
                    report["runs"].append(row)
                    print(kind, fixture, mode, repeat, row["wall_s"], flush=True)
                groups[mode] = {key: statistics.median(r[key] for r in rows)
                                for key in ("wall_s", "peak_rss_mb") + (("padding_s",) if kind == "carbon" else ())}
            report["summary"].append({"kind": kind, "fixture": fixture, **groups})
            (out / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    (out / "values.txt").unlink()


if __name__ == "__main__":
    main()
