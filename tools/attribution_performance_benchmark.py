"""AUD-097 baseline/candidate fixed-state attribution benchmark.

Build from an existing Release compile database, preserving dependency guards.
All result fields are hashed; total process RSS also includes setup and PF.
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
    parser.add_argument("--mode", choices=["baseline", "candidate"], required=True)
    parser.add_argument("--baseline", default="48e0cf62")
    parser.add_argument("--build-dir", default="build/macos-release")
    parser.add_argument("--output", default="output/attribution-performance")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = root / args.build_dir
    out = root / args.output
    out.mkdir(parents=True, exist_ok=True)
    source = "src/model/result_attribution.cpp"
    code = (subprocess.check_output(["git", "show", f"{args.baseline}:{source}"], cwd=root)
            if args.mode == "baseline" else (root / source).read_bytes())
    generated = out / (args.mode + ".cpp")
    generated.write_bytes(code)
    database = json.loads((build / "compile_commands.json").read_text())
    entry = next(row for row in database if Path(row["file"]) == root / source)
    command = shlex.split(entry["command"])
    obj = out / (args.mode + ".o")
    command[command.index("-c") + 1] = str(root / "tools/attribution_performance_benchmark.cpp")
    command[command.index("-o") + 1] = str(obj)
    command.append(f'-DBENCH_SOURCE="{generated}"')
    records = [{"argv": command, "cwd": entry["directory"]}]
    subprocess.run(command, cwd=entry["directory"], check=True)
    link = shlex.split((build / "tests/CMakeFiles/test_graph.dir/link.txt").read_text())
    link[next(i for i, x in enumerate(link) if x.endswith("test_graph.cpp.o"))] = str(obj)
    binary = out / args.mode
    link[link.index("-o") + 1] = str(binary)
    for item in link:
        if Path(item).name == "libdss_capi.dylib":
            for library in Path(item).parent.glob("*.dylib"):
                shutil.copy2(library, out / library.name)
    records.append({"argv": link, "cwd": str(build / "tests")})
    subprocess.run(link, cwd=build / "tests", check=True)
    report = {"mode": args.mode, "baseline": args.baseline, "platform": platform.platform(),
              "source_sha256": hashlib.sha256(code).hexdigest(), "commands": records,
              "scope": "Current attribution unit; other Release archives retained. Setup/PF excluded from wall time.",
              "runs": [], "summary": []}
    fixtures = [("synthetic", n) for n in (14, 1000, 2000, 10000)]
    fixtures += [("external_data/matpower/" + name, 0)
                 for name in ("case118.m", "case_ACTIVSg2000.m", "case9241pegase.m")]
    baseline_path = out / "baseline.json"
    baseline = json.loads(baseline_path.read_text()) if args.mode == "candidate" else None
    for index, (fixture, size) in enumerate(fixtures):
        rows = []
        for repeat in range(3):
            evidence = out / f"{args.mode}-{index}.json"
            command = [str(binary), fixture, str(size), str(evidence)]
            result = subprocess.run(command, cwd=root, text=True, capture_output=True)
            if result.returncode:
                raise RuntimeError(f"{command} exit {result.returncode}\n{result.stdout}\n{result.stderr}")
            row = json.loads(result.stdout.splitlines()[-1])
            row.update(fixture=fixture, size=size, repeat=repeat, argv=command,
                       output_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
            if rows:
                assert row["output_sha256"] == rows[0]["output_sha256"]
            if baseline:
                reference = baseline["runs"][index * 3 + repeat]
                assert row["output_sha256"] == reference["output_sha256"], (fixture, size, "output mismatch")
            rows.append(row)
            report["runs"].append(row)
            print(args.mode, fixture, size, repeat, row["seconds"], flush=True)
        report["summary"].append({"fixture": fixture, "size": size,
            "seconds": statistics.median(r["seconds"] for r in rows),
            "peak_rss_mb": statistics.median(r["peak_rss_mb"] for r in rows)})
        (out / (args.mode + ".json")).write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
