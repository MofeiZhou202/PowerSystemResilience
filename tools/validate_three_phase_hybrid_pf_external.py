#!/usr/bin/env python3
"""Cross-check hybrid PF operating-point AC snapshots in OpenDSS and GridLAB-D."""

from __future__ import annotations

import argparse
import cmath
import csv
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_NATIVE = ROOT / "output/benchmarks/paper_hybrid_pf_external_snapshot.csv"
DEFAULT_OUTPUT = ROOT / "output/benchmarks/paper_hybrid_pf_external_validation.csv"
DEFAULT_TEX = ROOT / "docs/papers/phase_graph_reduced_hybrid_opf/external_validation_table.tex"


def complex_text(value: complex) -> str:
    return f"{value.real:.12g}{value.imag:+.12g}j"


def discover_gridlabd(explicit: str | None) -> Path | None:
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    for key in ("HACDCPF_GRIDLABD_BIN", "GRIDLABD_BIN"):
        if os.environ.get(key):
            candidates.append(Path(os.environ[key]))
    found = shutil.which("gridlabd")
    if found:
        candidates.append(Path(found))
    candidates.extend([
        Path("/tmp/gridlabd-5.3.0/GridLAB-D-5.3.0-MacOS/bin/gridlabd"),
        ROOT.parent / "gridlab-d/build/bin/gridlabd",
        ROOT.parent / "gridlab-d/build/source/gridlabd",
    ])
    return next((path for path in candidates if path.is_file()), None)


def load_native(path: Path) -> dict[str, list[dict[str, str]]]:
    rows = list(csv.DictReader(path.open(newline="")))
    grouped: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        grouped.setdefault(row["mode"], []).append(row)
    for values in grouped.values():
        values.sort(key=lambda row: int(row["phase"]))
    return grouped


def native_voltages(rows: list[dict[str, str]]) -> list[complex]:
    return [complex(float(row["voltage_re_pu"]), float(row["voltage_im_pu"]))
            for row in rows]


def compare(native: list[complex], external: list[complex]) -> tuple[float, float, float]:
    complex_error = max(abs(a - b) for a, b in zip(native, external))
    magnitude_error = max(abs(abs(a) - abs(b)) for a, b in zip(native, external))
    angle_error = max(
        abs(math.degrees(cmath.phase(a / b))) if abs(b) > 1e-12 else math.inf
        for a, b in zip(native, external)
    )
    return complex_error, magnitude_error, angle_error


def run_opendss(rows: list[dict[str, str]], workdir: Path) -> dict:
    try:
        import opendssdirect as dss
    except Exception as exc:
        return {"status": "unavailable", "detail": str(exc)}
    base_mva = float(rows[0]["base_mva"])
    base_kv = float(rows[0]["base_kv_ll"])
    # The native phase equations put each phase power on the system MVA base,
    # so Z_base = V_LN^2 / S_base = V_LL^2 / (3 S_base).
    zbase = base_kv * base_kv / (3.0 * base_mva)
    r_ohm = float(rows[0]["line_r_pu"]) * zbase
    x_ohm = float(rows[0]["line_x_pu"]) * zbase
    commands = [
        "Clear",
        (f"New Circuit.hybrid phases=3 bus1=source basekv={base_kv:.12g} "
         "pu=1 angle=0 frequency=60 r1=1e-9 x1=1e-9 r0=1e-9 x0=1e-9"),
        (f"New Line.feeder bus1=source.1.2.3 bus2=load.1.2.3 phases=3 "
         f"rmatrix=[{r_ohm:.12g} | 0 {r_ohm:.12g} | 0 0 {r_ohm:.12g}] "
         f"xmatrix=[{x_ohm:.12g} | 0 {x_ohm:.12g} | 0 0 {x_ohm:.12g}] "
         "cmatrix=[0 | 0 0 | 0 0 0] length=1 units=none"),
    ]
    kv_ln = base_kv / math.sqrt(3.0)
    for row in rows:
        phase = int(row["phase"])
        p_kw = float(row["net_p_pu"]) * base_mva * 1000.0
        q_kvar = float(row["net_q_pu"]) * base_mva * 1000.0
        commands.append(
            f"New Load.p{phase} bus1=load.{phase + 1} phases=1 conn=wye "
            f"model=1 kv={kv_ln:.12g} kw={p_kw:.12g} kvar={q_kvar:.12g}"
        )
    commands.extend([
        f"Set voltagebases=[{base_kv:.12g}]",
        "CalcVoltageBases",
        "Set controlmode=off",
        "Set maxiterations=100",
        "Solve mode=snapshot",
    ])
    try:
        dss.Basic.ClearAll()
        for command in commands:
            dss.Text.Command(command)
        if not dss.Solution.Converged():
            return {"status": "not_converged", "detail": "OpenDSS solve failed"}
        dss.Circuit.SetActiveBus("load")
        polar = dss.Bus.puVmagAngle()
        voltages = [cmath.rect(float(polar[2 * k]), math.radians(float(polar[2 * k + 1])))
                    for k in range(3)]
        return {"status": "solved", "voltages": voltages,
                "version": dss.Basic.Version().strip().splitlines()[0]}
    except Exception as exc:
        return {"status": "failed", "detail": str(exc)}


def parse_gridlabd_complex(text: str) -> complex:
    value = text.strip().replace("i", "j")
    if value.endswith(" V"):
        value = value[:-2]
    polar = re.fullmatch(r"([+-]?[0-9.eE]+)([+-][0-9.eE]+)d", value)
    if polar:
        return cmath.rect(float(polar.group(1)), math.radians(float(polar.group(2))))
    return complex(value)


def run_gridlabd(rows: list[dict[str, str]], executable: Path | None,
                  workdir: Path) -> dict:
    if executable is None:
        return {"status": "unavailable", "detail": "GridLAB-D binary not found"}
    base_mva = float(rows[0]["base_mva"])
    base_kv = float(rows[0]["base_kv_ll"])
    zbase = base_kv * base_kv / (3.0 * base_mva)
    z = complex(float(rows[0]["line_r_pu"]) * zbase,
                float(rows[0]["line_x_pu"]) * zbase)
    vln = base_kv * 1000.0 / math.sqrt(3.0)
    source = [cmath.rect(vln, 0.0), cmath.rect(vln, -2.0 * math.pi / 3.0),
              cmath.rect(vln, 2.0 * math.pi / 3.0)]
    powers = [complex(float(row["net_p_pu"]), float(row["net_q_pu"]))
              * base_mva * 1e6 for row in rows]
    glm = workdir / "hybrid_snapshot.glm"
    recorder = workdir / "voltage.csv"
    glm.write_text(f"""module powerflow {{ solver_method NR; NR_iteration_limit 1000; }}
module tape;
clock {{ timezone GMT0; starttime '2000-01-01 00:00:00'; stoptime '2000-01-01 00:00:01'; }}
object node {{
  name source; phases ABCN; bustype SWING; nominal_voltage {vln:.12g};
  voltage_A {complex_text(source[0])}; voltage_B {complex_text(source[1])};
  voltage_C {complex_text(source[2])};
}}
object node {{ name loadbus; phases ABCN; nominal_voltage {vln:.12g}; }}
object line_configuration {{
  name cfg; z11 {complex_text(z)}; z22 {complex_text(z)}; z33 {complex_text(z)};
  z12 0+0j; z13 0+0j; z21 0+0j; z23 0+0j; z31 0+0j; z32 0+0j;
}}
object overhead_line {{
  name feeder; phases ABCN; from source; to loadbus; length 5280 ft; configuration cfg;
}}
object load {{
  name demand; parent loadbus; phases ABCN; nominal_voltage {vln:.12g};
  constant_power_A {complex_text(powers[0])};
  constant_power_B {complex_text(powers[1])};
  constant_power_C {complex_text(powers[2])};
}}
object recorder {{
  parent loadbus; property voltage_A,voltage_B,voltage_C;
  interval 1; file \"{recorder.name}\";
}}
""", encoding="ascii")
    proc = subprocess.run([str(executable), glm.name], cwd=workdir, text=True,
                          capture_output=True, timeout=60)
    if proc.returncode != 0 or not recorder.exists():
        detail = (proc.stderr or proc.stdout).strip().splitlines()
        return {"status": "failed", "detail": detail[-1] if detail else "no recorder"}
    data = []
    with recorder.open(newline="") as stream:
        for raw in csv.reader(stream):
            if raw and not raw[0].lstrip().startswith("#"):
                data.append(raw)
    if not data or len(data[-1]) < 4:
        return {"status": "failed", "detail": "GridLAB-D recorder has no voltage row"}
    try:
        voltages = [parse_gridlabd_complex(data[-1][k]) / vln for k in range(1, 4)]
    except Exception as exc:
        return {"status": "failed", "detail": f"recorder parse: {exc}: {data[-1]}"}
    return {"status": "solved", "voltages": voltages,
            "version": subprocess.run([str(executable), "--version"], text=True,
                                      capture_output=True).stdout.strip().splitlines()[-1]}


def latex_sci(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "--"
    if value == 0.0:
        return "$0$"
    exponent = math.floor(math.log10(abs(value)))
    mantissa = value / 10 ** exponent
    return rf"${mantissa:.2f}\times10^{{{exponent}}}$"


def write_outputs(rows: list[dict], output: Path, tex: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    tex.parent.mkdir(parents=True, exist_ok=True)
    with tex.open("w") as stream:
        stream.write("% Generated by validate_three_phase_hybrid_pf_external.py.\n")
        stream.write("\\begin{table}[t]\n\\caption{External ac-snapshot validation}\n")
        stream.write("\\label{tab:external_validation}\\centering\\scriptsize\n")
        stream.write("\\setlength{\\tabcolsep}{3.2pt}\n")
        stream.write("\\begin{tabular}{llrrr}\n\\toprule\n")
        stream.write("Mode & Reference & Max $|\\Delta V|$ & Max $|\\Delta|V||$ & Max $|\\Delta\\theta|$ (deg.)\\\\\n")
        stream.write("\\midrule\n")
        for row in rows:
            label = "GFL--$V_{dc}Q$" if row["mode"] == "gfl_vdc_q" else "GFM--$V_{dc}|E|$"
            engine = row["engine"].replace("_", "\\_")
            stream.write(
                f"{label} & {engine} & {latex_sci(row['max_complex_voltage_error_pu'])} & "
                f"{latex_sci(row['max_magnitude_error_pu'])} & "
                f"{latex_sci(row['max_angle_error_deg'])}\\\\\n"
            )
        stream.write("\\bottomrule\n\\end{tabular}\n")
        stream.write("\\vspace{1mm}\\parbox{0.98\\columnwidth}{\\scriptsize "
                     "OpenDSS and GridLAB-D receive the native PF operating-point net phase injections. "
                     "They validate the overlapping three-phase AC snapshot, not the DC network or off-point converter control law.}\n")
        stream.write("\\end{table}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", type=Path, default=DEFAULT_NATIVE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--tex", type=Path, default=DEFAULT_TEX)
    parser.add_argument("--gridlabd-bin")
    args = parser.parse_args()
    if not args.native.exists():
        binary = ROOT / "build/macos-release/three_phase_hybrid_pf_external_case"
        subprocess.run([str(binary), str(args.native)], cwd=ROOT, check=True)
    native = load_native(args.native)
    gridlabd = discover_gridlabd(args.gridlabd_bin)
    results = []
    with tempfile.TemporaryDirectory(prefix="hybrid_pf_external_") as directory:
        work = Path(directory)
        for mode, phase_rows in native.items():
            reference = native_voltages(phase_rows)
            mode_dir = work / mode
            mode_dir.mkdir(parents=True, exist_ok=True)
            for engine, solved in (
                ("OpenDSS", run_opendss(phase_rows, work)),
                ("GridLAB-D", run_gridlabd(phase_rows, gridlabd, mode_dir)),
            ):
                row = {"mode": mode, "engine": engine, "status": solved["status"],
                       "max_complex_voltage_error_pu": None,
                       "max_magnitude_error_pu": None, "max_angle_error_deg": None,
                       "version": solved.get("version", ""),
                       "detail": solved.get("detail", "")}
                if solved["status"] == "solved":
                    errors = compare(reference, solved["voltages"])
                    row["max_complex_voltage_error_pu"] = errors[0]
                    row["max_magnitude_error_pu"] = errors[1]
                    row["max_angle_error_deg"] = errors[2]
                results.append(row)
    write_outputs(results, args.output, args.tex)
    for row in results:
        print(row)
    return 0 if all(row["status"] == "solved" for row in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
