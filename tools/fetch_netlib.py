#!/usr/bin/env python3
"""Fetch a reproducible NETLIB LP corpus and generate benchmark metadata."""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import io
import re
import shutil
import tempfile
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path


DATA_NETLIB_COMMIT = "f1cc423067407d55d579c9c35fb01edf860dbc24"
DATA_NETLIB_ARCHIVE = (
    "https://github.com/coin-or-tools/Data-Netlib/archive/"
    f"{DATA_NETLIB_COMMIT}.zip"
)
OFFICIAL_README = "https://www.netlib.org/lp/data/readme"


@dataclass(frozen=True)
class Reference:
    official_name: str
    rows: int
    columns: int
    nonzeros: int
    objective: float


SUMMARY_ROW = re.compile(
    r"^(?P<name>[A-Z0-9][A-Z0-9.\-]*)\s+"
    r"(?P<rows>\d+)\s+(?P<columns>\d+)\s+(?P<nonzeros>\d+)\s+"
    r"(?:\d+|\(see NOTES\))\s+(?:[BR]+\s+)?"
    r"(?P<objective>[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:E[+-]?\d+)?)"
)

CPLEX_TABLE_HEADER = "Problem        CPLEX(Sparc)          MINOS(MIPS)"


def download(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "MIPSolvers-NETLIB-fetch"})
    with urllib.request.urlopen(request, timeout=120) as response:
        return response.read()


def canonical_name(name: str) -> str:
    return name.lower().replace(".", "")


def parse_references(text: str) -> dict[str, Reference]:
    references: dict[str, Reference] = {}
    for line in text.splitlines():
        match = SUMMARY_ROW.match(line.strip())
        if not match:
            continue
        official_name = match.group("name")
        references[canonical_name(official_name)] = Reference(
            official_name=official_name,
            rows=int(match.group("rows")),
            columns=int(match.group("columns")),
            nonzeros=int(match.group("nonzeros")),
            objective=float(match.group("objective")),
        )
    if len(references) < 80:
        raise RuntimeError(f"parsed only {len(references)} official NETLIB references")

    # The first summary table contains 1988 MINOS/VAX values.  The same
    # official readme later publishes double-precision CPLEX/Sparc revisions
    # for the numerically sensitive models.  Prefer that first revised column
    # when present; the second MINOS/MIPS column is an independent comparison,
    # not a replacement for CPLEX.  Fixed column boundaries are required here
    # because some rows intentionally leave the CPLEX field blank.
    lines = text.splitlines()
    try:
        header_index = next(
            i for i, line in enumerate(lines) if CPLEX_TABLE_HEADER in line
        )
    except StopIteration:
        raise RuntimeError("official NETLIB CPLEX revision table is missing")
    # Numeric rows use a 12-character problem field followed by two
    # 21-character right-aligned result fields.  The labels are centered, so
    # their text positions are not valid field boundaries.
    problem_column_end = 12
    minos_column = 33
    revised = 0
    for line in lines[header_index + 1 :]:
        if line.startswith("The above CPLEX"):
            break
        if not line.strip() or len(line) <= problem_column_end:
            continue
        name = line[:problem_column_end].strip()
        cplex_value = line[problem_column_end:minos_column].strip()
        key = canonical_name(name)
        if key not in references or not cplex_value:
            continue
        try:
            objective = float(cplex_value)
        except ValueError:
            continue
        old = references[key]
        references[key] = Reference(
            official_name=old.official_name,
            rows=old.rows,
            columns=old.columns,
            nonzeros=old.nonzeros,
            objective=objective,
        )
        revised += 1
    if revised < 8:
        raise RuntimeError(f"parsed only {revised} CPLEX NETLIB revisions")
    return references


def source_members(source_dir: Path | None) -> tuple[dict[str, bytes], str]:
    if source_dir is not None:
        files = {
            path.name: path.read_bytes()
            for path in sorted(source_dir.glob("*.mps.gz"))
        }
        return files, str(source_dir.resolve())

    archive = download(DATA_NETLIB_ARCHIVE)
    files: dict[str, bytes] = {}
    with zipfile.ZipFile(io.BytesIO(archive)) as package:
        for member in package.infolist():
            name = Path(member.filename).name
            if name.endswith(".mps.gz"):
                files[name] = package.read(member)
    return files, DATA_NETLIB_ARCHIVE


def validate_mps(name: str, payload: bytes) -> None:
    text = payload.decode("ascii", errors="strict")
    first_content = next((line.strip() for line in text.splitlines() if line.strip()), "")
    if not first_content.startswith("NAME"):
        raise RuntimeError(f"{name}: first MPS record is not NAME")
    if not any(line.strip() == "ENDATA" for line in text.splitlines()):
        raise RuntimeError(f"{name}: missing ENDATA record")


def write_provenance(path: Path, count: int, source: str) -> None:
    path.write_text(
        "# NETLIB 测试数据来源\n\n"
        f"本目录包含 {count} 个解压后的标准 NETLIB 线性规划 MPS 算例。\n\n"
        f"- 数据镜像：`coin-or-tools/Data-Netlib@{DATA_NETLIB_COMMIT}`\n"
        f"- 本次输入：`{source}`\n"
        f"- 官方规模与参考目标：{OFFICIAL_README}\n"
        "- 获取工具：`tools/fetch_netlib.py`\n\n"
        "`mps_manifest.csv` 保存每个解压文件的 SHA-256。参考目标来自 NETLIB "
        "官方 README：默认取问题汇总表，对后续 CPLEX(Sparc) 修订表中有数值的"
        "案例优先取双精度 CPLEX 结果。部分数值困难实例在不同求解器和容差下可能得到略有差异的"
        "目标值，因此 benchmark 还会独立检查原模型可行性。\n\n"
        "Data-Netlib 仓库明确说明其 EPL-2.0 许可适用于构建系统而不适用于 "
        "`*.mps.gz` 数据文件。重新分发算例前应单独确认原始问题数据的许可条件；"
        "本目录默认被项目 `.gitignore` 排除，仅作为本机测试数据。\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("tests/data"),
        help="directory containing netlib/ and mps_manifest.csv",
    )
    parser.add_argument(
        "--source-dir",
        type=Path,
        help="optional existing Data-Netlib checkout; otherwise download pinned archive",
    )
    args = parser.parse_args()

    references = parse_references(download(OFFICIAL_README).decode("ascii"))
    members, source = source_members(args.source_dir)
    if len(members) < 80:
        raise RuntimeError(f"source contains only {len(members)} .mps.gz files")

    output_dir = args.output_dir.resolve()
    netlib_dir = output_dir / "netlib"
    netlib_dir.mkdir(parents=True, exist_ok=True)
    manifest_rows: list[list[str]] = []

    with tempfile.TemporaryDirectory(prefix="mipsolvers-netlib-") as temp_name:
        staged = Path(temp_name)
        for compressed_name, compressed in sorted(members.items()):
            name = compressed_name.removesuffix(".mps.gz")
            reference = references.get(canonical_name(name))
            if reference is None:
                print(f"skip {name}: no official numeric reference objective")
                continue
            payload = gzip.decompress(compressed)
            validate_mps(name, payload)
            digest = hashlib.sha256(payload).hexdigest()
            staged_path = staged / f"{name}.mps"
            staged_path.write_bytes(payload)
            manifest_rows.append(
                [
                    "netlib",
                    name,
                    str(reference.rows),
                    str(reference.columns),
                    str(reference.nonzeros),
                    (
                        "https://github.com/coin-or-tools/Data-Netlib/blob/"
                        f"{DATA_NETLIB_COMMIT}/{compressed_name}"
                    ),
                    f"{reference.objective:.17g}",
                    digest,
                ]
            )

        for path in netlib_dir.glob("*.mps"):
            path.unlink()
        for path in staged.glob("*.mps"):
            shutil.copy2(path, netlib_dir / path.name)

    manifest_path = output_dir / "mps_manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            [
                "suite",
                "name",
                "rows",
                "columns",
                "nonzeros",
                "source",
                "reference_objective",
                "sha256",
            ]
        )
        writer.writerows(manifest_rows)

    write_provenance(output_dir / "NETLIB_PROVENANCE.md", len(manifest_rows), source)
    print(f"installed {len(manifest_rows)} NETLIB cases in {netlib_dir}")
    print(f"wrote {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
