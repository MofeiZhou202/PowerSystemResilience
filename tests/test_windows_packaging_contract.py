#!/usr/bin/env python3
"""Static cross-platform contract checks for the Windows release scripts."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    presets = json.loads((ROOT / "CMakePresets.json").read_text(encoding="utf-8"))
    configure = {row["name"]: row for row in presets["configurePresets"]}
    build = {row["name"]: row for row in presets["buildPresets"]}
    test = {row["name"]: row for row in presets["testPresets"]}

    source = configure["windows-source-release"]
    assert source["inherits"] == "windows-msvc-release"
    assert source["cacheVariables"]["MIPSOLVERS_USE_PREBUILT_THIRD_PARTY"] == "OFF"
    assert "HACDCPF_USE_CPLEX" not in source["cacheVariables"]
    assert build["windows-source-release"]["configurePreset"] == "windows-source-release"
    assert test["windows-source-release"]["configurePreset"] == "windows-source-release"

    package = (ROOT / "tools/package_windows.ps1").read_text(encoding="utf-8")
    verifier = (ROOT / "tools/verify_windows_release.ps1").read_text(encoding="utf-8")
    assert "[switch]$SourceDependencies" in package
    assert '"windows-source-release"' in package
    assert "MIPSOLVERS_USE_PREBUILT_THIRD_PARTY" in package
    assert "windows_licenses/*" in package
    assert "Expand-Archive" in verifier
    assert "package_manifest.csv" in verifier
    assert "$env:SystemRoot\\System32" in verifier
    assert "CPLEX" not in package.upper()
    assert "CPLEX" not in verifier.upper()

    top_level_cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    dependencies = (ROOT / "cmake/Dependencies.cmake").read_text(encoding="utf-8")
    assert "_hacdcpf_default_vcpkg_prefix" not in top_level_cmake
    assert "_hacdcpf_default_vcpkg_prefix" not in dependencies
    print("Windows packaging contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
