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
    resilience = configure["windows-resilience-release"]
    assert resilience["inherits"] == "windows-source-release"
    assert resilience["cacheVariables"]["HACDCPF_RESILIENCE_EDITION"] == "ON"
    assert resilience["cacheVariables"]["HACDCPF_TRIAL_EDITION"] == "OFF"
    release = configure["windows-msvc-release"]
    assert release["cacheVariables"]["MIPSOLVERS_MKL_ROOT"] == "${sourceDir}/build/windows-dependencies/oneapi-mkl"
    assert release["cacheVariables"]["ZLIB_ROOT"] == "${sourceDir}/build/windows-dependencies/zlib"
    assert release["cacheVariables"]["MIPSOLVERS_PREBUILT_THIRD_PARTY_PREFIX"] == "${sourceDir}/build/windows-dependencies/mipsolvers-third-party"
    assert configure["base"]["cacheVariables"]["MIPSOLVERS_SOURCE_DIR"] == "${sourceDir}/MIPSolvers"
    assert "HACDCPF_USE_CPLEX" not in source["cacheVariables"]
    assert build["windows-source-release"]["configurePreset"] == "windows-source-release"
    assert test["windows-source-release"]["configurePreset"] == "windows-source-release"
    assert build["windows-resilience-release"]["configurePreset"] == "windows-resilience-release"
    assert test["windows-resilience-release"]["configurePreset"] == "windows-resilience-release"

    full_package = (ROOT / "tools/package_windows.ps1").read_text(encoding="utf-8")
    full_verifier = (ROOT / "tools/verify_windows_release.ps1").read_text(
        encoding="utf-8"
    )
    trial_package = (ROOT / "tools/package_trial_windows.ps1").read_text(
        encoding="utf-8"
    )
    resilience_package = (
        ROOT / "tools/package_resilience_windows.ps1"
    ).read_text(encoding="utf-8")
    resilience_verifier = (
        ROOT / "tools/verify_resilience_windows_release.ps1"
    ).read_text(encoding="utf-8")

    # Every package script must fail closed on its exact edition profile and
    # prove the locked, committed in-repository MIPSolvers import.
    assert "HACDCPF_TRIAL_EDITION:BOOL=OFF" in full_package
    assert "HACDCPF_RESILIENCE_EDITION:BOOL=OFF" in full_package
    assert "HACDCPF_TRIAL_EDITION:BOOL=ON" in trial_package
    assert "HACDCPF_RESILIENCE_EDITION:BOOL=OFF" in trial_package
    assert "HACDCPF_RESILIENCE_EDITION:BOOL=ON" in resilience_package
    assert "HACDCPF_TRIAL_EDITION:BOOL=OFF" in resilience_package
    for script in (full_package, trial_package, resilience_package):
        assert "HEAD:MIPSolvers" in script
        assert "MIPSolvers.lock.json" in script
        assert "MIPSOLVERS_SOURCE_DIR" in script
        assert '"../MIPSolvers"' not in script
        assert "test_edition_profile" in script
        assert script.index("cmake --build") < script.index("ctest --test-dir")

    assert "edition-full-unit" in full_package
    assert "test_solver_capabilities" in full_package
    assert "edition-trial-unit" in trial_package
    assert "edition-trial-api-e2e" in trial_package
    assert "edition-resilience-unit" in resilience_package
    assert "edition-resilience-api-e2e" in resilience_package
    assert "test_solver_capabilities" in resilience_package

    assert "[switch]$SourceDependencies" in full_package
    assert '"windows-source-release"' in full_package
    assert "MIPSOLVERS_USE_PREBUILT_THIRD_PARTY" in full_package
    assert "windows_licenses/*" in full_package
    assert "build/windows-dependencies/oneapi-mkl/licensing" in full_package
    assert "MIPSolvers/third_party/oneapi-mkl/licensing" not in full_package
    assert "CPLEX" not in full_package.upper()
    assert "CPLEX" not in full_verifier.upper()

    # Runtime DLL staging must name files explicitly rather than copying every
    # DLL found in a build, vcpkg, or redistributable directory.
    assert "$requiredDlls = @(" in trial_package
    assert "Get-ChildItem -Path $dir -Filter '*.dll'" not in trial_package
    assert "Get-ChildItem -LiteralPath $runtimeDir -Filter \"*.dll\"" not in full_package
    assert "$vcRuntimeAllowlist = @(" in resilience_package
    assert "Runtime dependency is not allowlisted for Resilience" in resilience_package
    assert "Get-PeDependencies" in resilience_package

    # Resilience resource staging is per-file, with a second denylist for
    # disabled feature families. Recursive copies remain allowed for license
    # material only, not product data/docs/external_data/web trees.
    assert "$resourceFiles = @(" in resilience_package
    assert "external_data/typhoon/sst_monthly_south_china_sea.json" in resilience_package
    assert "typhoon_track_catalog_m1-12_n100_h48_dt1.00_seed203000.json" in resilience_package
    assert "$helpSections = @(" in resilience_package
    assert "web/help_docs.json" in resilience_package
    assert "$forbiddenWebModules = @(" in resilience_package
    assert "docs/modules/harmonics_power_flow" in resilience_package
    assert "web/examples/ev_traffic_scenario_template.json" in resilience_package
    for forbidden_copy in (
        'Copy-RequiredTree (Join-Path $repoRoot "web")',
        'Copy-RequiredTree (Join-Path $repoRoot "data")',
        'Copy-RequiredTree (Join-Path $repoRoot "docs")',
        'Copy-RequiredTree (Join-Path $repoRoot "external_data")',
    ):
        assert forbidden_copy not in resilience_package
    assert "ResourcePolicy=explicit-allowlist" in resilience_package
    assert "RuntimeDllPolicy=explicit-allowlist+recursive-dumpbin-audit" in resilience_package
    assert "package_manifest.csv" in resilience_package
    assert "Get-FileHash" in resilience_package
    assert '"$zipPath.sha256"' in resilience_package
    assert "verify_resilience_windows_release.ps1" in resilience_package

    # The archive verifier must validate integrity before executing, extract to
    # a clean path containing spaces, isolate PATH, and exercise both retained
    # and fail-closed HTTP behavior from packaged resources.
    assert "[System.IO.Path]::IsPathRooted($ZipPath)" in resilience_verifier
    assert "Expand-Archive" in resilience_verifier
    assert "package_manifest.csv" in resilience_verifier
    assert "Verification path must contain spaces" in resilience_verifier
    assert '$env:PATH = "$packageBin;$env:SystemRoot\\System32;$env:SystemRoot"' in resilience_verifier
    assert "/api/edition" in resilience_verifier
    assert '"$base/xjtu/"' in resilience_verifier
    assert "Assert-ServedFileMatchesPackage" in resilience_verifier
    assert "docs/modules/resilience/README.md" in resilience_verifier
    assert "/api/session/run_distribution_resilience" in resilience_verifier
    assert "/api/session/run_market_clearing" in resilience_verifier
    assert "EDITION_FEATURE_DISABLED" in resilience_verifier
    assert "analysis = 'carbon_flow'" in resilience_verifier
    assert "details.feature -ne 'carbon_flow'" in resilience_verifier
    assert "StatusCode -ne 403" in resilience_verifier
    assert "analysis = 'not_a_registered_analysis'" in resilience_verifier
    assert "StatusCode -ne 400" in resilience_verifier
    assert "analysis = 'power_flow'" in resilience_verifier
    assert "power_flow_result_v1" in resilience_verifier
    assert "/api/session/generate_typhoon_faults" in resilience_verifier
    assert "catalog_loaded_from_disk" in resilience_verifier
    assert "external_data/typhoon/sst_monthly_south_china_sea.json" in resilience_verifier
    assert "GetResponseStream" in resilience_verifier
    assert "ReadAsStringAsync" in resilience_verifier

    assert "Expand-Archive" in full_verifier
    assert "package_manifest.csv" in full_verifier
    assert "$env:SystemRoot\\System32" in full_verifier

    top_level_cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    dependencies = (ROOT / "cmake/Dependencies.cmake").read_text(encoding="utf-8")
    provenance = (ROOT / "cmake/MIPSolversProvenance.cmake").read_text(encoding="utf-8")
    lock = json.loads((ROOT / "cmake/MIPSolvers.lock.json").read_text(encoding="utf-8"))
    assert "_hacdcpf_default_vcpkg_prefix" not in top_level_cmake
    assert "_hacdcpf_default_vcpkg_prefix" not in dependencies
    assert "../MIPSolvers" not in dependencies
    assert "MIPSolversProvenance.cmake" in dependencies
    assert "file(SHA256" in provenance
    assert lock["schema"] == "hacdcpf.dependency-lock.v1"
    assert lock["upstream"]["commit"] == "c6f77f297350b357ff30cc96d9234b2031fd316c"
    assert lock["import"]["git_tree"] == "782f8745b7e3d39754d02999eeccfe11c04528dc"
    print("Windows packaging contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
