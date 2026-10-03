[CmdletBinding()]
param(
  [string]$BuildDir = "build/windows-resilience-release",
  [string]$OutputDir = "dist/PowerSystemResilience-Windows-x64",
  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$buildRoot = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "build"))
$distRoot = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "dist"))
$buildPath = [System.IO.Path]::GetFullPath((Join-Path $repoRoot $BuildDir))
$outputPath = [System.IO.Path]::GetFullPath((Join-Path $repoRoot $OutputDir))

if (-not $buildPath.StartsWith($buildRoot + [System.IO.Path]::DirectorySeparatorChar,
    [System.StringComparison]::OrdinalIgnoreCase)) {
  throw "Resilience build directory must be below $buildRoot"
}
if (-not $outputPath.StartsWith($distRoot + [System.IO.Path]::DirectorySeparatorChar,
    [System.StringComparison]::OrdinalIgnoreCase)) {
  throw "Resilience package directory must be below $distRoot"
}

function Assert-CacheEntry([string]$Cache, [string]$Entry) {
  if (-not $Cache.Contains($Entry)) {
    throw "Resilience build invariant missing: $Entry"
  }
}

function Copy-RequiredFile([string]$Source, [string]$Destination) {
  if (-not (Test-Path -LiteralPath $Source -PathType Leaf)) {
    throw "Required Resilience package file is missing: $Source"
  }
  $destinationParent = Split-Path -Parent $Destination
  if ($destinationParent) {
    New-Item -ItemType Directory -Path $destinationParent -Force | Out-Null
  }
  Copy-Item -LiteralPath $Source -Destination $Destination -Force
}

function Copy-RequiredTree([string]$Source, [string]$Destination) {
  if (-not (Test-Path -LiteralPath $Source -PathType Container)) {
    throw "Required Resilience package directory is missing: $Source"
  }
  New-Item -ItemType Directory -Path $Destination -Force | Out-Null
  Copy-Item -Path (Join-Path $Source "*") -Destination $Destination -Recurse -Force
}

function Find-Dumpbin {
  $candidate = Get-Command dumpbin.exe -ErrorAction SilentlyContinue |
    Select-Object -First 1 -ExpandProperty Source
  if ($candidate) { return $candidate }

  $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio/Installer/vswhere.exe"
  if (Test-Path -LiteralPath $vswhere -PathType Leaf) {
    $installRoots = @(& $vswhere -all -products * -property installationPath)
    foreach ($installRoot in ($installRoots | Sort-Object -Descending)) {
      $candidate = Get-ChildItem -LiteralPath (Join-Path $installRoot "VC/Tools/MSVC") `
        -Filter dumpbin.exe -Recurse -File -ErrorAction SilentlyContinue |
        Where-Object { $_.FullName -match 'Hostx64[\\/]x64' } |
        Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
      if ($candidate) { return $candidate }
    }
  }
  return $null
}

function Get-PeDependencies([string]$Dumpbin, [string]$Binary) {
  $output = (& $Dumpbin /DEPENDENTS $Binary 2>&1 | Out-String)
  if ($LASTEXITCODE -ne 0) {
    throw "dumpbin dependency inspection failed for $Binary"
  }
  return @([regex]::Matches(
      $output, '(?im)^\s+([A-Za-z0-9_.-]+\.dll)\s*$') |
    ForEach-Object { $_.Groups[1].Value.ToLowerInvariant() } |
    Sort-Object -Unique)
}

if (-not $SkipBuild) {
  & cmake --preset windows-resilience-release -S $repoRoot -B $buildPath
  if ($LASTEXITCODE -ne 0) { throw "Resilience CMake configure failed" }
}

$cachePath = Join-Path $buildPath "CMakeCache.txt"
if (-not (Test-Path -LiteralPath $cachePath -PathType Leaf)) {
  throw "Missing Resilience CMake cache: $cachePath"
}
$cache = Get-Content -LiteralPath $cachePath -Raw
@(
  'HACDCPF_RESILIENCE_EDITION:BOOL=ON',
  'HACDCPF_TRIAL_EDITION:BOOL=OFF',
  'HACDCPF_ENABLE_IPO:BOOL=OFF',
  'HACDCPF_ENABLE_OPENDSS:BOOL=OFF',
  'HACDCPF_USE_GUROBI:BOOL=OFF',
  'HACDCPF_USE_CPLEX:BOOL=OFF',
  'HACDCPF_USE_SUITESPARSE:BOOL=ON',
  'MIPSOLVERS_ENABLE_IPO:BOOL=OFF',
  'MIPSOLVERS_MKL_THREADING:STRING=SEQUENTIAL',
  'MIPSOLVERS_USE_GUROBI:BOOL=OFF',
  'MIPSOLVERS_USE_CPLEX:BOOL=OFF',
  'MIPSOLVERS_USE_PREBUILT_THIRD_PARTY:BOOL=OFF'
) | ForEach-Object { Assert-CacheEntry $cache $_ }
if ($cache -notmatch '(?m)^MIPSOLVERS_IPOPT_LINEAR_SOLVER:[^=]+=pardisomkl\r?$') {
  throw "Resilience releases require PardisoMKL"
}

$mipSourceMatch = [regex]::Match($cache, '(?m)^MIPSOLVERS_SOURCE_DIR:PATH=(.+)\r?$')
if ($mipSourceMatch.Success -and $mipSourceMatch.Groups[1].Value.Trim()) {
  $mipSolversDir = [System.IO.Path]::GetFullPath($mipSourceMatch.Groups[1].Value.Trim())
} else {
  $mipSolversDir = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "MIPSolvers"))
}
$vendoredMipsolversDir = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "MIPSolvers"))
if (-not $mipSolversDir.Equals($vendoredMipsolversDir,
    [System.StringComparison]::OrdinalIgnoreCase)) {
  throw "Resilience releases must use the locked in-repository MIPSolvers import."
}
if (-not (Test-Path -LiteralPath (Join-Path $mipSolversDir "CMakeLists.txt") -PathType Leaf)) {
  throw "Locked in-repository MIPSolvers source is missing."
}

$expectedCommitMatch = [regex]::Match(
  $cache, '(?m)^_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT:STRING=(.+)\r?$')
$expectedTreeMatch = [regex]::Match(
  $cache, '(?m)^_HACDCDSS_MIPSOLVERS_EXPECTED_TREE:STRING=(.+)\r?$')
if (-not $expectedCommitMatch.Success -or -not $expectedTreeMatch.Success) {
  throw "MIPSolvers provenance values are missing from the Resilience CMake cache."
}
$lockPath = Join-Path $repoRoot "cmake/MIPSolvers.lock.json"
$lock = Get-Content -LiteralPath $lockPath -Raw | ConvertFrom-Json
if ($lock.schema -ne 'hacdcpf.dependency-lock.v1' -or $lock.dependency -ne 'MIPSolvers') {
  throw "Invalid MIPSolvers dependency lock."
}
if ($expectedCommitMatch.Groups[1].Value.Trim() -ne $lock.upstream.commit -or
    $expectedTreeMatch.Groups[1].Value.Trim() -ne $lock.import.git_tree) {
  throw "MIPSolvers CMake provenance values do not match the dependency lock."
}
$trackedTree = (& git -C $repoRoot rev-parse "HEAD:MIPSolvers" 2>$null)
if ($LASTEXITCODE -ne 0 -or -not $trackedTree) {
  throw "Commit the prefixed MIPSolvers import before creating a Resilience release."
}
if ($trackedTree.Trim() -ne $lock.import.git_tree) {
  throw "Committed MIPSolvers subtree does not match the dependency lock."
}
$pendingMipsolvers = & git -C $repoRoot status --porcelain --untracked-files=all -- MIPSolvers
if ($LASTEXITCODE -ne 0 -or $pendingMipsolvers) {
  throw "MIPSolvers import must have no staged, unstaged, or untracked changes before creating a Resilience release."
}

if (-not $SkipBuild) {
  & cmake --build $buildPath --config Release --target run_gui_server test_edition_profile test_solver_capabilities --parallel 6
  if ($LASTEXITCODE -ne 0) { throw "Resilience server and edition tests failed to build" }
  & ctest --test-dir $buildPath -C Release -L edition-resilience-unit `
      --no-tests=error --output-on-failure
  if ($LASTEXITCODE -ne 0) { throw "Resilience edition unit test failed" }
  & ctest --test-dir $buildPath -C Release -R '^SolverCapabilities:' `
      --no-tests=error --output-on-failure
  if ($LASTEXITCODE -ne 0) { throw "Resilience solver capability test failed" }
  & ctest --test-dir $buildPath -C Release -L edition-resilience-api-e2e `
      --no-tests=error --output-on-failure
  if ($LASTEXITCODE -ne 0) { throw "Resilience edition API test failed" }
}

$exeCandidates = @(
  (Join-Path $buildPath "run_gui_server.exe"),
  (Join-Path $buildPath "tests/Release/run_gui_server.exe"),
  (Join-Path $buildPath "Release/run_gui_server.exe")
)
$serverExe = $exeCandidates | Where-Object {
  Test-Path -LiteralPath $_ -PathType Leaf
} | Select-Object -First 1
if (-not $serverExe) { throw "run_gui_server.exe was not found below $buildPath" }

if (Test-Path -LiteralPath $outputPath) {
  Remove-Item -LiteralPath $outputPath -Recurse -Force
}
New-Item -ItemType Directory -Path (Join-Path $outputPath "bin") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $outputPath "licenses") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $outputPath "data") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $outputPath "external_data/matpower") -Force | Out-Null
Copy-RequiredFile $serverExe (Join-Path $outputPath "bin/run_gui_server.exe")

# Resilience resources are an explicit per-file allowlist. Do not replace this
# with a recursive copy of data/, docs/, external_data/, or web/.
$resourceFiles = @(
  @{ Source = 'external_data/matpower/case9.m'; Destination = 'external_data/matpower/case9.m' },
  @{ Source = 'external_data/matpower/case14.m'; Destination = 'external_data/matpower/case14.m' },
  @{ Source = 'external_data/matpower/case24_ieee_rts.m'; Destination = 'external_data/matpower/case24_ieee_rts.m' },
  @{ Source = 'external_data/matpower/case30.m'; Destination = 'external_data/matpower/case30.m' },
  @{ Source = 'external_data/typhoon/sst_monthly_south_china_sea.json'; Destination = 'external_data/typhoon/sst_monthly_south_china_sea.json' },
  @{ Source = 'external_data/typhoon/typhoon_track_catalog_m1-12_n100_h48_dt1.00_seed203000.json'; Destination = 'external_data/typhoon/typhoon_track_catalog_m1-12_n100_h48_dt1.00_seed203000.json' },
  @{ Source = 'web/index.html'; Destination = 'web/index.html' },
  @{ Source = 'web/css/style.css'; Destination = 'web/css/style.css' },
  @{ Source = 'web/css/resilience_portal.css'; Destination = 'web/css/resilience_portal.css' },
  @{ Source = 'web/js/app.js'; Destination = 'web/js/app.js' },
  @{ Source = 'web/js/canvas.js'; Destination = 'web/js/canvas.js' },
  @{ Source = 'web/js/components.js'; Destination = 'web/js/components.js' },
  @{ Source = 'web/js/search_registry.js'; Destination = 'web/js/search_registry.js' },
  @{ Source = 'web/js/core/accessibility.js'; Destination = 'web/js/core/accessibility.js' },
  @{ Source = 'web/js/core/analysis_contracts.js'; Destination = 'web/js/core/analysis_contracts.js' },
  @{ Source = 'web/js/core/api_client.js'; Destination = 'web/js/core/api_client.js' },
  @{ Source = 'web/js/core/help_center.js'; Destination = 'web/js/core/help_center.js' },
  @{ Source = 'web/js/core/help_panel.js'; Destination = 'web/js/core/help_panel.js' },
  @{ Source = 'web/js/core/iec_symbols.js'; Destination = 'web/js/core/iec_symbols.js' },
  @{ Source = 'web/js/core/layout_engine.js'; Destination = 'web/js/core/layout_engine.js' },
  @{ Source = 'web/js/core/layout_graph.js'; Destination = 'web/js/core/layout_graph.js' },
  @{ Source = 'web/js/core/local_bus_diagram.js'; Destination = 'web/js/core/local_bus_diagram.js' },
  @{ Source = 'web/js/core/network_overview.js'; Destination = 'web/js/core/network_overview.js' },
  @{ Source = 'web/js/core/one_line_store.js'; Destination = 'web/js/core/one_line_store.js' },
  @{ Source = 'web/js/core/result_mapping.js'; Destination = 'web/js/core/result_mapping.js' },
  @{ Source = 'web/js/core/resilience_portal.js'; Destination = 'web/js/core/resilience_portal.js' },
  @{ Source = 'web/js/core/resilience_workspace.js'; Destination = 'web/js/core/resilience_workspace.js' },
  @{ Source = 'web/js/core/runtime_diagnostics.js'; Destination = 'web/js/core/runtime_diagnostics.js' },
  @{ Source = 'web/js/core/task_manager.js'; Destination = 'web/js/core/task_manager.js' },
  @{ Source = 'web/js/core/timeseries_window.js'; Destination = 'web/js/core/timeseries_window.js' },
  @{ Source = 'web/vendor/elk-worker.min.js'; Destination = 'web/vendor/elk-worker.min.js' },
  @{ Source = 'web/vendor/elk.bundled.js'; Destination = 'web/vendor/elk.bundled.js' },
  @{ Source = 'web/vendor/marked.min.js'; Destination = 'web/vendor/marked.min.js' },
  @{ Source = 'web/vendor/sql-wasm.js'; Destination = 'web/vendor/sql-wasm.js' },
  @{ Source = 'web/vendor/sql-wasm.wasm'; Destination = 'web/vendor/sql-wasm.wasm' },
  @{ Source = 'web/vendor/ELK-LICENSE.md'; Destination = 'web/vendor/ELK-LICENSE.md' },
  @{ Source = 'web/vendor/SQLJS-LICENSE'; Destination = 'web/vendor/SQLJS-LICENSE' },
  @{ Source = 'web/examples/ac_radial_feeder_example.json'; Destination = 'web/examples/ac_radial_feeder_example.json' },
  @{ Source = 'web/examples/hybrid_acdc_microgrid_example.json'; Destination = 'web/examples/hybrid_acdc_microgrid_example.json' }
)
foreach ($resource in $resourceFiles) {
  Copy-RequiredFile (Join-Path $repoRoot $resource.Source) `
    (Join-Path $outputPath $resource.Destination)
}

# The shared page contains disabled market workspaces, but Resilience must not
# ship their executable feature modules. Remove only those script elements from
# the staged copy; edition-profile gating hides the disabled controls.
$forbiddenWebModules = @(
  'market_navigation.js', 'market_activity.js', 'market_canvas.js',
  'southern_market.js', 'market_weekly_plan.js', 'market_operation.js',
  'market_boundary_controls.js', 'market_forecast.js', 'market_study.js',
  'market_ancillary.js', 'market_realtime.js'
)
$stagedIndexPath = Join-Path $outputPath 'web/index.html'
$stagedIndex = Get-Content -LiteralPath $stagedIndexPath -Raw
foreach ($module in $forbiddenWebModules) {
  $escapedModule = [regex]::Escape($module)
  $stagedIndex = [regex]::Replace(
    $stagedIndex,
    "(?m)^\s*<script\s+src=`"js/core/[^`"]*$escapedModule[^`"]*`"></script>\s*\r?\n?",
    '')
}
Set-Content -LiteralPath $stagedIndexPath -Value $stagedIndex -Encoding utf8
foreach ($module in $forbiddenWebModules) {
  if ($stagedIndex.Contains($module)) {
    throw "Disabled Resilience web module is still referenced: $module"
  }
}

# Ship only retained help content, and generate a manifest that cannot point to
# disabled-feature documentation or source-tree-only files.
$helpSections = @(
  @{ Id = 'guides'; Title = '用户指南'; Entries = @(
      @{ Title = '文档中心总览'; Path = 'README.md'; Tags = @('导航', '入门') },
      @{ Title = '参数系统'; Path = 'guides/parameter_system.zh.md'; Tags = @('参数', '默认值'); Modules = @('parameterLibrary') }
    ) },
  @{ Id = 'tutorials'; Title = '示例教程'; Entries = @(
      @{ Title = '内置算例目录'; Path = 'overview/case_catalog.md'; Tags = @('算例', '测试系统'); Modules = @('modelIO') },
      @{ Title = '可靠性计算与算例指南'; Path = 'guides/reliability_calculation_workflow_and_case_guide.md'; Tags = @('可靠性', '工作流'); Modules = @('reliability') },
      @{ Title = '配电参数补全标准'; Path = 'guides/distribution_parameter_completion_standards.zh.md'; Tags = @('参数', '配电'); Modules = @('parameterLibrary') }
    ) },
  @{ Id = 'modules'; Title = '模块手册'; Entries = @(
      @{ Title = '潮流计算'; Path = 'modules/power_flow/README.md'; Tags = @('潮流', 'NR', 'ACDC'); Modules = @('powerFlow') },
      @{ Title = '最优潮流'; Path = 'modules/optimal_power_flow/README.md'; Tags = @('OPF', 'RPO', '无功'); Modules = @('opf', 'rpo') },
      @{ Title = '短路计算'; Path = 'modules/short_circuit/README.md'; Tags = @('短路', 'IEC60909'); Modules = @('shortCircuit') },
      @{ Title = '可靠性分析'; Path = 'modules/reliability/README.md'; Tags = @('可靠性', 'MC', 'FMEA'); Modules = @('reliability') },
      @{ Title = '弹性恢复'; Path = 'modules/resilience/README.md'; Tags = @('弹性', '恢复', 'MESS'); Modules = @('resilience') },
      @{ Title = '网络重构'; Path = 'modules/network_reconfiguration/README.md'; Tags = @('重构', 'ONR', 'MILP'); Modules = @('topology') },
      @{ Title = '图与拓扑'; Path = 'modules/graph/README.md'; Tags = @('图', '拓扑', '降阶'); Modules = @('topologyAnalysis') },
      @{ Title = '场景生成'; Path = 'modules/scenario_generation/README.zh.md'; Tags = @('场景', '台风'); Modules = @('scenarioGeneration') }
    ) },
  @{ Id = 'api'; Title = 'API 契约'; Entries = @(
      @{ Title = '运行时 HTTP API'; Path = 'reference/runtime_api.zh.md'; Tags = @('HTTP', '路由', 'API') }
    ) },
  @{ Id = 'theory'; Title = '理论模型'; Entries = @(
      @{ Title = 'VSC 限值 NCP 潮流契约'; Path = 'theory/vsc_limit_ncp_power_flow_contract.zh.md'; Tags = @('VSC', 'NCP', '潮流'); Modules = @('powerFlow') },
      @{ Title = '交直流短路推导'; Path = 'theory/short_circuit_rich_acdc_derivation.zh.md'; Tags = @('短路', '推导'); Modules = @('shortCircuit') },
      @{ Title = '网络重构模型'; Path = 'theory/network_reconfiguration_models.zh.md'; Tags = @('重构', '模型'); Modules = @('topology') }
    ) }
)
$helpManifest = [ordered]@{
  schema = 'hysim_help_docs_v1'
  title = 'PowerSystemResilience 文档帮助中心'
  docsBase = '/xjtu/docs/'
  sections = @()
}
foreach ($section in $helpSections) {
  $manifestEntries = @()
  foreach ($entry in $section.Entries) {
    $relativeDoc = $entry.Path
    Copy-RequiredFile (Join-Path $repoRoot "docs/$relativeDoc") `
      (Join-Path $outputPath "docs/$relativeDoc")
    $manifestEntry = [ordered]@{
      title = $entry.Title
      path = $relativeDoc
      tags = @($entry.Tags)
    }
    if ($entry.Modules) { $manifestEntry.modules = @($entry.Modules) }
    $manifestEntries += $manifestEntry
  }
  $helpManifest.sections += [ordered]@{
    id = $section.Id
    title = $section.Title
    entries = $manifestEntries
  }
}
$helpManifest | ConvertTo-Json -Depth 12 |
  Set-Content -LiteralPath (Join-Path $outputPath 'web/help_docs.json') -Encoding utf8

# Reject known disabled feature resources even if a future allowlist edit tries
# to stage one. The denylist complements, but never replaces, the allowlist.
$forbiddenRelativePaths = @(
  'data/dsp',
  'external_data/harmonics_validation',
  'external_data/transient_validation',
  'external_data/transportation_networks',
  'external_data/opendss_ieee_pes',
  'web/examples/ev_traffic_scenario_template.json',
  'web/schemas/ev_traffic_scenario.schema.json',
  'docs/modules/harmonics_power_flow',
  'docs/modules/dynamics',
  'docs/modules/market',
  'docs/modules/integrated_energy',
  'docs/modules/ev_power_traffic',
  'docs/modules/carbon_analysis',
  'docs/modules/analysis'
)
foreach ($relative in $forbiddenRelativePaths) {
  if (Test-Path -LiteralPath (Join-Path $outputPath $relative)) {
    throw "Forbidden Resilience resource was staged: $relative"
  }
}
$forbiddenExtensions = @('.dat', '.dss', '.glm', '.xlsx', '.xml')
$forbiddenFiles = @(Get-ChildItem -LiteralPath $outputPath -Recurse -File | Where-Object {
  $_.Extension.ToLowerInvariant() -in $forbiddenExtensions
})
if ($forbiddenFiles.Count -ne 0) {
  throw "Forbidden Resilience resource was staged: $($forbiddenFiles[0].FullName)"
}

Copy-RequiredFile (Join-Path $repoRoot "LICENSE") `
  (Join-Path $outputPath "licenses/PowerSystemResilience-LICENSE")
Copy-RequiredFile (Join-Path $repoRoot "third_party/OpenXLSX-master/LICENSE.md") `
  (Join-Path $outputPath "licenses/OpenXLSX-LICENSE.md")
$mklLicenseRoot = Join-Path $repoRoot "build/windows-dependencies/oneapi-mkl/licensing"
Copy-RequiredTree $mklLicenseRoot (Join-Path $outputPath "licenses/oneapi-mkl")

$dependencyLicenseSources = @(
  'third_party/boost_papilo/LICENSE_1_0.txt',
  'highs/io/filereaderlp/LICENSE',
  'scip/amplmp/LICENSE.rst',
  'scip/cppad/COPYING',
  'scip/dejavu/LICENSE',
  'scip/tclique/LICENSE',
  'third_party/eigen/COPYING.MPL2',
  'third_party/eigen/COPYING.BSD',
  'third_party/nlohmann_json/LICENSE.MIT',
  'third_party/catch2/LICENSE.txt',
  'third_party/papilo/LICENSE',
  'third_party/fmt/LICENSE',
  'mumps/LICENSE',
  'suitesparse/KLU/Doc/License.txt',
  'suitesparse/UMFPACK/Doc/License.txt',
  'suitesparse/CHOLMOD/Doc/License.txt'
)
foreach ($relative in $dependencyLicenseSources) {
  $safeName = $relative.Replace('/', '-').Replace('\', '-')
  Copy-RequiredFile (Join-Path $mipSolversDir $relative) `
    (Join-Path $outputPath "licenses/$safeName")
}
Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot "windows_licenses") -File |
  ForEach-Object {
    Copy-RequiredFile $_.FullName (Join-Path $outputPath "licenses/$($_.Name)")
  }

$vcRuntimeAllowlist = @(
  'concrt140.dll',
  'msvcp140.dll',
  'msvcp140_1.dll',
  'msvcp140_2.dll',
  'msvcp140_atomic_wait.dll',
  'msvcp140_codecvt_ids.dll',
  'vccorlib140.dll',
  'vcomp140.dll',
  'vcruntime140.dll',
  'vcruntime140_1.dll',
  'vcruntime140_threads.dll'
)
$vcRedistCandidates = @()
if ($env:VCToolsRedistDir) { $vcRedistCandidates += $env:VCToolsRedistDir }
$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio/Installer/vswhere.exe"
if (Test-Path -LiteralPath $vswhere -PathType Leaf) {
  foreach ($installRoot in @(& $vswhere -all -products * -property installationPath)) {
    $redistRoot = Join-Path $installRoot "VC/Redist/MSVC"
    if (-not (Test-Path -LiteralPath $redistRoot -PathType Container)) { continue }
    $vcRedistCandidates += Get-ChildItem -LiteralPath $redistRoot -Directory |
      ForEach-Object { Join-Path $_.FullName "x64" } |
      Where-Object { Test-Path -LiteralPath $_ -PathType Container } |
      Sort-Object -Descending
  }
}
$vcRedistRoot = $vcRedistCandidates | Where-Object {
  Test-Path -LiteralPath (Join-Path $_ "Microsoft.VC143.CRT") -PathType Container
} | Select-Object -First 1
if (-not $vcRedistRoot) {
  throw "Visual C++ 2022 x64 redistributable files were not found"
}
foreach ($dll in $vcRuntimeAllowlist) {
  $source = Get-ChildItem -LiteralPath $vcRedistRoot -Filter $dll -Recurse -File |
    Select-Object -First 1 -ExpandProperty FullName
  if ($source) {
    Copy-RequiredFile $source (Join-Path $outputPath "bin/$dll")
  }
}

$dumpbin = Find-Dumpbin
if (-not $dumpbin) { throw "dumpbin.exe is required for the runtime dependency audit" }
$stagedExe = Join-Path $outputPath "bin/run_gui_server.exe"
$systemDllAllowlist = @(
  'advapi32.dll', 'bcrypt.dll', 'comdlg32.dll', 'crypt32.dll', 'dbghelp.dll',
  'gdi32.dll', 'imm32.dll', 'kernel32.dll', 'ntdll.dll', 'ole32.dll',
  'oleaut32.dll', 'rpcrt4.dll', 'secur32.dll', 'shell32.dll', 'shlwapi.dll',
  'user32.dll', 'userenv.dll', 'ucrtbase.dll', 'version.dll', 'winhttp.dll',
  'winmm.dll', 'ws2_32.dll'
)
$explicitPackageDllAllowlist = @($vcRuntimeAllowlist)
$dependencyQueue = [System.Collections.Generic.Queue[string]]::new()
$dependencyQueue.Enqueue($stagedExe)
$auditedBinaries = [System.Collections.Generic.HashSet[string]]::new(
  [System.StringComparer]::OrdinalIgnoreCase)
$requiredPackageDlls = [System.Collections.Generic.HashSet[string]]::new(
  [System.StringComparer]::OrdinalIgnoreCase)
while ($dependencyQueue.Count -gt 0) {
  $binary = $dependencyQueue.Dequeue()
  if (-not $auditedBinaries.Add($binary)) { continue }
  foreach ($dll in (Get-PeDependencies $dumpbin $binary)) {
    if ($dll -like 'api-ms-win-*.dll' -or $dll -like 'ext-ms-win-*.dll' -or
        $dll -in $systemDllAllowlist) {
      continue
    }
    if ($dll -notin $explicitPackageDllAllowlist) {
      throw "Runtime dependency is not allowlisted for Resilience: $dll"
    }
    $packagedDll = Join-Path $outputPath "bin/$dll"
    if (-not (Test-Path -LiteralPath $packagedDll -PathType Leaf)) {
      throw "Allowlisted Resilience runtime dependency is missing: $dll"
    }
    [void]$requiredPackageDlls.Add($dll)
    $dependencyQueue.Enqueue($packagedDll)
  }
}
Get-ChildItem -LiteralPath (Join-Path $outputPath "bin") -Filter '*.dll' -File |
  ForEach-Object {
    if (-not $requiredPackageDlls.Contains($_.Name)) {
      Remove-Item -LiteralPath $_.FullName -Force
    }
  }
if (@(Get-ChildItem -LiteralPath (Join-Path $outputPath "bin") -Filter '*.dll' -File |
    Where-Object { $_.Name -notin $explicitPackageDllAllowlist }).Count -ne 0) {
  throw "A non-allowlisted DLL was staged in the Resilience package"
}

$startScript = @'
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$url = "http://127.0.0.1:8088/xjtu/"
Push-Location $root
try {
  $server = Start-Process -FilePath (Join-Path $root "bin/run_gui_server.exe") `
    -ArgumentList @('--host', '127.0.0.1', '--port', 8088,
      '--data-dir', (Join-Path $root 'data'),
      '--matpower-dir', (Join-Path $root 'external_data/matpower')) `
    -WorkingDirectory $root -PassThru
  for ($attempt = 0; $attempt -lt 100; $attempt++) {
    if ($server.HasExited) { throw "PowerSystemResilience server exited before startup." }
    try {
      $profile = Invoke-RestMethod -Uri "http://127.0.0.1:8088/api/edition" -TimeoutSec 1
      if ($profile.edition -eq 'resilience') { Start-Process $url; break }
    } catch { Start-Sleep -Milliseconds 100 }
  }
  if ($attempt -ge 100) { throw "PowerSystemResilience server did not become ready." }
} finally { Pop-Location }
'@
Set-Content -LiteralPath (Join-Path $outputPath "Start-PowerSystemResilience.ps1") `
  -Value $startScript -Encoding utf8
$cmdScript = '@powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Start-PowerSystemResilience.ps1"'
Set-Content -LiteralPath (Join-Path $outputPath "Start-PowerSystemResilience.cmd") `
  -Value $cmdScript -Encoding ascii

$hysimCommit = (& git -C $repoRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or -not $hysimCommit) {
  throw "Unable to record the HySim source commit"
}
$buildInfo = @(
  'Product=PowerSystemResilience',
  'Edition=resilience',
  'Architecture=Windows-x64',
  'Configuration=Release',
  "HySimCommit=$hysimCommit",
  "MIPSolversCommit=$($lock.upstream.commit)",
  "MIPSolversTree=$($lock.import.git_tree)",
  'DependencyMode=source',
  'Ipopt=ON',
  'IpoptLinearSolver=PardisoMKL',
  'MKLThreading=SEQUENTIAL',
  'SuiteSparse=ON',
  'Gurobi=OFF',
  'CPLEX=OFF',
  'ResourcePolicy=explicit-allowlist',
  'RuntimeDllPolicy=explicit-allowlist+recursive-dumpbin-audit',
  'PackageAcceptance=edition-unit+solver-capabilities+resilience-api-e2e+clean-extracted-verification'
)
Set-Content -LiteralPath (Join-Path $outputPath "BUILD_INFO.txt") `
  -Value $buildInfo -Encoding ascii

$zipPath = "$outputPath.zip"
if (Test-Path -LiteralPath $zipPath) { Remove-Item -LiteralPath $zipPath -Force }
$manifestPath = Join-Path $outputPath "package_manifest.csv"
$manifest = Get-ChildItem -LiteralPath $outputPath -Recurse -File |
  Where-Object { $_.FullName -ne $manifestPath } |
  ForEach-Object {
    [pscustomobject]@{
      RelativePath = $_.FullName.Substring($outputPath.Length + 1)
      SizeBytes = $_.Length
      Sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
    }
  }
$manifest | Sort-Object RelativePath |
  Export-Csv -LiteralPath $manifestPath -NoTypeInformation -Encoding utf8
Compress-Archive -LiteralPath $outputPath -DestinationPath $zipPath -CompressionLevel Optimal
$zipHash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash
Set-Content -LiteralPath "$zipPath.sha256" `
  -Value "$zipHash  $([System.IO.Path]::GetFileName($zipPath))" -Encoding ascii

# Verify the exact archive, not merely the staging directory.
& (Join-Path $PSScriptRoot "verify_resilience_windows_release.ps1") -ZipPath $zipPath
if ($LASTEXITCODE -ne 0) { throw "Resilience archive verification failed" }
Write-Host "Resilience package: $zipPath"
Write-Host "SHA-256: $zipHash"
