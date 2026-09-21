[CmdletBinding()]
param(
  [string]$BuildDir = "build/windows-trial-release",
  [string]$OutputDir = "dist/HySim-Trial-Windows-x64",
  [string]$VcpkgRoot = $env:VCPKG_ROOT,
  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$buildPath = [System.IO.Path]::GetFullPath((Join-Path $repoRoot $BuildDir))
$buildRoot = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "build"))
$outputPath = [System.IO.Path]::GetFullPath((Join-Path $repoRoot $OutputDir))
$distRoot = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "dist"))
if (-not $buildPath.StartsWith($buildRoot + [System.IO.Path]::DirectorySeparatorChar,
    [System.StringComparison]::OrdinalIgnoreCase)) {
  throw "Trial build directory must be below $buildRoot"
}
if (-not $outputPath.StartsWith($distRoot + [System.IO.Path]::DirectorySeparatorChar,
    [System.StringComparison]::OrdinalIgnoreCase)) {
  throw "Trial package directory must be below $distRoot"
}
if (-not $VcpkgRoot -and (Test-Path "C:\vcpkg\scripts\buildsystems\vcpkg.cmake")) {
  $VcpkgRoot = "C:\vcpkg"
}
if ($VcpkgRoot) { $env:VCPKG_ROOT = $VcpkgRoot }

if (-not $SkipBuild -and -not (Test-Path (Join-Path $buildPath "CMakeCache.txt"))) {
  & cmake --preset windows-trial-release
  if ($LASTEXITCODE -ne 0) { throw "Trial CMake configure failed" }
}
$cachePath = Join-Path $buildPath "CMakeCache.txt"
if (-not (Test-Path $cachePath)) { throw "Missing Trial CMake cache: $cachePath" }
$cache = Get-Content -LiteralPath $cachePath -Raw
$requiredCache = @(
  'HACDCPF_TRIAL_EDITION:BOOL=ON',
  'HACDCPF_RESILIENCE_EDITION:BOOL=OFF',
  'HACDCPF_ENABLE_IPO:BOOL=OFF',
  'HACDCPF_ENABLE_OPENDSS:BOOL=ON',
  'HACDCPF_USE_GUROBI:BOOL=OFF',
  'MIPSOLVERS_ENABLE_IPO:BOOL=OFF',
  'MIPSOLVERS_USE_GUROBI:BOOL=OFF'
)
foreach ($entry in $requiredCache) {
  if (-not $cache.Contains($entry)) { throw "Trial build invariant missing: $entry" }
}

$mipSourceMatch = [regex]::Match($cache, '(?m)^MIPSOLVERS_SOURCE_DIR:PATH=(.+)$')
if ($mipSourceMatch.Success -and $mipSourceMatch.Groups[1].Value.Trim()) {
  $mipSolversDir = [System.IO.Path]::GetFullPath($mipSourceMatch.Groups[1].Value.Trim())
} else {
  $mipSolversDir = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "MIPSolvers"))
}
$vendoredMipsolversDir = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "MIPSolvers"))
if (-not $mipSolversDir.Equals($vendoredMipsolversDir,
    [System.StringComparison]::OrdinalIgnoreCase)) {
  throw "Trial releases must use the locked in-repository MIPSolvers import."
}
if (-not (Test-Path (Join-Path $mipSolversDir "CMakeLists.txt"))) {
  throw "Locked in-repository MIPSolvers source is missing."
}
$expectedCommitMatch = [regex]::Match(
  $cache, '(?m)^_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT:STRING=(.+)$')
$expectedTreeMatch = [regex]::Match(
  $cache, '(?m)^_HACDCDSS_MIPSOLVERS_EXPECTED_TREE:STRING=(.+)$')
if (-not $expectedCommitMatch.Success -or -not $expectedTreeMatch.Success) {
  throw "MIPSolvers provenance values are missing from the Trial CMake cache."
}
$lock = Get-Content -LiteralPath (Join-Path $repoRoot "cmake/MIPSolvers.lock.json") |
  ConvertFrom-Json
if ($expectedCommitMatch.Groups[1].Value.Trim() -ne $lock.upstream.commit -or
    $expectedTreeMatch.Groups[1].Value.Trim() -ne $lock.import.git_tree) {
  throw "MIPSolvers CMake provenance values do not match the dependency lock."
}
$trackedTree = (& git -C $repoRoot rev-parse "HEAD:MIPSolvers" 2>$null)
if ($LASTEXITCODE -ne 0) {
  throw "Commit the prefixed MIPSolvers import before creating a Trial release."
}
if ($trackedTree.Trim() -ne $lock.import.git_tree) {
  throw "Committed MIPSolvers subtree does not match the dependency lock."
}
$pendingMipsolvers = & git -C $repoRoot status --porcelain --untracked-files=all -- MIPSolvers
if ($LASTEXITCODE -ne 0 -or $pendingMipsolvers) {
  throw "MIPSolvers import must have no staged, unstaged, or untracked changes before creating a Trial release."
}

if (-not $SkipBuild) {
  & cmake --build $buildPath --config Release --target run_gui_server test_edition_profile --parallel 8
  if ($LASTEXITCODE -ne 0) { throw "Trial server and edition test failed to build" }
  & ctest --test-dir $buildPath -C Release -L edition-trial-unit `
      --no-tests=error --output-on-failure
  if ($LASTEXITCODE -ne 0) { throw "Trial edition unit test failed" }
  & ctest --test-dir $buildPath -C Release -L edition-trial-api-e2e `
      --no-tests=error --output-on-failure
  if ($LASTEXITCODE -ne 0) { throw "Trial acceptance tests failed" }
}

$exeCandidates = @(
  (Join-Path $buildPath "run_gui_server.exe"),
  (Join-Path $buildPath "tests/Release/run_gui_server.exe"),
  (Join-Path $buildPath "Release/run_gui_server.exe")
)
$serverExe = $exeCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $serverExe) { throw "run_gui_server.exe was not found below $buildPath" }

if (Test-Path $outputPath) { Remove-Item -LiteralPath $outputPath -Recurse -Force }
New-Item -ItemType Directory -Path (Join-Path $outputPath "bin") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $outputPath "licenses") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $outputPath "data") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $outputPath "external_data") -Force | Out-Null
Copy-Item -LiteralPath $serverExe -Destination (Join-Path $outputPath "bin/run_gui_server.exe")
Copy-Item -LiteralPath (Join-Path $repoRoot "web") -Destination (Join-Path $outputPath "web") -Recurse
Copy-Item -LiteralPath (Join-Path $repoRoot "docs/operations/trial_edition_design.md") -Destination $outputPath

$dllSearchDirs = @(
  $buildPath,
  (Join-Path $buildPath "tests/Release"),
  (Join-Path $buildPath "Release"),
  (Join-Path $repoRoot "third_party/dss_capi/dss_capi/lib/win_x64")
)
if ($VcpkgRoot) { $dllSearchDirs += Join-Path $VcpkgRoot "installed/x64-windows/bin" }
$requiredDlls = @(
  'altdss_capi.dll',
  'altdss_capi_loader.dll',
  'altdss_oddie_capi.dll',
  'CppIndMach012_AltDSS.dll',
  'CppIndMach012_OpenDSSv10.dll',
  'CppIndMach012_OpenDSSv7.dll',
  'CppIndMach012_OpenDSSv8v9.dll',
  'DSSExtensions.dll',
  'libklusolvex.dll',
  'libwinpthread-1.dll'
)
foreach ($dll in $requiredDlls) {
  $source = $dllSearchDirs | ForEach-Object { Join-Path $_ $dll } |
    Where-Object { Test-Path $_ } | Select-Object -First 1
  if (-not $source) { throw "Required Trial runtime DLL is missing: $dll" }
  Copy-Item -LiteralPath $source -Destination (Join-Path $outputPath "bin/$dll")
}
Copy-Item -LiteralPath (Join-Path $repoRoot "LICENSE") `
  -Destination (Join-Path $outputPath "licenses/HySim-LICENSE")
@('LICENSE.BSD3', 'LICENSE.LGPL3', 'OPENDSS_LICENSE') | ForEach-Object {
  $source = Join-Path $repoRoot "third_party/dss_capi/dss_capi/$_"
  if (-not (Test-Path -LiteralPath $source -PathType Leaf)) {
    throw "Required Trial DSS C-API license is missing: $_"
  }
  Copy-Item -LiteralPath $source -Destination (Join-Path $outputPath "licenses/DSS-CAPI-$_")
}

$externalDataDirs = @(
  'classical_example', 'matpower', 'opendss_ieee_pes', 'profiles',
  'short_circuit_example', 'short_circuit_validation', 'typical_cable_networks'
)
foreach ($name in $externalDataDirs) {
  $source = Join-Path $repoRoot "external_data/$name"
  if (-not (Test-Path $source)) { throw "Required Trial data directory is missing: external_data/$name" }
  Copy-Item -LiteralPath $source -Destination (Join-Path $outputPath "external_data/$name") -Recurse
}
$typhoonOut = Join-Path $outputPath "external_data/typhoon"
New-Item -ItemType Directory -Path $typhoonOut -Force | Out-Null
@('sst_monthly_south_china_sea.json',
  'typhoon_track_catalog_m1-12_n100_h48_dt1.00_seed203000.json') | ForEach-Object {
  $source = Join-Path $repoRoot "external_data/typhoon/$_"
  if (-not (Test-Path $source)) { throw "Required Trial typhoon sample is missing: $_" }
  Copy-Item -LiteralPath $source -Destination $typhoonOut
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
    if ($server.HasExited) { throw "HySim Trial server exited before startup." }
    try {
      $profile = Invoke-RestMethod -Uri "http://127.0.0.1:8088/api/edition" -TimeoutSec 1
      if ($profile.edition -eq 'trial') { Start-Process $url; break }
    } catch { Start-Sleep -Milliseconds 100 }
  }
  if ($attempt -ge 100) { throw "HySim Trial server did not become ready." }
} finally { Pop-Location }
'@
Set-Content -LiteralPath (Join-Path $outputPath "Start-HySim-Trial.ps1") -Value $startScript -Encoding utf8

# Prove that the staged package starts without source/build-directory resources.
$portProbe = [System.Net.Sockets.TcpListener]::new(
  [System.Net.IPAddress]::Loopback, 0)
$portProbe.Start()
$smokePort = ([System.Net.IPEndPoint]$portProbe.LocalEndpoint).Port
$portProbe.Stop()
$smokeProcess = Start-Process -FilePath (Join-Path $outputPath "bin/run_gui_server.exe") `
  -ArgumentList @('--host', '127.0.0.1', '--port', $smokePort, '--data-dir', (Join-Path $outputPath 'data'), '--matpower-dir', (Join-Path $outputPath 'external_data/matpower')) `
  -WorkingDirectory $outputPath -PassThru -WindowStyle Hidden
try {
  $ready = $false
  for ($attempt = 0; $attempt -lt 100; $attempt++) {
    try {
      $profile = Invoke-RestMethod -Uri "http://127.0.0.1:$smokePort/api/edition" -TimeoutSec 1
      if ($profile.edition -eq 'trial') { $ready = $true; break }
    } catch { Start-Sleep -Milliseconds 100 }
  }
  if (-not $ready) { throw "Staged Trial package failed its standalone startup smoke test" }
} finally {
  if (-not $smokeProcess.HasExited) { Stop-Process -Id $smokeProcess.Id -Force }
}

$hysimCommit = (& git -C $repoRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or -not $hysimCommit) {
  throw "Unable to record the HySim source commit"
}
$buildInfo = @(
  'Product=HySim-XJTU-HRPES',
  'Edition=trial',
  'Architecture=Windows-x64',
  'Configuration=Release',
  "HySimCommit=$hysimCommit",
  "MIPSolversCommit=$($lock.upstream.commit)",
  "MIPSolversTree=$($lock.import.git_tree)",
  'OpenDSS=ON',
  'Gurobi=OFF',
  'RuntimeDllPolicy=explicit-allowlist',
  'PackageAcceptance=edition-unit+trial-api-e2e+standalone-smoke'
)
Set-Content -LiteralPath (Join-Path $outputPath "BUILD_INFO.txt") `
  -Value $buildInfo -Encoding ascii

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
$manifest | Sort-Object RelativePath | Export-Csv -LiteralPath $manifestPath -NoTypeInformation -Encoding utf8
$zipPath = "$outputPath.zip"
if (Test-Path $zipPath) { Remove-Item -LiteralPath $zipPath -Force }
Compress-Archive -LiteralPath $outputPath -DestinationPath $zipPath -CompressionLevel Optimal
$zipHash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash
Set-Content -LiteralPath "$zipPath.sha256" -Value "$zipHash  $([System.IO.Path]::GetFileName($zipPath))" -Encoding ascii
Write-Host "Trial package: $zipPath"
Write-Host "SHA-256: $zipHash"
