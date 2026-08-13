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
} elseif (Test-Path (Join-Path $repoRoot "MIPSolvers/CMakeLists.txt")) {
  $mipSolversDir = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "MIPSolvers"))
} else {
  $mipSolversDir = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "../MIPSolvers"))
}
if (-not (Test-Path (Join-Path $mipSolversDir "CMakeLists.txt"))) {
  throw "MIPSolvers source cannot be resolved from the Trial cache or repository layout."
}
if (Test-Path (Join-Path $mipSolversDir ".git")) {
  $mipStatus = & git -C $mipSolversDir status --porcelain
  if ($LASTEXITCODE -ne 0 -or $mipStatus) {
    throw "MIPSolvers must be a clean checkout before creating a Trial release."
  }
  $expectedMatch = [regex]::Match($cache, '(?m)^_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT:STRING=(.+)$')
  $actualCommit = (& git -C $mipSolversDir rev-parse HEAD).Trim()
  if (-not $expectedMatch.Success -or $actualCommit -ne $expectedMatch.Groups[1].Value.Trim()) {
    throw "MIPSolvers HEAD does not match the HySim dependency pin."
  }
}

if (-not $SkipBuild) {
  & cmake --build --preset windows-trial-release --target run_gui_server
  if ($LASTEXITCODE -ne 0) { throw "Trial run_gui_server build failed" }
  & ctest --preset windows-trial-release -L trial --output-on-failure
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
New-Item -ItemType Directory -Path (Join-Path $outputPath "data") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $outputPath "external_data") -Force | Out-Null
Copy-Item -LiteralPath $serverExe -Destination (Join-Path $outputPath "bin/run_gui_server.exe")
Copy-Item -LiteralPath (Join-Path $repoRoot "web") -Destination (Join-Path $outputPath "web") -Recurse
Copy-Item -LiteralPath (Join-Path $repoRoot "docs/trial_edition_design.md") -Destination $outputPath

$dllSearchDirs = @($buildPath, (Join-Path $buildPath "tests/Release"), (Join-Path $buildPath "Release"))
if ($VcpkgRoot) { $dllSearchDirs += Join-Path $VcpkgRoot "installed/x64-windows/bin" }
$requiredDlls = @('altdss_capi.dll')
foreach ($dll in $requiredDlls) {
  $source = $dllSearchDirs | ForEach-Object { Join-Path $_ $dll } |
    Where-Object { Test-Path $_ } | Select-Object -First 1
  if (-not $source) { throw "Required Trial runtime DLL is missing: $dll" }
  Copy-Item -LiteralPath $source -Destination (Join-Path $outputPath "bin/$dll")
}
foreach ($dir in $dllSearchDirs) {
  if (Test-Path $dir) {
    Get-ChildItem -LiteralPath $dir -Filter '*.dll' -File | ForEach-Object {
      Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $outputPath "bin/$($_.Name)") -Force
    }
  }
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
