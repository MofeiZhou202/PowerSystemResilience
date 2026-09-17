[CmdletBinding()]
param(
  [string]$BuildDir = "build/windows-msvc-release",
  [string]$OutputDir = "dist/HySim-Windows-x64",
  [switch]$SkipBuild,
  [switch]$SourceDependencies
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$defaultBuildDir = "build/windows-msvc-release"
if ($SourceDependencies -and $BuildDir -eq $defaultBuildDir) {
  $BuildDir = "build/windows-source-release"
}
$buildRoot = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "build"))
$distRoot = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "dist"))
$buildPath = [System.IO.Path]::GetFullPath((Join-Path $repoRoot $BuildDir))
$outputPath = [System.IO.Path]::GetFullPath((Join-Path $repoRoot $OutputDir))

if (-not $buildPath.StartsWith($buildRoot + [System.IO.Path]::DirectorySeparatorChar,
    [System.StringComparison]::OrdinalIgnoreCase)) {
  throw "Windows build directory must be below $buildRoot"
}
if (-not $outputPath.StartsWith($distRoot + [System.IO.Path]::DirectorySeparatorChar,
    [System.StringComparison]::OrdinalIgnoreCase)) {
  throw "Windows package directory must be below $distRoot"
}

function Assert-CleanGitCheckout([string]$Path, [string]$Name) {
  if (-not (Test-Path (Join-Path $Path ".git"))) { return }
  $status = & git -C $Path status --porcelain
  if ($LASTEXITCODE -ne 0 -or $status) {
    throw "$Name must be a clean checkout before creating a release package."
  }
}

Assert-CleanGitCheckout $repoRoot "HySim"

if (-not $SkipBuild) {
  $preset = if ($SourceDependencies) { "windows-source-release" } else { "windows-msvc-release" }
  & cmake --preset $preset -S $repoRoot -B $buildPath
  if ($LASTEXITCODE -ne 0) { throw "Windows CMake configure failed" }
}

$cachePath = Join-Path $buildPath "CMakeCache.txt"
if (-not (Test-Path $cachePath)) {
  throw "Missing Windows CMake cache: $cachePath"
}
$cache = Get-Content -LiteralPath $cachePath -Raw
$requiredCache = @(
  'HACDCPF_ENABLE_IPO:BOOL=OFF',
  'HACDCPF_ENABLE_IPOPT:BOOL=ON',
  'HACDCPF_TRIAL_EDITION:BOOL=OFF',
  'HACDCPF_USE_GUROBI:BOOL=OFF',
  'HACDCPF_USE_SUITESPARSE:BOOL=ON',
  'MIPSOLVERS_ENABLE_IPO:BOOL=OFF',
  'MIPSOLVERS_MKL_THREADING:STRING=SEQUENTIAL',
  'MIPSOLVERS_USE_GUROBI:BOOL=OFF'
)
foreach ($entry in $requiredCache) {
  if (-not $cache.Contains($entry)) {
    throw "Windows release invariant missing: $entry"
  }
}
if ($cache -notmatch '(?m)^MIPSOLVERS_IPOPT_LINEAR_SOLVER:[^=]+=pardisomkl\r?$') {
  throw "PardisoMKL is required"
}
$expectedPrebuilt = if ($SourceDependencies) { "OFF" } else { "ON" }
if ($cache -notmatch "(?m)^MIPSOLVERS_USE_PREBUILT_THIRD_PARTY:[^=]+=$expectedPrebuilt\r?`$") {
  throw "Dependency build mode mismatch: expected prebuilt=$expectedPrebuilt"
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
  throw "MIPSolvers source cannot be resolved from the CMake cache or repository layout."
}
Assert-CleanGitCheckout $mipSolversDir "MIPSolvers"

if (Test-Path (Join-Path $mipSolversDir ".git")) {
  $expectedMatch = [regex]::Match(
    $cache, '(?m)^_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT:STRING=(.+)$')
  $actualCommit = (& git -C $mipSolversDir rev-parse HEAD).Trim()
  if (-not $expectedMatch.Success -or
      $actualCommit -ne $expectedMatch.Groups[1].Value.Trim()) {
    throw "MIPSolvers HEAD does not match the HySim dependency pin."
  }
}

if (-not $SkipBuild) {
  & cmake --build $buildPath --config Release --parallel 8
  if ($LASTEXITCODE -ne 0) { throw "Windows Release build failed" }
  & ctest --test-dir $buildPath -C Release -R '^SolverCapabilities:' `
      --no-tests=error --output-on-failure
  if ($LASTEXITCODE -ne 0) { throw "Windows package acceptance tests failed" }
}

$exeCandidates = @(
  (Join-Path $buildPath "tests/Release/run_gui_server.exe"),
  (Join-Path $buildPath "Release/run_gui_server.exe"),
  (Join-Path $buildPath "run_gui_server.exe")
)
$serverExe = $exeCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $serverExe) { throw "run_gui_server.exe was not found below $buildPath" }

if (Test-Path $outputPath) {
  Remove-Item -LiteralPath $outputPath -Recurse -Force
}
New-Item -ItemType Directory -Path (Join-Path $outputPath "bin") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $outputPath "licenses") -Force | Out-Null
Copy-Item -LiteralPath $serverExe -Destination (Join-Path $outputPath "bin/run_gui_server.exe")
foreach ($resource in @("data", "docs", "external_data", "web")) {
  $source = Join-Path $repoRoot $resource
  if (-not (Test-Path $source)) { throw "Required package resource is missing: $resource" }
  Copy-Item -LiteralPath $source -Destination (Join-Path $outputPath $resource) -Recurse
}
Copy-Item -LiteralPath (Join-Path $repoRoot "LICENSE") -Destination (Join-Path $outputPath "licenses/HySim-LICENSE")
Copy-Item -LiteralPath (Join-Path $repoRoot "README.md") -Destination $outputPath
Copy-Item -LiteralPath (Join-Path $repoRoot "third_party/OpenXLSX-master/LICENSE.md") `
  -Destination (Join-Path $outputPath "licenses/OpenXLSX-LICENSE.md")

$mklLicenseRoot = Join-Path $mipSolversDir "third_party/install/share/mipsolvers-third-party/licenses/oneapi-mkl"
if ($SourceDependencies) {
  $mklLicenseRoot = Join-Path $mipSolversDir "third_party/oneapi-mkl/licensing"
}
if (-not (Test-Path $mklLicenseRoot)) { throw "Packaged oneMKL license material is missing" }
Copy-Item -LiteralPath $mklLicenseRoot -Destination (Join-Path $outputPath "licenses/oneapi-mkl") -Recurse

$dependencyLicenseSources = @(
  "third_party/boost_papilo/LICENSE_1_0.txt",
  "highs/io/filereaderlp/LICENSE",
  "scip/amplmp/LICENSE.rst",
  "scip/cppad/COPYING",
  "scip/dejavu/LICENSE",
  "scip/tclique/LICENSE",
  "third_party/eigen/COPYING.MPL2",
  "third_party/eigen/COPYING.BSD",
  "third_party/nlohmann_json/LICENSE.MIT",
  "third_party/catch2/LICENSE.txt",
  "third_party/papilo/LICENSE",
  "third_party/fmt/LICENSE",
  "mumps/LICENSE",
  "suitesparse/KLU/Doc/License.txt",
  "suitesparse/UMFPACK/Doc/License.txt",
  "suitesparse/CHOLMOD/Doc/License.txt"
)
foreach ($relative in $dependencyLicenseSources) {
  $source = Join-Path $mipSolversDir $relative
  if (-not (Test-Path $source)) { throw "Dependency license is missing: $relative" }
  $safeName = $relative.Replace('/', '-').Replace('\', '-')
  Copy-Item -LiteralPath $source -Destination (Join-Path $outputPath "licenses/$safeName")
}
Copy-Item -Path (Join-Path $PSScriptRoot "windows_licenses/*") `
  -Destination (Join-Path $outputPath "licenses")

$vcRedistCandidates = @()
if ($env:VCToolsRedistDir) { $vcRedistCandidates += $env:VCToolsRedistDir }
$vsRoot = Join-Path ${env:ProgramFiles} "Microsoft Visual Studio/2022"
$vsInstallRoots = @()
if (Test-Path $vsRoot) {
  $vsInstallRoots += Get-ChildItem -LiteralPath $vsRoot -Directory |
    Select-Object -ExpandProperty FullName
}
$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio/Installer/vswhere.exe"
if (Test-Path $vswhere) {
  $vsInstallRoots += & $vswhere -all -products * -property installationPath
}
foreach ($vsInstallRoot in ($vsInstallRoots | Sort-Object -Unique)) {
  $msvcRedist = Join-Path $vsInstallRoot "VC/Redist/MSVC"
  if (-not (Test-Path $msvcRedist)) { continue }
  $vcRedistCandidates += Get-ChildItem -LiteralPath $msvcRedist -Directory |
    ForEach-Object { Join-Path $_.FullName "x64" } |
    Where-Object { Test-Path (Join-Path $_ "Microsoft.VC143.CRT") } |
    Sort-Object -Descending
}
$vcRedistRoot = $vcRedistCandidates | Where-Object {
  Test-Path (Join-Path $_ "Microsoft.VC143.CRT")
} | Select-Object -First 1
if (-not $vcRedistRoot) { throw "Visual C++ 2022 x64 redistributable files were not found" }
foreach ($runtimeGroup in @("Microsoft.VC143.CRT", "Microsoft.VC143.OpenMP")) {
  $runtimeDir = Join-Path $vcRedistRoot $runtimeGroup
  if (Test-Path $runtimeDir) {
    Get-ChildItem -LiteralPath $runtimeDir -Filter "*.dll" -File |
      ForEach-Object {
        Copy-Item -LiteralPath $_.FullName `
          -Destination (Join-Path $outputPath "bin") -Force
      }
  }
}

$dumpbin = Get-Command dumpbin.exe -ErrorAction SilentlyContinue |
  Select-Object -First 1 -ExpandProperty Source
if (-not $dumpbin -and (Test-Path $vsRoot)) {
  $dumpbin = Get-ChildItem -LiteralPath $vsRoot -Filter dumpbin.exe -Recurse -File |
    Where-Object { $_.FullName -match 'Hostx64[\\/]x64' } |
    Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
}
if (-not $dumpbin) { throw "dumpbin.exe is required for the runtime dependency audit" }
$stagedExe = Join-Path $outputPath "bin/run_gui_server.exe"
$dumpbinOutput = (& $dumpbin /DEPENDENTS $stagedExe 2>&1 | Out-String)
$dependencies = [regex]::Matches(
  $dumpbinOutput, '(?im)^\s+([A-Za-z0-9_.-]+\.dll)\s*$') |
  ForEach-Object { $_.Groups[1].Value } | Sort-Object -Unique
if ($dependencies -match '^gurobi') {
  throw "The distributable executable must not depend on a local Gurobi runtime"
}
$systemDlls = @('KERNEL32.dll', 'WS2_32.dll', 'ADVAPI32.dll', 'SHELL32.dll',
  'USER32.dll', 'OLE32.dll', 'OLEAUT32.dll', 'CRYPT32.dll', 'bcrypt.dll',
  'ntdll.dll', 'UCRTBASE.dll')
foreach ($dll in $dependencies) {
  if ($dll -like 'api-ms-win-*.dll' -or $dll -in $systemDlls) { continue }
  if (-not (Test-Path (Join-Path $outputPath "bin/$dll"))) {
    throw "Unpackaged runtime dependency: $dll"
  }
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
    if ($server.HasExited) { throw "HySim server exited before startup." }
    try {
      $profile = Invoke-RestMethod -Uri "http://127.0.0.1:8088/api/edition" -TimeoutSec 1
      if ($profile.edition -eq 'full') { Start-Process $url; break }
    } catch { Start-Sleep -Milliseconds 100 }
  }
  if ($attempt -ge 100) { throw "HySim server did not become ready." }
} finally { Pop-Location }
'@
Set-Content -LiteralPath (Join-Path $outputPath "Start-HySim.ps1") -Value $startScript -Encoding utf8
$cmdScript = '@powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Start-HySim.ps1"'
Set-Content -LiteralPath (Join-Path $outputPath "Start-HySim.cmd") -Value $cmdScript -Encoding ascii

$hysimCommit = if (Test-Path (Join-Path $repoRoot ".git")) {
  (& git -C $repoRoot rev-parse HEAD).Trim()
} else { "source-archive" }
$mipCommit = if (Test-Path (Join-Path $mipSolversDir ".git")) {
  (& git -C $mipSolversDir rev-parse HEAD).Trim()
} else { "source-archive" }
$buildInfo = @(
  "Product=HySim-XJTU-HRPES",
  "Edition=full",
  "Architecture=Windows-x64",
  "Configuration=Release",
  "HySimCommit=$hysimCommit",
  "MIPSolversCommit=$mipCommit",
  "Ipopt=ON",
  "IpoptLinearSolver=PardisoMKL",
  "MKLThreading=SEQUENTIAL",
  "SuiteSparse=ON",
  "Gurobi=OFF",
  "DependencyMode=$(if ($SourceDependencies) { 'source' } else { 'prebuilt' })",
  "PackageAcceptance=solver-capabilities+standalone-edition-and-frontend-smoke"
)
Set-Content -LiteralPath (Join-Path $outputPath "BUILD_INFO.txt") -Value $buildInfo -Encoding ascii

$portProbe = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, 0)
$portProbe.Start()
$smokePort = ([System.Net.IPEndPoint]$portProbe.LocalEndpoint).Port
$portProbe.Stop()
$smokeProcess = Start-Process -FilePath $stagedExe `
  -ArgumentList @('--host', '127.0.0.1', '--port', $smokePort,
    '--data-dir', (Join-Path $outputPath 'data'),
    '--matpower-dir', (Join-Path $outputPath 'external_data/matpower')) `
  -WorkingDirectory $outputPath -PassThru -WindowStyle Hidden
try {
  $ready = $false
  for ($attempt = 0; $attempt -lt 100; $attempt++) {
    if ($smokeProcess.HasExited) { break }
    try {
      $profile = Invoke-RestMethod -Uri "http://127.0.0.1:$smokePort/api/edition" -TimeoutSec 1
      $frontend = Invoke-WebRequest -UseBasicParsing -Uri "http://127.0.0.1:$smokePort/xjtu/" -TimeoutSec 1
      if ($profile.edition -eq 'full' -and $frontend.StatusCode -eq 200) {
        $ready = $true
        break
      }
    } catch { Start-Sleep -Milliseconds 100 }
  }
  if (-not $ready) { throw "Staged Windows package failed its standalone startup smoke test" }
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
$manifest | Sort-Object RelativePath |
  Export-Csv -LiteralPath $manifestPath -NoTypeInformation -Encoding utf8

$zipPath = "$outputPath.zip"
if (Test-Path $zipPath) { Remove-Item -LiteralPath $zipPath -Force }
Compress-Archive -LiteralPath $outputPath -DestinationPath $zipPath -CompressionLevel Optimal
$zipHash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash
Set-Content -LiteralPath "$zipPath.sha256" `
  -Value "$zipHash  $([System.IO.Path]::GetFileName($zipPath))" -Encoding ascii
Write-Host "Windows package: $zipPath"
Write-Host "SHA-256: $zipHash"
