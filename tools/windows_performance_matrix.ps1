param(
    [ValidateSet('Threads', 'IPO', 'AVX2', 'PGO', 'Restore')]
    [string]$Stage = 'Threads',
    [string]$BuildDir = 'build/windows-msvc-release',
    [string]$OutputDir = 'reports/windows_fix_20260911/matrix'
)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
Set-Location -LiteralPath $repoRoot
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
$OutputDir = (Resolve-Path $OutputDir).Path
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$vsRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsRoot) { throw 'A local x64 MSVC toolchain is required' }
$vsCommand = '"' + $vsRoot + '\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul && set'
foreach ($line in (cmd /c $vsCommand)) {
    if ($line -match '^([^=]+)=(.*)$') {
        [Environment]::SetEnvironmentVariable($matches[1], $matches[2], 'Process')
    }
}
$env:OMP_NUM_THREADS = '1'
$env:MKL_DYNAMIC = 'FALSE'
$env:MKL_NUM_THREADS = '1'
$env:MIPSOLVERS_BENCH_GIT_COMMIT = git rev-parse HEAD
# R5 in docs/archive/windows_remediation_2026-09-11.md fixes the protocol.
function Invoke-Logged([string]$Name, [string]$Executable, [string[]]$Arguments) {
    $logPath = Join-Path $OutputDir ($Name + '.log')
    [ordered]@{ executable=$Executable; arguments=$Arguments; commit=$env:MIPSOLVERS_BENCH_GIT_COMMIT;
        omp_threads=$env:OMP_NUM_THREADS; mkl_threads=$env:MKL_NUM_THREADS;
        started_utc=[DateTime]::UtcNow.ToString('o') } | ConvertTo-Json -Depth 5 |
        Set-Content (Join-Path $OutputDir ($Name + '.command.json'))
    & $Executable @Arguments > $logPath 2>&1
    if ($LASTEXITCODE -ne 0) { throw "$Name failed with exit $LASTEXITCODE; see $logPath" }
}
function Configure-Build([string]$Name, [string]$Threading, [string]$IPO='OFF', [string]$Arch='OFF', [string]$PGO='OFF') {
    Invoke-Logged "$Name-configure" 'cmake' @('-S', '.', '-B', $BuildDir,
        "-DMIPSOLVERS_MKL_THREADING=$Threading", "-DMIPSOLVERS_ENABLE_IPO=$IPO",
        "-DMIPSOLVERS_ENABLE_NATIVE_ARCH=$Arch", "-DMIPSOLVERS_PGO_MODE=$PGO",
        "-DMIPSOLVERS_PGO_PROFILE=$OutputDir/pgo")
    $targets = @('netlib_solver_benchmark', 'nlp_open_benchmark')
    if ($PGO -ne 'OFF') { $targets = @('netlib_solver_benchmark') }
    Invoke-Logged "$Name-build" 'cmake' (@('--build', $BuildDir, '--config', 'Release', '--parallel', '6', '--target') + $targets)
    Copy-Item -LiteralPath "$BuildDir/CMakeCache.txt" -Destination "$OutputDir/$Name-cache.txt"
    Get-FileHash tests/Release/netlib_solver_benchmark.exe -Algorithm SHA256 |
        ConvertTo-Json | Set-Content "$OutputDir/$Name-binary.json"
}
function Measure-Netlib([string]$Name, [string]$Cases='') {
    $benchmarkArgs = @('--data-dir', 'tests/data', '--solvers', 'native-ipm-direct',
        '--repeat', '3', '--time-limit', '15', '--max-iterations', '100000', '--json', "$OutputDir/$Name.json")
    if ($Cases) { $benchmarkArgs += @('--cases', $Cases) }
    Invoke-Logged $Name 'tests/Release/netlib_solver_benchmark.exe' $benchmarkArgs
    $data = Get-Content "$OutputDir/$Name.json" -Raw | ConvertFrom-Json
    if (@($data.runs | Where-Object { -not $_.accurate }).Count -ne 0) {
        throw "$Name failed the fixed NETLIB accuracy gate"
    }
}
switch ($Stage) {
    'Threads' {
        Configure-Build 'intel' 'INTEL'
        foreach ($threads in 1,2,4,8) {
            $env:MKL_NUM_THREADS = [string]$threads
            Measure-Netlib "intel-$threads"
            Invoke-Logged "intel-$threads-nlp" 'tests/Release/nlp_open_benchmark.exe' @('5', '1e-7')
        }
    }
    'IPO' {
        Configure-Build 'ipo' 'SEQUENTIAL' 'ON'
        Measure-Netlib 'ipo'
        Invoke-Logged 'ipo-nlp' 'tests/Release/nlp_open_benchmark.exe' @('5', '1e-7')
    }
    'AVX2' {
        Configure-Build 'avx2' 'SEQUENTIAL' 'OFF' 'ON'
        Measure-Netlib 'avx2'
        Invoke-Logged 'avx2-nlp' 'tests/Release/nlp_open_benchmark.exe' @('5', '1e-7')
    }
    'PGO' {
        Configure-Build 'pgo-generate' 'SEQUENTIAL' 'OFF' 'OFF' 'GENERATE'
        Measure-Netlib 'pgo-training' 'afiro,adlittle,blend,sc50a,sc105'
        Configure-Build 'pgo-use' 'SEQUENTIAL' 'OFF' 'OFF' 'USE'
        Measure-Netlib 'pgo-use'
        # Holdout analysis excludes the five training cases above.
    }
    'Restore' { Configure-Build 'restored' 'SEQUENTIAL' }
}
