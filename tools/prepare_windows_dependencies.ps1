<#
.SYNOPSIS
Stage the local Windows dependencies required by the repository presets.

.DESCRIPTION
Builds zlib from the MIPSolvers vendored source, stages a relocatable static
oneMKL bundle from an installed oneMKL tree, and optionally builds the complete
MIPSolvers third-party package. Generated files stay below build/ and never
modify the locked MIPSolvers import.
#>
[CmdletBinding()]
param(
  [string]$MklRoot = $env:MKLROOT,
  [ValidateRange(1, 1024)]
  [int]$Jobs = [Math]::Min(8, [Environment]::ProcessorCount),
  [switch]$BuildPrebuiltPackage,
  [switch]$Fresh
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$mipSolversRoot = Join-Path $repoRoot "MIPSolvers"
$dependencyRoot = Join-Path $repoRoot "build/windows-dependencies"
$zlibSource = Join-Path $mipSolversRoot "third_party/zlib-1.3.1"
$zlibBuild = Join-Path $dependencyRoot "zlib-build"
$zlibPrefix = Join-Path $dependencyRoot "zlib"
$mklPrefix = Join-Path $dependencyRoot "oneapi-mkl"
$prebuiltPrefix = Join-Path $dependencyRoot "mipsolvers-third-party"

if (-not (Test-Path (Join-Path $zlibSource "CMakeLists.txt"))) {
  throw "Vendored zlib source is missing from the locked MIPSolvers import."
}
if ([string]::IsNullOrWhiteSpace($MklRoot)) {
  throw "MklRoot is required. Install oneMKL or initialize MKLROOT first."
}

if ($Fresh -and (Test-Path $dependencyRoot)) {
  Remove-Item -LiteralPath $dependencyRoot -Recurse -Force
}
New-Item -ItemType Directory -Path $dependencyRoot -Force | Out-Null

& cmake -S $zlibSource -B $zlibBuild -G "Visual Studio 17 2022" -A x64 `
  "-DCMAKE_INSTALL_PREFIX=$zlibPrefix"
if ($LASTEXITCODE -ne 0) { throw "zlib configure failed" }
& cmake --build $zlibBuild --config Release --target INSTALL --parallel $Jobs
if ($LASTEXITCODE -ne 0) { throw "zlib build/install failed" }

& (Join-Path $mipSolversRoot "third_party/stage_onemkl.ps1") `
  -SourceRoot $MklRoot -Destination $mklPrefix -Threading sequential -Force
if ($LASTEXITCODE -ne 0) { throw "oneMKL staging failed" }

if ($BuildPrebuiltPackage) {
  & (Join-Path $mipSolversRoot "third_party/build_third_party.ps1") `
    -Prefix $prebuiltPrefix -OneMklRoot $mklPrefix -MklThreading SEQUENTIAL `
    -Jobs $Jobs -BuildType Release -Fresh
  if ($LASTEXITCODE -ne 0) {
    throw "MIPSolvers prebuilt third-party package failed"
  }
}

Write-Host "Windows dependency staging complete: $dependencyRoot"
Write-Host "zlib: $zlibPrefix"
Write-Host "oneMKL: $mklPrefix"
if ($BuildPrebuiltPackage) {
  Write-Host "MIPSolvers third-party package: $prebuiltPrefix"
} else {
  Write-Host "The source preset is ready. Re-run with -BuildPrebuiltPackage for the prebuilt preset."
}
