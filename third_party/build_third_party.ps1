<#
.SYNOPSIS
Build and install MIPSolvers' vendored third-party package on Windows.

.EXAMPLE
.\third_party\build_third_party.ps1 -Jobs 8 -BuildType Release -Fresh

.EXAMPLE
.\third_party\build_third_party.ps1 -Prefix C:\mipsolvers-deps -BuildType RelWithDebInfo
#>
[CmdletBinding()]
param(
    [ValidateRange(1, 1024)]
    [int]$Jobs = [Environment]::ProcessorCount,

    [string]$Prefix = "",

    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType = "Release",

    [switch]$Fresh
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir "..")).Path
$BuildDir = Join-Path $RepoRoot "third_party\build"
if ([string]::IsNullOrWhiteSpace($Prefix)) {
    $Prefix = Join-Path $RepoRoot "third_party\install"
} elseif (-not [System.IO.Path]::IsPathRooted($Prefix)) {
    $Prefix = [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $Prefix))
}

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    throw "cmake is required but was not found in PATH."
}

if ($Fresh -and (Test-Path -LiteralPath $BuildDir)) {
    Write-Host ">> wiping $BuildDir"
    Remove-Item -LiteralPath $BuildDir -Recurse -Force
}

$Timer = [System.Diagnostics.Stopwatch]::StartNew()
Write-Host ">> configuring third-party-only build ($BuildType) in $BuildDir"
& cmake -S $RepoRoot -B $BuildDir `
    "-DMIPSOLVERS_THIRD_PARTY_ONLY=ON" `
    "-DMIPSOLVERS_THIRD_PARTY_BUILD_CONFIG=$BuildType" `
    "-DCMAKE_CONFIGURATION_TYPES=$BuildType" `
    "-DCMAKE_BUILD_TYPE=$BuildType" `
    "-DCMAKE_INSTALL_PREFIX=$Prefix"
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed with exit code $LASTEXITCODE." }

Write-Host ">> building vendored third-party libraries with $Jobs jobs"
& cmake --build $BuildDir --config $BuildType --parallel $Jobs
if ($LASTEXITCODE -ne 0) { throw "CMake build failed with exit code $LASTEXITCODE." }

Write-Host ">> installing to $Prefix"
& cmake --install $BuildDir --config $BuildType
if ($LASTEXITCODE -ne 0) { throw "CMake install failed with exit code $LASTEXITCODE." }
$Timer.Stop()

$PackageDirs = @(
    (Join-Path $Prefix "lib\cmake\mipsolvers-third-party"),
    (Join-Path $Prefix "lib64\cmake\mipsolvers-third-party"),
    (Join-Path $Prefix "share\mipsolvers-third-party")
)
$PackageDir = $PackageDirs | Where-Object {
    Test-Path -LiteralPath (Join-Path $_ "mipsolversThirdPartyConfig.cmake")
} | Select-Object -First 1

Write-Host ""
Write-Host "== prebuilt third-party summary =="
Write-Host "prefix:     $Prefix"
Write-Host "build type: $BuildType"
Write-Host ("wall time:  {0:mm\:ss}" -f $Timer.Elapsed)
$Libraries = Get-ChildItem -LiteralPath $Prefix -Recurse -File -ErrorAction SilentlyContinue |
    Where-Object { $_.Extension -in @(".lib", ".dll", ".a", ".so", ".dylib") }
if ($Libraries) {
    Write-Host "libraries installed:"
    $Libraries | Sort-Object FullName | ForEach-Object {
        Write-Host ("  {0,10:N0}  {1}" -f $_.Length, $_.FullName)
    }
}
if ($PackageDir) {
    Write-Host "package config: $(Join-Path $PackageDir 'mipsolversThirdPartyConfig.cmake')"
    Write-Host "manifest:       $(Join-Path $PackageDir 'manifest.cmake')"
} else {
    Write-Warning "Installed package config was not found under $Prefix."
}
