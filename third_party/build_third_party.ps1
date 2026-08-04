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
    [int]$Jobs = [Math]::Min(8, [Environment]::ProcessorCount),

    [string]$Prefix = "",

    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType = "Release",

    [string]$OneMklRoot = "",

    [string]$Generator = "Visual Studio 17 2022",

    [string]$Architecture = "x64",

    [switch]$Fresh
)

$ErrorActionPreference = "Stop"
$PSVersionRequired = [Version]"5.1"
if ($PSVersionTable.PSVersion -lt $PSVersionRequired) {
    throw "PowerShell 5.1 or newer is required."
}
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir "..")).Path
$BuildDir = Join-Path $RepoRoot "third_party\build"
if ([string]::IsNullOrWhiteSpace($Prefix)) {
    $Prefix = Join-Path $RepoRoot "third_party\install"
} elseif (-not [System.IO.Path]::IsPathRooted($Prefix)) {
    $Prefix = [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $Prefix))
}
$Prefix = [System.IO.Path]::GetFullPath($Prefix)

if ([string]::IsNullOrWhiteSpace($OneMklRoot)) {
    $OneMklRoot = Join-Path $RepoRoot "third_party\oneapi-mkl"
} elseif (-not [System.IO.Path]::IsPathRooted($OneMklRoot)) {
    $OneMklRoot = [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $OneMklRoot))
}
if (-not [string]::IsNullOrWhiteSpace($Architecture) -and
    $Architecture -ne "x64") {
    throw "The staged static oneMKL bundle supports only the x64 architecture."
}
if (-not (Test-Path -LiteralPath (Join-Path $OneMklRoot "manifest.cmake") -PathType Leaf) -or
    -not (Test-Path -LiteralPath (Join-Path $OneMklRoot "SHA256SUMS") -PathType Leaf)) {
    throw "A staged oneMKL bundle is required at $OneMklRoot. Run third_party\stage_onemkl.ps1 first."
}
$OneMklRoot = (Resolve-Path -LiteralPath $OneMklRoot).Path

$HashFile = Join-Path $OneMklRoot "SHA256SUMS"
$VerifiedFiles = 0
$ExpectedFiles = @{}
foreach ($HashLine in (Get-Content -LiteralPath $HashFile)) {
    if ($HashLine -notmatch '^([0-9a-fA-F]{64})  (.+)$') {
        throw "Malformed SHA256SUMS line: $HashLine"
    }
    $ExpectedHash = $Matches[1].ToLowerInvariant()
    $ManifestPath = $Matches[2].Replace('\', '/')
    if ([System.IO.Path]::IsPathRooted($ManifestPath) -or
        $ManifestPath.Contains(':') -or
        $ManifestPath -match '(^|/)\.\.(/|$)') {
        throw "Unsafe path in SHA256SUMS: $ManifestPath"
    }
    if ($ExpectedFiles.ContainsKey($ManifestPath)) {
        throw "Duplicate path in SHA256SUMS: $ManifestPath"
    }
    $ExpectedFiles[$ManifestPath] = $true
    $RelativePath = $ManifestPath.Replace(
        [char]'/', [System.IO.Path]::DirectorySeparatorChar)
    $BundleFile = Join-Path $OneMklRoot $RelativePath
    if (-not (Test-Path -LiteralPath $BundleFile -PathType Leaf)) {
        throw "oneMKL bundle file listed in SHA256SUMS is missing: $RelativePath"
    }
    $ActualHash = (Get-FileHash -LiteralPath $BundleFile -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($ActualHash -ne $ExpectedHash) {
        throw "oneMKL bundle hash mismatch: $RelativePath"
    }
    $VerifiedFiles++
}
if ($VerifiedFiles -eq 0) {
    throw "SHA256SUMS contains no files."
}
$BundlePrefix = $OneMklRoot.TrimEnd('\') + '\'
foreach ($BundleEntry in (Get-ChildItem -LiteralPath $OneMklRoot -Recurse -File)) {
    if ($BundleEntry.FullName -eq $HashFile) {
        continue
    }
    $ActualRelativePath = $BundleEntry.FullName.Substring($BundlePrefix.Length).Replace('\', '/')
    if (-not $ExpectedFiles.ContainsKey($ActualRelativePath)) {
        throw "oneMKL bundle contains a file not listed in SHA256SUMS: $ActualRelativePath"
    }
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
$ConfigureArgs = @(
    "-S", $RepoRoot,
    "-B", $BuildDir,
    "-G", $Generator,
    "-DMIPSOLVERS_THIRD_PARTY_ONLY=ON",
    "-DMIPSOLVERS_THIRD_PARTY_BUILD_CONFIG=$BuildType",
    "-DMIPSOLVERS_BUILD_LOCAL_IPOPT=ON",
    "-DMIPSOLVERS_IPOPT_LINEAR_SOLVER=pardisomkl",
    "-DMIPSOLVERS_MKL_ROOT=$OneMklRoot",
    "-DMIPSOLVERS_ENABLE_IPO=OFF",
    "-DCMAKE_CONFIGURATION_TYPES=$BuildType",
    "-DCMAKE_BUILD_TYPE=$BuildType",
    "-DCMAKE_INSTALL_PREFIX=$Prefix"
)
if (-not [string]::IsNullOrWhiteSpace($Architecture)) {
    $ConfigureArgs += @("-A", $Architecture)
}
& cmake @ConfigureArgs
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed with exit code $LASTEXITCODE." }

Write-Host ">> building vendored third-party libraries with $Jobs jobs"
& cmake --build $BuildDir --config $BuildType --parallel $Jobs
if ($LASTEXITCODE -ne 0) { throw "CMake build failed with exit code $LASTEXITCODE." }

Write-Host ">> installing to $Prefix"
& cmake --install $BuildDir --config $BuildType
if ($LASTEXITCODE -ne 0) { throw "CMake install failed with exit code $LASTEXITCODE." }

$PackageDirs = @(
    (Join-Path $Prefix "lib\cmake\mipsolvers-third-party"),
    (Join-Path $Prefix "lib64\cmake\mipsolvers-third-party"),
    (Join-Path $Prefix "share\mipsolvers-third-party")
)
$PackageDir = $PackageDirs | Where-Object {
    Test-Path -LiteralPath (Join-Path $_ "mipsolversThirdPartyConfig.cmake")
} | Select-Object -First 1
if (-not $PackageDir) {
    throw "Installed package config was not found under $Prefix."
}

$SmokeBuildDir = Join-Path $BuildDir "prebuilt-consumer-smoke"
if (Test-Path -LiteralPath $SmokeBuildDir) {
    Remove-Item -LiteralPath $SmokeBuildDir -Recurse -Force
}
Write-Host ">> configuring the relocatable prebuilt consumer smoke test"
$SmokeArgs = @(
    "-S", (Join-Path $RepoRoot "cmake\prebuilt_consumer_smoke"),
    "-B", $SmokeBuildDir,
    "-G", $Generator,
    "-DMIPSOLVERS_PREBUILT_CONFIG=$(Join-Path $PackageDir 'mipsolversThirdPartyConfig.cmake')",
    "-DMIPSOLVERS_EXPECTED_PREFIX=$Prefix"
)
if (-not [string]::IsNullOrWhiteSpace($Architecture)) {
    $SmokeArgs += @("-A", $Architecture)
}
& cmake @SmokeArgs
if ($LASTEXITCODE -ne 0) {
    throw "Prebuilt consumer smoke test failed with exit code $LASTEXITCODE."
}
$Timer.Stop()

Write-Host ""
Write-Host "== prebuilt third-party summary =="
Write-Host "prefix:     $Prefix"
Write-Host "build type: $BuildType"
Write-Host "oneMKL:     $OneMklRoot"
Write-Host ("wall time:  {0:mm\:ss}" -f $Timer.Elapsed)
$Libraries = Get-ChildItem -LiteralPath $Prefix -Recurse -File -ErrorAction SilentlyContinue |
    Where-Object { $_.Extension -in @(".lib", ".dll", ".a", ".so", ".dylib") }
if ($Libraries) {
    Write-Host "libraries installed:"
    $Libraries | Sort-Object FullName | ForEach-Object {
        Write-Host ("  {0,10:N0}  {1}" -f $_.Length, $_.FullName)
    }
}
Write-Host "package config: $(Join-Path $PackageDir 'mipsolversThirdPartyConfig.cmake')"
Write-Host "manifest:       $(Join-Path $PackageDir 'manifest.cmake')"
