<#
.SYNOPSIS
Stage a relocatable static oneMKL bundle for hermetic Windows builds.

.DESCRIPTION
Copies the complete oneMKL header tree, the LP64/sequential/core static
libraries, and the controlling license material into third_party/oneapi-mkl.
The resulting bundle contains no machine-specific absolute paths.
With -Threading intel or both, the Intel threading layer (mkl_intel_thread)
and the Intel OpenMP runtime (libiomp5md import library + DLL) are staged as
well; the DLL is required at runtime by executables linked against
mkl_intel_thread.

.EXAMPLE
.\third_party\stage_onemkl.ps1 -SourceRoot $env:MKLROOT -Force

.EXAMPLE
.\third_party\stage_onemkl.ps1 `
  -SourceRoot C:\Intel\oneapi\mkl\2026.0 `
  -LicensePath C:\Intel\oneapi\licensing `
  -Destination D:\offline-deps\oneapi-mkl
#>
[CmdletBinding()]
param(
    [string]$SourceRoot = $env:MKLROOT,
    [string]$Destination = "",
    [string]$Version = "",
    [string]$LicensePath = "",
    [string]$CompilerRoot = "",
    [ValidateSet("sequential", "intel", "both")]
    [string]$Threading = "sequential",
    [switch]$Force
)

$ErrorActionPreference = "Stop"
$PSVersionRequired = [Version]"5.1"
if ($PSVersionTable.PSVersion -lt $PSVersionRequired) {
    throw "PowerShell 5.1 or newer is required."
}
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir "..")).Path

if ([string]::IsNullOrWhiteSpace($SourceRoot)) {
    throw "SourceRoot is required. Install/extract oneMKL, or initialize MKLROOT first."
}
$SourceRoot = (Resolve-Path -LiteralPath $SourceRoot).Path

if ([string]::IsNullOrWhiteSpace($Destination)) {
    $Destination = Join-Path $RepoRoot "third_party\oneapi-mkl"
} elseif (-not [System.IO.Path]::IsPathRooted($Destination)) {
    $Destination = [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $Destination))
}
$Destination = [System.IO.Path]::GetFullPath($Destination)

$IncludeSource = Join-Path $SourceRoot "include"
$PardisoHeader = Join-Path $IncludeSource "mkl_pardiso.h"
if (-not (Test-Path -LiteralPath $PardisoHeader -PathType Leaf)) {
    throw "mkl_pardiso.h was not found under $IncludeSource."
}

$LibraryDirs = @(
    (Join-Path $SourceRoot "lib"),
    (Join-Path $SourceRoot "lib\intel64")
)
$RequiredLibraries = @(
    "mkl_intel_lp64.lib",
    "mkl_core.lib"
)
if ($Threading -in @("sequential", "both")) {
    $RequiredLibraries += "mkl_sequential.lib"
}
if ($Threading -in @("intel", "both")) {
    $RequiredLibraries += "mkl_intel_thread.lib"
}
$LibrarySources = @{}
foreach ($LibraryName in $RequiredLibraries) {
    $ResolvedLibrary = $null
    foreach ($LibraryDir in $LibraryDirs) {
        $Candidate = Join-Path $LibraryDir $LibraryName
        if (Test-Path -LiteralPath $Candidate -PathType Leaf) {
            $ResolvedLibrary = (Resolve-Path -LiteralPath $Candidate).Path
            break
        }
    }
    if (-not $ResolvedLibrary) {
        throw "Required static oneMKL library '$LibraryName' was not found under $SourceRoot."
    }
    $LibrarySources[$LibraryName] = $ResolvedLibrary
}

$OpenMpRuntime = $null
if ($Threading -in @("intel", "both")) {
    # mkl_intel_thread needs the Intel OpenMP runtime: the import library at
    # link time and the DLL at run time. Both live under the oneAPI compiler
    # installation, which is a sibling of the MKL installation.
    if ([string]::IsNullOrWhiteSpace($CompilerRoot)) {
        $OneApiRoot = Split-Path -Parent (Split-Path -Parent $SourceRoot)
        $CompilerCandidates = @(
            $env:INTEL_COMPILER_ROOT,
            (Join-Path $OneApiRoot "compiler\latest"),
            "C:\Program Files (x86)\Intel\oneAPI\compiler\latest",
            "C:\Program Files\Intel\oneAPI\compiler\latest"
        )
        $CompilerRoot = $CompilerCandidates |
            Where-Object { -not [string]::IsNullOrWhiteSpace($_) -and
                (Test-Path -LiteralPath (Join-Path $_ "lib\libiomp5md.lib") -PathType Leaf) } |
            Select-Object -First 1
    }
    if ([string]::IsNullOrWhiteSpace($CompilerRoot)) {
        throw "Threading '$Threading' requires libiomp5md; no oneAPI compiler installation found. Pass -CompilerRoot explicitly."
    }
    $CompilerRoot = (Resolve-Path -LiteralPath $CompilerRoot).Path
    $OpenMpImportLibrary = Join-Path $CompilerRoot "lib\libiomp5md.lib"
    $OpenMpDllCandidates = @(
        (Join-Path $CompilerRoot "bin\libiomp5md.dll"),
        (Join-Path $CompilerRoot "redist\intel64\compiler\libiomp5md.dll")
    )
    $OpenMpDll = $OpenMpDllCandidates |
        Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } |
        Select-Object -First 1
    if (-not $OpenMpDll) {
        throw "libiomp5md.dll was not found under $CompilerRoot."
    }
    $OpenMpRuntime = @{
        ImportLibrary = $OpenMpImportLibrary
        Dll = $OpenMpDll
    }
}

if ([string]::IsNullOrWhiteSpace($LicensePath)) {
    $LicenseCandidates = @(
        (Join-Path $SourceRoot "licensing"),
        (Join-Path $SourceRoot "share\doc\mkl\licensing"),
        (Join-Path (Split-Path -Parent $SourceRoot) "licensing"),
        (Join-Path (Split-Path -Parent (Split-Path -Parent $SourceRoot)) "licensing"),
        (Join-Path $SourceRoot "LICENSE.txt"),
        (Join-Path $SourceRoot "license.txt")
    )
    $LicensePath = $LicenseCandidates |
        Where-Object { Test-Path -LiteralPath $_ } |
        Select-Object -First 1
}
if ([string]::IsNullOrWhiteSpace($LicensePath) -or
    -not (Test-Path -LiteralPath $LicensePath)) {
    throw "oneMKL license material was not found. Pass -LicensePath explicitly."
}
$LicensePath = (Resolve-Path -LiteralPath $LicensePath).Path

if ([string]::IsNullOrWhiteSpace($Version)) {
    $VersionHeader = Join-Path $IncludeSource "mkl_version.h"
    if (Test-Path -LiteralPath $VersionHeader -PathType Leaf) {
        $VersionText = Get-Content -LiteralPath $VersionHeader -Raw
        $Major = [regex]::Match($VersionText, '#define\s+__INTEL_MKL__\s+(\d+)')
        $Minor = [regex]::Match($VersionText, '#define\s+__INTEL_MKL_MINOR__\s+(\d+)')
        $Update = [regex]::Match($VersionText, '#define\s+__INTEL_MKL_UPDATE__\s+(\d+)')
        if ($Major.Success -and $Minor.Success -and $Update.Success) {
            $Version = "{0}.{1}.{2}" -f $Major.Groups[1].Value,
                $Minor.Groups[1].Value, $Update.Groups[1].Value
        }
    }
}
if ([string]::IsNullOrWhiteSpace($Version)) {
    throw "Unable to infer the oneMKL version from mkl_version.h. Pass -Version explicitly."
}
if ($Version -notmatch '^[0-9A-Za-z][0-9A-Za-z._+-]*$') {
    throw "Version contains unsupported characters: '$Version'."
}

if (Test-Path -LiteralPath $Destination) {
    if (-not $Force) {
        throw "Destination already exists: $Destination. Re-run with -Force to replace it."
    }
    Remove-Item -LiteralPath $Destination -Recurse -Force
}

$IncludeDestination = Join-Path $Destination "include"
$LibraryDestination = Join-Path $Destination "lib"
$LicenseDestination = Join-Path $Destination "licensing"
New-Item -ItemType Directory -Path $IncludeDestination, $LibraryDestination,
    $LicenseDestination -Force | Out-Null

Copy-Item -Path (Join-Path $IncludeSource "*") -Destination $IncludeDestination -Recurse -Force
foreach ($LibraryName in $RequiredLibraries) {
    Copy-Item -LiteralPath $LibrarySources[$LibraryName] `
        -Destination (Join-Path $LibraryDestination $LibraryName) -Force
}
if ($OpenMpRuntime) {
    $BinDestination = Join-Path $Destination "bin"
    New-Item -ItemType Directory -Path $BinDestination -Force | Out-Null
    Copy-Item -LiteralPath $OpenMpRuntime.ImportLibrary `
        -Destination (Join-Path $LibraryDestination "libiomp5md.lib") -Force
    Copy-Item -LiteralPath $OpenMpRuntime.Dll `
        -Destination (Join-Path $BinDestination "libiomp5md.dll") -Force
}
if (Test-Path -LiteralPath $LicensePath -PathType Container) {
    Copy-Item -Path (Join-Path $LicensePath "*") -Destination $LicenseDestination -Recurse -Force
} else {
    Copy-Item -LiteralPath $LicensePath -Destination $LicenseDestination -Force
}

$ManifestPath = Join-Path $Destination "manifest.cmake"
$ManifestLines = @(
    '# Generated by third_party/stage_onemkl.ps1.',
    'set(MIPSOLVERS_LOCAL_MKL_MANIFEST_VERSION "1")',
    ('set(MIPSOLVERS_LOCAL_MKL_VERSION "{0}")' -f $Version),
    'set(MIPSOLVERS_LOCAL_MKL_ARCHITECTURE "x64")',
    'set(MIPSOLVERS_LOCAL_MKL_LINKAGE "static")',
    ('set(MIPSOLVERS_LOCAL_MKL_THREADING "{0}")' -f $Threading)
)
[System.IO.File]::WriteAllLines(
    $ManifestPath, $ManifestLines, [System.Text.Encoding]::ASCII)

$DestinationPrefix = $Destination.TrimEnd('\') + '\'
$HashLines = Get-ChildItem -LiteralPath $Destination -Recurse -File |
    Where-Object { $_.Name -ne "SHA256SUMS" } |
    Sort-Object FullName |
    ForEach-Object {
        $RelativePath = $_.FullName.Substring($DestinationPrefix.Length).Replace('\', '/')
        $Hash = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
        "$Hash  $RelativePath"
    }
[System.IO.File]::WriteAllLines(
    (Join-Path $Destination "SHA256SUMS"), $HashLines, [System.Text.Encoding]::ASCII)

Write-Host "Staged static oneMKL $Version bundle at $Destination"
Write-Host "Validate SHA256SUMS before transferring it into a sealed environment."
