<#
.SYNOPSIS
Stage a relocatable static oneMKL bundle for hermetic Windows builds.

.DESCRIPTION
Copies the complete oneMKL header tree, the LP64/sequential/core static
libraries, and the controlling license material into third_party/oneapi-mkl.
The resulting bundle contains no machine-specific absolute paths.

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
    "mkl_sequential.lib",
    "mkl_core.lib"
)
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
    'set(MIPSOLVERS_LOCAL_MKL_THREADING "sequential")'
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
