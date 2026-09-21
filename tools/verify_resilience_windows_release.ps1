[CmdletBinding()]
param([string]$ZipPath = "dist/PowerSystemResilience-Windows-x64.zip")

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$zipInput = if ([System.IO.Path]::IsPathRooted($ZipPath)) {
  $ZipPath
} else {
  Join-Path $repoRoot $ZipPath
}
$zip = (Resolve-Path -LiteralPath $zipInput).Path
$hashPath = "$zip.sha256"
if (-not (Test-Path -LiteralPath $hashPath -PathType Leaf)) {
  throw "Missing ZIP SHA-256 sidecar: $hashPath"
}
$hashFields = (Get-Content -LiteralPath $hashPath -Raw).Trim() -split '\s+'
if ($hashFields.Count -lt 2 -or $hashFields[1] -ne [System.IO.Path]::GetFileName($zip)) {
  throw "Malformed ZIP SHA-256 sidecar"
}
$expectedZipHash = $hashFields[0].ToUpperInvariant()
if ($expectedZipHash -notmatch '^[0-9A-F]{64}$' -or
    (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash -ne $expectedZipHash) {
  throw "ZIP SHA-256 mismatch"
}

# Extract to a new path containing spaces. It must not inherit source/build
# resources, and package bin plus Windows system directories are the only PATH.
$verificationName = "Resilience release verification " + [guid]::NewGuid().ToString("N")
$extractRoot = Join-Path (Join-Path $repoRoot "build") $verificationName
New-Item -ItemType Directory -Path $extractRoot -Force | Out-Null
Expand-Archive -LiteralPath $zip -DestinationPath $extractRoot
$roots = @(Get-ChildItem -LiteralPath $extractRoot -Directory)
if ($roots.Count -ne 1) { throw "Expected exactly one Resilience package root" }
$packageRoot = $roots[0].FullName
if ($packageRoot -notmatch '\s') { throw "Verification path must contain spaces" }

$manifestPath = Join-Path $packageRoot "package_manifest.csv"
if (-not (Test-Path -LiteralPath $manifestPath -PathType Leaf)) {
  throw "Missing package manifest"
}
$manifest = @(Import-Csv -LiteralPath $manifestPath)
if ($manifest.Count -eq 0) { throw "Empty package manifest" }
$expectedFiles = [System.Collections.Generic.HashSet[string]]::new(
  [System.StringComparer]::OrdinalIgnoreCase)
foreach ($entry in $manifest) {
  if ($entry.RelativePath -eq 'package_manifest.csv') {
    throw "The manifest must not include itself"
  }
  $artifact = [System.IO.Path]::GetFullPath(
    (Join-Path $packageRoot $entry.RelativePath))
  if (-not $artifact.StartsWith(
      $packageRoot + [System.IO.Path]::DirectorySeparatorChar,
      [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Manifest path escapes package: $($entry.RelativePath)"
  }
  if (-not $expectedFiles.Add($artifact)) {
    throw "Duplicate manifest path: $($entry.RelativePath)"
  }
  if (-not (Test-Path -LiteralPath $artifact -PathType Leaf) -or
      (Get-Item -LiteralPath $artifact).Length -ne [long]$entry.SizeBytes -or
      (Get-FileHash -LiteralPath $artifact -Algorithm SHA256).Hash -ne $entry.Sha256) {
    throw "Manifest mismatch: $($entry.RelativePath)"
  }
}
$actualFiles = @(Get-ChildItem -LiteralPath $packageRoot -Recurse -File |
  Where-Object { $_.FullName -ne $manifestPath })
if ($actualFiles.Count -ne $manifest.Count) {
  throw "Package contains files absent from the manifest"
}
foreach ($file in $actualFiles) {
  if (-not $expectedFiles.Contains($file.FullName)) {
    throw "Unmanifested package file: $($file.FullName)"
  }
}

$requiredFiles = @(
  'bin/run_gui_server.exe',
  'BUILD_INFO.txt',
  'Start-PowerSystemResilience.ps1',
  'Start-PowerSystemResilience.cmd',
  'web/index.html',
  'web/css/style.css',
  'web/js/app.js',
  'web/js/canvas.js',
  'web/help_docs.json',
  'web/vendor/sql-wasm.js',
  'web/vendor/sql-wasm.wasm',
  'docs/modules/resilience/README.md',
  'external_data/matpower/case14.m',
  'external_data/typhoon/sst_monthly_south_china_sea.json',
  'external_data/typhoon/typhoon_track_catalog_m1-12_n100_h48_dt1.00_seed203000.json',
  'licenses/PowerSystemResilience-LICENSE'
)
foreach ($relative in $requiredFiles) {
  if (-not (Test-Path -LiteralPath (Join-Path $packageRoot $relative) -PathType Leaf)) {
    throw "Required Resilience artifact is absent: $relative"
  }
}
$buildInfo = Get-Content -LiteralPath (Join-Path $packageRoot "BUILD_INFO.txt") -Raw
foreach ($entry in @(
    'Product=PowerSystemResilience', 'Edition=resilience',
    'DependencyMode=source', 'Gurobi=OFF', 'CPLEX=OFF',
    'ResourcePolicy=explicit-allowlist',
    'RuntimeDllPolicy=explicit-allowlist+recursive-dumpbin-audit')) {
  if (-not $buildInfo.Contains($entry)) {
    throw "BUILD_INFO invariant missing: $entry"
  }
}

$forbiddenRelativePaths = @(
  'data/dsp',
  'external_data/harmonics_validation',
  'external_data/transient_validation',
  'external_data/transportation_networks',
  'external_data/opendss_ieee_pes',
  'web/examples/ev_traffic_scenario_template.json',
  'web/schemas/ev_traffic_scenario.schema.json',
  'web/js/core/market_navigation.js',
  'web/js/core/market_operation.js',
  'web/js/core/market_realtime.js',
  'docs/modules/harmonics_power_flow',
  'docs/modules/dynamics',
  'docs/modules/market',
  'docs/modules/integrated_energy',
  'docs/modules/ev_power_traffic',
  'docs/modules/carbon_analysis',
  'docs/modules/analysis'
)
foreach ($relative in $forbiddenRelativePaths) {
  if (Test-Path -LiteralPath (Join-Path $packageRoot $relative)) {
    throw "Forbidden Resilience resource is present: $relative"
  }
}
$forbiddenExtensions = @('.dat', '.dss', '.glm', '.xlsx', '.xml')
$forbiddenFiles = @(Get-ChildItem -LiteralPath $packageRoot -Recurse -File |
  Where-Object { $_.Extension.ToLowerInvariant() -in $forbiddenExtensions })
if ($forbiddenFiles.Count -ne 0) {
  throw "Forbidden Resilience resource is present: $($forbiddenFiles[0].FullName)"
}
$forbiddenDllPatterns = @('altdss*', 'gurobi*', 'cplex*', 'mkl_rt*', 'libiomp5md*')
foreach ($pattern in $forbiddenDllPatterns) {
  if (Get-ChildItem -LiteralPath (Join-Path $packageRoot "bin") -Filter "$pattern.dll" -File) {
    throw "Forbidden Resilience runtime DLL is present: $pattern.dll"
  }
}

function Invoke-JsonRequest(
  [string]$Method,
  [string]$Uri,
  [object]$Body = $null,
  [hashtable]$Headers = @{},
  [int]$TimeoutSec = 30
) {
  $parameters = @{
    Method = $Method
    Uri = $Uri
    Headers = $Headers
    TimeoutSec = $TimeoutSec
    UseBasicParsing = $true
  }
  if ($null -ne $Body) {
    $parameters.ContentType = 'application/json'
    $parameters.Body = ($Body | ConvertTo-Json -Depth 20 -Compress)
  }
  try {
    $response = Invoke-WebRequest @parameters
  } catch {
    if (-not $_.Exception.Response) { throw }
    $errorResponse = $_.Exception.Response
    $content = ''
    if ($errorResponse.PSObject.Methods.Name -contains 'GetResponseStream') {
      $reader = [System.IO.StreamReader]::new($errorResponse.GetResponseStream())
      try { $content = $reader.ReadToEnd() } finally { $reader.Dispose() }
    } elseif ($errorResponse.Content) {
      $content = $errorResponse.Content.ReadAsStringAsync().GetAwaiter().GetResult()
    }
    return [pscustomobject]@{
      StatusCode = [int]$errorResponse.StatusCode
      Headers = $errorResponse.Headers
      Payload = if ($content) { $content | ConvertFrom-Json } else { $null }
    }
  }
  return [pscustomobject]@{
    StatusCode = [int]$response.StatusCode
    Headers = $response.Headers
    Payload = if ($response.Content) { $response.Content | ConvertFrom-Json } else { $null }
  }
}

function Assert-NestedEdition403([string]$Base, [string]$Method,
                                 [string]$Path, [string]$Feature) {
  $response = Invoke-JsonRequest $Method "$Base$Path"
  if ($response.StatusCode -ne 403 -or
      $response.Payload.error.code -ne 'EDITION_FEATURE_DISABLED' -or
      $response.Payload.error.edition -ne 'resilience' -or
      $response.Payload.error.feature -ne $Feature) {
    throw "$Method $Path did not return the expected Resilience 403"
  }
}

function Assert-ServedFileMatchesPackage([string]$Base, [string]$UrlPath,
                                         [string]$RelativePath) {
  $packagedPath = Join-Path $packageRoot $RelativePath
  $downloadPath = Join-Path $extractRoot (
    'served-' + [guid]::NewGuid().ToString('N') + [System.IO.Path]::GetExtension($RelativePath))
  try {
    $response = Invoke-WebRequest -UseBasicParsing -Uri "$Base$UrlPath" `
      -TimeoutSec 15 -OutFile $downloadPath -PassThru
    if ($response.StatusCode -ne 200 -or
        -not (Test-Path -LiteralPath $downloadPath -PathType Leaf) -or
        (Get-FileHash -LiteralPath $downloadPath -Algorithm SHA256).Hash -ne
          (Get-FileHash -LiteralPath $packagedPath -Algorithm SHA256).Hash) {
      throw "Served content does not match packaged file: $UrlPath"
    }
  } finally {
    if (Test-Path -LiteralPath $downloadPath) {
      Remove-Item -LiteralPath $downloadPath -Force
    }
  }
}

# The checked-in catalog records build-machine paths. Adapt only this clean
# extracted verification copy so the server can prove it loaded the shipped
# catalog and SST resource through explicit paths without source-root fallback.
$sstPath = Join-Path $packageRoot `
  'external_data/typhoon/sst_monthly_south_china_sea.json'
$catalogPath = Join-Path $packageRoot `
  'external_data/typhoon/typhoon_track_catalog_m1-12_n100_h48_dt1.00_seed203000.json'
$catalogText = [System.IO.File]::ReadAllText($catalogPath)
$catalogJsonPath = $catalogPath | ConvertTo-Json -Compress
$sstJsonPath = $sstPath | ConvertTo-Json -Compress
$catalogText = [regex]::Replace(
  $catalogText, '"catalog_path"\s*:\s*""', '"catalog_path": ' + $catalogJsonPath, 1)
$catalogText = [regex]::Replace(
  $catalogText, '"sst_resource_path"\s*:\s*"[^"]*"',
  '"sst_resource_path": ' + $sstJsonPath)
[System.IO.File]::WriteAllText(
  $catalogPath, $catalogText, [System.Text.UTF8Encoding]::new($false))

$portProbe = [System.Net.Sockets.TcpListener]::new(
  [System.Net.IPAddress]::Loopback, 0)
$portProbe.Start()
$port = ([System.Net.IPEndPoint]$portProbe.LocalEndpoint).Port
$portProbe.Stop()
$base = "http://127.0.0.1:$port"
$server = $null
$originalPath = $env:PATH
try {
  $packageBin = Join-Path $packageRoot "bin"
  $env:PATH = "$packageBin;$env:SystemRoot\System32;$env:SystemRoot"
  $server = Start-Process -FilePath (Join-Path $packageBin "run_gui_server.exe") `
    -ArgumentList @('--host', '127.0.0.1', '--port', $port,
      '--api-job-workers', 1, '--data-dir', (Join-Path $packageRoot 'data'),
      '--matpower-dir', (Join-Path $packageRoot 'external_data/matpower')) `
    -WorkingDirectory $packageRoot -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput (Join-Path $extractRoot "server.stdout.log") `
    -RedirectStandardError (Join-Path $extractRoot "server.stderr.log")

  $edition = $null
  for ($attempt = 0; $attempt -lt 150; $attempt++) {
    if ($server.HasExited) { throw "Packaged server exited: $($server.ExitCode)" }
    try {
      $editionResponse = Invoke-JsonRequest GET "$base/api/edition" $null @{} 1
      if ($editionResponse.StatusCode -eq 200 -and
          $editionResponse.Payload.edition -eq 'resilience') {
        $edition = $editionResponse.Payload
        break
      }
    } catch { Start-Sleep -Milliseconds 100 }
  }
  if (-not $edition) { throw "Resilience server did not become ready" }
  if ($edition.route_policy.mode -ne 'fail_closed' -or
      $edition.route_policy.unknown_api_routes -ne 'disabled') {
    throw "Resilience edition route policy is not fail-closed"
  }

  $frontend = Invoke-WebRequest -UseBasicParsing -Uri "$base/xjtu/" -TimeoutSec 15
  if ($frontend.StatusCode -ne 200 -or $frontend.RawContentLength -eq 0) {
    throw "Packaged /xjtu/ frontend is unavailable"
  }
  $servedFileChecks = @(
    @{ Url = '/xjtu/'; Relative = 'web/index.html' },
    @{ Url = '/xjtu/css/style.css'; Relative = 'web/css/style.css' },
    @{ Url = '/xjtu/js/app.js'; Relative = 'web/js/app.js' },
    @{ Url = '/xjtu/js/canvas.js'; Relative = 'web/js/canvas.js' },
    @{ Url = '/xjtu/help_docs.json'; Relative = 'web/help_docs.json' },
    @{ Url = '/xjtu/vendor/sql-wasm.js'; Relative = 'web/vendor/sql-wasm.js' },
    @{ Url = '/xjtu/vendor/sql-wasm.wasm'; Relative = 'web/vendor/sql-wasm.wasm' },
    @{ Url = '/xjtu/docs/modules/resilience/README.md'; Relative = 'docs/modules/resilience/README.md' }
  )
  foreach ($check in $servedFileChecks) {
    Assert-ServedFileMatchesPackage $base $check.Url $check.Relative
  }

  # Retained legacy Resilience handler must reach its implementation, not the
  # edition guard. An empty body is expected to produce a handler-level 4xx.
  $retainedResponse = Invoke-JsonRequest POST `
    "$base/api/session/run_distribution_resilience" @{}
  if ($retainedResponse.StatusCode -eq 403 -and
      $retainedResponse.Payload.error.code -eq 'EDITION_FEATURE_DISABLED') {
    throw "Retained Resilience handler was blocked by the edition guard"
  }
  if ($retainedResponse.StatusCode -ge 500) {
    throw "Retained Resilience handler returned an unexpected status: $($retainedResponse.StatusCode)"
  }

  Assert-NestedEdition403 $base POST '/api/session/run_market_clearing' 'market'
  Assert-NestedEdition403 $base POST '/api/session/run_transient' 'dynamics'
  Assert-NestedEdition403 $base POST '/api/session/load_opendss' 'advanced_io'
  Assert-NestedEdition403 $base GET '/api/session/not_registered_in_profile' 'unclassified_api'

  $discoveryResponse = Invoke-JsonRequest GET "$base/api/v1"
  if ($discoveryResponse.StatusCode -ne 200 -or
      $discoveryResponse.Payload.schema -ne 'hysim_api_v1' -or
      @($discoveryResponse.Payload.analyses).Count -eq 0 -or
      'power_flow' -notin @($discoveryResponse.Payload.analyses)) {
    throw "Runtime API v1 discovery is invalid"
  }
  $createResponse = Invoke-JsonRequest POST "$base/api/v1/sessions" `
    @{ case = 'ieee14_acdc' }
  if ($createResponse.StatusCode -ne 201 -or
      -not $createResponse.Payload.session_id -or
      $createResponse.Payload.model_revision -ne 1) {
    throw "Runtime API v1 session creation failed"
  }
  $sessionId = $createResponse.Payload.session_id
  $etag = $createResponse.Headers['ETag']
  if (-not $etag) { $etag = $createResponse.Payload.etag }
  if (-not $etag) { throw "Runtime API v1 session did not provide an ETag" }

  $knownDisabled = Invoke-JsonRequest POST `
    "$base/api/v1/sessions/$sessionId/jobs" `
    @{ analysis = 'carbon_flow'; request = @{} } @{ 'If-Match' = $etag }
  if ($knownDisabled.StatusCode -ne 403 -or
      $knownDisabled.Payload.schema -ne 'hysim_error_v1' -or
      $knownDisabled.Payload.error -ne 'edition_feature_disabled' -or
      $knownDisabled.Payload.details.edition -ne 'resilience' -or
      $knownDisabled.Payload.details.feature -ne 'carbon_flow') {
    throw "Known-disabled runtime API v1 analysis did not return 403"
  }
  $unknown = Invoke-JsonRequest POST "$base/api/v1/sessions/$sessionId/jobs" `
    @{ analysis = 'not_a_registered_analysis'; request = @{} } `
    @{ 'If-Match' = $etag }
  if ($unknown.StatusCode -ne 400 -or
      $unknown.Payload.schema -ne 'hysim_error_v1' -or
      $unknown.Payload.error -ne 'unsupported_analysis') {
    throw "Unknown runtime API v1 analysis did not return 400"
  }

  $jobResponse = Invoke-JsonRequest POST "$base/api/v1/sessions/$sessionId/jobs" `
    @{ analysis = 'power_flow'; request = @{
        method = 'ac_newton'; options = @{ max_iter = 100; tol = 1.0e-8 }
      } } @{ 'If-Match' = $etag } 60
  if ($jobResponse.StatusCode -ne 202 -or -not $jobResponse.Payload.job_id) {
    throw "Runtime API v1 power-flow submission failed"
  }
  $jobId = $jobResponse.Payload.job_id
  $job = $null
  for ($attempt = 0; $attempt -lt 600; $attempt++) {
    $jobResponse = Invoke-JsonRequest GET "$base/api/v1/jobs/$jobId" $null @{} 5
    if ($jobResponse.StatusCode -ne 200) {
      throw "Runtime API v1 power-flow polling failed"
    }
    $job = $jobResponse.Payload
    if ($job.state -in @('succeeded', 'failed', 'cancelled')) { break }
    Start-Sleep -Milliseconds 100
  }
  if ($job.state -ne 'succeeded' -or $job.result.converged -ne $true -or
      $job.result.schema -ne 'power_flow_result_v1' -or
      $job.result._result_contract.schema -ne 'hysim_result_v1') {
    throw "Packaged runtime API v1 power flow did not converge"
  }

  $loadResponse = Invoke-JsonRequest POST "$base/api/session/load_builtin" `
    @{ case = 'ieee14_acdc' }
  if ($loadResponse.StatusCode -ne 200) {
    throw "Built-in case load for packaged typhoon verification failed"
  }
  $typhoonResponse = Invoke-JsonRequest POST `
    "$base/api/session/generate_typhoon_faults" @{
      intensity_category = 'TY'
      horizon_hours = 48
      time_step_hr = 1.0
      seed = 203000
      catalog_samples_per_month = 100
      catalog_first_month = 1
      catalog_last_month = 12
      catalog_base_seed = 203000
      sst_resource_path = $sstPath
      catalog_path = $catalogPath
    } @{} 120
  if ($typhoonResponse.StatusCode -ne 200 -or
      $typhoonResponse.Payload.used_catalog_sample -ne $true -or
      $typhoonResponse.Payload.catalog_loaded_from_disk -ne $true -or
      [System.IO.Path]::GetFullPath($typhoonResponse.Payload.catalog_path) -ne
        [System.IO.Path]::GetFullPath($catalogPath) -or
      [System.IO.Path]::GetFullPath($typhoonResponse.Payload.sst_resource_path) -ne
        [System.IO.Path]::GetFullPath($sstPath)) {
    throw "Packaged typhoon resources were not loaded from explicit extracted paths"
  }

  [ordered]@{
    schema = 'resilience_windows_release_verification_v1'
    zip = $zip
    sha256 = $expectedZipHash
    package_root = $packageRoot
    manifest_files = $manifest.Count
    isolated_path = $env:PATH
    edition = $edition.edition
    frontend = '/xjtu/:200'
    retained_resilience_handler = $retainedResponse.StatusCode
    disabled_legacy = 403
    known_disabled_v1 = 403
    unknown_v1 = 400
    power_flow = $job.state
    typhoon_catalog = 'loaded-from-package'
  } | ConvertTo-Json -Depth 8 |
    Set-Content -LiteralPath (Join-Path $extractRoot "verification.json") -Encoding utf8
  Write-Host "Verified Resilience release from clean extracted path: $packageRoot"
  Write-Host "Evidence: $extractRoot"
} finally {
  $env:PATH = $originalPath
  if ($server -and -not $server.HasExited) {
    Stop-Process -Id $server.Id -Force
  }
}
