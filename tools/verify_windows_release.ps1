[CmdletBinding()]
param([string]$ZipPath = "dist/HySim-Windows-x64.zip")

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$zip = (Resolve-Path (Join-Path $repoRoot $ZipPath)).Path
$hashPath = "$zip.sha256"
if (-not (Test-Path -LiteralPath $hashPath -PathType Leaf)) {
  throw "Missing ZIP SHA-256 sidecar: $hashPath"
}
$expected = ((Get-Content -LiteralPath $hashPath -Raw).Trim() -split '\s+')[0]
if ((Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash -ne $expected) {
  throw "ZIP SHA-256 mismatch"
}

# Keep the extracted package and verification report for release inspection.
# The path deliberately contains spaces to exercise launcher quoting.
$verificationName = "Windows package verification " + [guid]::NewGuid().ToString("N")
$extractRoot = Join-Path (Join-Path $repoRoot "build") $verificationName
Expand-Archive -LiteralPath $zip -DestinationPath $extractRoot
$roots = @(Get-ChildItem -LiteralPath $extractRoot -Directory)
if ($roots.Count -ne 1) { throw "Expected exactly one package root" }
$packageRoot = $roots[0].FullName

$manifestPath = Join-Path $packageRoot "package_manifest.csv"
$manifest = @(Import-Csv -LiteralPath $manifestPath)
if ($manifest.Count -eq 0) { throw "Empty package manifest" }
foreach ($entry in $manifest) {
  $artifact = [IO.Path]::GetFullPath((Join-Path $packageRoot $entry.RelativePath))
  if (-not $artifact.StartsWith($packageRoot + [IO.Path]::DirectorySeparatorChar,
      [StringComparison]::OrdinalIgnoreCase)) {
    throw "Manifest path escapes package: $($entry.RelativePath)"
  }
  if (-not (Test-Path -LiteralPath $artifact -PathType Leaf) -or
      (Get-Item -LiteralPath $artifact).Length -ne [long]$entry.SizeBytes -or
      (Get-FileHash -LiteralPath $artifact -Algorithm SHA256).Hash -ne $entry.Sha256) {
    throw "Manifest mismatch: $($entry.RelativePath)"
  }
}

$portProbe = [Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback, 0)
$portProbe.Start()
$port = $portProbe.LocalEndpoint.Port
$portProbe.Stop()
$base = "http://127.0.0.1:$port"
$server = $null
$originalPath = $env:PATH
try {
  # Only Windows system directories and the package bin are visible. This
  # rejects accidental dependencies on the build machine's PATH.
  $packageBin = Join-Path $packageRoot "bin"
  $env:PATH = "$packageBin;$env:SystemRoot\System32;$env:SystemRoot"
  $server = Start-Process -FilePath (Join-Path $packageBin "run_gui_server.exe") `
    -ArgumentList @("--host", "127.0.0.1", "--port", $port,
      "--data-dir", ('"' + (Join-Path $packageRoot "data") + '"'),
      "--matpower-dir", ('"' + (Join-Path $packageRoot "external_data/matpower") + '"')) `
    -WorkingDirectory $packageRoot -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput (Join-Path $extractRoot "server.stdout.log") `
    -RedirectStandardError (Join-Path $extractRoot "server.stderr.log")

  $ready = $false
  for ($attempt = 0; $attempt -lt 100; $attempt++) {
    if ($server.HasExited) { throw "Packaged server exited: $($server.ExitCode)" }
    try {
      $edition = Invoke-RestMethod "$base/api/edition" -TimeoutSec 1
      if ($edition.edition -eq "full") { $ready = $true; break }
    } catch { Start-Sleep -Milliseconds 100 }
  }
  if (-not $ready) { throw "Full-edition server did not become ready" }

  $assets = @("/xjtu/", "/xjtu/js/app.js", "/xjtu/js/canvas.js",
    "/xjtu/js/core/southern_market.js", "/xjtu/vendor/sql-wasm.wasm",
    "/xjtu/help_docs.json", "/xjtu/docs/README.md")
  foreach ($asset in $assets) {
    $response = Invoke-WebRequest -UseBasicParsing "$base$asset" -TimeoutSec 15
    if ($response.StatusCode -ne 200 -or $response.RawContentLength -eq 0) {
      throw "Missing GUI/help asset: $asset"
    }
  }
  $cases = Invoke-RestMethod "$base/api/cases"
  if (@($cases.cases).Count -eq 0) { throw "Empty built-in case catalog" }
  $loaded = Invoke-RestMethod "$base/api/session/load_builtin" -Method Post `
    -ContentType "application/json" -Body '{"case":"ieee14_acdc"}'
  if ($loaded.counts.ac_buses -ne 14) { throw "Built-in hybrid case failed to load" }
  $pf = Invoke-RestMethod "$base/api/session/pf" -Method Post -TimeoutSec 60 `
    -ContentType "application/json" -Body '{"method":"ac_newton","options":{}}'
  if ($pf.converged -ne $true) { throw "Packaged hybrid power flow did not converge" }

  [ordered]@{
    zip = $zip
    sha256 = $expected
    manifest_files = $manifest.Count
    package_root = $packageRoot
    edition = $edition.edition
    gui_assets = $assets
    builtin_cases = @($cases.cases).Count
    hybrid_pf_converged = $pf.converged
    isolated_path = $env:PATH
  } | ConvertTo-Json -Depth 5 |
    Set-Content -LiteralPath (Join-Path $extractRoot "verification.json") -Encoding utf8
  Write-Host "Verified $($manifest.Count) files, full GUI/help, and hybrid PF."
  Write-Host "Evidence: $extractRoot"
} finally {
  $env:PATH = $originalPath
  if ($server -and -not $server.HasExited) { Stop-Process -Id $server.Id -Force }
}
