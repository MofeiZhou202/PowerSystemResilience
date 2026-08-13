# Trial Edition Contract

Status: implemented as a capability profile on the shared `main` source tree.

The Trial edition is a build and packaging profile. It does not fork or delete
the full-edition implementation. The first stage enforces capabilities at the
HTTP boundary and filters the GUI; disabled library source groups are still
compiled into the common static library and may be pruned in a later measured
size-optimization stage.

## Workflow and indicators

The Trial GUI exposes five stages: modeling, parameter validation, indicator
design, panoramic simulation, and weak-link identification. It exposes eight
indicators: economic, carbon, reliability, and resilience at both system and
user levels.

`POST /api/edition/analysis_plan` is the source of truth for mapping selected
indicator IDs to ordered analysis modules. The GUI renders those steps as
module links. Solver execution remains user-confirmed because PF, OPF,
reliability, resilience, and fault analyses have method-specific inputs and
limitations; the profile does not invent an automatic calculation result.

## Capability boundary

Retained capabilities are built-in cases, JSON/MATPOWER/GridLAB-D/OpenDSS IO,
parameter validation, topology analysis, PF, OPF, AC/DC short circuit, scenario
generation, hosting capacity, static carbon flow, reliability, distribution
resilience, and multidimensional weak-link identification.

Disabled capabilities are RPO, harmonics, transient and small-signal dynamics,
time-series/annual/lifecycle analysis, dynamic carbon, markets and SCUC, campus
integrated energy, EV-traffic, network reconfiguration/reduction,
counterfactual planning, SPPT Agent, and ETAP/CIM/SVG/BPA/PSD or scenario-
workbook IO.

The backend policy is fail-closed. Known disabled routes return HTTP 403:

```json
{
  "error": {
    "code": "TRIAL_FEATURE_DISABLED",
    "feature": "market",
    "message": "This capability is not included in the Trial edition."
  }
}
```

An API route that is not classified by the retained or disabled registry also
returns 403 with `feature=unclassified_api`. This makes new endpoint exposure a
test and review decision instead of enabling it by default. Static `/xjtu/`
content and CORS preflight remain available.

## Build and package

The Windows profile is configured and tested with:

```powershell
cmake --preset windows-trial-release
cmake --build --preset windows-trial-release --target run_gui_server
ctest --preset windows-trial-release -L trial --output-on-failure
powershell -ExecutionPolicy Bypass -File tools/package_trial_windows.ps1
```

The package script refuses build/output paths outside `build/` and `dist/`, a
dirty or mismatched MIPSolvers checkout, incorrect Trial/IPO/Gurobi/OpenDSS
cache settings, failed Trial tests, missing required DLLs/data, or a staged
package that cannot start independently. It then records SHA-256 for every
shipped file and for the final ZIP.

Only allowlisted retained data is copied. A lightweight typhoon catalog and SST
sample remain for unexpected-fault scenarios; disabled-only transient,
harmonic, transport, PSD, XML/CIM, and large typhoon corpora are excluded.

## Acceptance boundary

Registered coverage includes a C++ policy/plan test, an HTTP profile and
fail-closed route test, and a Playwright DOM/navigation/responsive test. A
Windows release is not verified until those tests pass in the Windows Trial
preset and the package startup smoke test succeeds.

Commercial limits are outside this profile. Expiration, model-size limits,
scenario/horizon limits, export policy, watermarking, license binding, and
product version text require separate product decisions. The embedded profile
is capability enforcement, not a cryptographic licensing system.
