# Shaanxi climate-data pilot

This directory defines the reproducible data-entry gate for the paper
"Climate-State-Conditioned Compound Weather-to-Grid Scenario Generation for
Long-Term Distribution Planning." It deliberately starts with four
representative sites in three Shaanxi climate zones. It does not download a
province-wide or global archive.

## Locked pilot scope

- Historical reference: 1985-2014.
- Future pilot: 2031-2060.
- Climate states: SSP2-4.5 and SSP5-8.5, kept separate by GCM and epoch.
- Preliminary GCMs: ACCESS-CM2, EC-Earth3, GFDL-ESM4, MPI-ESM1-2-HR, and
  MRI-ESM2-0. These five models have development or confirmation roles and are
  not an untouched final test set.
- GCM-level outer evaluation: CanESM5, CNRM-CM6-1, IPSL-CM6A-LR, MIROC6,
  NorESM2-MM, and UKESM1-0-LL. Only catalog completeness was inspected before
  staging. CESM2 was excluded because the NEX product has `tas` but lacks the
  required `tasmax` and `tasmin` directories.
- Sites: Yulin, Xi'an, Hanzhong, and Ankang, grouped into northern Shaanxi,
  Guanzhong, and Qinba zones.
- Core variables: daily maximum/minimum temperature, relative humidity, surface
  wind, downward shortwave radiation, and precipitation. A drought state is
  derived only after the PET formulation and accumulation-scale sensitivity are
  locked.
- Hourly reconstruction library: ERA5-Land. Future daily states:
  NEX-GDDP-CMIP6, subject to endpoint and variable-availability verification.
- Independent evaluation: licensed CMA stations and/or an authorized CN05.1
  release. These observations must not be silently replaced by ERA5-Land.

The rectangles in the manifest are acquisition supports. They are not official
administrative boundaries, terrain-aware climate regions, or asset exposure
polygons. The representative-point pilot cannot support asset-scale fragility
claims.

## Files

- `manifests/shaanxi_pilot.json`: spatial, temporal, variable, climate-state,
  quality-control, and probability contract.
- `manifests/sources.json`: access, licensing, version, native variable, and unit
  metadata for every planned source.
- `manifests/historical_diagnostics.json`: predeclared reference-period
  percentiles, absolute thresholds, compound events, spell rules, and trend
  reporting limits.
- `manifests/hourly_heat_stress.json`: concurrent-hour wet-bulb and shade
  apparent-temperature formulas, validity domains, event definition, and
  interpretation limits.
- `manifests/gcm_skill_screening.json`: historical-reference metrics,
  normalization, category weights, calendar alignment, blocked stability, and
  provisional selection semantics.
- `manifests/observation_validation.json`: CMA/CN05.1 independent-evaluation
  schema, coverage gates, spatial matching, metrics, and provenance contract.
- `manifests/weather_generator_baseline.json`: preserved legacy four-state
  development baseline (`v1`).
- `manifests/weather_generator_baseline_frozen.json`: frozen one-state full-VAR
  and seasonal residual-block baseline (`v2`), selected before confirmation.
- `manifests/weather_generator_evaluation.json`: six-fold held-out evaluation,
  six component variants, aggregation, and interpretation contract.
- `manifests/weather_generator_state_sensitivity.json`: controlled `K=1..5`
  weather-state sensitivity and post-selection validation guard.
- `manifests/weather_generator_frozen_confirmation.json`: hash-locked `K=1`
  method freeze and untouched-GCM confirmation protocol.
- `manifests/weather_generator_knn_analog.json`: season-conditioned multivariate
  KNN analog advanced candidate and hyperparameter-status contract.
- `manifests/weather_generator_knn_development.json` and
  `manifests/weather_generator_knn_frozen_confirmation.json`: neighbor-count
  development and hash-frozen confirmation protocols.
- `manifests/weather_generator_knn_outer_evaluation.json`: hash-locked,
  GCM-level outer comparison of the frozen KNN K=10 and frozen VAR baseline.
- `templates/observation_daily.example.csv` and
  `templates/observation_provenance.example.json`: canonical licensed-data
  handoff examples. They contain placeholders and are not valid observations.
- `climate_data.py`: offline validation and volume planning, exact ERA5-Land
  request generation, opt-in CDS retrieval, NetCDF QC, and point subsetting.
- `weather_generator.py`: fit and sample the first climate-state-conditioned
  weather-generator baseline with deterministic seeds and checksum provenance.
- `weather_generator_evaluate.py`: five-year-blocked historical validation and
  component ablation against held-out GCM segments.
- `weather_generator_aggregate.py`: checksum-gated, unit-preserving aggregation
  across declared GCM-site evaluation reports.
- `weather_generator_knn.py`: cKDTree-backed multivariate analog fitting,
  generation, deterministic replay, and transition-level audit trail.
- `requirements.txt`: isolated optional runtime for retrieval and NetCDF checks.

Large and licensed data belong under `data/climate_shaanxi/`, which is already
excluded by the repository's `data/` ignore policy. Do not commit raw CMA data or
credentials.

## Offline first run

From the repository root:

```bash
python3 tools/climate_data/climate_data.py validate
python3 tools/climate_data/climate_data.py plan
python3 tools/climate_data/climate_data.py plan \
  --output data/climate_shaanxi/provenance/pilot_plan.json
python3 tools/climate_data/climate_data.py era5-requests --site xian --year 1985
```

The last command writes an exact request record but performs no network access.
The default pilot has 16 reference-period ERA5-Land point/variable-group requests and 90 logical
NEX-GDDP-CMIP6 model/experiment/variable subsets. The volume estimate represents
selected values; allow the documented multiplier for coordinates, NetCDF
metadata, chunking, and intermediate daily products.

For full NEX retrieval, the contract selects one annual corridor bounding-box
request per model/experiment/variable/year and then extracts the four registered
sites. A live 1985 precipitation preflight measured 231,453 bytes. Extrapolated
to the 2,700 annual pilot objects, this is about 596 MiB, versus 10,800 separate
point requests. This estimate must be refreshed before bulk execution because
compression differs by variable.

## Controlled retrieval

Use a dedicated environment so the core Python SDK remains dependency-free:

```bash
python3 -m venv .venv-climate
.venv-climate/bin/pip install -r tools/climate_data/requirements.txt
```

For ERA5-Land, create a CDS account, accept the current dataset terms, and
configure the credentials required by `cdsapi`. First retrieve exactly one
site-year:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py \
  era5-requests --site xian --year 1985 --group temperature_humidity --execute
```

The original gridded six-variable request exceeded the CDS per-request cost limit.
The pilot therefore uses `reanalysis-era5-land-timeseries`, which selects the
nearest 0.1-degree grid point to each submitted WGS84 coordinate. Requests are
sharded by site and the four product-compatible variable groups:
temperature/humidity, precipitation, solar radiation, and wind. Do not start the
16-request pilot until one annual file from every group passes QC and the
de-accumulated precipitation/radiation semantics are validated. Retrieval
never overwrites an existing raw target; a changed source or query requires a new
versioned path. `--overwrite` applies only to generated request/report records.
Each successful request record stores retrieval time, byte size, and SHA-256.
CDS returns a ZIP containing one NetCDF file for each compatible variable group;
the pipeline preserves both with independent checksums. Tiny negative values in
precipitation and radiation caused by GRIB packing remain unchanged in raw data.
QC reports them as warnings only within the source-specific absolute tolerance;
harmonization must clip those values to zero before daily aggregation.

After all four annual groups pass raw QC, generate the six-variable canonical
daily dataset and validate it independently:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py harmonize-era5 \
  --site xian --year 1985
.venv-climate/bin/python tools/climate_data/climate_data.py qc-daily \
  data/climate_shaanxi/processed/daily/era5_land/site=xian/year=1985/daily_era5_land_xian_1985.nc \
  --expected-year 1985
```

The harmonizer requires identical complete hourly axes and grid coordinates,
derives relative humidity from 2 m temperature/dewpoint, computes wind-vector
magnitude before averaging, and records every packing-noise or humidity clamp in
the output provenance sidecar.

After the annual smoke test passes, retrieve the full reference period in four
long point-series requests per site. The command preserves the CDS ZIP and
period NetCDF, verifies the exact hourly axis, and creates lossless annual time
partitions. An existing annual file is reused only after every variable and time
value matches the period response exactly:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py era5-period \
  --site xian --group temperature_humidity \
  --start-year 1985 --end-year 2014 --execute
```

Repeat for `precipitation`, `solar_radiation`, and `wind`, then build and
independently QC every canonical year:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py \
  harmonize-era5-range --site xian --start-year 1985 --end-year 2014
```

After all four sites are complete, verify every stored checksum and expected
object count in one reproducible audit:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py \
  combine-era5-reference
.venv-climate/bin/python tools/climate_data/climate_data.py \
  audit-era5-reference
```

## Reference-period diagnostics

After the four-site audit passes, derive the heat-stress series from the original
hourly temperature, dewpoint, and wind files. This must precede the annual
diagnostics:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py \
  derive-era5-heat-stress
```

The derivation computes relative humidity from concurrent 2 m temperature and
dewpoint, applies the Stull (2011) wet-bulb approximation only within its
declared `-20..50 degC` and `5..99%` domain, and retains the temperature,
humidity, wind speed, and UTC hour at each daily maximum. Out-of-domain hours
are missing for wet-bulb calculations and counted in the provenance record.
The additional apparent-temperature series is a shade metric; it does not
include solar radiation, clothing, or physiological response. Daily extrema
use UTC days, so events near the day boundary can differ from China Standard
Time aggregation.

Then compute the locked historical and compound-event indices:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py \
  diagnose-era5-reference
```

The command requires the exact continuous 1985-2014 daily axes and writes one
365-day calendar-threshold NetCDF per site, a 120-row annual-index CSV, and a
checksum-bearing JSON report. The definition uses a circular five-day window,
maps February 29 to the February 28 threshold, and resets spells at each calendar
year boundary. It reports TX90p/TN90p, RX1day/RX5day, R10mm/R20mm/R95pTOT, CDD,
CWD, WSDI, grid-heatwave duration and severity, and the predeclared hot-dry,
hot-dry-low-wind, concurrent humid-heat, and hot-warm-night intersections. The
humid-heat event is at least two consecutive May-September days whose daily
maximum hourly wet-bulb temperature is strictly above the site's 1985-2014
warm-season q90. Years without an event use blank conditional-mean fields, not
numeric zero. Trend estimates are descriptive Theil-Sen slopes without
unqualified p-values.

This output is an internal ERA5-Land baseline, not independent observational
validation. Percentile thresholds are estimated from and evaluated within the
same baseline, so publication-grade in-base percentile-index comparisons should
add an explicitly documented bootstrap treatment. The former non-concurrent
pairing of daily maximum temperature and daily mean relative humidity has been
removed from the diagnostic output. The replacement wet-bulb metric is
concurrent but remains a weather-stress screening index rather than a medical
heat-health threshold.

The independent evaluation remains gated on licensed CMA station data or an
authorized CN05.1 release. `manifests/observation_validation.json` pre-registers
the required variables, at least 20 overlapping years, completeness thresholds,
station/grid distance and elevation reporting, event-skill metrics, and
year-blocked bias-adjustment evaluation. Until compliant observations are staged,
the diagnostic report must retain
`independent_observation_validation_completed: false`; ERA5-Land cannot validate
itself.

Stage each licensed station or extracted CN05.1 grid cell as a separate canonical
daily CSV. Use `location_id` for either the CMA station identifier or a stable
CN05.1 grid-cell identifier; do not relabel a grid cell as a station. Keep the
native files outside version control and record every native-to-canonical
transformation in the provenance JSON. The canonical table may include the
optional `hurs`, `sfcWind`, and `rsds` columns, but `tasmax`, `tasmin`, and `pr`
are mandatory.

Run the admission gate before computing any ERA5 bias metric:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py qc-observations \
  data/climate_shaanxi/observations/canonical/source=cma_station/site=xian/daily.csv \
  --source cma_station --site xian \
  --provenance data/climate_shaanxi/observations/provenance/cma_xian.json \
  --output data/climate_shaanxi/provenance/qc/cma_xian_admission.json
```

The gate verifies the CSV checksum and license provenance, requires one stable
location per file, excludes every row whose quality flag is not explicitly
provider-approved, and evaluates missingness after reconstructing the complete
daily calendar. At least 20 calendar years must pass the per-year completeness
gate. Overall and seasonal completeness, wet-day sample size, duplicate dates,
temperature ordering, negative precipitation, distance to the downloaded ERA5
grid cell, and observation-minus-grid elevation are reported. The reference-grid
elevation must come from a named, versioned orography source; it must not be
guessed from the station elevation. A passing admission report establishes data
fitness only and is not itself a bias evaluation.

The NCCS catalog and point-subset endpoint were verified on 2026-07-24. The
study pins annual v2.0 files and a model-specific realization in the study
registry (`r1i1p1f2` for CNRM-CM6-1 and UKESM1-0-LL; `r1i1p1f1` for the other
retained outer models). Resolve the exact
catalog object and create a request record without downloading it:

```bash
python3 tools/climate_data/climate_data.py nex-request \
  --model ACCESS-CM2 --experiment historical --variable pr \
  --site xian --year 1985
```

Add `--execute` only for a controlled point-retrieval smoke test. The resolver
reads the live variable catalog and rejects missing or ambiguous annual v2.0
objects instead of guessing a filename.

Retrieve several variables for one state without broadening the spatial or
temporal scope:

```bash
python3 tools/climate_data/climate_data.py nex-bundle \
  --model ACCESS-CM2 --experiment historical --site xian --year 1985 \
  --variables tasmax tasmin hurs sfcWind rsds --execute
```

The full pilot uses one annual corridor object rather than four point requests.
The following command downloads the registered bounding box, validates the
three-dimensional field, extracts the nearest cell for all four sites, and
records checksums for the corridor and each point file:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py nex-corridor \
  --model ACCESS-CM2 --experiment historical --variable pr --year 1985 \
  --execute
```

Repeat the corridor command for all six variables and declared years. Existing
raw corridor or point files are never overwritten: a rerun validates and reuses
them only when time, coordinates, and every selected value agree. The 1985
ACCESS-CM2 historical six-variable preflight passed for all four sites on
2026-07-24. Plan the full 90-catalog, 2,700-object archive without network
transfer, then explicitly execute its resumable process-based downloader:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py nex-pilot-download
.venv-climate/bin/python tools/climate_data/climate_data.py nex-pilot-download \
  --workers 4 --execute --progress
```

The model, experiment, variable, and year range can be filtered for controlled
batches. Catalog access uses threads, while annual NetCDF validation and site
extraction use separate processes because common HDF5/NetCDF backends are not
thread-safe. A failed run writes its exact failed-object list and can be rerun;
existing valid raw objects are checked and reused. Do not call the complete
historical and future archive downloaded until the final summary reports all
2,700 objects complete.

Commands default to the original five-model `pilot` set. The frozen outer set
must be selected explicitly. Its 108 catalogs passed the complete 30-year
variable audit on 2026-07-24 and it contains 3,240 annual corridor objects:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py nex-audit \
  --model-set outer --workers 12
.venv-climate/bin/python tools/climate_data/climate_data.py nex-pilot-download \
  --model-set outer --workers 8 --execute --progress
```

Once all six raw variables pass their individual QC gates, align and convert them
to the canonical daily units:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py harmonize-nex \
  --model ACCESS-CM2 --experiment historical --site xian --year 1985
```

After the full archive is present, harmonize all 1,800
model/experiment/site/year records with a resumable process pool:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py \
  harmonize-nex-pilot --workers 8
```

The corresponding outer archive produces 2,160 canonical station-year records:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py \
  harmonize-nex-pilot --model-set outer --workers 8
```

The harmonizer requires identical time axes and selected grid cells, converts
temperature from K to degrees Celsius and precipitation flux to mm/day, checks
that daily minimum temperature never exceeds daily maximum temperature, and
writes an input-checksum/transformation sidecar. It respects each file's
declared calendar, including GFDL-ESM4's 365-day years. Raw relative humidity is
preserved with a two-percentage-point QC tolerance for small bias-adjustment
overshoots; canonical relative humidity is clipped to 0-100% and the number of
changed values is reported. The harmonizer never assigns probabilities across
GCMs or SSPs.

## Historical GCM reference screening

Run the predeclared historical screen only after all 600 historical station-year
files have passed harmonization:

```bash
.venv-climate/bin/python tools/climate_data/gcm_screen.py --overwrite
```

The screen removes February 29 from every series so Gregorian and `365_day`
models share a common calendar. It compares marginal quantiles, monthly seasonal
cycles, contemporaneous rank dependence, within-year lag-one persistence, and
annual extremes/compound-event frequencies. It deliberately excludes paired
daily RMSE and daily correlation because a free-running GCM is not expected to
reproduce the reference day's weather. Six leave-one-five-year-block-out runs
report rank stability.

This is a comparison with an ERA5-Land reanalysis reference, not observational
validation. NEX-GDDP-CMIP6 is already bias adjusted, so good marginal scores are
not independent evidence of GCM quality. The current pilot places EC-Earth3 and
MPI-ESM1-2-HR stably in the first two positions. Ranks three through five are
less stable and close in composite score. The lowest three scores form a
provisional method-development set only; all five GCMs continue to the licensed
CMA/CN05.1 observation gate, and none receives a probability weight.

Independently validate the resulting canonical file:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py qc-daily \
  data/climate_shaanxi/processed/daily/nex_gddp_cmip6/model=ACCESS-CM2/experiment=historical/site=xian/year=1985/daily_nex_gddp_cmip6_ACCESS-CM2_historical_xian_1985_v2.0.nc \
  --expected-year 1985
```

For a small set of already validated files, `summarize-daily` computes marginal
quantiles and provisional hot/dry/low-wind counts while preserving each climate
state as a separate record. Its output explicitly forbids climate inference from
single-year slices and does not pool SSP or GCM probabilities.

Audit all 90 pilot catalogs (five GCMs, historical plus two SSPs, six variables)
and require all 30 declared years in each catalog. This audit passed 90/90 on
2026-07-24 and its full record is written to the runtime provenance directory:

```bash
python3 tools/climate_data/climate_data.py nex-audit --progress
```

## First weather-generator baseline

Fit each GCM, experiment, site, and 30-year epoch separately. The command below
is the initial one-member smoke test; it does not pool models, assign GCM/SSP
probabilities, or create the final planning ensemble:

```bash
.venv-climate/bin/python tools/climate_data/weather_generator.py \
  --model EC-Earth3 --experiment historical --site xian --members 1
```

To replay members from a checksum-verified fitted parameter archive, add
`--reuse-fit`. The command rechecks the method, parameter, and all 30 input-file
hashes before sampling.

The fitted variables are `tasmax`, `tasmax - tasmin`, `hurs`, `sfcWind`, `rsds`,
and `pr`. Monthly empirical midranks are mapped to Gaussian space. The frozen
`v2` baseline uses one state, a full ridge VAR(1), and complete residual vectors
resampled in seven-day meteorological-season blocks. The legacy four-state `v1`
definition remains available for exact reproduction of development results. The
empirical step inverse retains the zero-precipitation mass, while the DTR representation enforces
`tasmin <= tasmax`. Outputs use a 365-day calendar and record the method,
conditioning state, stable member seed, input checksums, parameter checksum,
software versions, and physical quality gates. These statistical state labels
have no physical names, and site-wise fits do not yet model spatial dependence.

Run the six-fold historical distributional validation and component ablation
before expanding the scenario ensemble:

```bash
.venv-climate/bin/python tools/climate_data/weather_generator_evaluate.py \
  --model EC-Earth3 --site xian
```

Each fold removes one complete five-year block and excludes the transition over
the resulting gap. Twenty deterministic members per fold are scored against the
held-out GCM block. A fold-member pair uses the same seed across variants to
reduce Monte Carlo noise in their differences. The six variants isolate the state model, cross-variable VAR
lags, residual-column coupling, and seven-day blocks, with monthly independent
marginals as a lower bound. Member and fold results remain available; the
summary rank is diagnostic only. A held-out GCM segment is a distributional
generator target, not an observation, and this evaluation assigns no GCM or SSP
probabilities.

The four-state baseline is a hypothesis, not a fixed truth. Run the state-count
sensitivity with all other components unchanged:

```bash
.venv-climate/bin/python tools/climate_data/weather_generator_evaluate.py \
  --definition tools/climate_data/manifests/weather_generator_state_sensitivity.json \
  --model EC-Earth3 --site xian --quiet
```

This `K=1..5` sweep is for method development. After selecting a state count,
estimate final generalization with nested outer validation or freeze the method
before evaluating additional GCM-site units; do not report the tuned inner-fold
score as unbiased final performance.

After every declared GCM-site unit is complete, aggregate without pooling their
member rows or treating the units as independent observations:

```bash
.venv-climate/bin/python tools/climate_data/weather_generator_aggregate.py
```

Fit and sample the advanced seasonal multivariate analog candidate:

```bash
.venv-climate/bin/python tools/climate_data/weather_generator_knn.py \
  --model EC-Earth3 --experiment historical --site xian
```

Its exact cKDTree search operates in monthly Gaussian-rank space, conditions the
destination on meteorological season, samples a complete six-variable successor
vector with inverse-neighbor-rank probabilities, and records every chosen
transition index. Tune neighbor count only on the declared development GCMs:

```bash
.venv-climate/bin/python tools/climate_data/weather_generator_evaluate.py \
  --definition tools/climate_data/manifests/weather_generator_knn_development.json \
  --model EC-Earth3 --site xian --quiet
```

K=10 and the one-state VAR baseline are now frozen. After the six outer models
are harmonized, run the outer protocol with an explicit access gate for every
model-site unit; do not change methods or hyperparameters from these results:

```bash
.venv-climate/bin/python tools/climate_data/weather_generator_evaluate.py \
  --definition tools/climate_data/manifests/weather_generator_knn_outer_evaluation_calendar_amended.json \
  --baseline-definition tools/climate_data/manifests/weather_generator_baseline_frozen.json \
  --model-set outer --model CanESM5 --site xian --quiet

.venv-climate/bin/python tools/climate_data/weather_generator_aggregate.py \
  --evaluation-id shaanxi_weather_generator_knn_gcm_outer_evaluation_v1 \
  --model-set outer \
  --models CanESM5 CNRM-CM6-1 IPSL-CM6A-LR MIROC6 NorESM2-MM FGOALS-g3 \
  --reference-variant full \
  --aggregate-id frozen_gcm_outer_calendar_eligible_six_gcms_four_sites
```

UKESM1-0-LL was excluded before any generator score because its `360_day`
calendar is outside the frozen 365-day protocol. FGOALS-g3 replaced it under
the calendar amendment. Its upstream `tasmin` files omit the units attribute;
the evidence-gated repair accepts only the missing attribute as K, preserves all
source corridor files, rejects explicit conflicts, and is frozen in
`manifests/fgoals_g3_tasmin_metadata_repair.json`.

The completed frozen outer evaluation is mixed. KNN and VAR each rank first in
12 of 24 GCM-site units. Equal-unit mean composite loss is 0.088484 for KNN and
0.089017 for VAR, a KNN delta of -0.000533 (about -0.60%). KNN improves the
mean annual-extreme/compound-event score but worsens mean daily persistence and
multivariate dependence. This is evidence for a trade-off, not universal KNN
superiority; no outer result was used to retune either method. The immutable
summary and output checksums are in
`manifests/weather_generator_knn_outer_results.json`.

## Frozen future ensemble and resilience demonstration

The paper's future experiment retains both frozen generators rather than
selecting one from the outer-evaluation or electrical results. It separately
fits six outer GCMs, two SSPs, four sites, and two methods, then generates ten
30-year members per conditional state. Run the complete 96-task, 960-member
batch with:

```bash
.venv-climate/bin/python tools/climate_data/future_weather_batch.py \
  --execute --workers 4 --progress --overwrite
```

The completed report must state `status: complete`, `tasks_completed: 96`,
`member_count: 960`, and an empty `failures` array. Generated paths and the
checksum-bearing report are written under
`data/climate_shaanxi/processed/weather_generator_future/` and
`data/climate_shaanxi/provenance/future_weather_batch.json`, respectively.

Apply the predeclared weather-to-grid mapping without refitting any transfer
coefficient from grid outcomes:

```bash
.venv-climate/bin/python tools/climate_data/weather_to_grid.py --overwrite
```

This produces summaries for all 960 weather members and a fixed set of 240
Xi'an inputs: six GCMs, two SSPs, two generators, and ten paired members. The
mapping reconstructs 72 hourly values for the maximum three-day compound-stress
window, applies fixed load/PV/wind transfer functions, keeps the fault count and
branch identities fixed, and modifies only the declared repair durations with
weather stress. It is a sensitivity mapping, not a calibrated Shaanxi asset
fragility model.

Build and run the single downstream case with the existing heuristic sequential
resilience assessment:

```bash
cmake --preset macos-release
cmake --build build/macos-release --target weather_resilience_case_study -j4
build/macos-release/weather_resilience_case_study \
  data/climate_shaanxi/processed/future_weather_study/resilience_case_bundle.json \
  data/climate_shaanxi/processed/future_weather_study/resilience_results.json
```

The accepted run contains 240 cases and zero failures. The algorithm,
reconfiguration setting, network, renewable capacities, two branch faults, and
72-hour horizon are fixed by
`manifests/future_weather_resilience_study.json`; none is tuned from KNN-versus-VAR
outcomes.

Generate the paired paper tables, PNG figures, PDF figures, and final frozen
result inventory with:

```bash
MPLCONFIGDIR=/tmp/shaanxi_matplotlib \
  .venv-climate/bin/python tools/climate_data/paper_figures.py --overwrite
```

The output directory is
`data/climate_shaanxi/processed/future_weather_study/paper/`. It contains 480
weather pairs, 120 Xi'an resilience pairs, three figures in both PNG and PDF,
two member-level paired CSV tables, and `paper_results_summary.json`. The JSON
inventory verifies and hashes the future-batch report, mapping provenance,
resilience bundle, results, implementation sources, figures, and tables. It also
records that no GCM/SSP/generator probabilities, automatic generator selection,
or pseudo-replication significance claim are used.

## Quality control and subsetting

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py qc-netcdf \
  data/climate_shaanxi/raw/era5_land/site=xian/year=1985/era5_land_timeseries_xian_1985_temperature_humidity.nc \
  --source era5_land \
  --expected-year 1985 \
  --output data/climate_shaanxi/provenance/qc/era5_land_xian_1985.json
```

`qc-netcdf` rejects unknown variables, undeclared or unsupported calendars,
non-monotonic or incomplete annual time axes, native-unit conflicts, excessive
missingness, and coordinates outside the buffered Shaanxi support. It always
reports the file SHA-256. Supply `--expected-year` for every annual raw file.

For a larger source NetCDF file, subset it without modifying the source:

```bash
.venv-climate/bin/python tools/climate_data/climate_data.py subset-netcdf input.nc \
  --output data/climate_shaanxi/interim/example_sites.nc
```

The subset command only permits outputs under `interim/` or `processed/` and
writes a sidecar provenance record. Unit conversion, calendar harmonization,
bias adjustment, and weather generation are later transformations; they must not
be hidden inside acquisition.

## Acceptance sequence

1. Validate manifests and inspect the volume plan.
2. Verify one ERA5-Land site-year request and run NetCDF QC.
3. Confirm CMA/CN05.1 access and document the exact licensed product.
4. Resolve NEX-GDDP-CMIP6 endpoint/version and verify all six variables for every
   pilot GCM, historical run, SSP, and calendar.
5. Retrieve the four-site pilot and create immutable checksum inventory.
6. Harmonize daily variables and validate against independent observations.
7. Screen GCMs using predeclared metrics before expanding to 8-15 models or the
   2071-2100 window.

No SSP or GCM probabilities are defined in this pipeline. All generated weather
probabilities remain conditional on `(SSP, GCM, epoch)`.

## Research implementation sequence

The paper workflow is staged so that each later claim has an auditable input and
an explicit comparison:

1. **Concurrent heat stress (implemented):** derive hourly wet-bulb and shade
   apparent temperature from ERA5-Land, preserve the covariates at the extreme
   hour, and use the wet-bulb event definition in the 1985-2014 diagnostics.
2. **Climate-state acquisition:** retrieve and harmonize the NEX-GDDP-CMIP6
   1985-2014 historical run and 2031-2060 SSP2-4.5/SSP5-8.5 runs for each
   candidate GCM. Keep every `(GCM, experiment, epoch)` separate.
3. **Historical skill screening:** compare candidate-GCM historical simulations
   with the ERA5-Land reanalysis reference and, when admitted, CMA/CN05.1
   observations. Call ERA5-Land a reference, never observational truth, and use
   year-blocked evaluation for any fitted adjustment.
4. **Generator baseline (frozen):** use monthly empirical marginal transforms,
   one full VAR(1), and seven-day multivariate residual blocks. The tested
   latent-state layer was rejected because it degraded persistence.
5. **Advanced candidate and outer evaluation:** use the seasonal multivariate
   KNN analog transition with K=10, then compare it with the frozen VAR baseline
   on the six newly acquired GCMs. Retain every GCM-site unit and report
   marginals, dependence, spells, tails, and compound-event frequency without
   pseudo-replication significance claims.
6. **Weather-to-grid coupling:** convert each generated path into load,
   photovoltaic/wind availability, and weather-dependent failure/repair inputs;
   then report electrical outcomes conditional on climate state and model scope.

Steps 2-6 are not validated merely because acquisition or interface code
exists. Their acceptance requires the corresponding checksums, held-out scores,
ablation results, and end-to-end grid tests.
