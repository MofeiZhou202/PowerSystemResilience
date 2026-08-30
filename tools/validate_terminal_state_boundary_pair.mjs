#!/usr/bin/env node

import fs from 'node:fs';

const [coarsePath, finePath] = process.argv.slice(2);
if (!coarsePath || !finePath) {
  throw new Error(
    'usage: validate_terminal_state_boundary_pair.mjs coarse.json fine.json');
}

const coarse = JSON.parse(fs.readFileSync(coarsePath, 'utf8'));
const fine = JSON.parse(fs.readFileSync(finePath, 'utf8'));
const ensToleranceMwh = 1e-12;
const biasTolerancePercentagePoints = 0.25;
const scenarioKey = row => [
  row.fault_r_pu,
  row.clearing_duration_s,
  row.current_limit_priority,
].join('|');
const fineByKey = new Map(fine.scenarios.map(row => [scenarioKey(row), row]));

let terminalClassMismatchCount = 0;
let tripTimeComparedCount = 0;
let maximumTripTimeDifferenceMs = 0;
let tripTimeDifferenceSumMs = 0;
let maximumRideThroughIncrementMwh = 0;
let minimumOrderingMarginMwh = Number.POSITIVE_INFINITY;
let maximumAnalyticalReconstructionErrorMwh = 0;

for (const coarseRow of coarse.scenarios) {
  const fineRow = fineByKey.get(scenarioKey(coarseRow));
  if (!fineRow || coarseRow.frt_terminal_class !== fineRow.frt_terminal_class ||
      coarseRow.trip_reason !== fineRow.trip_reason) {
    terminalClassMismatchCount += 1;
    continue;
  }
  if (coarseRow.trip_time_s !== null && fineRow.trip_time_s !== null) {
    const differenceMs = 1000 * Math.abs(
      coarseRow.trip_time_s - fineRow.trip_time_s);
    maximumTripTimeDifferenceMs = Math.max(
      maximumTripTimeDifferenceMs, differenceMs);
    tripTimeDifferenceSumMs += differenceMs;
    tripTimeComparedCount += 1;
  }
  for (const row of [coarseRow, fineRow]) {
    for (const comparison of row.finite_horizon_comparison) {
      minimumOrderingMarginMwh = Math.min(
        minimumOrderingMarginMwh,
        comparison.transient_ens_mwh - comparison.static_ens_mwh);
      if (!row.ever_protection_tripped) {
        maximumRideThroughIncrementMwh = Math.max(
          maximumRideThroughIncrementMwh,
          Math.abs(comparison.incremental_ens_mwh));
      }
    }
  }
}

if (fineByKey.size !== coarse.scenarios.length ||
    fine.scenarios.length !== coarse.scenarios.length) {
  terminalClassMismatchCount += Math.abs(
    fine.scenarios.length - coarse.scenarios.length) + 1;
}

let maximumBiasDifferencePercentagePoints = 0;
const coarseSummary = coarse.finite_horizon_static_transient_comparison;
const fineSummary = fine.finite_horizon_static_transient_comparison;
if (coarseSummary.length !== fineSummary.length) {
  throw new Error('finite-horizon summary lengths differ');
}
for (let index = 0; index < coarseSummary.length; index += 1) {
  maximumBiasDifferencePercentagePoints = Math.max(
    maximumBiasDifferencePercentagePoints,
    100 * Math.abs(
      coarseSummary[index].relative_static_underestimate -
      fineSummary[index].relative_static_underestimate));
  maximumAnalyticalReconstructionErrorMwh = Math.max(
    maximumAnalyticalReconstructionErrorMwh,
    Math.abs(coarseSummary[index].analytical_reconstruction_error_mwh),
    Math.abs(fineSummary[index].analytical_reconstruction_error_mwh));
}

const report = {
  coarse_dt_s: coarse.settings.dt_s,
  fine_dt_s: fine.settings.dt_s,
  terminal_class_mismatch_count: terminalClassMismatchCount,
  compared_trip_count: tripTimeComparedCount,
  maximum_trip_time_difference_ms: maximumTripTimeDifferenceMs,
  mean_trip_time_difference_ms: tripTimeComparedCount > 0
    ? tripTimeDifferenceSumMs / tripTimeComparedCount
    : null,
  maximum_ride_through_increment_mwh: maximumRideThroughIncrementMwh,
  minimum_transient_minus_static_mwh: minimumOrderingMarginMwh,
  maximum_bias_difference_percentage_points:
    maximumBiasDifferencePercentagePoints,
  maximum_analytical_reconstruction_error_mwh:
    maximumAnalyticalReconstructionErrorMwh,
  thresholds: {
    ens_tolerance_mwh: ensToleranceMwh,
    bias_tolerance_percentage_points: biasTolerancePercentagePoints,
  },
};

console.log(JSON.stringify(report, null, 2));

if (terminalClassMismatchCount !== 0 ||
    maximumRideThroughIncrementMwh > ensToleranceMwh ||
    minimumOrderingMarginMwh < -ensToleranceMwh ||
    maximumAnalyticalReconstructionErrorMwh > ensToleranceMwh ||
    maximumBiasDifferencePercentagePoints > biasTolerancePercentagePoints) {
  process.exitCode = 2;
}
