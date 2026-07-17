// @ts-check

import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { runInNewContext } from 'node:vm';

const ROOT = path.resolve(path.dirname(new URL(import.meta.url).pathname), '..', '..');
const TEST_XML = path.join(ROOT, 'data', 'test.xml');

function arg(name, fallback) {
  const index = process.argv.indexOf(`--${name}`);
  return index >= 0 && index + 1 < process.argv.length ? process.argv[index + 1] : fallback;
}

async function jsonRequest(base, endpoint, options = {}) {
  const response = await fetch(base + endpoint, options);
  const data = await response.json();
  if (!response.ok || data.error) {
    throw new Error(`${endpoint}: ${data.error || response.statusText}`);
  }
  return data;
}

function post(base, endpoint, body = {}) {
  return jsonRequest(base, endpoint, {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify(body),
  });
}

function digest(text) {
  return createHash('sha256').update(text).digest('hex');
}

async function main() {
  const base = arg('base-url', 'http://127.0.0.1:18085');
  const xml = readFileSync(TEST_XML, 'utf8');
  const beforeHash = digest(xml);

  const loaded = await post(base, '/api/session/load_cim_dist', { xml_string: xml });
  if (loaded._recommended_reliability_model !== 'restoration_milp') {
    throw new Error('CIM import did not recommend the three-stage restoration model');
  }
  try {
    const before = await jsonRequest(base, '/api/session/reliability/data_quality');
    if (!(before.components_total > 0 && before.components_with_reliability_data === 0)) {
      throw new Error(`expected empty reliability data before apply, got ${JSON.stringify(before)}`);
    }

    const applied = await post(base, '/api/session/parameter_library/apply');
    const after = await jsonRequest(base, '/api/session/reliability/data_quality');
    if (!(applied.parameter_apply?.fields_changed > 0)) {
      throw new Error('parameter library did not change any fields');
    }
    if (!(after.components_with_reliability_data > 0)) {
      throw new Error(`reliability data remains empty after apply: ${JSON.stringify(after)}`);
    }

    const system = JSON.parse(applied._raw_json);
    const branches = system.ac?.branches || [];
    const totalLengthKm = branches.reduce((sum, branch) => sum + Number(branch.length_km || 0), 0);
    const aggregateFailureRate = branches.reduce((sum, branch) => sum + Number(branch.failure_rate || 0), 0);
    const pairs = new Set(branches.map(branch =>
      `${Number(branch.failure_rate).toFixed(6)}/${Number(branch.mttr_hr).toFixed(6)}`));
    if (branches.some(branch => !(branch.failure_rate > 0) || !(branch.mttr_hr > 0))) {
      throw new Error('one or more AC branches still lack failure rate or MTTR');
    }
    if (pairs.size < 2) {
      throw new Error(`expected equipment-specific branch defaults, got ${[...pairs].join(', ')}`);
    }
    for (const branch of branches) {
      const geometry = `${branch.line_type || ''} ${branch.conductor_model || ''}`.toUpperCase();
      const overhead = String(branch.line_type || '').includes('架空') ||
        geometry.includes('OVERHEAD') || geometry.includes('OHL');
      const cable = String(branch.line_type || '').includes('电缆') ||
        geometry.includes('CABLE') || geometry.includes('YJV') || geometry.includes('XLPE');
      const ratePerKm = overhead ? 0.50 : (cable ? 0.08 : 0.30);
      const exposureKm = Number(branch.length_km) > 1e-9 ? Number(branch.length_km) : 1.0;
      const expected = ratePerKm * exposureKm;
      if (Math.abs(Number(branch.failure_rate) - expected) > 1e-9) {
        throw new Error(`branch ${branch.index} failure rate is not length-scaled`);
      }
    }

    const reliability = await post(base, '/api/session/run_reliability', {
      method: 'three_stage',
      data_policy: 'case_data_only',
      load: { hours_per_year: 8760 },
      execution: { parallel: true, parallel_threads: 0 },
      restoration: {
        max_switch_operations: 2,
        include_dc_power_flow: true,
        parallel: true,
        parallel_threads: 0,
      },
    });
    const faults = reliability.faults || [];
    const faultRates = new Set(faults.map(fault => Number(fault.failure_rate).toFixed(6)));
    const repairTimes = new Set(faults.map(fault => Number(fault.tau_rep_hr).toFixed(6)));
    const positiveEens = new Set(faults
      .map(fault => Number(fault.eens_contribution_mwh_yr))
      .filter(value => value > 1e-12)
      .map(value => value.toFixed(9)));
    if (faultRates.size < 2 || repairTimes.size < 2) {
      throw new Error('three-stage reliability did not consume equipment-specific parameters');
    }
    if (positiveEens.size < 2) {
      throw new Error('positive EENS contributions remain uniform after parameter completion');
    }
    const frontend = { window: {} };
    runInNewContext(readFileSync(path.join(ROOT, 'web/js/core/result_mapping.js'), 'utf8'), frontend);
    runInNewContext(readFileSync(path.join(ROOT, 'web/js/search_registry.js'), 'utf8'), frontend);
    const bucketMaps = Object.fromEntries(
      frontend.window.HACDCSearch.SOURCES.map(source => [source.bucket, {}]));
    const firstFault = faults.find(fault => fault.canvas_type === 'branch');
    const mappedFault = frontend.window.HACDCSearch.resultRow(
      system, firstFault,
      type => frontend.window.HySimCore.ResultMapping.bucketFor(type, bucketMaps));
    if (!mappedFault || mappedFault.source.bucket !== 'branch' ||
        Number(mappedFault.item.index) !== Number(firstFault.canvas_index)) {
      throw new Error('headless reliability result did not map to its topology branch');
    }

    const nsq = await post(base, '/api/session/run_reliability', {
      method: 'nsq',
      data_policy: 'case_data_only',
      load: { hours_per_year: 8760 },
      execution: { parallel: true, parallel_threads: 0 },
      monte_carlo: {
        max_iterations: 2000,
        cov_threshold: 0.05,
        seed: 20260717,
        parallel: true,
        parallel_threads: 0,
      },
    });
    const nsqBaseline = Number(nsq.metrics?.baseline_eens_mwh_yr || 0);
    const nsqRaw = Number(nsq.metrics?.raw_eens_mwh_yr || 0);
    const nsqIncremental = Number(nsq.metrics?.incremental_eens_mwh_yr || 0);
    if (!(nsqBaseline > 0 && nsqRaw >= nsqBaseline && nsqIncremental >= 0)) {
      throw new Error('NSQ did not separate N-0 baseline and incremental EENS');
    }
    if (Math.abs(Number(nsq.metrics?.eens_mwh_yr) - nsqIncremental) > 1e-9) {
      throw new Error('GUI reliability metric is not using incremental NSQ EENS');
    }

    const afterHash = digest(readFileSync(TEST_XML, 'utf8'));
    if (afterHash !== beforeHash) throw new Error('data/test.xml was modified');

    console.log(JSON.stringify({
      before: `${before.components_with_reliability_data}/${before.components_total}`,
      fields_changed: applied.parameter_apply.fields_changed,
      after: `${after.components_with_reliability_data}/${after.components_total}`,
      branch_parameter_set_count: pairs.size,
      branch_parameter_samples: [...pairs].sort().slice(0, 10),
      reliability_fault_rate_count: faultRates.size,
      reliability_fault_rate_samples: [...faultRates].sort().slice(0, 10),
      reliability_repair_times_hr: [...repairTimes].sort(),
      distinct_positive_eens_contributions: positiveEens.size,
      total_line_length_km: Number(totalLengthKm.toFixed(6)),
      aggregate_branch_failure_rate_per_year: Number(aggregateFailureRate.toFixed(6)),
      total_eens_mwh_per_year: reliability.metrics?.eens_mwh_yr ?? reliability.eens_mwh_yr,
      nsq_incremental_eens_mwh_per_year: nsqIncremental,
      nsq_baseline_eens_mwh_per_year: nsqBaseline,
      nsq_raw_eens_mwh_per_year: nsqRaw,
      nsq_model_scope: nsq.model_scope,
      headless_mapping: `${mappedFault.source.bucket}:${mappedFault.item.index}`,
      original_xml_unchanged: true,
    }, null, 2));
  } finally {
    await post(base, '/api/session/load_cim_dist', { xml_string: xml });
  }
}

main().catch(error => {
  console.error(error.message || error);
  process.exitCode = 1;
});
