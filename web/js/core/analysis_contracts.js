/** HySim analysis endpoint metadata and result-contract helpers. */
'use strict';

(function initAnalysisContracts(global) {
  const core = global.HySimCore = global.HySimCore || {};

  const endpoints = new Map([
    ['/api/session/pf', 'power_flow'],
    ['/api/session/opf', 'optimal_power_flow'],
    ['/api/session/sc', 'short_circuit'],
    ['/api/session/sc_detailed', 'short_circuit'],
    ['/api/session/dc_sc', 'dc_short_circuit'],
    ['/api/session/harmonics', 'harmonics'],
    ['/api/session/harmonics_3ph', 'harmonics'],
    ['/api/session/harmonics_newton', 'harmonics'],
    ['/api/session/harmonics_freqscan', 'harmonics'],
    ['/api/session/harmonics_metrics', 'harmonics'],
    ['/api/session/run_transient', 'transient'],
    ['/api/session/run_ts_pf', 'time_series_power_flow'],
    ['/api/session/run_annual_sim', 'annual_production'],
    ['/api/session/topology', 'topology_analysis'],
    ['/api/session/network_reduction', 'network_reduction'],
    ['/api/session/run_reconfig', 'topology_reconfiguration'],
    ['/api/session/run_hosting_capacity', 'hosting_capacity'],
    ['/api/session/run_reliability', 'reliability'],
    ['/api/session/run_distribution_resilience', 'resilience'],
    ['/api/session/run_carbon', 'carbon_flow'],
    ['/api/session/run_dynamic_carbon', 'dynamic_carbon_flow'],
    ['/api/session/run_campus_ies', 'integrated_energy'],
    ['/api/session/run_ev_traffic', 'ev_power_traffic'],
    ['/api/session/generate_scenarios', 'scenario_generation'],
    ['/api/session/generate_typhoon_faults', 'scenario_generation'],
  ]);

  const labels = Object.freeze({
    power_flow: '潮流', optimal_power_flow: '最优潮流', short_circuit: '短路',
    dc_short_circuit: 'DC短路', harmonics: '谐波', transient: '暂态仿真',
    time_series_power_flow: '时序潮流', annual_production: '年度生产模拟',
    topology_analysis: '拓扑分析', network_reduction: '网络化简',
    topology_reconfiguration: '拓扑重构', hosting_capacity: '承载力分析',
    reliability: '可靠性分析', resilience: '弹性分析', carbon_flow: '碳流分析',
    dynamic_carbon_flow: '动态碳流', integrated_energy: '综合能源分析',
    ev_power_traffic: '电力-交通分析', scenario_generation: '场景生成',
  });

  function analysisForPath(path) {
    return endpoints.get(path);
  }

  function labelForAnalysis(analysis) {
    return labels[analysis] || analysis || '分析';
  }

  function reliabilityControlState({ physicalModel = 'auto', method = 'nsq',
                                     cyberEnabled = false } = {}) {
    const useThreeStage = physicalModel === 'restoration_milp';
    const useCyberPhysical = method === 'fmea' && !useThreeStage;
    return {
      useThreeStage,
      useSequential: method === 'seq' && !useThreeStage,
      useCyberPhysical,
      cyberEffective: useCyberPhysical && cyberEnabled,
      cyberToggleEnabled: true,
      cyberParametersVisible: useCyberPhysical,
      cyberParametersEnabled: useCyberPhysical,
    };
  }

  function attachResultContract({ path, data, requestId, modelRevision, currentModelRevision }) {
    const analysis = analysisForPath(path);
    if (!analysis || !data || typeof data !== 'object' || Array.isArray(data)) return false;
    const stale = modelRevision !== currentModelRevision;
    const status = data.error ? 'failed'
      : data.converged === false || data.feasible === false ? 'infeasible'
      : 'completed';
    data._result_contract = {
      schema: 'hysim_result_v1',
      component_ref_schema: 'hysim_canvas_ref_v1',
      analysis,
      request_id: requestId,
      model_revision: modelRevision,
      current_model_revision: currentModelRevision,
      stale,
      status: stale ? 'stale' : status,
      received_at: new Date().toISOString(),
    };
    return stale;
  }

  core.AnalysisContracts = Object.freeze({
    schema: 'hysim_analysis_contracts_v1',
    analysisForPath,
    labelForAnalysis,
    attachResultContract,
    reliabilityControlState,
  });
})(window);
