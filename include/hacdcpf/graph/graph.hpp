#pragma once

/// graph/graph.hpp
/// ================
/// Umbrella include for the hacdcpf graph analysis & reduction module.
///
/// Usage:
///   #include "hacdcpf/graph/graph.hpp"
///
/// Provides:
///   hacdcpf::graph::build_power_system_graph()
///   hacdcpf::graph::analyze_topology()
///   hacdcpf::graph::contract_zero_impedance_edges()
///   hacdcpf::graph::classify_reduction_candidates()
///   hacdcpf::graph::make_reduction_plan()
///   hacdcpf::graph::apply_series_reduction()
///   hacdcpf::graph::apply_pendant_reduction()
///   hacdcpf::graph::apply_kron_reduction()
///   hacdcpf::graph::recover_*()

#include "hacdcpf/graph/kron_reduction.hpp"
#include "hacdcpf/graph/power_system_graph.hpp"
#include "hacdcpf/graph/reduction_mapping.hpp"
#include "hacdcpf/graph/reduction_plan.hpp"
#include "hacdcpf/graph/result_recovery.hpp"
#include "hacdcpf/graph/series_reduction.hpp"
#include "hacdcpf/graph/switch_contraction.hpp"
#include "hacdcpf/graph/topology_analysis.hpp"
