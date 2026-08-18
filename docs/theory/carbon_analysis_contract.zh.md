> 本文档为 [carbon_analysis_contract.md](carbon_analysis_contract.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

# 碳分析运行时契约

本文档涵盖 `carbon_analysis/` 中的快照碳流追踪、年度聚合、储能碳存量，
以及用户/节点绿色电力证书（green-energy-certificate, GEC）核算。

## 输入与算法

`compute_carbon_analysis(sys, pf_result, options)` 要求输入一个已收敛、
包含支路与换流器传输量的 `PowerFlowResult`。便捷重载会先运行潮流计算。
返回两个相关计算结果：

- 比例追踪法按已求解的潮流方向分摊电源功率与损耗责任；
- 稀疏线性方程组 `A w = b` 求解节点碳势（nodal carbon intensity）。

两条路径均保持稀疏。比例追踪法使用 SparseLU 对其稀疏分摊矩阵只做一次
分解，然后将分解因子应用于所有电源右端项。节点碳势求解使用秩揭示
SparseQR，因此 `matrix_rank`、主元条件数估计与残差有效性仍是结果契约的
一部分。任何生产路径都不会物化稠密的 `node_count x node_count` 矩阵。

模型消费系统与求解结果中显式存在的发电机、外部电网、可再生能源、
负负荷、储能、换流器、能量路由器与负荷语义。它是后处理：不会重新调度
功率，也不会修复失败的潮流。

## 单位与标识

| 量 | 单位或索引空间 |
|---|---|
| 有功功率、需求、损耗、失配 | MW |
| 电量与 GEC | MWh |
| 排放量与碳存量 | tCO2 |
| 碳强度/排放因子 | tCO2/MWh |
| `load_index`、`branch_index`、换流器/储能/路由器索引 | 稳定的源组件 `.index` |
| `bus`、`bus_index` | 稳定的源母线 `.index`，由 AC/DC 字段或结果集合加以限定 |
| `source_id` | 碳追踪内部 ID，不是组件 `.index`；归因请使用 `source_type` 与 `component_index`。 |

结果向量遵循其文档约定的源集合顺序，并同时携带稳定 ID。AC 与 DC 的
负荷、支路和母线是各自独立的集合。`generator_supply_mw` 与损耗分摊
映射以碳 `source_id` 为键。

## 有效性契约

不存在单一的隐式成功标志。消费方必须检查：

- `power_balance_verified` 与节点失配诊断；
- `matrix_solved`、秩、相对残差与条件数估计；
- `tracing_verified`，它要求物理平衡与分摊检查均通过。

如果提供的潮流结果未收敛，快照分析返回默认的未验证输出。矩阵奇异、
秩亏、条件数过大或残差不合格会保持可见；不会被转换为成功的零碳结果。
`loss_allocation_alpha` 改变的是损耗的责任归属，而不是物理支路损耗。

## 年度与储能核算

年度重载接受单一静态系统、每个潮流步一个系统，或一个
`TimeSeriesPFResult`。`step_duration_hr` 必须为正且有限。静态系统重载
仅在调度、负荷状态与储能状态均不变化时才正确。动态储能碳核算要求每个
潮流结果都对应一个实际的 `pf_system_snapshots[t]`。

`step_results[t]` 保留收敛性与碳有效性信息。未收敛的步会被计数，但不
被视为已验证；可选的小时级矩阵在这些步上为 NaN。矩阵的行是时间步，
列遵循 `bus_stats` 或 `load_stats` 的顺序。储能结果暴露初始/期末碳
存量、充电、放电与自放电损耗，以及电量与碳存量两方面的平衡残差。

## GEC 核算与导入格式

用户 GEC 将稳定的 `{load_index,is_dc}` 引用映射到外部 `user_id`。
节点 GEC 使用稳定的 `{bus_index,is_dc}` 键。GEC 核算会降低报告的净
排放量，但不改变物理总碳流。小时级核算要求来自年度分析的对应小时级
负荷数组。

CSV 接受严格文档约定的表头与严格的列数。字段可使用 RFC 4180 风格的
引号与双引号转义。整数、布尔与非负浮点字段必须完整消费解码后的字段；
引号格式错误、尾部多余文本、NaN/Inf、重复键以及冲突的用户行都会被
以 `std::invalid_argument` 拒绝。JSON 解析器应用相同的值域与非负性
检查。

## 近似边界

比例分摊是一种分摊约定，而非因果调度模型。矩阵法是有功稳态模型；它
不分配无功功率、动态因果性或边际排放。平衡电源的碳取决于显式的外部
电网/DC 参考排放因子。未建模的外送或缺失的电源会导致节点平衡有效性
门槛不通过。

## 已注册的验证

- `test_carbonflow_tracing`：解析电源分摊、损耗、AC/DC 映射、换流器、
  储能、富组件以及无效平衡。
- `test_carbonflow_dynamic_storage`：时间序列快照、SOC/碳递推、平衡
  残差以及严格 CSV 解析。
- `test_carbonflow_case_validation`：内置与 MATPOWER 算例在多步储能
  工作流上的验证。
- `test_crossmodule_integration`：潮流/最优潮流/重构/碳分析的一致性。

## 性能基线

在 Release 版云南算例上，规范碳矩阵为 `3927 x 3927`，报告秩为 3927。
稀疏实现完成生产 HTTP 请求耗时 `1.469 s`，`matrix_solved=true`，相对
残差为 `0`，条件数估计有限（`5.173`），功率平衡与追踪验证均为 true。
此前的稠密 QR 实现实测为 `11.35 s`。该算例没有编写的碳因子，因此该
测量验证的是数值管线与性能，而非回退排放因子的证据质量。
