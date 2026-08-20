# 第 11 章 理论与设计参考

> 本章整合自: docs/ 下带日期的推导/设计记录与 docs/todo/ 研究笔记（索引性质）

手册前 10 章面向"如何使用"。本章索引仓库中保留的理论推导与设计记录，
供需要理解求解器内部决策依据的算法研究者查阅。按仓库 AGENTS.md 的
theory-guided 规则，所有算法/数值改动必须以这些推导文档为依据，新结论
也应写回相应文档。

## 11.1 推导与设计文档（docs/archive/）

这些文档是手册内容的原始来源，现已冻结归档；其中相互引用可能仍指向
归档前的路径。

### 数值方法与求解器设计

| 文档 | 内容 |
|---|---|
| [numerical_methods.md](../archive/numerical_methods.md) | scaling、KKT、稀疏分解、迭代改进等公共数值约定 |
| [solvers.md](../archive/solvers.md) | 求解器实现与算法审查手册（总架构、数学原理、源码定位） |
| [native_ipm_design.md](../archive/native_ipm_design.md) | Native LP 内点法设计：现代 IPM 文献、HSD/IP-PMM 路线、验收门槛 |
| [conic_sdp.md](../archive/conic_sdp.md) | LP/SOCP/SDP 锥内点法完整推导 |
| [engine.md](../archive/engine.md) | Engine API 详细参考 |

### IPM / OPF 专项推导

| 文档 | 内容 |
|---|---|
| [opf_native_ipm_structural_derivation.md](../archive/opf_native_ipm_structural_derivation.md) | OPF 原生 IPM 结构化推导；§8 记录中心性、缩放与接受门的轨迹修正（含 2026-08-18 冷启动初始化器观测） |
| [lp_tail_elimination_2026-08-18.md](../archive/lp_tail_elimination_2026-08-18.md) | LP 长尾歼灭工作流：NETLIB 90 全口径领先的量化预测与 measured-vs-predicted 记录 |
| [native_presolve_lp_2026-08-18.md](../archive/native_presolve_lp_2026-08-18.md) | 原生 LP presolve 立项推导与设计：桥死亡实测、成本模型、量化预测与 P0-P4 分阶段验收协议 |
| [ipm_structural_performance_2026-08-06.md](../archive/ipm_structural_performance_2026-08-06.md) | IPM 结构化性能分析（2026-08-06 快照） |
| [lp_kernel_selector_2026-08-11.md](../archive/lp_kernel_selector_2026-08-11.md) | LP 内核选择器：成本模型与并发组合 portfolio 的决策记录 |
| [native_ipm_windows_integration_2026-08-13.md](../archive/native_ipm_windows_integration_2026-08-13.md) | 原生 IPM 的 Windows 集成记录 |
| [klu_numeric_refactor_2026-08-20.md](../archive/klu_numeric_refactor_2026-08-20.md) | 固定模式 LE/NLE 的 KLU 数值重分解契约、成本模型、回退事务与验收协议 |

### Native MILP 专项（2026-08-13 系列）

| 文档 | 内容 |
|---|---|
| [native_milp_root_quality_restart_prerequisites_2026-08-13.md](../archive/native_milp_root_quality_restart_prerequisites_2026-08-13.md) | 根节点质量与重启前置条件（最完整的一篇） |
| [native_milp_root_source_eligibility_2026-08-13.md](../archive/native_milp_root_source_eligibility_2026-08-13.md) | 根节点来源资格判定 |
| [native_milp_cutpool_proof_ownership_2026-08-13.md](../archive/native_milp_cutpool_proof_ownership_2026-08-13.md) | 割池证明所有权 |
| [native_milp_degenerate_cutpool_fixed_point_2026-08-13.md](../archive/native_milp_degenerate_cutpool_fixed_point_2026-08-13.md) | 退化割池不动点分析 |
| [native_milp_presolve_varbound_coordinates_2026-08-13.md](../archive/native_milp_presolve_varbound_coordinates_2026-08-13.md) | presolve varbound 坐标系 |
| [native_milp_reliability_branching_2026-08-13.md](../archive/native_milp_reliability_branching_2026-08-13.md) | 可靠性分支策略 |
| [native_milp_highs_lp_root_profile_2026-08-13.md](../archive/native_milp_highs_lp_root_profile_2026-08-13.md) | HiGHS LP 根节点剖面 |

### 用户文档原始版本

`user_manual.md`、`build_and_deploy.md`、`python_api.md`、`aml_python_api.md`、
`case_builder.md`、`data_format_spec.md`、`testing.md`、`netlib_benchmark.md`、
`windows_ci_validation.md` 均保存在 [docs/archive/](../archive/)，为手册相应
章节的原始来源；内容以手册为准，归档件仅作历史追溯。

## 11.2 研究笔记（docs/todo/）

未完成的方法论研究，不代表当前实现状态：

- GPU 电力优化方法论：凝缩-分解-稠密化的理论与实现
- OPF 定制化加速技术方案
- 高性能线性代数内核理论分析与工程化差距评估
- 高性能线性代数求解器进展与电力系统应用综述
- linear_algebria.md（线性代数内核笔记）

## 11.3 教程资源（docs/tutorial/）

- `demo_01`–`demo_05`：可运行的 Python 演示（见
  [第 3 章 快速上手](03-quickstart.md)）。
- `slides.tex` + `build.sh`：Beamer 教程幻灯片源文件与编译脚本。
