# AGENTS.md — HySim-XJTU-HRPES 项目记忆

> 供 AI 代理与新加入开发者快速建立全局认识。与代码冲突时，以 `src/`、`include/`、`tests/`、`CMakeLists.txt` 为准。
> 最后核实：2026-08-09。当前验证基线与未闭环调试见 `docs/development_status.md`。

## 项目速览

- **HySim-XJTU-HRPES**：交直流混合高弹性能源电力系统仿真分析平台（西安交通大学 HRPES 团队）。
- 核心是 C++20 静态库 `hacdcpf`（`src/` + `include/hacdcpf/`），外加独立 HTTP 后端可执行 `run_gui_server`（`tests/run_gui_server.cpp`，约 2.1 万行）与原生 JS 单页 GUI（`web/`，挂载于 `/xjtu/`）。
- 唯一必需外部依赖是**兄弟仓库 `../MIPSolvers`**（本地源码，非系统安装；提供 Eigen3、fmt、nlohmann_json、HiGHS、Ipopt/SCIP 与 AML 建模层）。路径可用 `MIPSOLVERS_SOURCE_DIR` 覆盖。

## AI 快速入口

| 入口 | 用途 |
|---|---|
| `docs/development_status.md` | 最近验证的构建/测试基线、当前脏工作树与未闭环问题 |
| `docs/README.md` | 实现契约与理论材料的唯一文档导航入口 |
| `.github/skills/manage-codebase-context/SKILL.md` | 代码状态核实、交接和 AI 文档同步流程 |

### Skills

| Skill | When to use |
|---|---|
| [manage-codebase-context](.github/skills/manage-codebase-context/SKILL.md) | 仓库入门、当前代码/测试状态、架构交接、依赖升级或 AI 文档维护 |
| [develop-gui-backend-contract](.github/skills/develop-gui-backend-contract/SKILL.md) | HTTP/JSON、参数编辑、Canvas/拓扑定位、分析可视化、响应式 GUI 或前后端 E2E 的同步设计与实现 |

## 架构主线（务必先理解）

```text
Rich HybridPowerSystem（工程语义层）
  -> validation（Basic/Electrical/SolverReady/Strict 四级）
  -> pre-formulation（设备聚合 aggregate_* + 图建模 + 零阻抗合并/开关收缩）
  -> canonical projection（project_to_canonical_models，生成 BusMergeMap/BranchExpandMap/ComponentMapping）
  -> SolverData 装配（Ybus / DC 电导矩阵）
  -> 求解 / 分析（PF、OPF、重构、可靠性、弹性、市场……）
  -> projection back（unproject_bus_vector、结果归因回原始组件）
```

三条铁律：

1. **AC/DC 域用 domain-qualified map**（`ac_bus_id_to_node_idx` / `dc_bus_id_to_node_idx`），不要用裸 `int bus_id` 跨域传递。
2. **三类 ID 不混用**：组件 `.index`（对外稳定）、vector position（内部 0-based）、graph node/edge index（临时）。对外报告必须经 mapping 回到稳定 ID（如 `BranchRef`）。
3. **诚实结果口径**：近似/fallback/time-limit/模型覆盖不足必须写进 result 的 `model_scope`、`ValidityFlags` 或 `model_limitations`。新模块普遍遵循此模式（如市场运行时按资产组成改写 scope 字符串——混合系统为 `ac-dc-linear-v1:dc-voltage+bidirectional-converters`，仅 external_grid/energy_router 显式退回；可靠性 `ac-only-dcopf` 声明）。

## 目录地图（src/ ↔ include/hacdcpf/）

| 模块 | 内容 |
|---|---|
| `model/` | 全部组件 POD 模型与 `HybridPowerSystem` 容器；`defaults.hpp` 常量单一来源；`Result<T>` 错误类型 |
| `projection/` | rich→canonical 投影与映射类型（**实现在 `src/model/`**，无同名 src 目录） |
| `assembly/` | `SolverData`、索引映射、Ybus 构建（**实现在 `src/power_flow/`**） |
| `power_flow/` | 潮流全家族：NR、FDPF、DC、自适应孤岛、分布式松弛、HELM、同伦、Newton-Krylov；LM 信赖域/非单调线搜索全局化；CPF 电压稳定；三相 NR；换流器协调 |
| `optimal_power_flow/` | AC OPF（Native AC / Parity IPM / 嵌入式 Ipopt）、DC OPF、RPO（含 OLTC 离散档位）、三相混合 OPF `opf::phase_hybrid`（活跃研发） |
| `power_models/` | 基于 MIPSolvers AML 的求解器无关建模层（acopf/acdcopf/dc_opf/lindistflow/scuc builder） |
| `graph/` | 图建模、拓扑分析、开关收缩、Kron/series/pendant/sparse-Kron 降阶与结果恢复 |
| `network_reconfiguration/` | ONR 与故障后重构 MILP |
| `reliability/` | MC、FMEA（含 failure-mode 目录与信息物理 L1）、三阶段 MILP、F&D |
| `resilience/` | 启发式/多时段 MIP/分阶段 MILP 弹性恢复，MESS 路由 |
| `analysis/` | 承载力（DL/T 2041-2025）、反事实规划、多维薄弱环节 |
| `scenario_generation/` | 常规/可靠性/弹性场景 + 台风（Holland 风场） |
| `short_circuit/` | IEC 60909 简化+详细 AC 短路、DC 故障水平 |
| `harmonics_power_flow/` | 频域穿透/Newton/三相/AC-DC 耦合谐波、频扫、IEEE 519/GB/T 14549 |
| `dynamics/` | 机电暂态 DAE：7 类求解器（含 MassMatrixDae）、小信号、频率观测、丰富设备模型库、IEEE 1547 保护 |
| `time_series/` | UC→OPF→PF 流水线、年度分层生产模拟（可按日并行）、生命周期仿真 |
| `carbon_analysis/` | 碳流追踪、年度碳、用户/节点 GEC |
| `ev_power_traffic/` | EV-交通耦合 Formulation A–H（CTM/LTM、联合优化、选址定容、MPC） |
| `integrated_energy/` | 园区电-热-氢多能流 MILP |
| `market/` | 日前市场 SCUC→SCED/LMP→N-1 割→结算，实时双结算，重复博弈（DC 电压线性化的全混合出清；仅 external_grid/energy_router 退回） |
| `sppt/` | 语义保持投影理论的可执行验证层：MR1–MR8、证书语料、准入守卫、agent 循环 |
| `io/` | JSON、MATPOWER、CIM（CGMES3+配电）、GridLAB-D、PSD.jl、BPA/DSP dat（含 LCC 直流卡）；ETAP（OpenXLSX）与 OpenDSS（dss_capi）可选 |
| `api/` | 公共门面 `hacdcpf.hpp` + `solver_capabilities.hpp`（运行时后端查询） |
| `aml/`、`engine/`、`solver/` | **header-only 转发层**到 MIPSolvers；唯一例外 `engine/solver/native/nle/newton_solver.hpp` 是本项目 NR 潮流头 |

## 构建与测试

```bash
cmake --preset macos-release        # 或 linux-release / windows-vcpkg-release / full-dev
cmake --build --preset macos-release
ctest --preset macos-release
```

- CMake 选项：`HACDCPF_DEPENDENCY_PROFILE`（portable/full/minimal）、`HACDCPF_ENABLE_ETAP`（**默认 ON**，需 OpenXLSX，找不到则 FATAL_ERROR）、`HACDCPF_ENABLE_OPENDSS(_COMPARE)`（OFF）、`HACDCPF_ENABLE_IPOPT`（仅 macOS 默认 ON）、`HACDCPF_USE_SUITESPARSE`（ON，缺失回退 Eigen SparseLU）。
- 测试：100+ C++ 目标（约 1250 个 Catch2 用例）+ Node/Playwright/Python E2E。外部依赖（gridlabd、Julia、OpenDSS、chromium）缺失时自动 skip。浏览器 E2E 需 `npm i -D playwright && npx playwright install chromium`。
- GUI E2E：`ctest -R gui_api_e2e`（Python 全链路冒烟）。

## 常见坑（已核实）

- `include/` 与 `src/` 目录名不一一对应：`projection/`、`assembly/` 的实现在 `src/model/`、`src/power_flow/`；`scenario_generation`、`short_circuit` 的头文件在 `include/hacdcpf/analysis/`。
- `tests/short_circuit/` 与 `tests/short_circuit_case.json` 是**未跟踪的临时材料**，未接入构建。
- `tests/dynamics/` 是空目录；`web/js/workers/` 也是空目录。
- `CMakeLists.txt` 中 `src/power_flow/jacobian_check.cpp` 重复列出两次（无害）。
- CI（`.github/workflows/ci.yml`）公开仅 lint JS/Python；C++ build-test 被仓库变量 `ENABLE_FULL_CI` 门控且需私有 MIPSolvers。
- `tests/e2e/reliability_parameter_library_e2e.mjs` 是孤儿脚本（无任何构建/文档引用）。

## 文档维护原则

- 文档导航唯一入口 `docs/README.md`；区分**实现契约**（可更新）与**理论参考**（不承诺全部投产）。
- **任何后续修改都必须在文档中留下记录**：代码、配置、接口、数据结构、数学模型、算法行为、构建测试或运行边界发生变化时，必须同步更新对应的 `docs/` 分类文档或模块手册，记录变更内容、影响范围、验证结果与已知限制；未完成文档同步的修改不得视为完成。
- 不新增带日期的审计快照；不把 roadmap 写成当前行为；结果向量必须声明索引空间与单位；近似必须标注有效边界；HTTP 示例用 `tests/run_gui_server.cpp` 的生产路由。
- 大改代码后同步更新：根 `README.md`（状态快照表、能力表、实现地图）与 `docs/README.md` 索引日期。
- 修改 projection/graph 代码后，优先补 round-trip、结果恢复、AC/DC 同号 bus ID、dead-island、零阻抗合并测试。
