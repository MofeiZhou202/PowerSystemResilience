# HySim-XJTU-HRPES 项目系统说明

**交直流混合高弹性能源电力系统仿真分析平台**<br>
高弹性能源电力系统研究团队 · 西安交通大学

本文档面向工程使用者和开发者，说明 HySim-XJTU-HRPES 从“工程场景建模”到“规范模型求解”、再到“结果回投”的完整链路。文档入口见 `docs/README.md`，更底层的公式和接口见 `docs/technical_notebook/`。

## 文档同步状态（2026-07-19）

- `docs/README.md` 是当前文档的唯一导航入口，明确区分运行契约与理论参考。
- 已删除被实现取代的阶段计划、一次性代码审查和重复暂态设计稿；不再用历史 roadmap 描述当前行为。
- 本次同步（2026-07-22）补入 2026 年 5–7 月新增能力域：电力市场、园区综合能源、承载力/薄弱环节/反事实规划、场景生成与台风弹性、年度碳/GEC、SPPT 可执行理论层、三相混合 OPF、电压稳定 CPF，以及 CIM/GridLAB-D/PSD.jl 等 IO 通道；并补齐高级潮流求解器专属回归、CPF 弧长增广、FDPF 稀疏注入与统一求解器接口。
- 2026-07-19 增加 `/api/v1` 多会话、模型 revision/ETag、异步 PF/OPF 作业，以及对应 Python SDK 与 AI 工具层；设计边界见 `docs/python_api.md`。
- 同步依据为当前 `CMakeLists.txt`、`CMakePresets.json`、`tests/CMakeLists.txt`、`src/`、`include/`、GUI 路由和 E2E 验证。
- 如文档描述与代码行为冲突，以仓库实现为准：`src/`、`include/`、`tests/`、`CMake` 配置优先。

## 快速构建与验证（基于当前实现）

推荐使用 CMake preset（与 `docs/cross_platform_build.md` 保持一致）：

```bash
cmake --preset macos-release
cmake --build --preset macos-release
ctest --preset macos-release
```

```bash
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release
```

```powershell
cmake --preset windows-vcpkg-release
cmake --build --preset windows-vcpkg-release
ctest --preset windows-vcpkg-release
```

## 当前建模与仿真包状态快照（2026-07-19）

本节用于快速回答“现在这个包到底做到哪一步了”。结论基于当前仓库源码组织、CMake 选项与已注册测试目标，而不是历史规划文档。

### 1) 总体状态（按可用性分层）

| 能力域 | 当前状态 | 说明 |
|---|---|---|
| 混合 AC/DC 潮流与聚合建模 | 已实现并持续回归 | 覆盖 canonical projection、AC/DC 潮流、换流器协调与图分析链路；求解器族含 Newton、FDPF（稀疏 Ybus 注入）、DC、自适应孤岛、分布式松弛、HELM、同伦延拓与 Newton-Krylov，并有 LM 信赖域/非单调线搜索/非线性缩放全局化层。`PowerFlowSolverFactory` 六类入口已接线，Newton 跨调用复用 `SolverWorkspace` 与 Jacobian pattern。 |
| OPF 与约束优化 | 已实现并持续回归 | AC OPF / DC OPF / RPO（含 OLTC 离散档位控制）已集成，支持 Native AC、Parity IPM、嵌入式 Ipopt 等多后端路径；GUI 回显参数用途、请求值/生效值、对偶有效性和模型边界。 |
| 三相混合 PF / OPF | 活跃研发中，GUI 已接入 | `powerflow::solve_three_phase_hybrid_pf` 与 `opf::phase_hybrid` 已接入 `/xjtu/` 潮流/OPF 工具栏；OPF 提供 Full 与 GraphReduced（稀疏 Kron 降阶）、Ipopt/NativeIPM 双后端。GUI rich-model 适配范围见下文。 |
| 电压稳定 | 已实现 | 连续潮流（CPF）采用增广 `[state, lambda]` 弧长预测-校正，可越过 P-V 鼻点并保留下支采样；输出 P-V 曲线与 VSI 指标。 |
| 图建模、网络降阶、重构 | 已实现并持续回归 | 支持连通性、开关收缩、Kron/series/pendant/sparse-Kron reduction、ONR。 |
| 可靠性与弹性分析 | 已实现并持续回归 | 包含 MC、FMEA（含 failure-mode 目录路径与信息物理 Level 1 调节）、三阶段可靠性与配电弹性分析（含 MIP 路径），以及有限恢复动作目录的 cyber/MESS 可执行性--多保真动态 oracle 主从循环；后者明确区分采样常数与全局证明。 |
| 三相与短路分析 | 已实现并持续回归 | 三相 NR 与 AC/DC 短路分析（IEC 60909 简化与详细路径、DC 故障水平估计）均有独立测试族。 |
| 谐波分析 | 已实现（持续增强） | 频域穿透、Newton 非线性、三相 abc 与 AC/DC 耦合谐波潮流，频扫/谐振检测与 IEEE 519 / GB/T 14549 合规校核。 |
| 暂态动力学 | 已实现基础框架（持续增强） | 动态建模、事件、7 类求解器（含 MassMatrixDae 同时式 DAE）、DAE 诊断、小信号与频率观测；设备模型覆盖同步机/调速器/励磁/PSS、GFM/GFL 逆变器、DER 与 IEEE 1547 保护。显式三相网络自动启用 GFL 逐相电流状态与相域限流，GFM 采用序耦合 Norton 端口和最大相电流限流；三线制默认阻断零序电流。 |
| 时序与年度生产模拟 | 已实现并持续回归 | UC MILP → AC-OPF → PF 校验流水线；年度分层分解（支持按日并行）、多年生命周期仿真与容量扫描对比。 |
| 电力市场 | 已实现（交直流线性商业模型） | 日前混合 SCUC → 固定组合/换流与储能方向 SCED → AC/DC LMP → DC 储能跨期优化 → 结算/uplift → 非线性交直流认证；实时双结算与重复博弈。N-1 采用 AC 支路预防式切平面、发电机能力约束和全覆盖元件纠正式 SCED 校核。 |
| 园区综合能源 | 已实现 | 电-热-氢-燃料多能流 MILP 调度（CHP、热泵、电解/燃料电池、多层氢储能、CCUS、碳预算）。 |
| 承载力、薄弱环节与反事实规划 | 已实现 | DL/T 2041-2025 分布式电源承载力（含工程校核）、多维薄弱环节辨识、五类措施反事实对比。 |
| 场景生成与台风弹性 | 已实现 | 常规/可靠性/弹性三族场景生成与聚类缩减，Holland 风场台风故障序列。 |
| 碳流追踪 | 已实现并持续回归 | 比例/矩阵碳流追踪、年度碳核算（含储能碳库存动态）与用户/节点绿电证书（GEC）核算。 |
| EV-电力-交通耦合 | 已实现（持续扩展） | CTM/LTM 传播、Formulation A–H 联合优化家族、选址定容 MILP 与滚动时域 MPC；结果按“可证伪证书”口径区分全局最优/局部驻点/启发式。 |
| SPPT 可执行理论层 | 已实现（研究验证性质） | 语义保持投影理论（`docs/latex/sppt_theory.tex`）的 MR1–MR8 证伪套件、MR3 证书语料（CSV/LaTeX）、准入守卫与 agent 循环。 |
| Web GUI 服务 | 已集成可运行 | `run_gui_server` 为独立可执行服务，上述能力均经 HTTP API 暴露；前端为原生 JS 单页应用。 |
| Python SDK 与 AI 工具层 | v1 已实现 | `/api/v1` 独立会话、模型 revision/ETag、异步 PF/OPF 作业；Python 提供类型化客户端、结果诚实性检查、工具 Schema、影响分级与显式变更批准。 |

### 2) 依赖与功能开关状态（当前默认）

| 项 | 当前默认 | 影响 |
|---|---|---|
| 依赖模式 `HACDCPF_DEPENDENCY_PROFILE` | `portable` | 默认构建核心能力，避免强绑定开发型附加组件。 |
| ETAP Excel IO `HACDCPF_ENABLE_ETAP` | `ON` | 需要 OpenXLSX：依次尝试系统包、本仓库/兄弟规划仓库的 vendored 副本，最后回退到 GitHub 下载；全部失败则 configure 报错，可显式 `-DHACDCPF_ENABLE_ETAP=OFF` 关闭。 |
| OpenDSS bridge `HACDCPF_ENABLE_OPENDSS` | `OFF` | 需显式开启并提供 DSS C-API。 |
| OpenDSS compare `HACDCPF_ENABLE_OPENDSS_COMPARE` | `OFF` | 依赖 OpenDSS bridge。 |
| SuiteSparse `HACDCPF_USE_SUITESPARSE` | `ON` | 找到 UMFPACK/KLU 时作为 OPF 稀疏 KKT 后端；未找到自动回退 Eigen SparseLU。MIPSolvers 提供 MUMPS（LDLᵀ）时其为默认后端（可用 `HACDCPF_OPF_LINEAR_SOLVER=mumps/umfpack/klu/eigen` 指定）；Parity IPM 默认采用增广 Newton 形式（`HACDCPF_OPF_KKT_FORM=condensed` 可切回）。 |
| IPOPT `HACDCPF_ENABLE_IPOPT` | macOS 默认 `ON`，其他平台默认 `OFF` | 当前嵌入式 IPOPT 路径按平台受限。 |

### 3) 测试覆盖信号（如何判断“不是纸面功能”）

- 当前 `tests/CMakeLists.txt` 已注册 100 余个 C++ 测试目标（约 1250 个 Catch2 用例），覆盖 IO、PF/OPF、图分析、重构、可靠性、弹性、短路、谐波、三相、暂态、EV-交通耦合、市场、综合能源、SPPT 与跨模块一致性；另有 Node/Playwright 浏览器 E2E 与 Python GUI HTTP E2E 注册进 CTest。
- 部分测试有运行时外部依赖门控（gridlabd 可执行、Julia、OpenDSS、Playwright/chromium 等），依赖缺失时自动 skip，不构成失败。
- 这表示“代码路径已工程化并具备回归入口”，但不等同于“你当前机器/当前配置已全部跑通”。
- 对外汇报建议使用两层口径：
  - 能力存在性：以源码与测试目标注册为准。
  - 可复现实测结论：以你本地 preset 构建与 ctest 结果为准。

### 4) 当前边界与建议口径

- 对可选 IO（ETAP/OpenDSS）和外部比较（GridLAB-D/OpenDSS/PSD.jl）应明确“需启用对应编译开关和运行时依赖”。
- 对暂态/谐波/跨引擎一致性类结论，建议标注“持续增强中”，避免描述为已完全定型。
- 电力市场已覆盖 DC 母线/支路/固定资源、VSC、DC/DC 双向传输、DC 储能跨期优化和 AC/DC LMP；N-1 覆盖发电机、AC/DC 支路、VSC、DC/DC 与两类 DC 储能，但只有 AC 支路 LODF 割进入定价 LP，其余采用固定组合纠正式 SCED 校核。DC 支路商业网损、换流器报价、母线/负荷/开关与保护故障仍未建模，外部电网和能量路由器仍显式拒绝。
- 省级市场规模尚无无条件在线时延承诺：SCUC 已注入机组时序/容量/备用/报价结构、经固定整数 LP 验证的 MIP start 和分支优先级；大型模型自适应使用 StrictHiGHS，并在当前分支树内尝试提交 AC 基态热限全局割，最终执行全候选复核，未完成时拒绝定价。不能通过 presolve 精确投影的热限进入外层轮次，并复用原空间根割、配套 root basis 和伪成本；新增热限后的旧开放节点树不直接沿用。默认显式 1% MIP gap 与 120 s 总时限，并返回实际 gap、树内提交、状态复用、树重建、候选/激活/剩余超限和证明口径。大型定价 LP 按变量阈值直达 HiGHS；LODF 使用稀疏因子复用与候选列按需计算，全元件事故 SCED 使用默认 4-worker 有界并行。异步取消、滚动时域、跨运行 artifact 缓存、可认证树 checkpoint 和注册规模基准仍是生产化缺口；大系统应限制 `n1_max_contingencies` 并分层运行。
- OPF 结果按实际路径声明有效边界：DC OPF 的凸二次成本在 LP 回退时使用 `pwl_segments` 分段，QP 路径回显有效分段为 0；节点 LMP 与支路拥塞 `mu` 分开认证，当前支路 `mu` 始终未认证。AC OPF 是非凸局部 KKT 求解，Ipopt 适配器不返回乘子因而无 LMP；RPO 是受时限/评估预算约束的离散邻域搜索，不提供全局 MINLP 证书。快照 OPF 不含跨时段 SOC，能量路由器端口守恒不含内部损耗。AML builders 标记为实验链路，其中 AML SCUC 无网络约束且 MILP 价格未认证。`HACDCPF_OPF_*` 环境变量仅为调试通道，不是稳定 API。
- 三相混合 OPF 与 SPPT 层属活跃研发/论文验证性质，接口与产物格式仍可能调整。
- 当文档、报告、UI 文案与实现不一致时，以本仓库 `src/`、`include/`、`tests/` 与 CMake 配置为最终依据。

## 1. 工程场景

本项目是一个 C++20 静态库，核心目标是支撑混合 AC/DC 配电系统的稳态仿真、优化、可靠性和弹性分析。典型工程对象包括：

- 城市/园区配电网：AC 馈线、DC 母线、VSC 换流器、DC/DC 变换器、联络开关、断路器、分布式电源、储能和充电设施。
- 主动配电网：PV、风电等可再生电源，静态发电机、柔性负荷、可控负荷、移动储能、微电网和虚拟电厂。
- 故障恢复和运行优化：N-1 故障枚举、三阶段故障恢复、网络重构、弹性恢复、多时段生产模拟、OPF 和碳流追踪。
- 标准算例和工程导入：MATPOWER、JPC JSON、CIM/CGMES 3.0 与配电 CIM XML、GridLAB-D GLM、PowerSimulationsDynamics.jl snapshot、Excel(ETAP)/OpenDSS 可选接口，以及项目内部 rich component schema。
- 内置案例目录：能力导向的两级内置案例体系——旗舰 13 个（GUI「模型IO → 内置/算例」工具栏下拉）+ 扩展 8 个（「加载算例」模态框"更多算例"组，`GET /api/cases` 全量 21 个结构化目录均含 `featured` 标记）；每个案例的规模、数据亮点与推荐演示路径见 [docs/case_catalog.md](docs/case_catalog.md)，19 个能力域 × 案例 × 断言的覆盖矩阵由 `tools/validate_case_capabilities.py` 一键验证（输出 `output/capability_coverage.md`）。

工程上，本项目不直接把所有复杂设备塞进一个求解器模型，而是采用分层流程：

```text
Rich HybridPowerSystem
  -> validation / diagnostics
  -> pre-formulation: aggregation + graph analysis + network merging
  -> canonical projection
  -> solver data / optimization model assembly
  -> analysis or numerical solve
  -> projection back / result attribution
```

这种设计的核心价值是：输入层保留工程语义，求解层保持数值模型简洁，输出层再把电压、潮流、可靠性贡献和拓扑动作映射回原始设备。

## 2. Rich Component 与标准模型层

项目的顶层数据结构是 `HybridPowerSystem`，定义在 `include/hacdcpf/model/hybrid_power_system.hpp`。它包含 AC、DC、换流器、微电网、VPP、移动储能和可选三相系统。

| 模型域 | Rich component / 标准模型 | 工程语义 | Canonical 处理 |
|---|---|---|---|
| AC 网络 | `ACBus`, `ACBranch` | 母线、线路、基础负荷/并联参数 | 直接进入 AC 导纳矩阵和潮流/OPF 模型 |
| AC 电源 | `Generator`, `StaticGenerator`, `RenewableGen`, `PVSystem`, `ExternalGrid` | 常规机组、分布式电源、可再生电源、外部电网 | 聚合成每母线注入；外部电网可设置 slack / 电压设定 |
| AC 负荷 | `Load`, `FlexibleLoad`, `AsymmetricLoad`, `AsynchronousMotor` | 静态负荷、柔性负荷、不对称负荷、电机 | 转换为 canonical load 或等效注入；ZIP 系数在组装阶段加权聚合 |
| AC 设备 | `Transformer2W`, `Transformer3W`, `Switch`, `CircuitBreaker`, `Shunt` | 变压器、开关、断路器、并联补偿 | 变压器/开关可展开为等效 `ACBranch`；保留 provenance 映射 |
| 充电设施 | `ChargingStation`, `Charger` | 站级或桩级 EV 负荷 | 桩级可投影到站级；作为恒功率负荷进入组装 |
| DC 网络 | `DCBus`, `DCBranch`, `DCLoad` | DC 母线（`DC_P` 有功母线 / `DC_V` 电压参考母线 / `DC_ISOLATED` 停电隔离母线）、线路、负荷 | 进入 DC 电导矩阵和混合潮流/优化模型；`DC_ISOLATED` 母线在图/孤岛分析中按停电处理，不作为电压参考，且在潮流方程组中以固定电压剔除，避免雅可比奇异 |
| DC 电源/设备 | `StaticGeneratorDC`, `PVArrayDC`, `DCDCConverter`, `DCCircuitBreaker`, DC storage | DC 电源、PV 阵列、DC/DC、DC 开断设备 | 投影到 DC 注入、DC 边或耦合设备；DC/DC 保留拓扑和占空比可行性字段 |
| AC/DC 耦合 | `VSCConverter`, `LCCConverter`, `EnergyRouter` | VSC/LCC 换流器、能量路由器、多端口耦合 | VSC 保留为带控制角色的耦合元件；LCC 为准稳态外特性模型（α/γ 角、换相电抗、内生无功），用于 BPA/DSP 的 BD/LD 直流卡；EnergyRouter 展开为内部 DC 母线、VSC 和 DC/DC |
| 聚合资源 | `VirtualPowerPlant`, `Microgrid`, `MobileStorage` | VPP、微电网、移动储能 | VPP/Microgrid 可转换为 PCC 注入；移动储能按位置和状态注入 |
| 三相系统 | `ThreePhaseACSystem` | abc 三相馈线和设备 | 可投影或单独由三相 NR 分析处理 |

因此，rich component 层偏工程数据模型，canonical 层偏求解器模型。二者之间不能简单等同，必须通过 projection 和 mapping 保持可追溯性。

## 3. Pre-Formulation：聚合、图分析与网络合并

Pre-formulation 是正式建立 PF/OPF/MILP 前的结构化预处理，主要解决三个问题：设备聚合、拓扑图表达、数值网络合并。

### 3.1 节点和设备聚合

求解器通常需要“每个节点的净注入”或“每个节点的一组受约束设备”。项目在 `src/power_flow/solver_data.cpp` 中执行以下聚合：

- `aggregate_generation`：把 `Generator`、`StaticGenerator`、`RenewableGen`、`PVSystem`、`Storage`、`VPP`、`Microgrid`、`MobileStorage` 聚合为每个 AC 母线的 `pg/qg`。
- `aggregate_load_demand`：把 `Load` 和 `ChargingStation` 聚合为每母线 `pd/qd`，并按负荷权重计算 ZIP 系数。
- `make_solver_data`：先调用 `project_to_canonical_models`，再移动 canonical component 到 `SolverData`，最后建立 AC `Ybus` 和 DC `gdc`。

这一步要求 bus index、设备 bus 引用、base MVA 和 in-service 状态一致，否则会出现 silent drop 或结果错位。因此正式求解前建议使用 `validate_full` 或 `validation::validate(..., ValidationLevel::Strict)`。

### 3.2 图建模与拓扑分析

图层入口是 `include/hacdcpf/graph/power_system_graph.hpp` 和 `src/graph/power_system_graph.cpp`。`build_power_system_graph` 将 `HybridPowerSystem` 转为：

- `GraphNode`：AC/DC 母线节点，带有 domain、slack、load、generator、storage、converter 等标记。
- `GraphEdge`：AC line、transformer、switch、breaker、DC line、DC switch、VSC coupling、DCDC coupling。
- domain-qualified maps：`ac_bus_id_to_node_idx` 和 `dc_bus_id_to_node_idx`，用于避免 AC/DC 母线使用相同数字 ID 时发生混淆。

图分析支撑：

- 连通性、孤岛、径向性、桥边、割点等拓扑诊断。
- 开关收缩、零阻抗边识别、Kron/series/pendant reduction。
- 网络重构和弹性恢复中的连通性约束、候选开关动作和故障隔离。

### 3.3 节点聚合与网络合并

项目有两类容易混淆但用途不同的“合并”：

- Projection 里的 zero-impedance bus merge：`merge_zero_impedance_buses` 将由零阻抗支路连接的 AC 母线合并，避免 `Ybus` 病态。映射记录在 `BusMergeMap`。
- Graph 里的 switch contraction：`contract_zero_impedance_edges` 根据闭合开关、闭合断路器和零阻抗边形成 super-node，同时聚合负荷/电源/并联元件，并保留 `ContractionResult`。

二者都属于 pre-formulation，但服务对象不同：前者偏数值求解稳定性，后者偏拓扑/图算法和恢复过程。开发时应优先使用 domain-qualified map，避免用裸 `int bus_id` 跨 AC/DC 域传递。

## 4. Canonical Models（规范模型）

Canonical projection 的入口是 `project_to_canonical_models`，定义在 `include/hacdcpf/projection/project_to_canonical.hpp`，实现主要在 `src/model/network_utils.cpp`。其职责是把 rich component 展开或折叠为求解器可以直接处理的扁平模型。

典型转换包括：

- 统一 base MVA，并把 bus-level load 与显式 `Load` 表保持一致，避免双计。
- 实际工程值到标幺值转换：当支路只给出实际值（`r_ohm_per_km`、`x_ohm_per_km`、`b_us_per_km`、`length_km` 和母线 `base_kv`）而 `r_pu/x_pu` 仍为零时，`convert_actual_to_per_unit`（在 `project_to_canonical_models` 入口执行）按 `Z_base = base_kv² / base_mva` 计算 `r_pu/x_pu/b_pu`，AC 与 DC 支路同理。该步骤是非破坏性且幂等的：已给定标幺值的支路保持不变，因此既支持 ETAP/OpenDSS 风格的实际值输入，也完全兼容既有标幺值算例。
- `FlexibleLoad`、`AsymmetricLoad`、`AsynchronousMotor` 转换为等效 `Load`。
- `Transformer2W`、`Transformer3W`、`Switch`、`CircuitBreaker` 转换为等效 branch，并通过 `BranchExpandMap` 记录来源。
- `EnergyRouter` 展开为内部 DC 母线、VSC 和 DC/DC 耦合。
- VSC 控制模式不直接等同于 bus type；潮流/协调检查阶段通过 `resolve_device_control_role` 解析 PQ、AC_PV、VDC_Q、VDC_VAC、AC_GRID_FORMING、DC_V_DROOP_AC_V 等模式的受控量、自由量和岛参考能力。
- `VirtualPowerPlant`、`Microgrid`、`MobileStorage` 投影为 PCC 注入或储能等规范设备。
- 执行零阻抗母线合并，生成 `BusMergeMap`，并剔除无供电路径的 dead islands。
- 生成 `ComponentMapping` / `ProjectionReport` 能力，用于诊断和结果归因。

Canonical 层的一个重要设计原则是：求解器只看到必要的数学对象，但用户仍能通过 mapping 理解某条 branch、某个注入或某个恢复动作来自哪个原始组件。

## 5. 现有分析能力

| 模块 | 入口/路径 | 使用模型 | 输出 |
|---|---|---|---|
| AC/DC Power Flow | `solve_power_flow`, `solve_dc_power_flow`, `solve_power_flow_fdpf`, `solve_ac_dc_power_flow`, `solve_power_flow_adaptive`, `solve_power_flow_distributed_slack` | canonical AC/DC network + converter coupling | 电压、相角、支路/VSC/DC-DC/ER 潮流、收敛状态、converter coordination 诊断、`converter_model_scope` |
| 高级/回退潮流求解器 | `HelmSolver`、`HomotopyContinuationSolver`、`NewtonKrylovSolver`、`AdaptiveSolver`（`solver_factory.hpp`，`PowerFlowMethod`） | 全纯嵌入、同伦延拓、GMRES+Schur 预条件 | 难收敛算例的回退求解路径与诊断 |
| 三相潮流 | `analysis::solve_three_phase_nr` | `ThreePhaseACSystem` | abc 相电压、电流和三相收敛信息 |
| 三相混合 PF | `powerflow::solve_three_phase_hybrid_pf` | 原生相域 AC + DC 节点平衡 + equal-phase/GFL/GFM 变换器稳态闭合 | AC/DC 电压、逐相变换器功率/电流、VUF、分域物理残差；可从工程初值独立复核三相混合 OPF 点 |
| OPF | `solve_ac_opf`, `solve_dc_opf`, `solve_rpo` | AC/IPM、DC LP/QP、无功优化模型（RPO 含 OLTC 离散档位邻域搜索） | 调度、目标值、节点 LMP、约束诊断、solver path、audit/infeasibility hints、`converter_model_scope`；DCOPF branch congestion dual 仅在 `branch_mu_valid=true` 时可作工程解释 |
| 三相混合 OPF | `opf::phase_hybrid::solve_three_phase_hybrid_opf`（Full / GraphReduced 变体，Ipopt / NativeIPM 后端） | 相域 AC + DC 混合 OPF，可选稀疏 Kron 降阶 | 三相调度、约束诊断与同模型 PF 回放（活跃研发中） |
| 电压稳定 | `CpfSolver`、`compute_vsi`（`power_flow/voltage_stability.hpp`） | 连续潮流（CPF） | P-V 曲线、VSI 指标 |
| 网络重构 | `solve_optimal_reconfiguration`, `run_topology_reconfiguration` | LinDistFlow MILP + graph connectivity | 开/合支路集合、损耗 proxy、PF 校验 |
| 图分析/降阶 | `build_power_system_graph`, `contract_zero_impedance_edges`, Kron/series/pendant/sparse-Kron recovery | graph abstraction | 连通性、径向性、super-node、恢复映射 |
| 可靠性 MC | `run_nonsequential_mc`, `run_sequential_mc` | component outage sampling + DC OPF state evaluation | EENS、LOLE、LOLF、CoV、VaR/CVaR、关键元件 |
| FMEA 可靠性 | `run_distribution_fmea`, `run_failure_mode_fmea` | N-1/N-2 enumeration + switching/repair stage evaluation；可选信息物理 Level 1 调节（`CyberPhysicalFMEAOptions`） | contingency detail、EENS/EDNS/SAIFI/SAIDI |
| 三阶段可靠性 | `run_three_stage_reliability` | native C++ MILP via MIPSolvers | 三阶段失负荷、SOP 动作、节点可靠性指标 |
| 配电弹性 | `run_distribution_resilience_assessment`, `run_distribution_resilience_mip_assessment` | heuristic sequential 或 multi-period MIP LinDistFlow | 恢复曲线、MESS 状态、故障序列、弹性指标 |
| 短路分析 | `compute_short_circuit`, `run_short_circuit_detailed`, `dc_bus_fault_level` | Z-bus IEC 60909 简化 / 完整 IEC（c 因子、κ/ip/ib/ik/ith、变压器修正、电机与换流器贡献）；DC 为戴维南保守上限估计 | 故障电流、IEC 指标、DC 故障水平与开断 duty |
| 谐波潮流 | `solve_harmonic_power_flow`（及 `_newton` / `_3ph` / `_3ph_hybrid` / `_hybrid_newton` 变体）, `frequency_scan`, `check_harmonic_limits` | 频域穿透（NIC 双端口桥）、Newton 非线性、三相 abc、AC/DC 耦合 | 谐波电压/电流、频扫/谐振、IEEE 519 / GB/T 14549 合规、K 因子/TDD |
| 暂态仿真 | `run_transient_simulation`, `small_signal_analysis`, `computeFrequencyReport` | 机电暂态 DAE（7 类求解器，含 MassMatrixDae 同时式） | 轨迹、事件、COI/孤岛频率、小信号摘要 |
| 碳分析 | `run_carbon_analysis`, `compute_annual_carbon_analysis`, `compute_annual_user_gec_accounting` | PF result + proportional / matrix tracing；年度时序含储能碳库存 | 节点、支路、负荷碳流；年度碳与用户/节点 GEC 核算 |
| 时序/生产模拟 | `solve_time_series_pf`, `solve_unit_commitment`, `solve_annual_production_simulation`, `run_lifecycle_simulation`, `run_lifecycle_comparison` | 多时段负荷/资源曲线 + OPF/UC | 年度生产、成本、生命周期指标、容量扫描对比 |
| 电力市场 | `market::run_day_ahead_market`, `run_real_time_market`, `run_repeated_market_game` | 混合 AC/DC SCUC → 固定组合/换流与储能方向 SCED → AC/DC LMP → DC 储能跨期优化 → 混合口径全元件 N-1 → 非线性交直流认证 → 结算/uplift | AC/DC LMP 与分域结算、DC 储能 SOC/结算、全元件事故校核、换流器传输/损耗、模型边界、HHI 等市场力指标 |
| 园区综合能源 | `integrated_energy::solve_campus_ies` | 电-热-氢-燃料多能流 MILP（CHP、热泵、电解/燃料电池、氢储能、CCUS） | 多能流调度、成本/碳目标 |
| 承载力评估 | `assess_hosting_capacity`（DL/T 2041-2025） | 设备级区间公式 + 可选 PF/短路/谐波工程校核 | 逐变压器/逐区域承载区间与分级 |
| 薄弱环节辨识 | `run_multidimensional_weak_link_assessment` | 多维压力证据评分（severity / consensus / Pareto） | 薄弱环节排序与模式对比 |
| 反事实规划 | `run_counterfactual_planning_assessment` | 扩容/储能/联络/自动化/DER 五类措施多维对比 | 反事实指标与两两协同分析 |
| 场景生成 | `generate_scenarios`, `generate_typhoon_fault_sequence`, `enumerate_n1_contingencies` | 常规/可靠性/弹性三族场景 + Holland 风场台风模型 + k-medoids 缩减 | 场景目录、台风故障序列 |
| EV-交通耦合 | `simulate_ev_power_traffic`（A）、`_ctm_due`（B+）、`_ctm_joint`（C）、`solve_joint_optimizer`（D）、`solve_ctm_so_lp`/`solve_ltm_so_lp`（E）、`solve_ctm_due_vi`（F）、`solve_infra_design_milp`（G）、`solve_ltm_mpc`（H） | CTM/LTM 交通传播 + DC-OPF/LMP 联合优化 | 耦合仿真结果与最优性证书（区分全局/局部/启发式） |
| SPPT 验证层 | `sppt::run_core_metamorphic_suite`, `certify_corpus`, `guard_system`, `run_agent_loop` | MR1–MR8 蜕变关系、独立残差证书、三道准入守卫 | 证伪/认证产物（CSV/LaTeX，研究验证性质） |

优化和 MILP 模块依赖 sibling directory `../MIPSolvers` 提供的 Eigen、HiGHS、Ipopt 和 native branch-and-cut 后端。项目 CMake 默认从该 sibling 路径解析依赖，而不是搜索系统 solver。

跨平台构建建议使用仓库内的 CMake presets；macOS/Linux/Windows 的依赖安装、
`MIPSolvers` 源码路径、SuiteSparse 稀疏求解器和 Windows OPF 后端选择见
[`docs/cross_platform_build.md`](docs/cross_platform_build.md)。

## 6. Validation 与诊断

Validation 是从 rich component 到 canonical model 的安全门。主要入口：

- `hacdcpf::validate_full(sys)`：公共 API 中的完整校验。
- `validation::validate(sys, ValidationLevel::...)`：可选择 `Basic`、`Electrical`、`SolverReady`、`Strict`。
- `project_to_canonical_models` 内部也会生成 projection mapping 和 dead-island / merge 相关信息。

当前静态校验覆盖：

- AC/DC bus ID 重复、branch/converter 引用不存在、孤岛、slack 缺失或多 slack。
- 电压上下限、机组 P/Q 限值、branch 阻抗、transformer tap、base MVA 不一致。
- VSC、DC branch、DC converter 引用错误，以及 VSC 控制角色/构网互斥/AC_PV 自由度/DC 岛电压源协调问题。
- 零阻抗和死岛等会影响数值稳定性的拓扑问题。

建议的工程流程是：

```text
import/load system
  -> validation::validate(sys, SolverReady or Strict)
  -> project_to_canonical_models(sys)
  -> optional graph diagnostics
  -> solve / analysis
  -> result audit and projection back
```

对于可靠性、弹性和网络重构这类组合优化任务，还应把求解器状态、MIP gap、time limit、模型规模和 fallback 状态写入结果对象，避免把启发式、近似可行和最优解混为一谈。

潮流结果的 `SolverDiagnostics` 还会携带 `converter_coordination`、结构闭合扫描、自动提升的 VSC 索引和 `effective_converters`。下游报告应优先解释这些最终生效的换流器状态，而不是只看输入 JSON 中的原始控制模式。

换流器容量圆、AC/DC 电流、调制比和 DC/DC 占空比在确定型 PF 中没有可自动重调度的自由量。默认模式保持兼容口径（数值根收敛并发出 `ACDC-PHYS-*` / `DCDC-PHYS-*` 告警）；启用 `PowerFlowOptions::enforce_converter_physical_limits` 后，任何超限根会被硬性拒绝并报告物理不可行。需要在约束下调整 P/Q 或电压设定时应使用 OPF，而不是由 PF 静默改写设定值。

## 7. Projection Back 与结果归因

Projection back 的目标是把 solver 内部的 compact/canonical 结果映射回用户输入的 rich model。主要机制包括：

- `BusMergeMap`：记录 external bus index 到 internal merged position 的映射、合并组、dead bus、branch 原始位置到 projected 位置。
- `unproject_bus_vector`：把求解器返回的电压、相角、LMP 等 bus vector 扩展回原始 bus 数量；dead-island bus 返回 0。
- `BranchExpandMap`：记录 canonical `ACBranch` 来自 `Transformer2W`、`Transformer3W` 或 `Switch` 的哪一个原始组件。
- `ComponentMapping`：记录 rich component 到 canonical component 的来源关系，服务于诊断、碳流和用户可读报告。
- `ContractionResult` 与 `FullNetworkVoltages`：用于图降阶后的电压恢复，特别是 switch contraction、series reduction、pendant reduction 和 Kron reduction。

开发时需要区分三类 ID：

- component `.index`：用户/模型层的稳定组件编号。
- vector position：数组中的 0-based 位置，适合内部循环，不适合对外报告。
- graph node/edge index：图构建后的临时下标，必须通过 mapping 回到模型组件。

任何跨越 projection、graph contraction 或 canonical solver 的结果，都不应直接用数组下标对外解释，必须经过对应 map。

## 8. 典型端到端流程

### 8.1 潮流/OPF 流程

```text
HybridPowerSystem
  -> validate_full
  -> project_to_canonical_models
  -> make_solver_data
  -> aggregate_generation / aggregate_load_demand
  -> build_admittance_matrix / build_dc_conductance
  -> PF or OPF solve
  -> unproject_bus_vector / branch provenance
```

### 8.2 网络重构/弹性恢复流程

```text
HybridPowerSystem
  -> project_to_canonical_models
  -> build_power_system_graph
  -> identify faults, switches, islands, roots
  -> LinDistFlow / connectivity MILP or heuristic restoration
  -> solve with native B&C / HiGHS / selected backend
  -> report BranchRef / switch actions / restoration metrics
  -> optional PF validation on restored topology
```

### 8.3 可靠性流程

```text
HybridPowerSystem + reliability data
  -> validate component failure rates, MTTR, customer/load metadata
  -> enumerate or sample component outages
  -> stage evaluation using OPF / reconfiguration / restoration model
  -> accumulate frequency-weighted EENS, LOLE, SAIFI, SAIDI
  -> expose model_limitations when physics coverage is approximate
```

### 8.4 I/O 流程

```text
MATPOWER / JPC JSON / CIM（CGMES 3.0 与配电 CIM）/ GridLAB-D / PSD.jl / Excel(ETAP) / OpenDSS
  -> rich HybridPowerSystem
  -> validation and projection
  -> analysis
  -> JSON result export or report generation
```

JPC JSON export should be treated as a schema-preserving operation: rich component arrays must either be written faithfully or explicitly diagnosed as unsupported, because silently writing empty arrays can lose engineering data.

### 8.5 ETAP I/O（导入/导出）

ETAP 互操作由 `include/hacdcpf/io/etap_io.hpp` / `src/io/etap_io.cpp` 提供，编译开关
`HACDCPF_ENABLE_ETAP`（依赖 OpenXLSX；默认 ON，可用 `-DHACDCPF_ENABLE_ETAP=OFF` 关闭）。

支持三条输入路径，全部映射到同一 `HybridPowerSystem`：

1. **规范 ETAP 工作簿**（`save_etap` 产生、可无损 round-trip 的 schema）。
2. **原始 ETAP 工具箱导出**（`etap-main/etap_output.py` 的列名与单位：`OpVMag`/`VMag`
   为百分比、`NominalkV`、`RPos`/`XPos` 为欧姆、`AnsiPosZ`/`PosR` 为变压器 %Z/%R、
   `ZBaseMVA` 为 kVA、`LUMPEDLOAD` 用 `MVA`+`PF`）。导入器通过别名表同时识别两套列名。
3. **原生 ETAP 工程 XML**（如 `Feeder.xml`）：`load_etap_xml()` 直接解析 `<COMPONENTS>`
   元素属性，无需 Python 工具箱。

主要 API：

| 功能 | 入口 |
|---|---|
| Excel 导入 | `load_etap(path[, mode, report])` |
| Excel 导出 | `save_etap(sys, path[, report])` |
| 原生 XML 导入 | `load_etap_xml(path[, mode, report])` |
| 严格度 | `EtapImportMode::{Strict, Permissive}`（Strict 对悬空母线引用抛错，Permissive 记 warning） |
| 往返保真度报告 | `etap_fidelity_check(sys)` → `EtapFidelityReport`（逐字段 before→after 差异） |
| 诊断 | `EtapIoReport`（每个 sheet 计数 + warnings） |

无显式 `Type` 列时，母线类型由所连 utility（→SLACK）/generator（→PV）推导；标幺↔欧姆
阻抗用 `Z_base = base_kV² / base_MVA`（取自支路 from 母线），保证往返精确。

支持的元件 sheet：`BUS, XLINE, CABLE, XFORM2W, XFORM3W, UTIL, SYNGEN, MGSET(仅导入),
PVARRAY, WIND, LUMPEDLOAD(ZIP), CAPACITOR, HVCB, INDMOTOR, DCBUS, DCIMPEDANCE,
DCLUMPLOAD, DCCONVERTER, DCCB, INVERTER, CHARGER, BATTERY` 外加 `PROJECT`。

**逐字段保真**：除拓扑与核心电气量外，往返还无损保留——2 绕组/3 绕组变压器分接头
（`TapSide/TapPos/TapStepPct`，3W 含 `ShiftMV/LV`）、负荷 ZIP 模型与优先级、VSC 控制模式
与设定点、电池 SoC/效率，以及**短路数据**：外部电网 `S_sc_max/min_MVA`、`RX_max/min`、
零序 `R0/X0`；同步机次暂态/暂态/同步电抗 `Xdpp/Xdp/Xd`、`Ra`、零序 `R0/X0`；断路器额定/
开断电流 `I_rated_kA`/`I_breaking_kA`；变压器零序 `Z0_percent`。原生 XML 同时识别 ETAP
原始属性（`ZeroR/ZeroX`、断路器 `Rated`、`AnsiPosXR` 反推 %R 等）。

**3 绕组变压器潮流**：投影时 3W 被展开为三条等效支路（成对短路阻抗构成的 Δ）。有载调压
（OLTC）仅作用于与受调绕组端子相连的两条支路（`tap_side`：0=HV，1=MV，2=LV），不影响对边
支路，物理上更准确。

命令行工具 `etap_convert`（`-DHACDCPF_ENABLE_ETAP=ON` 时构建）：

```text
etap_convert etap2json  in.xlsx  out.json   [--strict]
etap_convert xml2json   in.xml   out.json   [--strict]
etap_convert json2etap  in.json  out.xlsx
etap_convert etap2etap  in.xlsx  out.xlsx   [--strict]   # 规范化
etap_convert fidelity   in.xlsx                          # 报告再导出会丢失的字段
```

Python 侧 `etap-main/src/canonical_schema.py` 提供与 C++ 完全一致的列定义
（`CANONICAL_COLUMNS`）、`write_canonical_workbook()` 与 `convert_raw_export()`，
用于从工具箱直接产出规范工作簿。

**Web GUI 集成**（`tests/run_gui_server.cpp`，`web/` 下的画布编辑器挂载于 `/xjtu/`）：

| 操作 | 入口 |
|---|---|
| 导出当前系统为 MATPOWER `.m` | `POST /api/session/export_matpower`（文本下载）；工具栏「导出MATPOWER」按钮 |
| 导出当前系统为 ETAP `.xlsx` | `POST /api/session/export_etap`（二进制下载）；工具栏「导出ETAP」按钮 |
| 导入 ETAP `.xlsx`（二进制上传） | `POST /api/session/load_etap_xlsx`；「加载算例」对话框「导入ETAP工作簿 (.xlsx)」 |
| 导入原生 ETAP `.xml` | `POST /api/session/load_etap_xml`；「加载算例」对话框「导入ETAP工程 (.xml)」 |

GUI 潮流入口 `POST /api/session/pf` 新增 `method=three_phase_hybrid`，在同一 Newton 系统中联立 abc 相域 AC、DC 节点平衡与 VSC 稳态方程。GUI OPF 入口 `POST /api/session/opf` 新增 `network_model=three_phase_hybrid`，可选择 Full / GraphReduced 与 NativeIPM / Ipopt，并返回逐相电压、DC 电压、逐相电源/VSC 调度及同模型 `post_pf` 回放。该 GUI 适配器当前精确覆盖恒功率星形相负荷、纯电阻 DC 支路与直接 VSC；Delta/ZIP、DC/DC、能量路由器和可调 DC 静态电源会在前端禁用并由后端显式拒绝。AC 支路热限与 VSC 调制比尚未进入相域混合 OPF，响应通过 `scope` / `model_limitations` 如实标注。

平衡聚合 OPF 仍支持 parity/native/dc 求解路径，并把实际约束范围以 `scope.model_scope` 与布尔 flags 返回。AC/parity OPF 收敛后，后端会在 OPF 调度点再跑一次 PF，并返回 `post_pf` 支路潮流/VSC 转移以及 best-effort `post_carbon` 碳流结果；前端将 `post_pf.branch_flows` 用于 OPF 解上的潮流/负载率热力图叠加。

GUI 第一阶段统一契约包括：`hysim_task_status_v1`（任务状态、耗时与模型版本）、`hysim_result_v1`（分析、请求 ID、结果状态与陈旧性）和 `hysim_canvas_ref_v1`（结果行的元件类型、模型索引与 Canvas ID）。PF/OPF 在计算期间检测到模型版本变化时会丢弃过期结果；结果表统一通过 `data-result-ref` / `data-comp-id` 定位 Canvas，并支持鼠标和键盘操作。

第二阶段任务执行层为所有主要分析请求分配 `X-HySim-Request-ID`，活动任务期间锁定运行按钮并提供“取消等待”。取消会中止浏览器请求、忽略该请求的后续结果，并轮询后端直到求解收尾；由于当前 C++ 求解器没有统一的取消令牌，这不是强制终止求解线程。模型版本在请求期间变化时，响应统一标记为 `stale` 且禁止进入 Dashboard 或 Canvas。

第三阶段将共享前端基础设施从 `web/js/app.js` 拆分到 `web/js/core/`：`analysis_contracts.js` 维护分析端点和结果契约，`task_manager.js` 管理单活动任务、取消与后端收尾，`api_client.js` 统一请求 ID、错误和陈旧结果处理，`result_mapping.js` 维护 Dashboard 到 Canvas 的类型/索引映射。此后 `web/js/core/` 又扩展了 `timeseries_window.js`（长时序窗口化与保峰降采样）、`layout_graph.js` / `layout_engine.js`（布局语义投影与 ELK 异步布局客户端）、`accessibility.js`（键盘导航与可访问性审计）、`runtime_diagnostics.js`（前端运行时诊断）和 `network_overview.js`（WebGL2 全网 LOD 总览）。`app.js` 只保留 UI 状态回调和薄适配层，这些核心脚本必须在 `app.js` 之前加载。

第四阶段针对中大型系统优化交互性能：普通规模仍使用 SVG 单线图编辑，并将连续鼠标移动合并到浏览器动画帧、对视口外元件执行可逆裁剪；超过规模阈值时不再显示空白“无画布”摘要，而由 WebGL2 点/线缓冲绘制 LOD0 域、LOD1 区域和 LOD2 母线全网总览，同时保持 SVG glyph 数为零。WebGL、局部 k 跳 SVG、虚拟拓扑表和结果导航统一使用 `{domain,index}` 母线引用。年度生产模拟按 7/30/90 天窗口浏览；选择全年时采用保留首尾及负荷极值的降采样，最多绘制 2000 点，同时保留完整原始序列供导出和后续分析使用。规模化 GUI E2E 对 WebGL 非空像素、选择同步、局部 SVG 和这些性能边界提供回归契约。

`/api/v1` 为大模型客户端提供后端分块：`sessions/{id}/topology` 支持 LOD、空间视口和分页，`sessions/{id}/subgraph` 按稳定母线引用提取有界邻域，`jobs/{id}/frames/{step}` 按时间/域/稳定索引/空间返回结果窗口，`jobs/{id}/violations` 返回最严重电压与负载率越限。静态 PF/OPF 使用第 0 帧，后续生产模拟沿同一帧协议扩展多时步。

Python v1 SDK 对上述接口提供 `TopologyChunk`、`SubgraphView`、`ResultFrameChunk` 和 `ViolationChunk` 类型校验，支持自动分页、`BusRef` 稳定引用、指数退避作业等待与请求哈希审计。AI 工具层只暴露有明确行数上限的拓扑、子图、帧、越限和作业摘要，避免把完整大模型意外装入上下文。

第五阶段补齐工程界面的可访问性：工作流、模块和页签采用 roving-tabindex 键盘导航，支持方向键、Home/End，提供主工作区跳转、清晰焦点环、对话框语义、控制台播报、减少动画和高对比度偏好。`hysim_accessibility_audit_v1` 会检查关键地标、导航和控件名称。

第六阶段增加本地运行时防护：`hysim_runtime_diagnostics_v1` 捕获全局脚本错误、未处理 Promise 拒绝、HTTP/网络失败及在线状态，在依赖栏显示“前端”健康芯片。诊断仅在内存中保留最近 25 条截断记录，不上传、不持久化，并可通过 `App.getRuntimeDiagnostics()` 检查或清空。

无 GIS 自动布局采用 `hysim_layout_graph_v1` 语义投影：AC、DC 和耦合域分开，VSC/DC-DC/多端 Energy Router 保持星形超边，域内馈线从 Slack/外部电网开始识别，并保留锁定节点。30 母线或 180 元件以上的系统由本地 ELK.js 0.9.3 Worker 执行 layered 骨架布局，普通支路和设备再按电力语义回挂；失败时自动回退原 BFS 布局。Canvas 提供增量布局、位置锁定、馈线折叠/局部展开，以及 `hysim_layout_metrics_v1` 的交叉、重叠、折点、面积和耗时指标。四个代表算例的 PNG 与指标预算位于 `tests/e2e/baselines/layout/`，验证入口为 `tests/e2e/layout_baseline_e2e.mjs`。

往返与摄入由 `tests/test_io_etap.cpp` 覆盖（Excel round-circle、真实导出摄入、
case14 潮流一致性、逐字段保真度、原生 XML、3 绕组变压器分接头/潮流、短路数据），
fixtures 见 `data/etap_sample.xlsx`、`data/etap_feeder.xml`。GUI 后端端到端冒烟测试见
`tools/gui_api_e2e.py`（启动服务并驱动 加载/导出ETAP/重新导入/XML导入/潮流/短路 全链路，
已接入 ctest 目标 `gui_api_e2e`）。画布层浏览器端到端测试见 `tests/e2e/canvas_3w_e2e.mjs`
（Playwright：在画布上放置并连线一台三绕组变压器+外网+负荷，同步后端并跑潮流；需
`npm i -D playwright && npx playwright install chromium`）。

## 9. 实现地图

| 主题 | 主要文件 |
|---|---|
| 顶层模型 | `include/hacdcpf/model/hybrid_power_system.hpp` |
| AC/DC/rich components | `include/hacdcpf/model/ac_components.hpp`, `include/hacdcpf/model/dc_components.hpp`, `include/hacdcpf/model/converter_components.hpp` |
| Converter control/scope | `include/hacdcpf/model/device_control_role.hpp`, `include/hacdcpf/model/converter_model_scope.hpp`, `include/hacdcpf/power_flow/converter_coordination.hpp`, `src/power_flow/converter_coordination.cpp` |
| Canonical projection | `include/hacdcpf/projection/project_to_canonical.hpp`, `src/model/network_utils.cpp` |
| Projection mapping | `include/hacdcpf/projection/canonical_network.hpp` |
| Result attribution | `include/hacdcpf/projection/result_attribution.hpp`, `src/model/result_attribution.cpp` |
| 参数注册库 | `include/hacdcpf/model/standard_parameter_library.hpp`, `src/model/standard_parameter_library.cpp` |
| Solver data assembly | `include/hacdcpf/assembly/solver_data.hpp`, `src/power_flow/solver_data.cpp` |
| Graph model | `include/hacdcpf/graph/power_system_graph.hpp`, `src/graph/power_system_graph.cpp` |
| Graph reduction/recovery | `include/hacdcpf/graph/switch_contraction.hpp`, `include/hacdcpf/graph/result_recovery.hpp`, `src/graph/` |
| Validation | `include/hacdcpf/validation/validate_system.hpp`, `src/validation/validate_system.cpp` |
| 公共 API 门面 | `include/hacdcpf/api/hacdcpf.hpp`, `include/hacdcpf/api/solver_capabilities.hpp`, `src/api/hacdcpf.cpp` |
| PF/OPF | `include/hacdcpf/power_flow/`, `include/hacdcpf/optimal_power_flow/`, `src/power_flow/`, `src/optimal_power_flow/` |
| 高级潮流求解器 | `include/hacdcpf/power_flow/solvers/`, `include/hacdcpf/power_flow/globalization/`（HELM、同伦、Newton-Krylov、LM 信赖域等） |
| 电压稳定 CPF | `include/hacdcpf/power_flow/voltage_stability.hpp`, `src/power_flow/voltage_stability.cpp` |
| AML 代数建模层 | `include/hacdcpf/power_models/`, `src/power_models/`（ACOPF/ACDCOPF/DCOPF/LinDistFlow/SCUC builder） |
| MIPSolvers 转发层 | `include/hacdcpf/aml/`, `include/hacdcpf/engine/`, `include/hacdcpf/solver/`（header-only 转发；实现在兄弟仓库 `../MIPSolvers`） |
| 暂态动力学 | `include/hacdcpf/dynamics/`, `src/dynamics/` |
| 谐波潮流 | `include/hacdcpf/analysis/harmonics_power_flow.hpp`, `src/harmonics_power_flow/` |
| 短路分析 | `include/hacdcpf/analysis/short_circuit.hpp`, `include/hacdcpf/analysis/dc_short_circuit.hpp`, `src/short_circuit/` |
| 时序/年度/生命周期 | `include/hacdcpf/time_series/`, `src/time_series/` |
| 碳流/年度碳 | `include/hacdcpf/carbon_analysis/`, `src/carbon_analysis/` |
| EV-交通耦合 | `include/hacdcpf/ev_power_traffic/`, `src/ev_power_traffic/` |
| 电力市场 | `include/hacdcpf/market/market_simulation.hpp`, `src/market/` |
| 园区综合能源 | `include/hacdcpf/integrated_energy/`, `src/integrated_energy/` |
| 承载力/薄弱环节/反事实 | `include/hacdcpf/analysis/hosting_capacity.hpp` 等, `src/analysis/` |
| 场景生成/台风 | `include/hacdcpf/analysis/scenario_generation.hpp`, `include/hacdcpf/analysis/typhoon_resilience.hpp`, `src/scenario_generation/` |
| SPPT 验证层 | `include/hacdcpf/sppt/`, `src/sppt/`（理论：`docs/latex/sppt_theory.tex`） |
| 网络重构 | `include/hacdcpf/network_reconfiguration/`, `src/network_reconfiguration/` |
| 可靠性 | `include/hacdcpf/reliability/`, `include/hacdcpf/analysis/three_stage_reliability.hpp`, `src/reliability/` |
| 弹性恢复 | `include/hacdcpf/resilience/resilience_assessment.hpp`, `src/resilience/` |
| I/O | `include/hacdcpf/io/`, `src/io/`（JSON、MATPOWER、CIM、GridLAB-D、PSD.jl；ETAP/OpenDSS 可选） |
| 并行工具 | `include/hacdcpf/util/thread_pool.hpp`（`ThreadPool`、`parallel_for`） |
| GUI 后端服务 | `tests/run_gui_server.cpp`（独立可执行，旧 GUI API + 静态挂载 `web/`） |
| v1 运行时 API | `src/server/runtime_api_v1.hpp`, `src/server/runtime_api_v1.cpp`（多会话、ETag、异步作业） |
| Python SDK / AI 工具层 | `python/src/hysim/`（类型化客户端、可替换传输、结果口径、工具策略与本地服务生命周期） |
| Diagnostics/benchmarks | `tools/`（`opendss_pf_compare`、`etap_convert`、`matpower_pf_compare`、`sppt_certify`/`sppt_ablation`/`sppt_benchmark`/`sppt_agent_demo`、`hybrid_acdc_pf_study`、`phase_graph_reduction_benchmark`、`phase_hybrid_opf_benchmark`、gridlabd/transient/short-circuit validation matrices 等） |
| 文档索引 | `docs/README.md` |
| 技术笔记 | `docs/technical_notebook/` |

## 10. 维护原则

- Rich component 层尽量保留工程语义，不把设备信息过早丢弃。
- Canonical model 层只保留求解必要变量，并通过 mapping 保留来源。
- Graph 层所有 AC/DC lookup 应使用 domain-qualified map。
- 对外报告使用稳定 component ID 或结构化引用，例如 `BranchRef`，不要暴露临时 vector position。
- 任何 fallback、近似模型、time-limit 解和模型覆盖不足都应进入 result status / diagnostics。
- 修改 projection 或 graph 代码后，应优先补充 round-trip、result recovery、AC/DC 同号 bus ID、dead-island、zero-impedance merge 等测试。
