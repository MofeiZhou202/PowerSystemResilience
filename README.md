# HySim-XJTU-HRPES 项目系统说明

**交直流混合高弹性能源电力系统仿真分析平台**<br>
高弹性能源电力系统研究团队 · 西安交通大学

本文档面向工程使用者和开发者，说明 HySim-XJTU-HRPES 从“工程场景建模”到“规范模型求解”、再到“结果回投”的完整链路。当前合同与实现参考统一从 `docs/README.md` 进入；历史审计、理论提案和旧技术总笔记隔离在 `docs/archive/`，不代表当前行为。

## 文档同步状态（2026-08-30）

- `docs/README.md` 是当前文档的唯一导航入口，明确区分运行契约与理论参考。
- 暂态保护新增固定前向事件聚类窗口与本地相量 CT/PT 频率/距离测量；EMT 测量
  在当前相量网络中显式拒绝，不以 COI 代替。
- 修正 case33bw AC/DC 算例新增可选的 GFL 直流欠压有功降额与定时闭锁，补齐
  平衡相量可靠性入口层的 DC 到 AC 功率/电压传播，并注册 C++、多步长和 Julia
  独立交叉验证。该能力不包含二极管故障馈流、MMC 阀级动态或 DCCB 电弧，不能
  表述为 EMT 级双向故障传播。
- N-k 研究驱动在固定进入事件内按 DAE 输出的不可用 VSC 集压缩恢复入口，
  不删除恢复 MILP 原有的 `sP/sQ` 切负荷变量或逐运行状态求解。case33 的
  92 个 N-1/N-2/N-3 事件与 6 个状态形成 552/552 个已解析条件，留一入口类
  准确率 100%，两次端到端投影加速为 5.76--5.99 倍；预注册 20 倍门槛失败，case123 尚未闭环。
  case123 的 1050 个条件中有 158 个在 DAE/事件时代数求解阶段失败；其余 892 个
  均进入并完成原切负荷恢复 MILP，因此该失败不应归因于恢复模型不支持负荷切除。
- `liuyanhui` 的 BPA/DSP 增量已同步到 `main`：新增 BA/BB/BM/LM/LY 受限导入、
  L/T 小电抗策略、BQ 来源限定 active-set、DC/LCC 来源字段和 JSON 往返。
  BM/LM 三端 DSP fixture 未入库，故该外部数值对拍仍明确标为未闭环；当前
  macOS Release 完整重链后为 1675 passed、4 conditional skips、0 failed。
- `main` 提供统一 Trial 能力清单、后端 fail-closed 403 防绕过、五阶段
  GUI、后端指标分析计划、Trial 专属验收测试和白名单 Windows 打包；
  维护边界见 `docs/trial_edition_design.md`。
- 历史审计、理论提案与旧技术总笔记已迁入 `docs/archive/`；受版本控制的
  PDF/LaTeX 中间产物已移除，活动索引不再直接导航到归档材料。
- `AGENTS.md`、`CLAUDE.md` 与项目级 `manage-codebase-context` skill
  为不同编码代理提供统一入口；动态验证基线和未闭环调试集中维护在
  `docs/development_status.md`，不再依赖按日期堆积的审计快照。
- 平衡 Newton 潮流默认使用固定 PV/PQ superset Jacobian、KLU 后向误差守卫的
  numeric refactor，以及“直接 Newton -> 半光滑 NCP -> DC 相角种子活动集 ->
  DC 种子 NCP -> 同伦”的数值升级梯级；五个困难平启动 MATPOWER 算例均已
  收敛并通过 Q 证书。重复求解可使用 `PreparedPowerFlowSession` 复用 projection、
  assembly、固定 pattern 与 symbolic analysis。依赖侧 KLU adapter 已提交至
  `../MIPSolvers` 的 `3bf1e66`；当前依赖 pin 已推进至包含后续线性代数更新的
  `4a0b16a`。
- 2026-08-08 将 MIPSolvers 固定到 `60f8bc4`：纳入系统更新后的 native
  dual simplex（分区 PRICE、驻留 pivot workspace、warm re-optimization）、
  LP IPM Gondzio multiple centrality correctors、MILP B&C 拆分与跨平台构建修复。
  `full-dev` 同时构建两仓库完整测试树；该历史升级回归 1429/1429 个已注册
  测试通过，3 个缺少外部运行时或条件不满足的用例明确跳过。当前 Trial
  Trial 集成先将依赖 pin 更新到 MIPSolvers `7b4cba8`；当前 pin 为
  `4a0b16a`，加入 Native IPM 的原坐标可行起点审计、方向性变量界契约、
  central warm-start 审计与 NativeLCQP 协作式墙钟截止。有限预算
  的稀疏 Phase I 已覆盖纯 AC、平衡 Hybrid AC/DC 与三相混合 OPF，在
  `mu0 = 0.1` 下以 `1e-1` 为 admission 门槛、`1e-2` 为 primal handoff
  corridor，并构造 central dual/slack；DCOPF 的 NativeLCQP 路径另采用逐连通
  分量功率配平与约化 Laplacian 初值。Phase II 最终容差不变，只有完整状态
  通过 MIPSolvers 独立审计后才使用，未接受时严格回到普通冷启动。当前代码的
  `macos-release` 全量 CTest 为 1482/1492 通过、7 个 OPF/RPO 失败、3 个条件跳过，
  不作全绿声明。大型纯 AC 可在 AC-PF 启动前条件式运行 5 次 compact DCOPF；
  DC 总预算覆盖 projection/formulation/symbolic/numeric，候选与 baseline 复用
  单个 Parity formulation：`case9241pegase` 的 Release/KLU 固定协议将端到端
  中位数降低 12.4%，Phase-II
  分解降低 27.6%，并保持最终 objective 与 primal/dual KKT；`case13659pegase`
  由 baseline-dual 预筛选零成本跳过。25,000-bus `ACTIVSg25k` 的同一
  Release/KLU 协议验证了可扩展性边界：普通 AC-PF 启动以 66 次迭代、65 次
  分解和 `183.808 s` 中位数稳定收敛，最终 primal/dual 为
  `3.24e-7/1.00e-7`；默认 2 秒 DC Phase I 因无法形成合格候选而诚实回退，
  中位数为 `186.588 s`（慢 1.51%），不能宣称该算例获得 Phase I 加速。
  零分解的分量 dispatch-dual predictor 也仅把初始 normalized dual
  `468.10733 -> 468.08762`（改善 0.0042%），被 5% 审计门槛拒绝；该研究路径
  默认关闭。现已加入显式 RAII `PreparedACOPFSession`：同一输入复用完整
  Parity formulation/maps，固定布局的参数变化只刷新 canonical numeric data，
  并以完整 KKT 坐标指纹审计后跨求解保留稀疏 symbolic ordering；只有布局签名完全一致时才整体复用
  `(x,lambda,mu,z)`，拓扑/服务状态/约束 family 变化会清空缓存。case118 Release
  固定协议中，普通 Phase I 三次中位数为 `16.407 ms`、27 次迭代、2 次 symbolic
  analyze；prepared 同实例为 `7.163 ms`、2 次迭代、0 次 analyze，五点
  `0/+0.5/-0.5/+2/-2%` 负荷序列中位数为 `8.434 ms`，末点 7 次迭代、0 次
  analyze。25,000-bus ACTIVSg25k 的两次同 session Release/KLU 验证为：首轮
  `184.645 s`、66 次迭代、65 次分解；第二轮 `8.666 s`、4 次迭代、3 次分解、
  0 次 symbolic analyze，墙钟下降 `95.31%`，最终 primal/dual 为
  `3.39e-8/2.09e-8`，峰值 RSS `4709.6 MiB`。当前后端仍逐次重算 numeric
  factor；请求 numeric reuse 时明确报告 unsupported，不虚报复用。AC-PF
  adjoint dual seed 仍不实施。
- 2026-07-28 增加 IEC-CGE 注释配电单线图 SVG 导入：从
  `cge:psr_ref` 与几何端点恢复导线、母线、开关、母联和配变拓扑，按
  `BestEffort` 口径补齐缺失电气参数，源外孤岛保持隔离；两个真实馈线
  SVG 已纳入结构与 AC 潮流回归。边界见 `docs/svg_distribution_import.md`。
- 配电 SVG 默认接入统一标准参数补齐：线路电阻表、设备适用范围和短路
  等值方法分别追溯至 GB/T 3956、GB/T 1179、GB/T 14049、GB/T 12706、
  GB/T 1094、GB/T 6451、GB/T 15544.1 与 DL/T 5220；电抗、载流量、
  电源强度和负荷系数仍明确标记为可配置工程假设。
- 配电 SVG 支持从核心 API、HTTP 会话和 GUI 反向导出；生成文件保留
  `cge:psr_ref` 稳定设备引用及 `hacdcpf:model` 电气参数扩展，可重新导入，
  对未覆盖的独立负荷和非 AC 资产显式返回遗漏计数与警告。
- 2026-07-26 完成 MIPSolvers 系统升级回归：Eigen 3.4.1（含
  `unsupported/`）、fmt、nlohmann/json、HiGHS、SCIP、MUMPS、Ipopt、
  SuiteSparse、Catch2、PaPILO 3.0.0 与所需 Boost 头均从本地源码解析；ETAP 所需 OpenXLSX 也已纳入本仓库，
  配置和构建过程不再下载依赖。
- 已删除被实现取代的阶段计划、一次性代码审查和重复暂态设计稿；不再用历史 roadmap 描述当前行为。
- 本次同步（2026-07-24）补入 2026 年 5–7 月新增能力域，并核实 BPA/DSP BD/LD/T 到原生 `LCCConverter` 的接口链：统一 Newton PF 消费 LCC 的 AC P/Q、DC 注入和交叉 Jacobian；平衡聚合 OPF 的 Parity/Ipopt 路径复用同一准稳态特性，不能保持物理闭合的后端显式拒绝。
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
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release
```

正式 Windows x64 分发包在完整构建后生成，并自动执行运行时依赖审计、
解压目录独立启动检查、逐文件清单和 ZIP SHA-256：

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_windows.ps1
```

产物位于 `dist/HySim-Windows-x64.zip`。解压后运行
`Start-HySim.cmd`；正式 Windows preset 固定使用 MIPSolvers 自带的
HiGHS/SCIP、Ipopt、sequential oneMKL/PardisoMKL 和 SuiteSparse，不链接
开发机上的 Gurobi DLL。

Trial Windows 包使用独立 preset，并在打包前强制运行 Trial 验收测试：

```powershell
cmake --preset windows-trial-release
cmake --build --preset windows-trial-release --target run_gui_server
ctest --preset windows-trial-release -L trial --output-on-failure
powershell -ExecutionPolicy Bypass -File tools/package_trial_windows.ps1
```

## 当前建模与仿真包状态快照（2026-08-23）

本节用于快速回答“现在这个包到底做到哪一步了”。结论基于当前仓库源码组织、CMake 选项与已注册测试目标，而不是历史规划文档。

### 1) 总体状态（按可用性分层）

| 能力域 | 当前状态 | 说明 |
|---|---|---|
| 混合 AC/DC 潮流与聚合建模 | 已实现并持续回归 | 覆盖 canonical projection、AC/DC 潮流、换流器协调与图分析链路；统一 Newton 默认采用固定 PV/PQ superset pattern、后向误差守卫的 KLU refactor 和 NCP/DC-seed/homotopy 升级梯级，`PreparedPowerFlowSession` 支持兼容快照重复求解。普通 `case2000_acdc` 的 Vdc-Q droop 由额定传输和统一 5% 电压带宽推导，并由关闭 fallback 的真实混合 NR 与浏览器默认按钮回归。平衡正序 PQ/Vdc-Q/GFM VSC 可选显式六变量限流 NCP 块，支持电流圆、Magnitude/P-first/Q-first、Vdc droop 饱和，以及 GFM 内部电势/角度和虚拟阻抗 Norton 端口；无 terminal SLACK 的 GFM 岛由固定内部相量锚定，保留全部终端电压未知量，支持多 Norton GFM、自适应分岛与暂态初始化。局部 Schur 与平滑 NCP 的准入/延拓策略由具名默认值统一管理，并可经 C++、HTTP 与 GUI 高级潮流覆盖；结果回显后端有效值和实际 Schur 证书，机器 epsilon 只读。该 GUI 数值策略不表示 NCP 已进入 OPF KKT，也不覆盖暂态设备参数。GPU 路径尚未实现。原生 LCC 的 AC P/Q、DC 注入及 `Vm/Vdc` 交叉 Jacobian 由统一 Newton 消费；FDPF 和独立 DC solver 不宣称等价覆盖。 |
| OPF 与约束优化 | 已实现并持续回归 | AC OPF / Hybrid AC/DC OPF 的 Parity Native IPM 前置有界稀疏 Phase I，认证 primal/dual warm start 后交给 Phase II；DC OPF / RPO（含 OLTC 离散档位控制）已集成。含在役 LCC 的平衡聚合 OPF 强制走共享 Parity/Ipopt NLP，控制指令与 tap 作为固定输入；不支持路径显式拒绝。 |
| 三相混合 PF / OPF | 活跃研发中，GUI 已接入 | `powerflow::solve_three_phase_hybrid_pf` 与 `opf::phase_hybrid` 已接入 `/xjtu/` 潮流/OPF 工具栏；OPF 提供 Full 与 GraphReduced（稀疏 Kron 降阶）、Ipopt/NativeIPM 双后端。GUI rich-model 适配范围见下文。 |
| 电压稳定 | 已实现 | 连续潮流（CPF）采用增广 `[state, lambda]` 弧长预测-校正，可越过 P-V 鼻点并保留下支采样；输出 P-V 曲线与 VSI 指标。 |
| 图建模、网络降阶、重构 | 已实现并持续回归 | 支持 domain-qualified 混合拓扑、开关收缩、Kron/series/pendant/sparse-Kron reduction、双向可复合映射与 ONR；Transformer3W/LCC/EnergyRouter 虚拟边仅表达连接性，无物理聚合规则的 phase/transformer/DCDC 收缩显式拒绝，HTTP Kron 保持 identify-only。AUD-024--AUD-031 已由 C++/validation/GUI E2E 与定向 sanitizer 回归关闭。 |
| 可靠性与弹性分析 | 已实现并持续回归 | 包含 MC、精确容量状态 COPT/F&D、FMEA、运行年/日历年故障率精确换算、物理成功路径容斥与稳定 ID 最小割集、三阶段可靠性。物理后果统一采用显式负荷削减：主网 HL-II 执行逐岛角度参考、零下界再调度、固定注入削减和两阶段字典序 DC-OPF；混合 AC/DC 执行同口径两阶段网络 LP；配网执行含有功/无功削减、AC/DC LinDistFlow、热限、辐射森林、Stage-2 运行拓扑重构及 Stage-3 拓扑保持的 MILP，纯无功负荷也具有显式削减可行点。无源/无 slack 状态由模型内生全切负荷，任何求解或证书失败立即报错，不伪造 EENS。保护不准入在安全拓扑上继续求解后果，并与 solver failure 分开标记。信息可靠性支持共享依赖路径、QoS、联合功能、最小割集、共因环境、通信设备供电与电池自治；在线保护链覆盖 CT/PT、主后备配合、重合/分段、DER-FRT 与 Mass-Matrix DAE。LCC、多端口路由器及显式三相保护动态不在当前物理后果口径内。 |
| 三相与短路分析 | 已实现并持续回归 | 三相 NR 与 AC/DC 短路分析均有独立测试族。详细短路模块覆盖 IEC 60909 三序网、$K_G/K_T/K_S$、两/三绕组 Y/YN/Z/ZN/$\Delta$ 零序、线路零序电纳、方法 A/B/C、公式 (77)、$I_{bmo}$、附录 A、导体温度、相/地电流、稀疏批量、authored 身份恢复，以及理想导体收缩和实际电阻边 DCCB 分流；57 个直接短路用例/917 断言、IEC 13 母线综合例、50 组 OpenDSS 完整网络、35 组 GridLAB-D 5.3.0 平衡探针和 IEEE 13/34/123 外部 Thevenin 核已验证。GridLAB-D 为短路交叉验证强制门禁；详细 IEC 60909 计算域无开放审计项，EMT、控制器、保护与 IEC 61660 属于各自模型族。 |
| 谐波分析 | 已实现并完成声明范围闭环 | 频域穿透、Newton 非线性、三相 abc/变压器零序与 AC/DC 耦合，频扫、IEEE 519 / GB/T 14549；14/14 跨域矩阵、24 个强制 GridLAB-D 切片、IEEE13/OpenDSS 164 点。 |
| 暂态动力学 | 机电暂态相量 DAE 声明范围已闭环（持续扩展设备） | 平衡正序/显式三相相量网络、AC/DC 代数网络、潮流一致初始化、7 类求解器（含 MassMatrixDae 同时式 DAE）、DAE 诊断、小信号与 COI 频率观测；DC/DC Power/Voltage/Droop 动态端口与稳态方程同源，DER_A 7/10 状态链已按 PSD/WECC 逐式实现并使用系统 COI 频率。MassMatrixDae 对 IEEE 1547 及直接 `DynamicSystem` API 装配的定时限欠压/频率/正序 Zone-1 继电器执行回滚/二分定位、固定前向窗口事件聚类、`ACLoadScale`/`ACBranchTrip` 重置和事件后代数残差审计；直接继电器使用本地正序 PT、CT 与相角频率滤波，不以系统 COI 代替本地频率。该闭环不包含 EMT、行波、开关波形或形式化 chronology certification；PSD Test 42 轨迹对照仍受外部 SciML 环境阻断。rich-model/JSON/HTTP/GUI 尚不自动装配外部继电器，其他积分器、反时限、多区距离及步内未采样脉冲仍为声明边界。 |
| 时序与年度生产模拟 | 核心已实现，年度/生命周期边界已深审计 | UC MILP → AC-OPF → PF 校验流水线；年度全耦合 UC、L0 能量/燃料代理预算、跨周 SOC、bottom-up feedback、物理生命周期 replay、抽样 PF correction、AC/DC/外部网碳分项均可执行。冻结 UC 已由 SciPy/HiGHS 和 256 序列穷举证明，6 步 AC 快照已与 OpenDSS/GridLAB-D 对照。制造/材料碳、网络损耗碳、燃料热率曲线和随机抽样 coverage 仍明确不属于当前范围；schedule-only 不能写成全年物理或生命周期外部认证。 |
| 电力市场 | 已实现（通用混合模型与南方规则研究执行入口） | 通用日前/实时/重复博弈保留；新增南方 98 点 SCUC → 调频容量改限 → SCED → AC 反馈 → 独立 LMP，以及 GUI 市场边界编辑、基准/情景差分。A1–A7 使用显式执行解释，未作正式市场等价认证，见 [执行契约](docs/modules/market/southern_execution_contract.md)。 |
| 园区综合能源 | 已实现 | 电-热-氢-燃料多能流 MILP 调度（CHP、热泵、电解/燃料电池、多层氢储能、CCUS、碳预算）。 |
| 承载力、薄弱环节与反事实规划 | 已实现 | DL/T 2041-2025 分布式电源承载力（含工程校核）、多维薄弱环节辨识、五类措施反事实对比。 |
| 场景生成与台风弹性 | 已实现并完成声明范围文档/数值闭环 | 16 专章/56 页手册覆盖三族条件概率、二元 AR(1)、风险边界锚点、Holland 风雨--易损--故障/修复--交通链与全字段契约；4096 步统计、128→12 聚类和固定公式由独立 Python 复算，hybrid 的尾部收益与运输/regime 退化同时披露。气象预报、易损现场校准及外部引擎等价认证不在声明范围。 |
| 碳流追踪 | 已实现并持续回归 | 比例/矩阵碳流追踪、年度碳核算（含储能碳库存动态）与用户/节点绿电证书（GEC）核算。 |
| EV-电力-交通耦合 | 已实现（持续扩展） | CTM/LTM 传播、Formulation A–H 联合优化家族、选址定容 MILP 与滚动时域 MPC；结果按“可证伪证书”口径区分全局最优/局部驻点/启发式。 |
| SPPT 可执行理论层 | 已实现（研究验证性质） | MR1–MR8 证伪套件、MR3 证书语料（CSV/LaTeX）、准入守卫与 agent 循环；当前缺少受版本控制的独立理论/运行契约，见 `docs/module_documentation_map.md`。 |
| Web GUI 服务 | 已集成可运行 | `run_gui_server` 为独立可执行服务，上述能力均经 HTTP API 暴露；前端为原生 JS 单页应用。 |
| Python SDK 与 AI 工具层 | v1 已实现 | `/api/v1` 独立会话、模型 revision/ETag、异步 PF/OPF 作业；Python 提供类型化客户端、结果诚实性检查、工具 Schema、影响分级与显式变更批准。 |

### 2) 依赖与功能开关状态（当前默认）

| 项 | 当前默认 | 影响 |
|---|---|---|
| 依赖模式 `HACDCPF_DEPENDENCY_PROFILE` | `portable` | 默认构建核心能力，避免强绑定开发型附加组件。 |
| ETAP Excel IO `HACDCPF_ENABLE_ETAP` | `ON` | 默认使用 `third_party/OpenXLSX-master`；不进行网络下载。源码副本缺失时配置失败，也可显式 `-DHACDCPF_ENABLE_ETAP=OFF` 关闭。 |
| OpenDSS bridge `HACDCPF_ENABLE_OPENDSS` | `OFF` | 需显式开启并提供 DSS C-API。 |
| OpenDSS compare `HACDCPF_ENABLE_OPENDSS_COMPARE` | `OFF` | 依赖 OpenDSS bridge。 |
| SuiteSparse `HACDCPF_USE_SUITESPARSE` | `ON` | 默认复用 MIPSolvers vendored UMFPACK/KLU；关闭时回退到 vendored Eigen SparseLU。MIPSolvers 提供 MUMPS（LDLᵀ）时其为默认后端（可用 `HACDCPF_OPF_LINEAR_SOLVER=mumps/umfpack/klu/eigen` 指定）；Parity IPM 默认采用增广 Newton 形式（`HACDCPF_OPF_KKT_FORM=condensed` 可切回）。 |
| IPOPT `HACDCPF_ENABLE_IPOPT` | macOS/Windows 默认及 preset `ON`，Linux preset `OFF` | Windows 使用 MIPSolvers 导出的本地 sequential oneMKL 静态 prebuilt 包与 PardisoMKL；包不完整时配置失败，不静默关闭。 |
| PaPILO `HACDCPF_USE_PAPILO` | `ON` | 默认使用 MIPSolvers 包内 PaPILO 与 Boost 头；关闭或 minimal profile 时回退到原生 presolve。 |
| Gurobi `HACDCPF_USE_GUROBI` | `ON` | 默认探测并优先使用本机 Gurobi；未安装、许可证不可用或求解失败时自动回退到包内 HiGHS/原生求解器。 |

### 3) 测试覆盖信号（如何判断“不是纸面功能”）

- 当前 `tests/CMakeLists.txt` 已注册 100 余个 C++ 测试目标（1300 余个 Catch2 用例），覆盖 IO、PF/OPF、图分析、重构、可靠性、弹性、短路、谐波、三相、暂态、EV-交通耦合、市场、综合能源、SPPT 与跨模块一致性；另有 Node/Playwright 浏览器 E2E 与 Python GUI HTTP E2E 注册进 CTest。
- 部分测试有运行时外部依赖门控（Julia、部分 GridLAB-D I/O 比较、OpenDSS、Playwright/chromium 等），依赖缺失时自动 skip；短路交叉验证例外，GridLAB-D 缺失、少于 35 个案例或误差超门均直接失败。
- 这表示“代码路径已工程化并具备回归入口”，但不等同于“你当前机器/当前配置已全部跑通”。
- 对外汇报建议使用两层口径：
  - 能力存在性：以源码与测试目标注册为准。
  - 可复现实测结论：以你本地 preset 构建与 ctest 结果为准。

### 4) 当前边界与建议口径

- 对可选 IO（ETAP/OpenDSS）和外部比较（GridLAB-D/OpenDSS/PSD.jl）应明确“需启用对应编译开关和运行时依赖”。
- 对暂态/谐波/跨引擎一致性类结论，建议标注“持续增强中”，避免描述为已完全定型。
- 电力市场已覆盖 DC 母线/支路/固定资源、VSC、DC/DC 双向传输、DC 储能跨期优化和 AC/DC LMP；N-1 覆盖发电机、AC/DC 支路、VSC、DC/DC 与两类 DC 储能，但只有 AC 支路 LODF 割进入定价 LP，其余采用固定组合纠正式 SCED 校核。DC 支路商业网损、换流器报价、母线/负荷/开关与保护故障仍未建模，外部电网和能量路由器仍显式拒绝。
- 省级市场规模尚无无条件在线时延承诺：SCUC 已注入机组时序/容量/备用/报价结构、经固定整数 LP 验证的 MIP start 和分支优先级；大型模型自适应使用 StrictHiGHS，并在当前分支树内尝试提交 AC 基态热限全局割，最终执行全候选复核，未完成时拒绝定价。不能通过 presolve 精确投影的热限进入外层轮次，并复用原空间根割、配套 root basis 和伪成本；新增热限后的旧开放节点树不直接沿用。默认显式 1% MIP gap 与 120 s 总时限，并返回实际 gap、树内提交、状态复用、树重建、候选/激活/剩余超限和证明口径。大型定价 LP 按变量阈值直达 HiGHS；LODF 使用稀疏因子复用与候选列按需计算，全元件事故 SCED 使用默认 4-worker 有界并行。异步取消、滚动时域、跨运行 artifact 缓存、可认证树 checkpoint 和注册规模基准仍是生产化缺口；大系统应限制 `n1_max_contingencies` 并分层运行。
- OPF 结果按实际路径声明有效边界：DC OPF 的凸二次成本在 LP 回退时使用 `pwl_segments` 分段，QP 路径回显有效分段为 0；节点 LMP 与支路拥塞 `mu` 分开认证，当前支路 `mu` 始终未认证。AC OPF 是非凸局部 KKT 求解，其 Ipopt 结果映射尚未发布乘子，因而无 LMP；RPO 是受时限/评估预算约束的离散邻域搜索，不提供全局 MINLP 证书。纯 AC、平衡 Hybrid AC/DC 与三相混合 NativeIPM 采用有迭代/分解/回溯预算的稀疏 Phase I；它只需进入与 `mu0` 协调的 primal/central corridor，不追求 Phase II 最终精度。完整 primal/equality-dual/positive inequality-dual/slack 状态通过 MIPSolvers 审计后才进入 Phase II，未接受状态严格退回普通冷启动；Phase I dual-fit 也不等同于完整 stationarity。三相混合 Ipopt 路径已支持原始变量、约束乘子和变量界乘子的完整热启动，由 Ipopt 自行重建内部 slack。默认 admission 会让初始违反度超过 `0.1` 的算例零分解跳过，因此该局部 Newton 恢复不会保证每个算例都发生，也不保证端到端加速。大型纯 AC 的可选 DC-dispatch Phase I 使用真实 Parity 初始残差比较候选与 baseline；它可能以更大的不等式 primal metric 换取更好的 dispatch dual，因此不冒充 central-state 证书，最终 KKT 仍完全由 Phase II 认证。快照 OPF 不含跨时段 SOC，能量路由器端口守恒不含内部损耗。AML builders 标记为实验链路，其中 AML SCUC 无网络约束且 MILP 价格未认证。`HACDCPF_OPF_*` 环境变量仅为调试通道，不是稳定 API。
- 三相混合 OPF 与 SPPT 层属活跃研发/论文验证性质，接口与产物格式仍可能调整。
- 当文档、报告、UI 文案与实现不一致时，以本仓库 `src/`、`include/`、`tests/` 与 CMake 配置为最终依据。

## 1. 工程场景

本项目是一个 C++20 静态库，核心目标是支撑混合 AC/DC 配电系统的稳态仿真、优化、可靠性和弹性分析。典型工程对象包括：

- 城市/园区配电网：AC 馈线、DC 母线、VSC 换流器、DC/DC 变换器、联络开关、断路器、分布式电源、储能和充电设施。
- 主动配电网：PV、风电等可再生电源，静态发电机、柔性负荷、可控负荷、移动储能、微电网和虚拟电厂。
- 故障恢复和运行优化：N-1 故障枚举、三阶段故障恢复、网络重构、弹性恢复、多时段生产模拟、OPF 和碳流追踪。
- 标准算例和工程导入：MATPOWER、JPC JSON、CIM/CGMES 3.0 与配电 CIM XML、IEC-CGE 注释配电 SVG、GridLAB-D GLM、PowerSimulationsDynamics.jl snapshot、Excel(ETAP)/OpenDSS 可选接口，以及项目内部 rich component schema。
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
| AC/DC 耦合 | `VSCConverter`, `LCCConverter`, `EnergyRouter` | VSC/LCC 换流器、能量路由器、多端口耦合 | VSC 保留为带控制角色的耦合元件；LCC 为本仓库 C++ 准稳态外特性模型（α/γ 角、换相电抗、内生无功），由统一 Newton PF 和 Parity/Ipopt OPF 消费；DSP 只用于 BD/LD 卡片语义与外部校准。EnergyRouter 展开为内部 DC 母线、VSC 和 DC/DC |
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
| AC/DC Power Flow | `solve_power_flow`, `solve_dc_power_flow`, `solve_power_flow_fdpf`, `solve_ac_dc_power_flow`, `solve_power_flow_adaptive`, `solve_power_flow_distributed_slack` | canonical AC/DC network + converter coupling | 统一 Newton 返回电压、相角、支路/VSC/LCC/DC-DC/ER 潮流、收敛状态、converter coordination 诊断和 `converter_model_scope`；其他求解器按各自范围解释 |
| 高级/回退潮流求解器 | `HelmSolver`、`HomotopyContinuationSolver`、`NewtonKrylovSolver`、`AdaptiveSolver`（`solver_factory.hpp`，`PowerFlowMethod`） | 全纯嵌入、同伦延拓、GMRES+Schur 预条件 | 难收敛算例的回退求解路径与诊断 |
| 三相潮流 | `analysis::solve_three_phase_nr` | `ThreePhaseACSystem` | abc 相电压、电流和三相收敛信息 |
| 三相混合 PF | `powerflow::solve_three_phase_hybrid_pf` | 原生相域 AC + DC 节点平衡 + equal-phase/GFL/GFM 变换器稳态闭合 | AC/DC 电压、逐相变换器功率/电流、VUF、分域物理残差；可从工程初值独立复核三相混合 OPF 点 |
| OPF | `solve_ac_opf`, `solve_dc_opf`, `solve_rpo` | AC/Hybrid AC/DC Parity IPM（含有界 Phase I）、DC LP/QP、无功优化模型（RPO 含 OLTC 离散档位邻域搜索） | Parity/Ipopt 对在役 LCC 返回稳定 ID 的 `lcc_transfers` 并闭合 AC/DC KCL；Native 路径另返回 Phase I primal/dual-fit/预算/接受诊断。不支持的线性路径拒绝。其余输出含调度、目标值、节点 LMP、约束诊断、solver path 与审计 |
| 三相混合 OPF | `opf::phase_hybrid::solve_three_phase_hybrid_opf`（Full / GraphReduced 变体，Ipopt / NativeIPM 后端） | 相域 AC + DC 混合 OPF，可选稀疏 Kron 降阶；NativeIPM 前置有硬工作预算的稀疏 Phase I；两后端均支持参数化原始--对偶热启动 | 三相调度、约束与 KKT 诊断、求解器乘子、Phase I primal/dual-fit/预算证书与同模型 PF 回放（活跃研发中） |
| 电压稳定 | `CpfSolver`、`compute_vsi`（`power_flow/voltage_stability.hpp`） | 连续潮流（CPF） | P-V 曲线、VSI 指标 |
| 网络重构 | `solve_optimal_reconfiguration`, `run_topology_reconfiguration` | LinDistFlow MILP + graph connectivity | 开/合支路集合、损耗 proxy、PF 校验 |
| 图分析/降阶 | `build_power_system_graph`, `contract_zero_impedance_edges`, Kron/series/pendant/sparse-Kron recovery | graph abstraction | 连通性、径向性、super-node、恢复映射 |
| 可靠性 MC/F&D | `run_nonsequential_mc`, `run_sequential_mc`, `run_frequency_duration_analysis`, `scan_ac_hlii_n2_states` | component outage sampling + HL-II DC OPF；在线机组从 0 再调度；两阶段字典序最小切负荷；独立两状态机组精确容量状态 COPT | EENS、LOLE、LOLF、CoV、严格经验 VaR/CVaR；COPT 状态概率/频率与缺失 MTTR 诊断；N-0/N-1/N-2 确定性零失败证书。状态求解失败直接报错，不计入 EENS |
| FMEA 与信息/保护可靠性 | `run_distribution_fmea`, `run_failure_mode_fmea`, `evaluate_physical_network_reliability`, `evaluate_joint_information_reliability`, `generate_protection_cyber_classes`, `compare_protection_cyber_reliability` | 全 rich-model 失效模式目录 + N-1/N-2 二阶交互；运行年/日历年频率换算；物理/信息成功路径容斥、QoS、共因与供电依赖的精确状态；给定轨迹的 L2 保护/FRT 联合事件树 | EENS/EDNS/SAIFI/SAIDI、二阶完整性、稳定 ID 最小割集、静态 FMEA/仅保护/联合 EENS/LOLE/LOLF 对比与有效性边界 |
| 三阶段可靠性 | `run_three_stage_reliability` | native C++ 联合 AC/DC LinDistFlow MILP；交流支路视在功率内接多边形；VSC/DC-DC 双向效率，AC/DC/DER/移动储能/VPP 调度与跨阶段 SOC；会话保护配置按自动重合、主保护、后备保护和未清除事件调节频率、清除时间、停运区与恢复准入 | 三阶段失负荷、热限边数与最大视在功率比、保护场景审计、有符号 VSC 调度、节点可靠性指标与有效性标志；该路由是概率条件化恢复 MILP，继电整定与 DER-FRT 动态由独立的在线保护—信息物理三级对照入口执行 |
| 配电弹性 | `run_distribution_resilience_assessment`, `run_distribution_resilience_mip_assessment`, `run_distribution_resilience_stage_milp_assessment`, `run_certified_distribution_resilience_mip` | heuristic sequential、multi-period hybrid AC/DC MIP 或 RA 分阶段拓扑/MESS；恢复状态与母线级调度送入多保真 DAE oracle，proof-valid Unsafe 自动形成 topology/service 割并重求解，异常 fail-closed | 恢复曲线、域限定元件/MESS 状态、故障序列、弹性指标、逐转换证书与已施加反馈；复杂案例 2 次 MIP/1 条割闭环，2048 对风险样本满足预声明准则，三冻结 AC 快照通过 OpenDSS/GridLAB-D 门。外部对照不覆盖 DC/保护/控制/DAE |
| 短路分析 | `compute_short_circuit`, `run_short_circuit_detailed`, `dc_bus_fault_level`, `dc_bus_fault_levels` | 稀疏 Z-bus IEC 60909 概览 / 详细序网（c 因子、κ/ip/ib/ik/ith、相电流、变压器修正、电机与换流器贡献）；DC 为电阻性刚性源准稳态批量模型，收缩理想导体并由故障后电压恢复 DCCB 实际边电流 | authored 母线/设备故障电流、IEC 指标、数值质量、DC 故障水平与准稳态开断 duty；EMT 与控制动态显式列为限制 |
| 谐波潮流 | `solve_harmonic_power_flow`（及 `_newton` / `_3ph` / `_3ph_hybrid` / `_hybrid_newton` 变体）, `frequency_scan`, `check_harmonic_limits` | 频域穿透（NIC 双端口桥）、Newton 非线性、三相 abc、AC/DC 耦合 | 谐波电压/电流、频扫/谐振、IEEE 519 / GB/T 14549 合规、K 因子/TDD |
| 暂态仿真 | `run_transient_simulation`, `small_signal_analysis`, `computeFrequencyReport` | 机电暂态相量 DAE（7 类求解器，含 MassMatrixDae 同时式）；AC/DC 一致初始化、共享 DC/DC 端口方程、DER_A COI 频率控制；IEEE 1547 及直接 API 装配的本地 CT/PT 定时限 UVLS/频率/正序 Zone-1 继电器可定位、前向窗口聚类并一致重启 | 轨迹、稳定 ID 事件记录、COI/孤岛遥测频率、小信号摘要、初始化残差所有权、事件定位次数/括号/簇跨度与事件后代数残差；COI 不作为直接继电器输入，EMT 测量拒绝，外部继电器尚无 rich-model/HTTP 自动装配 |
| 碳分析 | `run_carbon_analysis`, `compute_annual_carbon_analysis`, `compute_annual_user_gec_accounting` | PF result + proportional / matrix tracing；年度时序含储能碳库存 | 节点、支路、负荷碳流；年度碳与用户/节点 GEC 核算 |
| 时序/生产模拟 | `solve_time_series_pf`, `solve_unit_commitment`, `solve_annual_production_simulation`, `run_lifecycle_simulation`, `run_lifecycle_comparison` | 多时段负荷/资源曲线 + OPF/UC | 年度生产、成本、生命周期指标、容量扫描对比 |
| 电力市场 | `market::run_day_ahead_market`, `run_real_time_market`, `run_repeated_market_game`, `run_southern_day_ahead_market` | 通用混合市场与独立南方规则研究执行模型；后者使用版本化边界快照、三阶段模型和 AC 安全反馈 | 通用结算/市场力；南方边界校正、机组/储能/水库/交易曲线、独立 LMP、约束残差及情景差分 |
| 园区综合能源 | `integrated_energy::solve_campus_ies` | 电-热-氢-燃料多能流 MILP（CHP、热泵、电解/燃料电池、氢储能、CCUS） | 多能流调度、成本/碳目标 |
| 承载力评估 | `assess_hosting_capacity`（DL/T 2041-2025） | 设备级区间公式 + 可选 PF/短路/谐波工程校核 | 逐变压器/逐区域承载区间与分级 |
| 薄弱环节辨识 | `run_multidimensional_weak_link_assessment` | 多维压力证据评分（severity / consensus / Pareto） | 薄弱环节排序与模式对比 |
| 反事实规划 | `run_counterfactual_planning_assessment` | 扩容/储能/联络/自动化/DER 五类措施多维对比 | 反事实指标与两两协同分析 |
| 场景生成 | `generate_scenarios`, `generate_typhoon_fault_sequence`, `enumerate_n1_contingencies` | 常规/可靠性/弹性三族场景 + 二元相关扰动 + Holland 风场/交通耦合 + k-medoids/边界锚点 | 场景目录、台风故障/修复序列、交通影响与验证审计 |
| EV-交通耦合 | `simulate_ev_power_traffic`（A）、`_ctm_due`（B+）、`_ctm_joint`（C）、`solve_joint_optimizer`（D）、`solve_ctm_so_lp`/`solve_ltm_so_lp`（E）、`solve_ctm_due_vi`（F）、`solve_infra_design_milp`（G）、`solve_ltm_mpc`（H） | CTM/LTM 交通传播 + DC-OPF/LMP 联合优化 | 耦合仿真结果与最优性证书（区分全局/局部/启发式） |
| SPPT 验证层 | `sppt::run_core_metamorphic_suite`, `certify_corpus`, `guard_system`, `run_agent_loop` | MR1–MR8 蜕变关系、独立残差证书、三道准入守卫 | 证伪/认证产物（CSV/LaTeX，研究验证性质） |

优化和 MILP 模块依赖 sibling directory `../MIPSolvers`。该源码树自带完整
Eigen 3.4.1（包括 `unsupported/Eigen/MatrixFunctions`）、fmt、nlohmann/json、
HiGHS、SCIP、MUMPS、Ipopt、SuiteSparse、PaPILO 与所需 Boost 头；项目默认从本地源码解析这些依赖，
无需系统 Eigen 或网络包管理器。封闭环境须同时交付两个源码目录，也可通过
`MIPSOLVERS_SOURCE_DIR` 指向包内其他位置。

Windows 的嵌入式 Ipopt 使用 MIPSolvers 中预先 staging 的 sequential 静态 oneMKL
依赖包。联网准备机运行 `third_party/stage_onemkl.ps1` 和
`third_party/build_third_party.ps1` 后，封闭环境只消费带 manifest、SHA-256 与许可
材料的 `MIPSolvers/third_party/install`，不依赖机器级 oneAPI 安装。

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

平衡 Newton 潮流将 PV/PQ 转换作为有界活动集过程：在固定活动集收敛后批量钳位全部 Q 越限 PV 节点，只执行一次带保持时间和电压方向死区的 PQ→PV 恢复审计，并返回 `reactive_limits.certified`、重复活动集和外层预算诊断。GUI 默认关闭该校核以进行快速筛查；需要工程 Q 限值结论时必须显式启用并检查证书。理论与性能边界见 [PV/PQ 无功限值切换契约](docs/pv_pq_switching_contract.md)。

换流器调制比、DC 电流和 DC/DC 占空比默认仍采用收敛后物理审计；启用 `PowerFlowOptions::enforce_converter_physical_limits` 后，超限根会被硬性拒绝。平衡正序 `PQ_MODE` / `VDC_Q` / `AC_GRID_FORMING` VSC 可通过设备级 `enable_limit_ncp` 与正 `i_ac_max_pu` 启用固定六变量局部块，在统一 Newton 内求解电流圆、P/Q priority、Vdc droop 饱和及 GFM 内部电势—虚拟阻抗 Norton 端口，而非事后改写结果。目标方程使用精确 Fischer–Burmeister/中值 NCP；困难退化工况可选择固定结构的光滑 FB/CHKS `mu` 延拓，最终结果重新按精确方程认证。无 terminal SLACK 的 AC 岛保留全部母线 `Vm/θ` 与 P/Q 平衡，由 authored GFM 内部相量消除全局旋转零模；多个 Norton GFM 可共存，但严格协调要求 DC 侧有物理电压/功率支撑。该半光滑 Newton 是局部方法，远初值可能进入低压数学根。平衡 OPF 本体不含 GFM priority-NCP KKT 约束，最终由保留控制模式的 `post_pf` 回放认证。数学、接口和验证门槛见 [VSC 电流限值 NCP 潮流契约](docs/vsc_limit_ncp_power_flow_contract.md)。

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
  -> replay represented topology/MESS transitions on the same canonical model
  -> L3 scenario-specific DAE threshold certificate
  -> unresolved/failed when an action lacks an exact dynamic event or required evidence
```

### 8.3 可靠性流程

```text
HybridPowerSystem + reliability data
  -> stable-ID catalog + user mode/protection overlay validation
  -> resolve user override > case data > built-in template > policy fallback
  -> apply authored / forced-GFL / eligible-GFM control scenario to a private copy
  -> enumerate or sample component outages
  -> stage evaluation using OPF / reconfiguration / restoration model
  -> accumulate frequency-weighted EENS, LOLE, SAIFI, SAIDI
  -> compare same-basis methods by metric spread and stable-ID rank agreement
  -> expose model_limitations when physics coverage is approximate
```

### 8.4 I/O 流程

```text
MATPOWER / JPC JSON / CIM（CGMES 3.0 与配电 CIM）/ IEC-CGE 配电 SVG / GridLAB-D / PSD.jl / Excel(ETAP) / OpenDSS
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

GUI 潮流入口 `POST /api/session/pf` 新增 `method=three_phase_hybrid`，在同一 Newton 系统中联立 abc 相域 AC、DC 节点平衡与 VSC 稳态方程。GUI OPF 入口 `POST /api/session/opf` 新增 `network_model=three_phase_hybrid`，可选择 Full / GraphReduced 与 NativeIPM / Ipopt，并返回逐相电压、DC 电压、逐相电源/VSC 调度及同模型 `post_pf` 回放。该 GUI 适配器当前精确覆盖恒功率星形相负荷、纯电阻 DC 支路与直接 VSC；LCC、Delta/ZIP、DC/DC、能量路由器和可调 DC 静态电源不在其模型范围内，由后端显式拒绝。AC 支路热限与 VSC 调制比尚未进入相域混合 OPF，响应通过 `scope` / `model_limitations` 如实标注。

平衡聚合 OPF 支持 parity/native/dc 求解路径，并把实际约束范围以 `scope.model_scope` 与布尔 flags 返回；含在役 LCC 时只允许共享 Parity/Ipopt 模型，响应包含 `lcc_transfers` 与 `scope.lcc_quasi_steady`。AC/parity OPF 收敛后，后端会在 OPF 调度点再跑一次 PF，并返回 `post_pf` 支路潮流/VSC/LCC 转移以及 best-effort `post_carbon` 碳流结果；前端将 `post_pf.branch_flows` 用于 OPF 解上的潮流/负载率热力图叠加。

GUI 第一阶段统一契约包括：`hysim_task_status_v1`（任务状态、耗时与模型版本）、`hysim_result_v1`（分析、请求 ID、结果状态与陈旧性）和 `hysim_canvas_ref_v1`（结果行的元件类型、模型索引与 Canvas ID）。PF/OPF 在计算期间检测到模型版本变化时会丢弃过期结果；结果表统一通过 `data-result-ref` / `data-comp-id` 定位 Canvas，并支持鼠标和键盘操作。

第二阶段任务执行层为所有主要分析请求分配 `X-HySim-Request-ID`，活动任务期间锁定运行按钮并提供“取消等待”。取消会中止浏览器请求、忽略该请求的后续结果，并轮询后端直到求解收尾；由于当前 C++ 求解器没有统一的取消令牌，这不是强制终止求解线程。模型版本在请求期间变化时，响应统一标记为 `stale` 且禁止进入 Dashboard 或 Canvas。

第三阶段将共享前端基础设施从 `web/js/app.js` 拆分到 `web/js/core/`：`analysis_contracts.js` 维护分析端点和结果契约，`task_manager.js` 管理单活动任务、取消与后端收尾，`api_client.js` 统一请求 ID、错误和陈旧结果处理，`result_mapping.js` 维护 Dashboard 到 Canvas 的类型/索引映射。此后 `web/js/core/` 又扩展了 `timeseries_window.js`（长时序窗口化与保峰降采样）、`layout_graph.js` / `layout_engine.js`（布局语义投影与 ELK 异步布局客户端）、`accessibility.js`（键盘导航与可访问性审计）、`runtime_diagnostics.js`（前端运行时诊断）和 `network_overview.js`（WebGL2 全网 LOD 总览）。`app.js` 只保留 UI 状态回调和薄适配层，这些核心脚本必须在 `app.js` 之前加载。

第四阶段针对中大型系统优化交互性能：普通规模仍使用 SVG 单线图编辑，并将连续鼠标移动合并到浏览器动画帧、对视口外元件执行可逆裁剪；超过规模阈值时不再显示空白“无画布”摘要，而由 WebGL2 点/线缓冲绘制 LOD0 域、LOD1 区域和 LOD2 母线全网总览，同时保持 SVG glyph 数为零。WebGL、局部 k 跳 SVG、虚拟拓扑表和结果导航统一使用 `{domain,index}` 母线引用。5,000 节点以上的无画布潮流请求自动采用 `response_detail=compact`：保留完整 `vm`/`va`/`vdc`/`branch_abs` 数值向量，省略逐元件 GIS 与 rich attribution 展示行，并通过 `model_scope` / `model_limitations` 声明边界；小系统仍返回完整逐元件结果。年度生产模拟按 7/30/90 天窗口浏览；选择全年时采用保留首尾及负荷极值的降采样，最多绘制 2000 点，同时保留完整原始序列供导出和后续分析使用。规模化 GUI E2E 对 WebGL 非空像素、选择同步、局部 SVG 和这些性能边界提供回归契约。

`/api/v1` 为大模型客户端提供后端分块：`sessions/{id}/topology` 支持 LOD、空间视口和分页，`sessions/{id}/subgraph` 按稳定母线引用提取有界邻域，`jobs/{id}/frames/{step}` 按时间/域/稳定索引/空间返回结果窗口，`jobs/{id}/violations` 返回最严重电压与负载率越限。静态 PF/OPF 使用第 0 帧，后续生产模拟沿同一帧协议扩展多时步。

Python v1 SDK 对上述接口提供 `TopologyChunk`、`SubgraphView`、`ResultFrameChunk` 和 `ViolationChunk` 类型校验，支持自动分页、`BusRef` 稳定引用、指数退避作业等待与请求哈希审计。AI 工具层只暴露有明确行数上限的拓扑、子图、帧、越限和作业摘要，避免把完整大模型意外装入上下文。

第五阶段补齐工程界面的可访问性：工作流、模块和页签采用 roving-tabindex 键盘导航，支持方向键、Home/End，提供主工作区跳转、清晰焦点环、对话框语义、控制台播报、减少动画和高对比度偏好。`hysim_accessibility_audit_v1` 会检查关键地标、导航和控件名称。

第六阶段增加本地运行时防护：`hysim_runtime_diagnostics_v1` 捕获全局脚本错误、未处理 Promise 拒绝、HTTP/网络失败及在线状态，在依赖栏显示“前端”健康芯片。诊断仅在内存中保留最近 25 条截断记录，不上传、不持久化，并可通过 `App.getRuntimeDiagnostics()` 检查或清空。

第七阶段将 GUI 调整为画布优先的工程工作区：提供标准/紧凑密度，元件库、上下文功能区、右侧检查器和控制台可独立显隐；专注模式保留当前模块、页签和结果，仅隐藏辅助区域。布局写入浏览器本地状态并在刷新后恢复，移动端首次进入默认收起元件库、上下文功能区和控制台，模型参数与可靠性结果仍在右侧检查器中完整呈现。

模型参数页由后端 `model_catalog` 提供 44 类可序列化实体/系统模型，不再由标准规则数量反推元件列表。当前实例字段来自权威系统 JSON 序列化并按工程语义分组；可靠性失效模式归入对应实体的“可靠性”属性，不再作为 `Reliability - ...` 伪元件单列。未登记的典型范围或硬边界明确显示为未发布/未注册，避免前端虚构校核依据。

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
| 谐波潮流与 HSS | 逐阶/三相/非线性 HPF；一等 DC 电容、电抗器和 AC/DC 滤波器；两电平 VSC、MMC、LCC、DC/DC 开关函数频率耦合；`include/hacdcpf/analysis/harmonics_power_flow.hpp`, `src/harmonics_power_flow/` |
| 短路分析 | `include/hacdcpf/analysis/short_circuit.hpp`, `include/hacdcpf/analysis/dc_short_circuit.hpp`, `src/short_circuit/` |
| 时序/年度/生命周期 | `include/hacdcpf/time_series/`, `src/time_series/` |
| 碳流/年度碳 | `include/hacdcpf/carbon_analysis/`, `src/carbon_analysis/` |
| EV-交通耦合 | `include/hacdcpf/ev_power_traffic/`, `src/ev_power_traffic/` |
| 电力市场 | `include/hacdcpf/market/market_simulation.hpp`, `src/market/` |
| 园区综合能源 | `include/hacdcpf/integrated_energy/`, `src/integrated_energy/` |
| 承载力/薄弱环节/反事实 | `include/hacdcpf/analysis/hosting_capacity.hpp` 等, `src/analysis/` |
| 场景生成/台风 | `include/hacdcpf/analysis/scenario_generation.hpp`, `include/hacdcpf/analysis/typhoon_resilience.hpp`, `src/scenario_generation/` |
| SPPT 验证层 | `include/hacdcpf/sppt/`, `src/sppt/`（文档覆盖：`docs/module_documentation_map.md`） |
| 网络重构 | `include/hacdcpf/network_reconfiguration/`, `src/network_reconfiguration/` |
| 可靠性 | `include/hacdcpf/reliability/`, `include/hacdcpf/analysis/three_stage_reliability.hpp`, `src/reliability/` |
| 弹性恢复 | `include/hacdcpf/resilience/resilience_assessment.hpp`, `src/resilience/` |
| I/O | `include/hacdcpf/io/`, `src/io/`（JSON、MATPOWER、CIM、IEC-CGE 配电 SVG、GridLAB-D、PSD.jl；ETAP/OpenDSS 可选） |
| 并行工具 | `include/hacdcpf/util/thread_pool.hpp`（`ThreadPool`、`parallel_for`） |
| GUI 后端服务 | `tests/run_gui_server.cpp`（独立可执行，旧 GUI API + 静态挂载 `web/`） |
| v1 运行时 API | `src/server/runtime_api_v1.hpp`, `src/server/runtime_api_v1.cpp`（多会话、ETag、异步作业） |
| Python SDK / AI 工具层 | `python/src/hysim/`（类型化客户端、可替换传输、结果口径、工具策略与本地服务生命周期） |
| Diagnostics/benchmarks | `tools/`（`opendss_pf_compare`、`etap_convert`、`matpower_pf_compare`、`sppt_certify`/`sppt_ablation`/`sppt_benchmark`/`sppt_agent_demo`、`hybrid_acdc_pf_study`、`phase_graph_reduction_benchmark`、`phase_hybrid_opf_benchmark`、gridlabd/transient/short-circuit validation matrices 等） |
| 文档索引 | `docs/README.md` |
| 历史文档归档 | `docs/archive/`（仅用于追溯，不代表当前行为） |

## 10. 维护原则

- Rich component 层尽量保留工程语义，不把设备信息过早丢弃。
- Canonical model 层只保留求解必要变量，并通过 mapping 保留来源。
- Graph 层所有 AC/DC lookup 应使用 domain-qualified map。
- 对外报告使用稳定 component ID 或结构化引用，例如 `BranchRef`，不要暴露临时 vector position。
- 任何 fallback、近似模型、time-limit 解和模型覆盖不足都应进入 result status / diagnostics。
- 修改 projection 或 graph 代码后，应优先补充 round-trip、result recovery、AC/DC 同号 bus ID、dead-island、zero-impedance merge 等测试。
