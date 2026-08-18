# 数学模型文档 ↔ 代码一致性审计问题卡

> 本文件是受控改进项清单：每条问题卡含文档位置、文档陈述、代码证据与修正方向。
> 修复一条删一条；全部清零后本文件可归档。审计方法：19 个模块单元逐一核对公式/默认值/阈值/分支/引用行号，
> 仅当有双侧证据（文档位置 + 代码位置）时立案。UNCERTAIN 项（证据不足的疑点）记录于各模块审计底稿，未列入本文件。

## 一、实质性不一致（数学模型/行为/默认值）——已全部修复并销卡

首批 33 张卡（PF-1..11、OPF-1..2、PM-1、REL-1、MKT-1、SC-1..3、IE-1..2、CA-1、AN-1..2、EV-1、
TS-1..3、SG-1..3、TH-1..2）已于 2026-08-18 全部修复：每卡修复前重新核实代码证据，无一 INVALID。
修复要点：power_flow 手册撤销 LCC 移除声明并补写 LCC 小节（`\label{sec:hybrid-lcc}`）、ZIP 导数与
DC/DC droop 符号改正、线搜索按 Armijo-GLL 重写、`enable_auto_fallback_scheduling` 默认改 true、
DC 求解器改 SparseLU 口径、零阻抗/tap=0 改 fail-close 抛异常、CPF 补 PV 母线回退分支与 nose_found
真实置位条件、分布式松弛上限按现公式；OPF 三个选项表补齐字段（保住"全部字段"声明）且 DC 参考角
改逐连通分量锚定；market 手册摘要/§1.3/§4.2 按"线性化全混合出清 + 运行时 scope 改写"重写；
短路峰值电流改逐贡献求和 + κ 分支、SCTopology 收窄为仅影响 κ、换流器零贡献阈值改 1e-6；
integrated_energy PCC 符号改正并列举实际校验的 18 个效率字段；carbon BFS 损耗归送端；
analysis 参数表补 `default_dr_max_output_coeff` 并修正符号/范围列；ev `allow_v2g` 按结构分行；
time_series EOL 百分数归一化、SCIP 可按日并行、E' 阈值区分 1e-9/1e-12；scenario_generation
`min_fault_probability` 改"保留字段未消费"、`holland_b` 改实际钳位、聚类改"默认可覆盖"；
theory md 的 VSC 无序 droop 对与 κ 上限两处改正且 .zh.md 译文同步。

### 修复过程中新发现的候选卡（待立项核实）

- PF-N1：`newton_solver.cpp:2327-2328` 注释仍提已删除的"逃逸步"（代码注释清理，改代码）。
- PF-N2：`cpf.tex`、`hybrid_acdc.tex`、`newton.tex` 部分非卡内行号可能仍有漂移，未逐一重核。
- OPF-N1：`parity_ipm.tex:618-624,640-643` 既有表行行号整体约 +18 漂移；`dc_opf.tex:176` 等。
- OPF-N2：`dc_opf.cpp:518` 注释 "skip j==slack" 与实际循环（不再跳过）矛盾（代码注释）。
- MKT-N1：`docs/theory/market_simulation_mathematical_models.md`（含 .zh.md）若仍写 AC-only 口径需立项核查。
- IE-N1：`docs/theory/integrated_energy_contract.md:27-28`（含 .zh.md）的效率校验措辞对 `eta_wasteheat` 有歧义。
- EV-N1：`num_steps` 默认值两结构不同（24 vs 6），参数表默认列为"—"不构成错误但可补充分行。
- SG-N1：`source_equivalent_model.tex` 未转写 `holland_b(lat,rmw)` 生成式与 `rmw_from_delta_p`（`typhoon_resilience.cpp:314-320`）。
- CA-N1：`carbon_analysis` 章节 `edge_loss_carbon_intensity` 引用漂移 2 行（130-138）。
- SC-N1：方法 B+Meshed 的 `min(1.8,1.15κ)` 实际只作用非故障母线单 κ 近似；若 IEC 本意应作用故障点，属代码口径问题（CODE-FIX 候选）。
- REL-N1：无。PATH 类新发现已并入下方 PATH-1。

### 锚点校对批次新发现的散文级候选卡（2026-08-18，未修）

- PF-N3：`newton.tex` "迭代内仅在残差 <1e-3 时才允许 PV↔PQ 切换"已过时——当前无外环内切换，`check_q_limits_and_switch` 仅在外环结束后调用，守卫为残差 ≤0.1（`newton_solver.cpp:2484-2518`）。
- PF-N4：`robust_solvers.tex` 称 `PtcSerController` 未被主循环调用——已接线（`newton_solver.cpp:1858`）；仅 `LMController` 闲置。另 `ptc_delta0`/`ptc_growth` 在 src/ 无读取点，已成死字段（候选代码清理）。
- PF-N5：`classic_solvers.tex` 称自适应孤岛 `q_limits` 未接线——已接线（`adaptive_solver.cpp:316`）；同章"纯 DC 岛被丢弃"与 `detect_islands` 行为不符；分布式松弛失配公式口径应为全部 AC 母线求和（`distributed_slack_solver.cpp:210-214` 注释解释了原因）。
- PF-N6：`distribution.tex` 诚实口径与代码相反——非连通/非辐射拓扑现显式 fail-close（`distribution_power_flow.cpp:510-520`）；ZIP 负荷已是电压依赖（`run_bfs_sweep` 经 `voltage_dependent_net_demand` 每迭代更新）。
- PF-N7：`cpf.tex` 测试覆盖声明过时（现 5 个用例已含 PV 回退专项）；`overview.tex:133` "engine 规范头"表述与同文件自述矛盾（转发头 vs `engine/solver/native/nle/newton_solver.hpp`）；`hybrid_acdc.tex:32` 引用的 dc.hpp "linear approximation" 注释已改为 nonlinear。
- OPF-N3：`ac_opf_native.tex:207` 散文裸函数名 `clamp_in_interior` 应为 `clamp_interior`；`external_data/opf_hand_cases/` 目录整体不存在，`verification.tex` 手算基准整节失去目标（需确认是否有底稿存档）。
- OPF-N4：`overview.tex` L609-611 "非 Apple 平台开启 HACDCPF_ENABLE_IPOPT 直接 FATAL_ERROR" 与当前 CMakeLists 脱节。
- OPF-N5：`parity_formulation.cpp:36-38` 悬置软锚注释与 `equality_jacobian`:2329 残留注释有误导性（代码注释清理）。
- OPF-N6：`dc_opf.tex` "死岛剪枝"测试名与实际测试语义相反（测试验证的是有源孤岛保留）。
- MISC-2：各手册/测试文件仍残留散文中的零散纯行号引用（未走锚点格式），量大、语义上下文弱，低优先级。

## 二、机械性漂移（不影响语义，批量修）

- PATH-1【跨文档路径失效，docs 重组（5842164e）后未回改】——**已批量修复（2026-08-18）**：全量扫描 docs/ 下 tex/md 的 `docs/...` 引用，101 处唯一匹配的死链已按真实位置改写（tex 转义风格保留；规律主要为 `docs/X.md` → `docs/theory/X.md`，另有 developer/reference/guides/archive 归位），涉及 31 个文件。两个文件确认已从仓库删除且无替代（`docs/lcc_dat_opf_report/LCC_DAT_OPF_technical_report.tex` 10 处、`docs/numerical_methods.md` 1 处），引用处已就地标注"该文档已不在仓库中"——若这些验证报告另有存档，应恢复或改写对应段落。有意保留未改：`documentation_reorganization_report.md` 中的旧路径属历史叙述。
- LINE-1【行号大面积漂移】——**已根治（2026-08-18）**：不再逐行重刷数字，而是全量迁移为"文件:函数名"符号锚点。迁移脚本 `tools/doc_anchor_symbolize.py`（可重跑，dry-run 默认）把 65 个文档文件中 2102 处行号锚点按当前代码解析为所属函数/结构体/lambda 锚点（含同文件/bare 简写与已改名文件 opf.cpp→three_phase_hybrid_opf.cpp 等的重定向）；46 处指向头文件注释区/转发头的锚点保留纯文件名。随后 8 路语义校对修正了约 600 处因行号漂移导致的错误符号（典型：ac_opf.cpp 大重构后旧行号落进 solve_with_parity_ipm 的 lambda 里）。规范条文已同步：docs/modules/README.md 数学模型规范第 2 条改为"必须给出实现函数及文件:函数名锚点，禁止行号"，各手册前言的"文件:行号"体例声明同步更新。仍残留：各章散文中未走锚点格式的零散纯行号引用（如 `:852--2140`），数量大且语义上下文弱，留作后续低优先级清理。
- MISC-1：`harmonics_power_flow/source_equivalent_model.tex:70` LaTeX 排版缺陷 `,qquad` 漏反斜杠；`power_models` 顺带发现 `scuc_builder.hpp:51` 头文件注释与实现语义有张力（缺省 1e6 名义宽松 vs 注释"不施加"）；`graph_manual.tex:204` "改进方向"中"悬垂折叠迭代恢复"已实现（`result_recovery.cpp:276-301`），仅剩"报告收敛标志"未做。

## 三、审计覆盖说明

- 19 个审计单元（18 个模块手册 + theory/md 文档组）全部完成；公式核对比例各单元报告为 8/8（ev）、20/20（time_series）、全部（market/power_models/graph）等，dynamics/short_circuit 等大手册按数学密集章节优先。
- 未立案的"代码有而文档没有"项（按现行规范不构成不一致，但若规范要求全量转写则需补）：`analysis` 的 decision_hint 决策路由（`multidimensional_weak_link.cpp:68-110`）、短路稳态 I_k 的 λ_max 机制、谐波 THD_I/TDD 零守卫。
