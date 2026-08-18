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

### 候选卡处置记录（2026-08-18 第二批，已清零）

- PF-N1 ✅ FIXED（逃逸步残留注释改写为"正则化→LM→伪瞬态"恢复链）；PF-N2 ✅ 已随符号锚点迁移消解。
- OPF-N1 ✅ FIXED（部分随迁移消解；parity_ipm.tex 逗号分隔表行 11 行手工改符号锚点——并发现迁移工具 BARE 正则不覆盖 `（:NN，…）` 逗号列表形式，工具缺陷待修）；OPF-N2 ✅ FIXED（dc_opf.cpp:518 注释改"slack 列保留但数值惰性"）。
- MKT-N1 ✅ INVALID（market_simulation_mathematical_models.md 为中文主稿且已正确描述全混合口径）。
- IE-N1 ✅ FIXED（契约 md + .zh.md 补 eta_wasteheat/eta_carbon_to_fuel 校验排除说明）；EV-N1 ✅ FIXED（num_steps 按结构分行 24/6）；SG-N1 ✅ FIXED（补录 holland_b 生成式与 rmw_from_delta_p 小节，xelatex 编译通过）。
- CA-N1 ✅ 已随符号锚点迁移消解；SC-N1 ✅ FIXED（2026-08-18，见第三批记录）。
- PF-N3 ✅ FIXED（PV↔PQ 仅外环发起 + 残差≤0.1 守卫）；PF-N4 ✅ FIXED（PtcSerController 已接线、SER 公式按代码转写；死字段 ptc_delta0/ptc_growth 已加"保留兼容"注释，未删字段）。
- PF-N5 ✅ FIXED（q_limits 已接线、纯 DC 岛不丢弃、分布式松弛全母线求和口径）；PF-N6 ✅ FIXED（fail-close 口径、ZIP 电压依赖）；PF-N7 ✅ FIXED（CPF 5 用例、转发头表述、dc.hpp nonlinear）。
- OPF-N3 ✅ FIXED（clamp_interior 裸名改正；opf_hand_cases 四处引用就地标注"不在仓库检出中"，verification.tex 手算节加诚实声明——**若有底稿存档应恢复目录**）；OPF-N4 ✅ FIXED（IPOPT 平台口径按当前 CMake 改写）；OPF-N5 ✅ FIXED（软锚残留注释一删一改）；OPF-N6 ✅ FIXED（孤岛保留语义改写，判死/判活条件补齐）。
- 新遗留（下批候选）：已全部随第四批清零（见下）。

### 第三批处置记录（2026-08-18，SC-N1 + MISC-2 清零）

- **SC-N1 ✅ FIXED**：方法 B + 网状网的 κ 钳制 `min(1.8, 1.15κ)` 统一作用于故障点逐贡献 κ（`kappa_net`、`kappa_of(z_src)`）与非故障母线单 κ（`src/short_circuit/short_circuit.cpp:run_short_circuit_detailed_impl`，此前仅作用于非故障母线）。同步更新 `tests/test_short_circuit_crossval.cpp` 两个 method-B 用例期望（故障母线 κ_net 现落 1.8 封顶）与三份文档（`iec60909_detailed.tex`、`short_circuit_rich_acdc_derivation.md/.zh.md`）。Debug 构建验证 12/12 + `[peak]` 2/2 通过。顺带修复同文件潜伏缺陷：`SparseInverseSolver` 与 `compute_fault_at_bus` 在 KLU `analyzePattern` 失败（空稀疏模式，如无支路/外网的 network-only 矩阵）后仍调 `factorize`，Debug 下断言崩溃——加 `nonZeros()==0` 早退守卫（Release 下原表现为静默 invalid，行为不变）。
- **MISC-2 ✅ FIXED**：五轮清零（8 路并行 + 主循环），共约 750 处行号引用迁移为符号锚点：① 170 处 `（:数字` 裸引用；② 约 230 处 `，:数字` 宽模式裸引用；③ 378 处 `\srcpath{文件}:行号`；④ 约 100 处 `同一文件:NN`/`头文件:NN`/`cpp:NN`/`\file{:NN}` 等变体；⑤ 约 50 处单行号表格/枚举残留。合理保留（不属缺陷）：verification.tex 中 13 处指向已不存在文档（LCC 技术报告、opf_hand_cases README）的存档引用；classic_solvers.tex:338 `:764--767`（指向已移除的旧兜底逻辑，散文待裁决）；component_models_math_audit.tex:383 文件头注释引用；parity_ipm.tex:589/739 `CMakeLists.txt:122--127`（目标已移除，见下条登记）。迁移工具 `tools/doc_anchor_symbolize.py` 此前已改为多条目仅记录 manual-review，本轮全部人工迁移并抽查核实。
- **opf_hand_cases 结论**：无存档可恢复（git 全历史无记录），保留现有"不在仓库检出中"诚实标注，不再追。
- 新登记的内容级问题：6 项已全部随第四批清零（见下）。

## 二、机械性漂移（不影响语义，批量修）

- PATH-1【跨文档路径失效，docs 重组（5842164e）后未回改】——**已批量修复（2026-08-18）**：全量扫描 docs/ 下 tex/md 的 `docs/...` 引用，101 处唯一匹配的死链已按真实位置改写（tex 转义风格保留；规律主要为 `docs/X.md` → `docs/theory/X.md`，另有 developer/reference/guides/archive 归位），涉及 31 个文件。两个文件确认已从仓库删除且无替代（`docs/lcc_dat_opf_report/LCC_DAT_OPF_technical_report.tex` 10 处、`docs/numerical_methods.md` 1 处），引用处已就地标注"该文档已不在仓库中"——若这些验证报告另有存档，应恢复或改写对应段落。有意保留未改：`documentation_reorganization_report.md` 中的旧路径属历史叙述。
- LINE-1【行号大面积漂移】——**已根治（2026-08-18）**：不再逐行重刷数字，而是全量迁移为"文件:函数名"符号锚点。迁移脚本 `tools/doc_anchor_symbolize.py`（可重跑，dry-run 默认）把 65 个文档文件中 2102 处行号锚点按当前代码解析为所属函数/结构体/lambda 锚点（含同文件/bare 简写与已改名文件 opf.cpp→three_phase_hybrid_opf.cpp 等的重定向）；46 处指向头文件注释区/转发头的锚点保留纯文件名。随后 8 路语义校对修正了约 600 处因行号漂移导致的错误符号（典型：ac_opf.cpp 大重构后旧行号落进 solve_with_parity_ipm 的 lambda 里）。规范条文已同步：docs/modules/README.md 数学模型规范第 2 条改为"必须给出实现函数及文件:函数名锚点，禁止行号"，各手册前言的"文件:行号"体例声明同步更新。仍残留：各章散文中未走锚点格式的零散纯行号引用（如 `:852--2140`），数量大且语义上下文弱，留作后续低优先级清理。
- MISC-1：`harmonics_power_flow/source_equivalent_model.tex:70` LaTeX 排版缺陷 `,qquad` 漏反斜杠——✅ FIXED（改 `,\qquad`）；`power_models` `scuc_builder.hpp` ramp 注释与实现张力——✅ FIXED（注释改为"缺省 1e6 MW/h 名义不约束；显式 0 禁止爬坡"，up/dn 两处）；`graph_manual.tex` 改进方向——✅ FIXED（悬垂折叠迭代恢复标注为已实现于 `result_recovery.cpp`，剩余仅收敛标志上报）。

### 第四批处置记录（2026-08-18，登记项清零）

- **newton.tex PV 迟滞口径** ✅ FIXED：进入侧各调用点 `entry_margin_pu=0.0`（无边距）；`pv_q_hysteresis_pu` 仅作用于恢复侧"钉限判定"容差 `limit_tol=max(1e-10, q_hys×1e-6)`，散文已按此改写（`newton_solver.cpp:check_q_limits_and_switch（lambda）`）。
- **options_results.tex bad_condition_threshold / tiny_pivot_threshold** ✅ 核实声明属实：两字段仅在 GUI 服务器层读写（`run_gui_server.cpp` 选项进出），求解核无消费者，"当前未接线"表述准确，文档不改。
- **parity_ipm.tex IPOPT 平台口径** ✅ FIXED：`HACDCPF_ENABLE_IPOPT` 为 Apple 默认 ON 的普通选项（任何平台可显式开启，无 FATAL_ERROR），强制透传 `MIPSOLVERS_BUILD_LOCAL_IPOPT`；Ipopt 缺失的 FATAL_ERROR 在 `MIPSolvers/cmake/Dependencies.cmake`（语义为"必须用仓库内 Ipopt 源码"）。"嵌入式 Ipopt 仅 macOS 可构建"改为"取决于 MIPSolvers 检出内嵌 ipopt/ 源码，默认仅 Apple 开启"。章首"全章行号均指…（共 2250 行）"改为符号锚点表述并删除失真行数。
- **parity_formulation.tex:44** ✅ FIXED：引用的 "the reference formulation stores dg as n x neq" 注释已不存在，散文改为"也曾留有…对照痕迹；当前检出处这些注释均已删除"。
- **validation.tex workspace** ✅ FIXED：删除不存在的 `reset_iteration`（头文件仅有 prepare_state/prepare_equations）。
- **component_models_math_audit.tex A2** ✅ FIXED（代码侧早已修复）：charger-only 投影现于 `project_in_place` fail-close 抛 `std::invalid_argument`，条目改写为"修复前行为/现已不可复现"口径，锚点更新；同文件 B9 的 `connect（lambda）` 误植锚点改为 `add_equivalent_branch_from_switch`（现存唯一 connect lambda 属 `detect_dc_dead_buses` 并查集，与语义无关）。
- **classic_solvers.tex:338** ✅ FIXED：`:764--767` 指向的旧兜底逻辑已移除；该残句与上文"→ 首母线"冗余（fallback 在 `auto_select_swing_bus` 末尾），删句。
- **robust_solvers.tex 转发头路径** ✅ FIXED：`nonlinear_scaling.hpp`、`robust_nonlinear_options.hpp`、`helm_solver.hpp` 共 7 处改 canonical 路径（`globalization/`、`solvers/`）。
- 至此本文件登记的实质/机械/候选问题全部清零；第三节"未立案项"三项亦于第五批补齐文档（见下），本文件可归档。

### 第五批处置记录（2026-08-18，"代码有而文档没有"补齐）

- decision_hint 决策路由、短路 λ_max、谐波 THD_I/TDD 零守卫三项全部按代码事实补入对应模块手册，详见第三节记录。

## 三、审计覆盖说明

- 19 个审计单元（18 个模块手册 + theory/md 文档组）全部完成；公式核对比例各单元报告为 8/8（ev）、20/20（time_series）、全部（market/power_models/graph）等，dynamics/short_circuit 等大手册按数学密集章节优先。
- 未立案的"代码有而文档没有"项——**已全部补齐（2026-08-18 第五批）**：
  `analysis` 的 decision_hint 决策路由（operation/planning 两模式全分支、0.25 压差阈值，补入
  `docs/modules/analysis/chapters/source_equivalent_model.tex` 新增"决策路由"小节）；
  短路稳态 I_k 的 λ_max 机制（xd/xq≤1.2→2.8、≥1.5→5.0、线性插值、转移比折算，补入
  `iec60909_detailed.tex` 稳态电流条目）；谐波 THD_I/TDD 零守卫（$I_1\le10^{-12}$→THD_I=0、
  $I_{dem}$ 回退后仍≤$10^{-12}$→TDD=0，补入 harmonics `source_equivalent_model.tex` 并改为
  分段函数，顺带清除该处 `（2789--2849 行）` 行号残留）。
  若规范日后要求全量转写，仍需另立全模块普查。
