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
- 新遗留（下批候选）：newton.tex 进入侧迟滞 `pv_q_hysteresis_pu` 口径疑似过时（代码 entry_margin_pu=0.0 无死区）；`options_results.tex:120` bad_condition_threshold"未接线"未核。

### 第三批处置记录（2026-08-18，SC-N1 + MISC-2 清零）

- **SC-N1 ✅ FIXED**：方法 B + 网状网的 κ 钳制 `min(1.8, 1.15κ)` 统一作用于故障点逐贡献 κ（`kappa_net`、`kappa_of(z_src)`）与非故障母线单 κ（`src/short_circuit/short_circuit.cpp:run_short_circuit_detailed_impl`，此前仅作用于非故障母线）。同步更新 `tests/test_short_circuit_crossval.cpp` 两个 method-B 用例期望（故障母线 κ_net 现落 1.8 封顶）与三份文档（`iec60909_detailed.tex`、`short_circuit_rich_acdc_derivation.md/.zh.md`）。Debug 构建验证 12/12 + `[peak]` 2/2 通过。顺带修复同文件潜伏缺陷：`SparseInverseSolver` 与 `compute_fault_at_bus` 在 KLU `analyzePattern` 失败（空稀疏模式，如无支路/外网的 network-only 矩阵）后仍调 `factorize`，Debug 下断言崩溃——加 `nonZeros()==0` 早退守卫（Release 下原表现为静默 invalid，行为不变）。
- **MISC-2 ✅ FIXED**：五轮清零（8 路并行 + 主循环），共约 750 处行号引用迁移为符号锚点：① 170 处 `（:数字` 裸引用；② 约 230 处 `，:数字` 宽模式裸引用；③ 378 处 `\srcpath{文件}:行号`；④ 约 100 处 `同一文件:NN`/`头文件:NN`/`cpp:NN`/`\file{:NN}` 等变体；⑤ 约 50 处单行号表格/枚举残留。合理保留（不属缺陷）：verification.tex 中 13 处指向已不存在文档（LCC 技术报告、opf_hand_cases README）的存档引用；classic_solvers.tex:338 `:764--767`（指向已移除的旧兜底逻辑，散文待裁决）；component_models_math_audit.tex:383 文件头注释引用；parity_ipm.tex:589/739 `CMakeLists.txt:122--127`（目标已移除，见下条登记）。迁移工具 `tools/doc_anchor_symbolize.py` 此前已改为多条目仅记录 manual-review，本轮全部人工迁移并抽查核实。
- **opf_hand_cases 结论**：无存档可恢复（git 全历史无记录），保留现有"不在仓库检出中"诚实标注，不再追。
- 新登记的内容级问题（下批候选，均为散文事实性声明过时，非锚点问题）：
  1. `parity_ipm.tex:588--592、739` 散文称"非 Apple 显式开启 HACDCPF_ENABLE_IPOPT 直接 FATAL_ERROR"与"嵌入式 Ipopt 仅 macOS 可构建"——该 FATAL_ERROR 已从根 CMakeLists.txt 移除，现为 Apple 默认 ON 的普通选项，Ipopt 检测在 `MIPSolvers/cmake/Dependencies.cmake`（MIPSOLVERS_HAVE_IPOPT）；
  2. `parity_formulation.tex:44` 散文引用的 `parity_ipm.cpp` "the reference formulation stores dg as n x neq" 注释已不存在；
  3. `power_flow/chapters/validation.tex:241--245` 散文称 `workspace.hpp` 新增 `reset_iteration`，代码中不存在（仅剩 prepare_state/prepare_equations）；
  4. `parity_ipm.tex:9` "共 2250 行"与实际 3594 行不符（文件规模描述）；
  5. `component_models_math_audit.tex` A2 条锚点沿用的 `connect（lambda）` 已不存在（charger-only 现于 `project_in_place` 抛异常），建议散文改写为"修复前 connect（lambda），现 project_in_place 拒绝"；
  6. `robust_solvers.tex:32、:268` 等引用的 `include/hacdcpf/power_flow/nonlinear_scaling.hpp`、`helm_solver.hpp` 为 3 行转发头，可改 canonical 路径（`globalization/`、`solvers/` 子目录，低优先级）。

## 二、机械性漂移（不影响语义，批量修）

- PATH-1【跨文档路径失效，docs 重组（5842164e）后未回改】——**已批量修复（2026-08-18）**：全量扫描 docs/ 下 tex/md 的 `docs/...` 引用，101 处唯一匹配的死链已按真实位置改写（tex 转义风格保留；规律主要为 `docs/X.md` → `docs/theory/X.md`，另有 developer/reference/guides/archive 归位），涉及 31 个文件。两个文件确认已从仓库删除且无替代（`docs/lcc_dat_opf_report/LCC_DAT_OPF_technical_report.tex` 10 处、`docs/numerical_methods.md` 1 处），引用处已就地标注"该文档已不在仓库中"——若这些验证报告另有存档，应恢复或改写对应段落。有意保留未改：`documentation_reorganization_report.md` 中的旧路径属历史叙述。
- LINE-1【行号大面积漂移】——**已根治（2026-08-18）**：不再逐行重刷数字，而是全量迁移为"文件:函数名"符号锚点。迁移脚本 `tools/doc_anchor_symbolize.py`（可重跑，dry-run 默认）把 65 个文档文件中 2102 处行号锚点按当前代码解析为所属函数/结构体/lambda 锚点（含同文件/bare 简写与已改名文件 opf.cpp→three_phase_hybrid_opf.cpp 等的重定向）；46 处指向头文件注释区/转发头的锚点保留纯文件名。随后 8 路语义校对修正了约 600 处因行号漂移导致的错误符号（典型：ac_opf.cpp 大重构后旧行号落进 solve_with_parity_ipm 的 lambda 里）。规范条文已同步：docs/modules/README.md 数学模型规范第 2 条改为"必须给出实现函数及文件:函数名锚点，禁止行号"，各手册前言的"文件:行号"体例声明同步更新。仍残留：各章散文中未走锚点格式的零散纯行号引用（如 `:852--2140`），数量大且语义上下文弱，留作后续低优先级清理。
- MISC-1：`harmonics_power_flow/source_equivalent_model.tex:70` LaTeX 排版缺陷 `,qquad` 漏反斜杠；`power_models` 顺带发现 `scuc_builder.hpp:51` 头文件注释与实现语义有张力（缺省 1e6 名义宽松 vs 注释"不施加"）；`graph_manual.tex:204` "改进方向"中"悬垂折叠迭代恢复"已实现（`result_recovery.cpp:276-301`），仅剩"报告收敛标志"未做。

## 三、审计覆盖说明

- 19 个审计单元（18 个模块手册 + theory/md 文档组）全部完成；公式核对比例各单元报告为 8/8（ev）、20/20（time_series）、全部（market/power_models/graph）等，dynamics/short_circuit 等大手册按数学密集章节优先。
- 未立案的"代码有而文档没有"项（按现行规范不构成不一致，但若规范要求全量转写则需补）：`analysis` 的 decision_hint 决策路由（`multidimensional_weak_link.cpp:68-110`）、短路稳态 I_k 的 λ_max 机制、谐波 THD_I/TDD 零守卫。
