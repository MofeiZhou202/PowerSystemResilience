# 数学模型文档 ↔ 代码一致性审计问题卡

> 本文件是受控改进项清单：每条问题卡含文档位置、文档陈述、代码证据与修正方向。
> 修复一条删一条；全部清零后本文件可归档。审计方法：19 个模块单元逐一核对公式/默认值/阈值/分支/引用行号，
> 仅当有双侧证据（文档位置 + 代码位置）时立案。UNCERTAIN 项（证据不足的疑点）记录于各模块审计底稿，未列入本文件。

## 一、实质性不一致（数学模型/行为/默认值，必须修）

### power_flow（`docs/modules/power_flow/`）

- PF-1【声明失实】`power_flow_manual.tex:220-223` 摘要第 7 条称 LCC 已整体移除；实际 `src/power_flow/lcc_model.cpp/.hpp`、`solver_data.hpp:31`、`jacobian_builder.cpp:217-235,451-488`、`dc_solver.cpp:72-76`、`hacdcpf.cpp:1525-1576` 全部保留。→ 撤销声明并补写 LCC 小节。
- PF-2【公式符号】`chapters/newton.tex:85-86` ZIP 导数写 `-(p·w_p1+2V·p·w_p2)`；代码 `jacobian_builder.cpp:327,342` 为正号（与文档自述的 calc−spec 约定自洽）。→ 改 `+`。
- PF-3【旧实现描述】`newton.tex:153-155`、`robust_solvers.tex:74-76` 的"接受比分级 2.0/1.5/1.2/1.01 + 事后 φ≤φ_max(1+c1)"不存在；现为 Armijo-GLL（`newton_solver.cpp:1519-1535`、`nonmonotone_linesearch.hpp:84-94`）。→ 按 Armijo-GLL 重写。
- PF-4【默认值写反】`newton.tex:159`、`robust_solvers.tex:140`、`options_results.tex:175` 称 `enable_auto_fallback_scheduling` 默认 false；实际 `{true}`（`robust_nonlinear_options.hpp:268`）。→ 改 true。
- PF-5【代数内核】`hybrid_acdc.tex:60,70-71` 称纯 DC 求解器用致密 FullPivLU；实际稀疏 SparseLU（`dc_solver.cpp:9,89-92,181-195`）。
- PF-6【公式符号】`hybrid_acdc.tex:261` DC/DC droop 写 `+k_droop(v_out−v_ref)`；代码 `converter_model.cpp:249` 为减号（电压升→出力降，代码方向正确）。→ 文档改 `−`。
- PF-7【行为描述】`assembly.tex:97-98` 称 tap=0 按 1.0、|z|=0 跳过；实际均抛 `std::runtime_error` fail-close（`admittance_builder.cpp:51-61`，DC 侧 :128-132）。
- PF-8【关键分支缺失】`cpf.tex:10-13,70-72` 称默认弧长可翻越鼻点；实际任一在运 PV 母线即回退自然参数化（`voltage_stability.cpp:323-342`，`model_scope="cpf:natural-parameterization+pv-pq-active-set"`），文档通篇未提。→ 补回退分支与诚实口径。
- PF-9【置位声明】`cpf.tex:80-82,145-151` 称首步失败/legacy 停滞置 `nose_found=true`；实际不置（`voltage_stability.cpp:397-398,445-449`），且 legacy 终止串引用缺后缀；终止原因"全集"还缺 :462/:479 两条弧长割线坍缩。
- PF-10【旧实现描述】`classic_solvers.tex:222` 分布式松弛上限 1.5P/10pu 不存在；现为 `max(|pmin−psched|,|pmax−psched|)/Sbase`（`distributed_slack_solver.cpp:404-415`）。
- PF-11【次要】`cpf.tex:226-227` VSI 基态负荷口径漏 `has_component_loads` 聚合路径（`voltage_stability.cpp:562-567`）；`cpf.tex:325` 文件行数 534→实际 581。

### optimal_power_flow（`docs/modules/optimal_power_flow/`）

- OPF-1【行为不一致】`dc_opf.tex:181-185` 称相角参考为"第一台 SLACK 母线钉 [0,0]"；实际按连通分量逐分量锚定（`dc_opf.cpp:183-187,263-272`）。
- OPF-2【完整性声明失效】`dc_opf.tex:393` DCOPFOptions"全部字段"缺 `phase_one_linear_relaxation`（`opf_options.hpp:210-215`）；`parity_ipm.tex:617-634` IPMOptions"全字段"缺 10 个 prepared_numeric/phase_one 字段（`native_ipm_solver.hpp:51-72`）；`ac_opf_native.tex:654-699` ACOPFOptions 表缺 DC Phase I 七字段等。→ 补字段或收窄表述为"主要字段"。

### power_models

- PM-1【口径过度推广】`power_models_manual.tex:22,52-53,106,120-121` 称 model_scope 固定为 `experimental-aml-builder:not-production-runtime`；实际仅 AMLBuildResult 如此，混合 OPF 为 `experimental-aml-hybrid-acdc-opf`（`hybrid_opf_model_builder.hpp:83`），SCUC 为 `experimental-aml-scuc:system-balance-no-network`，DCOPF/LinDistFlow 无该字段。

### reliability

- REL-1【公式转写漏步】`source_equivalent_model.tex:132-133` 三阶段增量切负荷漏重标定步：`three_stage_reliability.cpp:3457-3460` 截非负后还按 `scale=Δ/positive_sum` 比例缩放。

### market

- MKT-1【能力口径过时】`market_manual.tex:22-24,50-53,113-115` 称"AC-only、显式拒绝混合资产"；代码已演进为 DC 电压线性化的全混合 SCUC/SCED，运行时改写 scope 为 `ac-dc-linear-v1:dc-voltage+bidirectional-converters` 并条件化 ValidityFlags（`market_simulation.cpp:4725-4735,5170`；仅 external_grid/energy_router 走 unsupported_assets 退回 :454-476）。→ 摘要/§1.3/§4.2 按当前行为重写。

### short_circuit

- SC-1【公式非源码等价】`iec60909_detailed.tex:28-30` 峰值电流为单 κ 教科书式；实际故障母线按 IEC 60909-0 (59) 逐贡献求和、各贡献取自身 κ_i（`short_circuit.cpp:1744-1766`），且方法 B+Meshed 的 `κ←min(1.8,1.15κ)` 分支未写入。
- SC-2【选项语义夸大】同章 :45 称 SCTopology 影响"κ 与开断电流网状衰减"；实际仅影响 κ（全模块唯一消费点 :1703）。
- SC-3【阈值量级错】`converter_contributions.tex:13` 跟网换流器零贡献阈值写 1e-12；实际生效阈值 p_rated≤1e-6（`short_circuit.cpp:1477-1484`）。

### integrated_energy

- IE-1【符号相反】`integrated_energy_manual.tex:47` PCC 功率 `p_pcc = p_imp − p_exp`；代码 `integrated_energy_optimizer.cpp:848` 为 export−import（章节文件 source_equivalent_milp.tex:152 已正确，主手册未同步）。
- IE-2【校验过度声称】`source_equivalent_milp.tex:12-13` 及主手册复述"被动效率必须在 (0,1] 否则抛异常"；实际校验清单（cpp:164-194）不含 `eta_wasteheat`（默认 0.0，本身就违反声称域）与 `eta_carbon_to_fuel`。→ 列举实际校验字段。

### carbon_analysis

- CA-1【语义误植】`carbon_analysis_manual.tex:92-93` 称 BFS 比例追踪按 `loss_allocation_alpha` 分摊损耗；实际 BFS 全额归送端（`carbon_analysis.cpp:1523-1536`），alpha 仅作用于矩阵法（:1735-1738）。

### analysis

- AN-1【符号张冠李戴】`analysis_manual.tex:137` 参数表把 `n1_loading_limit` 的符号写作 τ_max；真正 τ_max 是 `default_dr_max_output_coeff`（`hosting_capacity.cpp:214,221`），两者默认值碰巧都是 1.0 掩盖了错误。
- AN-2【范围列与代码/自身章节矛盾】`:136-137` β 与 N-1 限值范围写 0~1；代码仅下界钳位 ≥0（`hosting_capacity.cpp:74-75`），本手册公式章 :11-13 也是"下限为零"。

### ev_power_traffic

- EV-1【默认值错】`ev_power_traffic_manual.tex:120` 称 `allow_v2g` 默认 false；顶层 `EVPowerTrafficOptions::allow_v2g{true}`（`options.hpp:32`），false 的是子结构 `JointOptimizerOptions::allow_v2g`（:285）。→ 参数表按结构分行。

### time_series

- TS-1【公式缺百分数归一化】`source_equivalent_model.tex:202` EOL 阈值直接 clamp(EOL,0,1)；代码先将 >1 视为百分数除 100（`lifecycle_simulation.cpp:476-479`）。
- TS-2【边界声明过时】`time_series_manual.tex:130-131` 称按日并行仅 Native 可；实际 SCIP 同样可并行且 Auto 下 SCIP 优先（`time_series_pf.cpp:3245-3264`、`annual_production_sim.cpp:1175-1182`）。
- TS-3【阈值不一致】`source_equivalent_model.tex:133` 日循环上限用 E'（:113 定义阈值 1e-12）；代码此处为 1e-9（`time_series_pf.cpp:2732`，SOC 段才是 1e-12）。

### scenario_generation

- SG-1【参数语义不存在】`scenario_generation_manual.tex:121` `min_fault_probability` 声称"过滤"；src/ 内零消费点（仅声明与解析），与章节 :126 的正确声明自相矛盾。→ 参数表改"保留字段，当前未消费"。
- SG-2【范围非强制】`:123` `holland_b` 标 1~2.5；代码仅下钳 ≥0.8（生成 `typhoon_resilience.cpp:319`）/≥0.5（求值 :444），无上界。→ 改为实际钳位或注明经验范围。
- SG-3【"固定"overstate】`:129-130` 聚类"固定 hybrid_kmedoids_tail_5pct"；实际可经 JSON `method` 覆盖（`scenario_generation.cpp:2832`，另有 weighted_k_medoids 分支 :1665）。

### theory/ 的 md 文档

- TH-1【回退行为不符】`vsc_limit_ncp_power_flow_contract.md:33-34` 称无序 droop 对回退 pmin/pmax；实际无序（零宽）对不做钳制、P_ref=P_raw（`vsc_limit_ncp.cpp:90-102,205-210`）。
- TH-2【κ 上限描述错】`short_circuit_rich_acdc_derivation.md:767-768` 称方法 B meshed 乘 1.15 后 clamp [1.0,2.0]；实际为 `min(1.8, 1.15κ)`、无 [1,2] clamp（`short_circuit.cpp:1703-1707`）。

## 二、机械性漂移（不影响语义，批量修）

- PATH-1【跨文档路径失效，docs 重组（5842164e）后未回改，约 20 处】：reliability（overview.tex:6,53、verification.tex:42-43）、resilience（verification.tex:17,45-46）、market（manual.tex:4,21,121-122）、short_circuit（dc_fault.tex:38、verification.tex:36）、dynamics（manual.tex:4,22-23,42,126-127）、network_reconfiguration（manual.tex:22,37）、integrated_energy（manual.tex:4,24,37 + `integrated_energy_optimizer.cpp:49` 注释）、carbon_analysis（manual.tex:4,21,35）、analysis（manual.tex:41-42）、scenario_generation（manual.tex:4,23,42）、graph（manual.tex:43）、sppt（manual.tex:3,21-22,41,119）——规律统一：`docs/X.md` → `docs/theory/X.md`（graph 例外：`docs/developer/graph_runtime_contract.md`）。
- LINE-1【行号大面积漂移】：power_flow（newton_solver/jacobian_builder/fdpf_solver/dc_solver/defaults.hpp 引用普遍失效，集中在 2026-07-22 之后大改的文件；CPF 与 ac_linearized_pf 章基本准确）、optimal_power_flow（ac_opf.cpp 增约 700 行致 +2~+400 行漂移；dc_opf/parity_* 同病）、ev（:102 区间终点超文件末 20 行）、resilience/reliability/network_reconfiguration/dynamics 各 1-3 处 ±2~7 行。→ 以当前树重刷；长期建议手册构建加行号核验脚本，或只保留"文件+函数名"。
- MISC-1：`harmonics_power_flow/source_equivalent_model.tex:70` LaTeX 排版缺陷 `,qquad` 漏反斜杠；`power_models` 顺带发现 `scuc_builder.hpp:51` 头文件注释与实现语义有张力（缺省 1e6 名义宽松 vs 注释"不施加"）；`graph_manual.tex:204` "改进方向"中"悬垂折叠迭代恢复"已实现（`result_recovery.cpp:276-301`），仅剩"报告收敛标志"未做。

## 三、审计覆盖说明

- 19 个审计单元（18 个模块手册 + theory/md 文档组）全部完成；公式核对比例各单元报告为 8/8（ev）、20/20（time_series）、全部（market/power_models/graph）等，dynamics/short_circuit 等大手册按数学密集章节优先。
- 未立案的"代码有而文档没有"项（按现行规范不构成不一致，但若规范要求全量转写则需补）：`analysis` 的 decision_hint 决策路由（`multidimensional_weak_link.cpp:68-110`）、短路稳态 I_k 的 λ_max 机制、谐波 THD_I/TDD 零守卫。
