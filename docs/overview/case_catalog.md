# 内置案例目录（Case Catalog）

> 平台内置案例的能力导向索引：每个案例一张故事卡——规模、数据亮点、推荐演示路径。
> 案例分两级：**旗舰 13 个**（工具栏「内置算例」下拉，每个对应至少一个能力域的主验证案例）与
> **扩展 8 个**（「加载算例」模态框的"更多算例（扩展）"组，API 与模态框全量可达）。
> 程序化访问用生产路由 `GET /api/cases`（返回 `cases[]` 结构化目录含 `featured` 标记 +
> `case_names[]` 纯名称清单 + `default_case`），加载用 `POST /api/session/load_builtin {"case": "<name>"}`。
> 案例构建函数位于 `src/io/case_builders.cpp`，注册与目录元数据位于 `tests/run_gui_server.cpp`
> 的 `case_catalog()` / `build_case()`。

## 能力覆盖矩阵（19 能力域 × 验证案例 × 验证点）

下表为**手写 contract**，与验证运行器 `tools/validate_case_capabilities.py` 的断言一一对应；
运行器输出物（含最近一次 PASS/FAIL 状态）写入 `output/capability_coverage.md`。
索引空间与单位：电压 AC 母线 vm_pu（标幺）；功率 MW/MVar；电价 $/MWh；故障率 occ/yr。

| 能力域 | 验证案例 | 验证点（断言） |
|---|---|---|
| 混合潮流求解器族 | ieee14_acdc | NR/FDPF/DC 三法收敛，NR vs FDPF max\|ΔV\| < 1e-3 pu |
| OPF 与约束优化 | ieee24_3area_acdc_expanded | 收敛 + objective > 0（当前受 MIPSolvers IPM 回归阻断，KNOWN 标注） |
| RPO 无功优化 | ieee24_3area_acdc_expanded | OLTC 档位/并联动作非空（同上 KNOWN） |
| 三相混合 PF | urban_lvn_primary_secondary | 相域模型收敛，VUF 可输出 |
| 电压稳定 CPF | — | HTTP 未暴露（C++ 测试层覆盖），SKIP |
| 图建模与网络降阶 | comprehensive_hybrid_acdc | 化简节点合并记录非空，结果可恢复 |
| ONR 网络重构 | dist33_tie_demo | MILP 可行，重构后网损 ≤ 基态 |
| IEC 60909 短路 | comprehensive_hybrid_acdc | 各母线 Ik" > 0（kA） |
| 谐波分析 | ieee14_acdc | VSC 自动 NIC，THD > 0 且含 IEEE 519 校核字段 |
| 暂态动力学 | networked_microgrids_islanding | 发电机跳闸：频率轨迹非平线，被跳机组 P/I 末值归零 |
| 小信号模态分析 | hybrid_acdc_microgrid_island | 模态特征值/阻尼比可输出 |
| 时序生产模拟 | multiscale_comprehensive_acdc | 24 步全收敛 |
| 电力市场 | market_3bus_toy | 出清收敛，峰时 LMP 分裂，结算现金流残差 ≈ 0 |
| 承载力（DL/T 2041-2025） | comprehensive_hybrid_acdc | 变压器校核条目非空 |
| 薄弱环节与反事实规划 | dist33_microgrid_der | 证据实体 → 辨识输出非空；反事实措施评估系统数 > 0 |
| 可靠性评估 | dist33_microgrid_der | SAIDI > 0、ASAI < 1（支路故障率/用户数驱动） |
| 信息物理可靠性 L1 | cyber_physical_reliability_demo | cyber_physical 自动化增量字段族存在 |
| 弹性恢复 | dist33_microgrid_der | avg_restoration_ratio > 0，优先级层级生效（HiGHS 路径） |
| 场景生成与台风 | dist33_microgrid_der / five_province_acdc | 聚类缩减输出；Holland 风场故障序列非空（GIS 驱动） |
| 碳流追踪 | ieee24_3area_acdc_expanded | 节点碳势 > 0，源荷平衡残差 ≈ 0 |
| 真实区域电网 + GIS | five_province_acdc | FDPF 收敛，电压分布健康（NR 刚性已标注） |
| VSC 限值 NCP 规模基准 | case300_acdc_vsc_limit_ncp / case2000_acdc_vsc_limit_ncp | 真实 300/2000 AC + 6/8 DC，逐 VSC 显式 NCP；两例均 4 台同时限流、单次 Jacobian pattern 构建 |
| GFM Norton 生产演示 | gfm_norton_limit_demo | 2 AC + 2 DC + 2 VSC；独立 AC 角参考、VDC-Q 直流支撑和显式 GFM `E∠δ/Zv` 限流证书 |


## 快速上手与综合展示

### ieee14_acdc — IEEE14 混合入门 · 14 AC + 2 DC
- 最小交直流混合系统：IEEE 14 节点 + 双端 DC（2 台 VSC，挂 AC 5/9 号母线）。
- 推荐演示：潮流求解器家族对比（NR/FDPF/DC/自适应）；谐波模块的 VSC 自动 NIC（六脉波特征频谱）基例。
- 构建：`build_ieee14_acdc()`（src/io/case_builders.cpp:sum_bus_group_diesel_capacity_mw）。

### ieee24_3area_acdc_expanded — IEEE24 三区域扩展 · 默认 · 24 AC + 8 DC
- 默认基准案例。10 机全成本（c2/c1/c0 + 启停 + 爬坡 + 最小启停）与碳因子（煤 0.85 / 气 0.42 tCO2/MWh）；
  DC 侧 8 母线 9 支路 8 VSC + 2 DCDC + DC 负荷/光伏/储能；AC 侧 2 储能；3 台 OLTC + 4 台可投切并联电容器。
- 推荐演示：UC→OPF→时序潮流全流水线、RPO（OLTC 离散档位 + 电容器投切）、碳流追踪。
- 构建：`build_ieee24_3area_acdc_expanded()`（src/io/case_builders.cpp:build_ieee24_3area_acdc）。

### ieee24_3area_acdc（扩展组）— IEEE24 三区域基础版 · 24 AC + 4 DC
- 扩展版的轻量基座：同样的 10 机成本/碳因子，4 端 DC。适合 UC/OPF 快速验证。
- 构建：`build_ieee24_3area_acdc()`（src/io/case_builders.cpp:make_microgrid（lambda））。

### comprehensive_hybrid_acdc — 综合全组件秀场 · 21 AC + 4 DC
- 110kV 外部电网 + 20kV 三馈线：OLTC（±8 档）、三绕组变、2 可投切并联、10 个 Switch + 3 断路器、
  2 微网记录、PV/风电/小水电、3 AC 储能、3 充电桩、DC 子网（2 DCDC、grid-forming VSC）。
- **承载力旗舰**：OLTC 带 DL/T 2041-2025 `cap_*` 评估参数（β/τ_max/注册分布式/预期储能），
  骨干母线带断路器短路额定值，1 台储能为 static 充电策略（与 OPF 默认对照）。
- 推荐演示：承载力评估（变压器 N-1/反向负载/短路容量校核）、RPO、开关拓扑重构、碳流。
- 构建：`build_comprehensive_hybrid_acdc()`（src/io/case_builders.cpp:build_dist33_microgrid_der）。

### multiscale_comprehensive_acdc — 多尺度时序旗舰 · 21 AC + 4 DC
- 综合秀场 + VPP + 3 台能量路由器 + 移动储能 + DC 断路器；发电机补齐 UC 成本与爬坡。
- 推荐演示：日 24 步时序生产模拟、年度 8760h 分层仿真（既有回归基线）。
- 构建：`build_multiscale_comprehensive_acdc()`（src/io/case_builders.cpp:build_comprehensive_hybrid_acdc）。

### demo_multizone_acdc（扩展组）— 多电压等级 DER 演示 · 24 AC + 4 DC
- 多 zone / 多电压等级 + 静态发电机（PV/柴油/燃料电池）+ RenewableGen + PVSystem + 双侧储能。
- 推荐演示：DER 混合建模、多区功率交换、碳流（燃料混合机组）。
- 构建：`build_demo_multizone_acdc()`（src/io/case_builders.cpp:add_shunt（lambda））。

## 配电网与 DER

### case33bw_acdc（扩展组）— IEEE33 配电混合 · 33 AC + 2 DC
- 经典 33 节点配电（含 5 条常开联络支路）+ 双端 DC。配网潮流与重构入门。
- 构建：`build_case33bw_acdc()`（src/io/case_builders.cpp:add_oltc（lambda），MATPOWER 文件驱动）。

### case33mg_acdc（扩展组）— IEEE33 多微网 · 33 AC + 2 DC
- 33 节点 + 3 条微网记录（PCC 3/12/25，自动汇总负荷/装机/储能容量）。
- 构建：`build_case33mg_acdc()`（src/io/case_builders.cpp:add_oltc（lambda））。

### case69_acdc（扩展组）— IEEE69 配电混合 · 69 AC + 2 DC
- 69 节点径向配网 + 双端 DC，中等规模配网潮流。
- 构建：`build_case69_acdc()`（src/io/case_builders.cpp:add_oltc（lambda））。

### dist33_microgrid_der — Dist33 DER 可靠性·弹性旗舰 · 33 AC + 2 DC
- 33 节点 + 3 微网：4 PV、2 风、3 BESS（含充放电报价与寿命成本）、柴油/CHP 静态发电机、
  3 个需求响应 FlexibleLoad（居民温控/商业空调/工业过程）、7 个自动化 Switch（远方可操作）。
- **可靠性·弹性旗舰**：支路分层故障率/修复时间（主干 0.15/8h、分支 0.08/5h、联络 0.02/4h occ/yr·h），
  机组 FOR/MTBF，母线用户数与重要度分层。
- 推荐演示：可靠性评估（SAIDI/SAIFI/ASUI、FMEA）、弹性恢复（优先级加权 + 远方开关重构）、
  碳流、时序生产模拟。
- 构建：`build_dist33_microgrid_der()`（src/io/case_builders.cpp:make_dc_sgen（lambda））。

### dist33_tie_demo — Dist33 联络重构专用 · 33 AC 纯交流
- case33bw 纯 AC 化，5 条标准联络线全部断开（次优径向）；每条支路可投切。
- 推荐演示：ONR 网络重构——自动重选更低损耗的径向树，对比重构前后网损。
- 构建：`build_case()` 内由 case33bw 派生（run_gui_server.cpp）。

### urban_lvn_primary_secondary — 城市低压三相 338 节点 · 338 AC + 3 DC
- 230/13.8/0.48kV 三级城市低压网：4 馈线 × 6 区 × 12 街道，24 台配电变（Dd0/Dyn11）、
  312 个不平衡负荷（wye/delta、ZIP 比例、相分配）、屋顶 PV、社区储能、4 条常开联络、DC 快充走廊。
- **唯一带完整三相相域模型**（`three_phase_ac` 同步生成：相母线/相线路/相变压器/相负荷）。
- 推荐演示：三相潮流（balanced vs phase-domain 对拍）、OpenDSS 对比、承载力、电压不平衡评估。
- 构建：`build_urban_lvn_primary_secondary()`（src/io/case_builders.cpp:ac_line（lambda））。

## 微网与暂态动态

### hybrid_acdc_microgrid_island — 混合微网孤岛暂态 · 6 AC + 2 DC
- 柴油 genset（ClassicalMachine + TGOV1 + SEXS 完整参数）+ 构网型 BESS + VSC DC 子网 + PCC 断路器。
- 推荐演示：暂态 DAE——孤岛/并网切换、GFM 下垂响应、负荷阶跃后的频率/电压轨迹。
- 构建：`build_hybrid_acdc_microgrid_island()`（src/io/case_builders.cpp:NuclearDef）。

### networked_microgrids_islanding — 联网微网群多机暂态 · 8 AC + 2 DC
- 4 台同步机（utility + 3 微网机组，均带 TGOV1/SEXS）+ 3 个 PCC 开关 + 2 条常开联络。
- 推荐演示：多机系统暂态（slack 动态化语义）、发电机跳闸事件、微网群孤岛-重并全流程。
- 构建：`build_networked_microgrids_islanding()`（src/io/case_builders.cpp:dc_bus（lambda））。

## 可靠性·弹性·市场

### cyber_physical_reliability_demo — 信息物理 FLISR 基准 · 3 AC
- 3 节点 FLISR 基准：支路故障率（1.0 / 1e-6 / 0.0）+ 修复时间，2 负荷带用户数，1 储能。
- 推荐演示：信息物理可靠性 L1——量化"自动化可用 vs 不可用"的恢复增量。
- 构建：`build_cyber_physical_reliability_demo()`（src/io/case_builders.cpp:make_ac_sgen（lambda））。

### market_3bus_toy — 市场 3 节点阻塞 · 3 AC
- L3 限额 20 MVA 制造阻塞；3 机全 UC 成本（0-based 索引，60 Hz）。
- 推荐演示：SCUC→SCED→LMP 分裂→结算最小闭环。
- 构建：`build_market_3bus_toy()`（src/io/case_builders.cpp:build_comprehensive_hybrid_acdc）。

### market_5bus_acdc_toy（扩展组）— 市场 5 节点混合边界演示 · 5 AC + 2 DC
- AC 阻塞 + DC 通道 40 MVA 限额构造的混合拓扑（0-based，60 Hz）；5 机全成本。
- 注意：市场模块声明 **AC-only 边界**并拒绝混合资产（`status=unsupported_hybrid_market_assets`）——
  本例用于潮流/OPF 与"诚实结果口径"（model_scope）演示；市场出清请用 market_3bus_toy。
- 构建：`build_market_5bus_acdc_toy()`（src/io/case_builders.cpp:build_multiscale_comprehensive_acdc）。

## 输电网与性能基准

### five_province_acdc — 南方五省区域电网 · 52 AC + 6 DC
- 真实区域电网模型：5 区域 52 节点、70 台机组（核电/水电/火电模板）、3 条 HVDC + 16 回交流联络线，
  全部母线带 GIS 坐标。
- 注意：系统刚性强，统一 NR 潮流对该系统收敛性待改进，**推荐 FDPF**（10 迭代内收敛，
  电压分布 1.00–1.013 pu）。
- 推荐演示：GIS 地图渲染、FDPF 大系统快解、区域间功率交换分析。
- 构建：`build_five_province_acdc()`（src/io/case_builders.cpp:build_market_3bus_toy）。

### ieee118_acdc（扩展组）— IEEE118 混合 · 118 AC + 6 DC
- 118 节点 54 机（gencost）+ 9 台固定变比变压器 + 14 并联母线 + 6 端 DC。
- 推荐演示：中大型 OPF/潮流基准。
- 构建：`build_ieee118_acdc()`（src/io/case_builders.cpp:add_dc_storage（lambda），MATPOWER 文件驱动）。

### case300_acdc（扩展组）— IEEE300 + MTDC · 300 AC + 6 DC
- 300 节点 69 机 + 62 台变压器（前 3 台提升为 OLTC）+ 29 并联母线 + 6 端环网 MTDC（带三段损耗）。
- 推荐演示：大系统 RPO（OLTC + 固定并联）、OPF 多后端对比。
- 构建：`build_case300_acdc()`（src/io/case_builders.cpp:add_oltc（lambda））。

### gfm_norton_limit_demo — GFM Norton 限流演示 · 2 AC + 2 DC
- 可从 authored/平坦初值直接求解；一台 VDC-Q 站提供真实 DC 电压与能量支撑，一台并网 GFM 使用内部电势/角度、虚拟阻抗和固定六槽限流 NCP。
- 推荐演示：GUI 四参数编辑与 JSON round-trip、PF 证书、OPF post-PF replay、PF→暂态 Norton seed。
- 构建：`build_gfm_norton_limit_demo()`。

### case300_acdc_vsc_limit_ncp — IEEE300 VSC 限值基准 · 300 AC + 6 DC
- 在 case300 六端环网 MTDC 上启用每站六槽固定 NCP 块、三种 priority 与 Vdc droop 饱和；非 GFM 站的两个内部电势槽使用恒等约束。
- 注册回归中 6 台 VSC 全部进入扩展块、4 台同时触发电流圆，NCP 残差不超过 `1e-9`，pattern 只构建一次。
- 构建：`build_case300_acdc_vsc_limit_ncp()`。

### case2000_acdc — ACTIVSg2000 性能旗舰 · 2000 AC + 8 DC
- 2000 节点 544 机 + 149 并联母线 + 8 端 MTDC（按 area 自动选网孔最强的机端母线挂接）。
- 四台固定 P 站各传输 0.5 pu，四台 Vdc-Q 站按统一 5% Vdc droop 带宽分担；`k_vdc` 由平方电压控制律反算，不使用案例内裸调参。
- 注册生产回归使用统一混合 NR、`tol=1e-8`、`max_iter=100`，并显式关闭自动 fallback；实测 37 次、残差 `7.263e-9`、DC 电压 0.9491–0.9499 pu。该证据不接受 `pure_ac` 替代。
- 构建：`build_case2000_acdc()`（src/io/case_builders.cpp:make_ac_sgen（lambda））。

### case2000_acdc_vsc_limit_ncp — ACTIVSg2000 VSC 限值基准 · 2000 AC + 8 DC
- 使用仓库内真实 ACTIVSg2000 数据和按区域构造的八端 MTDC；不是纯 AC 结果，也不接受 case300 surrogate。
- 注册回归中 8 台 VSC 全部进入扩展块、4 台同时触发电流圆，NCP 残差不超过 `1e-9`，pattern 只构建一次。
- 该结果只证明当前 CPU 稀疏 MTDC/PQ-Vdc-Q 生产路径；GFM 证据来自专用并网双机、弱网、OPF/时序/暂态与 OpenDSS 测试，不作为大规模 GFM 或 GPU 加速证据。
- 构建：`build_case2000_acdc_vsc_limit_ncp()`。

### case300/case2000 GFM continuation 数据集
- `build_case300_acdc_gfm_limit_ncp()` / `build_case2000_acdc_gfm_limit_ncp()` 在对应真实 MTDC 限值基准上附加一台非绑定 GFM Norton 块。
- 注册测试先求原 MTDC 工况，再以其 `Vm/Va/Vdc` 作为公开 `InitialState` 求解 GFM 扩展；这是准稳态/OPF replay 型 continuation 证据，不是平坦启动证据。
- 该证据覆盖真实 300/2000 AC + 6/8 DC 的固定稀疏结构；多 GFM 同时绑定仍由小系统测试覆盖，GPU 未实现。

## 备注

- 谐波演示不在案例内嵌谐波源：任意含 VSC 的案例（上述绝大多数）在谐波模块勾选
  「换流器自动谐波源」即自动施加六脉波特征频谱（`HPFOptions.auto_nic_from_vscs` 默认开）。
- EV-交通耦合演示使用独立的 `evpt_demo_cases` 体系（GUI「电力-交通」模块内选择场景）。
- 各模块对案例数据字段的依赖（动态参数/故障率/cap_\*/碳因子/报价等）见
  `include/hacdcpf/model/ac_components.hpp` 与各 `src/` 模块实现。
