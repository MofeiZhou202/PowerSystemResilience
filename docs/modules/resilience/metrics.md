# Web 弹性指标：参考书对应关系与计算契约

定义版本：`book_ch3_2026.2`。本页记录当前实现与数值证据；参考文件为
`reference/弹性电力系统极端事件防御与恢复.pdf`，PDF 第 51–54 页第三章。

## 全流程与书中指标的关系

弹性仿真平台 Web 版已有五步：指标选择 → 场景生成与选择 → 主动防御 → 快速恢复 → 指标输出。
场景生成提供代表场景、故障序列与时序曲线；选择一个场景后进行一次恢复求解。
主动防御仍明确跳过，未新增防御优化算法。指标页引用这次运行的 `run_id`，不再次求解。

| 指标族 | 现有证据与输出边界 |
|---|---|
| 书中灾前 8 项 | LOLP、EDNS、ALRIL、PCFD、PSI、PIN 需要多场景后果与概率；仅生成代表场景并不等于已求解全部后果。GMA 需要区域与灾前/灾中发电；TMTS 需要断面容量、潮流。当前保留 null 与原因。 |
| 书中灾中 3 项 | CLLP 按式 (3.31) 输出最后观测时刻切负荷比例。ATCS 缺断面数据。APDA 严格定义需要区域实际发电；显式同意后可输出系统最大失供功率代理，标记 approximate。 |
| 书中灾后 7 项 | LEDSR、RLRO、t_sp、ARSS 按已知灾害/恢复时间及轨迹计算；RES 使用有来源的重要度加权失供序列，需同意近似。REI 式 (3.37) 的前导负号及分母量纲可疑；RSE 缺停电/修复/运行成本，继续不可用。 |
| 工程扩展 24 项 | 单次运行的电量、供电比例、失供时长、分级损失、故障/修复、拓扑操作和 MESS 统计；不使用书中概率/期望指标名称冒充结果。 |

此前目录只有 18 项且通常仅 CLLP 可用；还存在把时间点合计当作 CLLP、把供电比例面积当作 ARSS、忽略 RES 权重以及 APDA 同意开关未实现的问题。本版修正这些口径。

## 时间约定

- 用户未显式给出 `disaster_end_hr` 时，默认取 `max(fault_sequence.start_hr)`，即最后一次故障发生时刻。这是用户指定的平台约定，不是气象灾害结束时间的观测值。
- `disaster_end_source` 为 `explicit_request`、`last_fault_start_default` 或 `unavailable_no_faults`。无故障且无显式时间时保留 null。
- `last_repair_completion_hr=max(start_hr+repair_duration_hr)` 独立保留。
- `recovery_start_hr` 缺省时沿用灾害结束后第一次供电比例上升或修复数增加的前一观测点，并截到灾害结束时刻。
- Web 指标选择页只选择指标，不再要求填写“指标计算口径”。指标请求省略 `parameters`、`allow_apda_system_gap_approximation` 和 `allow_res_approximation`，由后端使用默认值：t_sp 目标比例 0.9；灾害结束取最后一次故障开始；恢复开始按轨迹推定；APDA/RES 代理授权均为 false。缺数据时保持 unavailable/null。后端 HTTP API 仍支持显式覆盖，供已有调用方使用；以后需要用户输入时再设计相应界面。
- 历史分析文件保留原计算口径及结果用于对比，但重新打开后清空旧覆盖值，下一次 Web 计算走当前默认值。
- LEDSR = 恢复开始 − 灾害结束。t_sp 以恢复开始时的失供功率 S0 为基准，首次满足 `1-S(t)/S0 >= target_ratio`，从灾害结束时刻计时。目标未达到返回 null + censored。
- ARSS = `1 − ∫S(t)dt / ((t_end−t_start) S0)`；RES 将 S 替换为加权失供功率。两者以最后观测点为评价终点，用线性插值与梯形积分；尚有失供时标记 censored，不能宣称恢复完成。恢复恶化时允许负值，不强行裁剪。
- RLRO 保留对书中含糊分母的版本化解释，仍标近似。

## 24 项运行指标及字段映射

运行步字段来自恢复求解的 `DistributionResilienceStep`，HTTP `steps` 添加
`duration_hr` 与 `weighted_shed_mw`；分级顺序为 Critical/High/Medium/Low。
单步表示从 `hour` 开始、长为 `duration_hr` 的完整时段，最后一个时段也计入。
令 T=Σdt，D=Σdemand·dt，E=Σserved·dt，U=Σshed·dt。

| ID（均带 run. 前缀） | 名称 / 公式 | 单位 / 来源 |
|---|---|---|
| duration | 覆盖时长 T | h，duration_hr |
| demand_energy | 需求电量 D | MWh，demand_mw |
| served_energy | 供电量 E | MWh，served_mw |
| ens | 未供电量 U | MWh，shed_mw；非 EENS |
| energy_supply_ratio | E/D | 无量纲 |
| energy_loss_ratio | U/D | 无量纲 |
| mean_shed | U/T | MW |
| peak_shed | max shed | MW |
| minimum_supply_ratio | 正需求时段 min(served/demand) | 无量纲 |
| final_supply_ratio | 最后时段 served/demand | 无量纲 |
| equivalent_outage_hours | U/(D/T) | h，非客户 SAIDI |
| interrupted_hours | Σdt·I(shed > 1e-9) | h |
| below_90_hours | Σdt·I(served < 0.9 demand − 1e-9)，仅正需求 | h |
| weighted_ens | Σweighted_shed_mw·dt | weighted_MWh |
| critical_ens / high_ens / medium_ens / low_ens | 各级 shed_by_priority 的时段积分 | MWh |
| peak_active_faults | max active_faults | 故障数 |
| repaired_faults | max repaired_faults（累计数） | 故障数 |
| peak_islands | max island_count | 岛数，不等于孤立节点概率 |
| switch_actions | Σswitch_actions（每步动作数） | 次 |
| mess_energy | 求解器 mess_energy_delivered_mwh | MWh |
| mess_distance | 求解器 mess_travel_distance_km | km |

缺失可选序列返回 unavailable；零分母返回 not_applicable；未知值不填 0。
仅点采样的 C++ 输入按相邻点梯形积分，不推断末端时段；时长阈值统计要求明确的 duration_hr。
时刻重复/倒序、非有限值、负功率、供需失配、重叠时段均为 invalid。
非可行或未完成求解的 artifact 不得产生可计算指标。
指标为系统级标量，分级数组使用明确的优先级次序，不涉及跨 AC/DC 裸 ID 合并。

## 实现、界面与接口

- 模型/计算：`include/hacdcpf/resilience/resilience_metrics.hpp`、`src/resilience/resilience_metrics.cpp`。
- 目录统一来源：`resilience_metric_catalog()`；版别 profile 与 `/api/session/resilience/metric_catalog` 都序列化这份目录。
- 结果：`POST /api/session/resilience/metrics`，42 项中仅返回所选项；旧 run eviction/model revision 拒绝规则保留。
- 界面：`web/js/core/resilience_portal.js`，支持“选择本次可算指标”、中文名称、来源、百分比显示、状态汇总、仅看有值项和 JSON/CSV 导出。CSV 保留原始数值与单位，界面百分比不改变后端值。
- `web/js/core/accessibility.js` 跳过工作台自管 tablist，避免重复处理方向键及重置 aria-selected。
- 普通恢复可行性不等于动态安全；近似、截尾、默认时间来源与缺失依赖均保留在输出证据中。

## 可复现数值核验

注册测试 `test_resilience_metrics` 使用三个非等间隔时段：起点 [0,0.5,2] h，
时长 [0.5,1.5,1] h，需求均 100 MW，供电 [50,80,100] MW。
预期 T=3 h，D=300 MWh，E=245 MWh，ENS=55 MWh，供给率=81.6667%，
失供累计时长=2 h，等效停电=0.55 h。关键/高/中/低损失为 5.5/11/16.5/22 MWh，
加权损失=110 weighted_MWh。对全部 24 项逐项断言，并覆盖缺字段、无负荷、无效时间、失败运行与近似授权。

真实 API 对照扩展在已注册的 `tools/resilience_edition_e2e.py`：恢复步骤独立积分、
与求解器总量核对、四级加总、默认/显式灾害结束时间和非法时间顺序。
注册 Chromium `tests/e2e/resilience_edition_gui_e2e.mjs` 覆盖推荐选择、24 项显示、
后端比值格式、null 筛选、CSV 原值与不重复恢复请求。实际执行状态见开发状态页。

执行结果：Release 单测 10 用例 / 961 断言、版别测试 6 用例 / 1458 断言、真实 HTTP E2E
及桌面/手机 Chromium 版别矩阵均通过。真实 33 节点 4 h 启发式恢复得到 27 项 computed、
1 项 approximate、13 项 unavailable、1 项 not_applicable；这是该算例的结果，不承诺任意
场景均有同样的可用数量。实际结果页检查无脚本错误，1440×1000 与 390×844 无页面级横向溢出。
