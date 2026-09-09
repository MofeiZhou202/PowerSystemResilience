# 南方区域日前执行模型与边界契约

## 周月任务的跨日状态继承

当前契约对应`src/market/market_operation.cpp`中的`apply`、`daily_window`、
`carry_from`、`step_market_operation`。每个场景独立顺序推进；第一天取建任务初态，
以后取前一天SCED实执行末态。数组0..95是96个15分钟实际时段，96/97为次日峰谷参考，
均不参与实际状态推进。实体按表内稳定`id`匹配，不能按结果vector position承接。

令k为日末相同开停机状态向前连续出现的时段数，1<=k<=96，则
`initial_state_minutes_next = 15*k + initial_state_minutes`，仅当k=96且当日日初状态
等于日末状态时加上旧时长，否则为15*k。该量是连续开/停机时长，不是累计运行小时。

| 对象 | 次日输入字段 | SCED来源 / 单位 |
|---|---|---|
| generators | initial_on | online[95]>0.5，0/1 |
| generators | initial_power_mw | 开机时power_mw[95]裁至[0,pmax_mw[95]]，停机为0；MW |
| generators | initial_state_minutes | 上述连续状态递推；分钟 |
| storage | initial_mwh | energy_mwh[95]裁至[0,rated_mwh]；MWh，SOC=能量/额定容量 |
| reservoirs | initial_level_m | level_m[95]；m |
| reservoirs | initial_release_m3_s | max(0,release_m3_s[95])；m3/s |
| reservoirs | release_history_m3_s | 前日release_m3_s[0..95]逐项取非负；m3/s |
| dc_links | initial_mw / initial_adjustment | max(0,power_mw[95])；MW；up/down得到1/-1/0 |

数值裁界对应源码，不能代替此前原单位残差审计。生成当日输入时先应用每日覆盖，
再由carry写入上述初值，保证初态由递推拥有。负荷、来水、报价、停运计划、每日SOC
终值等边界仍按每日申报生效。`terminal_mwh`不是继承字段，原SCED仍约束当天第96点
能量等于当日申报终值；相同终值可能令每天SOC初值相同。机组最小开停机/爬坡、水库
守恒/历史时滞在`southern_market.cpp::assemble_model`消费这些初值。

结果`days[d].state_start/state_end`分别记录日初/日末，成功日之后才将`state_end`
写回`job.carry`并增加`completed_days`。无效结果保留失败证据，不推进carry；恢复实验
从被检查日`state_start`重算，不修改原主任务承接。状态源是SCED而非LMP；定价规则
省略储能能量行不影响调度SOC。逐日98点模型不是全周联合最优模型，不能承诺最优跨日
蓄水/能量价值；启动、停机的非空功率轨迹在滚动任务准入时明确拒绝。非市场核电只作
外部计划注入，不含反应堆动态状态。

验证：`tests/test_southern_market.cpp`的“Southern assembly templates preserve rolling
objectives residuals and water SOC carry”连续七天对照reference/verify，核对资源、
节点价格、state_start/state_end/carry、目标和残差。最终74项测试/29807断言的构建证据
及2000节点完整七日尚未测量的范围见[性能记录](performance.md)。
用户操作见[周月教程](../../guides/market_simulation_workflow.zh.md)。

## SCED 到 LMP 有序模型派生

独立日前compact SCED在未裁行、无RT/云南辅助服务、无AC安全迭代及安全割时，可按原
pricing分支选择行列，A_price=R*A_sced*C，RHS同序选择，费用和冻结/功率邻域边界按原式
重新计算。共同矩阵系数不重新累加、不缩放、不消元；水库方程全部保留。
仅当原定价策略本就省略时，删除储能能量/终值/循环约束及对应能量列，或优先交易/新能源
约束及短缺列；这不改变SCED的水量/SOC输出及跨日承接。`reference`走原装配，`verify`
独立原装配逐项核对矩阵、RHS、row_lhs、目标、边界、语义和结果索引。其他路径自动使用
原装配。派生模板不进入跨运行缓存，双定价完整对偶核验规则不变。
实现为`src/market/southern_market.cpp::derive_lmp`；验证入口为`[lmp_reuse]`、
`[lmp_reuse_gurobi]`、覆盖交易费与非紧凑资产分支的`[lmp_reuse_branches]`、
七日`[assembly_cache]`及`tools/market_validation/probe_market_scale_price.cpp ... derive`。
完整计时和分解诊断见[性能记录](performance.md)。

## Deterministic Pricing

For Gurobi pricing LPs with at least 1000000 ordered columns,
`ordered-lp-barrier-8-v2` fixes Method=2, Threads=8 and Crossover=0; the fresh
environment is reset to default parameters (whose Seed default is 0) rather than
fixing Seed explicitly.
Two fresh environments solve the immutable LP concurrently, with at most one
large pair active per process. Both full original-unit duals still must match
exactly. This changes the representative policy on degenerate LPs; historical
v1 prices are not promised to match. Each solve has the requested soft optimizer
budget; `runtime_sec` is pair wall including verification and queue wait, while
repeat wall overlaps primary wall and must not be added to it.

For smaller LPs and HiGHS, `ordered-lp-dual-simplex-v1` fixes pricing to a fresh single-thread dual
simplex solve (seed 0). Gurobi resets environment parameter overrides to defaults,
then uses Method=1, Threads=1, OptimalityTol=1e-8 and the configured time budget;
HiGHS uses simplex_strategy=1, parallel=off, presolve=on and seed=0. Native market
pricing continues to use HiGHS. Requested `gurobi_method`/threads govern SCUC/SCED
dispatch only; the LMP pricing LP always uses the fixed size-based policy above
(single thread in the v1 path), regardless of the requested dispatch algorithm.
The LMP stage reports its effective algorithm and `price_consistency` policy.

For the unchanged ordered LP min c'x subject to Ax<=b, Ex=d, l<=x<=u,
an optimal face can contain several row multipliers y. This policy selects a
repeatable algorithmic representative; it does not prove that the LP has a unique
mathematical dual or require different backends/versions/platforms to agree.
No epsilon costs, rounding or cached prices alter c, A, E, bounds or prices.
Both fresh solves use the same Build. Row multipliers are lifted/unscaled before
comparison, with y_original=S*y_scaled for row scaling S. Compare every row
multiplier exactly, including reserve rows; net bound reduced costs c-A'y-E'z
are consequently the same. The exported node price retains the original formula
(balance+reserve_up-reserve_down)/weight_hr, in CNY/MWh.

`lmp.price_consistency.passed` requires two optimal statuses, finite full row
vectors with exact equality (tolerance zero), original-unit primal residuals
<=1e-6 and objective difference <=1e-6. Failure makes `prices_valid=false`, all
exported LMP values null and the main result `lmp_failed`; settlement remains
ineligible. A feasible schedule does not override this gate. Missing duals,
timeouts and nonfinite values fail it. The sequential v1 repeat uses the remaining
stage budget; the concurrent v2 pair uses the per-solve budgets described above.
Solver time limits cannot strictly bound import/audit wall time.

The operation/forecast summary retains the same `price_consistency` object.
`repeat_wall_sec` and `repeat_solver_timing` separate the extra solve; primary
`solver_timing` retains the first solve only. For sequential v1, `runtime_sec` sums
both solver calls; concurrent v2 records pair wall. `solve_wall_sec` includes
verification. Unexecuted checks have no valid
price. Historical results without this object are displayed as unchecked.
References: Gurobi Optimizer parameter reference, Method/Threads/Seed; HiGHS
options reference, simplex_strategy/parallel/random_seed; implementation
`solve_once`, `solve`, `stage_result` and `southern_price_consistency.hpp`.
Regression and measured evidence are recorded in [performance.md](performance.md).

## Certified Integer Repair

`execution.large_mip_strategy=auto|reference|certified_repair` defaults to auto.
Auto attempts repair only for compact Gurobi SCUC with >=1000000 columns and
positive requested gap; reference preserves the original MILP path. Explicit
certified_repair admits small compact Gurobi fixtures for cross-validation and
requires positive gap. Real-time and already coupled ancillary models are excluded.
Admission additionally requires a nonempty binary set and no general integer
columns. LP relaxation consumes at most30% of the configured optimizer budget and repair
at most40%; failed admission uses the remaining soft budget for the original MILP.
An LP relaxed barrier status alone never proves market feasibility or a lower bound.
When the certificate box cannot be constructed (a nonpositive-susceptance or
phase-shifting branch defeats the network bounds, or a column stays unbounded),
the repair is declined and the same fallback applies; this is an applicability
exit, not evidence about the market model.

The independently rounded Lagrangian bound, finite optimal-representative box,
fixed integer repair and numerical assumptions are derived in performance.md.
The original matrix, objective, integer, water and SOC constraints remain in the
1e-6 original-unit audit. `gap_certificate` reports acceptance, bound/upper CNY,
actual relative gap, relaxation/repair/certificate seconds and fallback. Accepted
solutions use `Optimality gap reached`; a positive gap is not exact optimality.

SCED reuse is restricted to the same call and requires exact CSC matrices, costs,
RHS, row lower sides, column names/order, constant objective, tighter variable
bounds, no new integer columns and a fresh full SCED residual check. It is only
attempted after an accepted certified-repair SCUC, whose Lagrangian bound is the
retained certificate. It marks `reused_from=scuc`, `matrix_cost_rhs_comparison=
exact_match`, `bound_subset=true`. Any mismatch invokes the ordinary SCED solver.
No previous run, day or changed water/SOC boundary can supply a cached certificate.
For derived SCED that passes the reuse gate, its stored preceding solution map
is moved to the next stage without rebuilding identical entries. All original
SCED audits still execute. Optimized assembly supplies the previous insertion
position per variable family as an ordered-map hint, with standard comparison
fallback whenever names are out of order. Solver column/row order and arithmetic
are unchanged. Reference assembly omits hints; verify compares full maps/matrices.
The operation summary carries the same `gap_certificate` object. The failed
network_bounds experiment is not a supported execution option.
Completed JSON results and entity rows transfer ownership at their final use.
The single-day HTTP route serializes under the same revision lock before moving
the result into `southern_latest`; stale results are returned but not stored.
No response fields or complete time-series vectors are omitted. Reload/export
continue to read the saved result; this is independent of numerical caching.

Candidate generation may first project ordinary zero-minimum-power storage onto
its original mode/SOC/terminal/cycle rows, and scale nonnegative controllable
load reduction to its authored bus and daily energy upper bounds. These only
restrict the candidate LP; the independent lower bound uses the complete
unrestricted original model. They are heuristics within the requested gap, not
claims of identical physical trajectories to another gap-feasible solution.
`storage_candidate_assets`, `load_candidate_assets`, projection seconds and
separate `relaxation_solver_timing`/`repair_solver_timing` expose this scope.

SCED can be derived directly when all commitment costs and regulation awards
are exactly zero, trade fees match, and no RT/ancillary/row-pruning semantics
apply. Only predecessor-frozen columns change; matrices remain unchanged.
The original LP comparison and feasibility gate still run. `assembly_template`
marks `derived_from=scuc`; verify additionally assembles the independent original
SCED and compares all coefficients, metadata and audits. Reference mode keeps
the full assembly. Empty zero-duration commitment windows are skipped only in
the optimized assembler; positive windows retain their original ordered rows.

本文件记录 `southern_market` 的执行解释、字段归属和验证门槛。
性能归因、Hourly Storage Mode 等价推导及 Paired Recovery 并行协议见[性能契约](performance.md)。
规范基准为正式附件第 2.4、2.6 节；原式和歧义见
[逐条比较](southern_rules_comparison.md)。执行解释不是发布机构勘误。

## Recovery Trigger and Timing

运行模拟恢复策略新增 `config.explain_trigger=anomaly|always|manual` 和
`config.recovery_pricing=dispatch_only|full`。新 GUI/forecast 默认 anomaly，
缺省 trigger 的旧配置保持 always；缺省 pricing 为 dispatch_only。
anomaly 在有效 SCED 的 96 个已实现点检查缺额/富余/越限和 >1e-6 MW，
未触发在 `day.cause_analysis.status` 显式标记 not_triggered，不能解释成恢复差值为零。
关闭 explain 或 manual 为 not_requested；主解失败为 unavailable。
完成/部分失败/无改变因素分别为 completed/partial_failure/no_interventions。
恢复证据 `pricing_scope`、`prices_valid` 明确范围，dispatch_only 无 lmp 阶段。
主出清的 LMP、SCED 水量/SOC 递推和残差验收均不改变。

`POST /api/session/market_operation` 的 `action:"explain"` 接受 run_id、整数 day、
pricing（缺省 dispatch_only）；`/api/session/market_forecast` 额外要求整数 scenario。
使用历史 day.state_start、原当日变化和原下一日预测，不改主结果、统计、completed_days、
carry 或运行状态。忙碌/旧 run_id/边界 revision 拒绝；未完成日、无效范围、取消任务拒绝。
operation GET 返回 `recovery_policies:true`；GUI 对未声明该能力的旧服务隐藏新策略控件、
禁用补算并省略新增配置字段，继续兼容旧的 explain 开关。
`manual_wall_sec` 单独记录，不回写原逐日 `execution_timing`。
每次恢复执行另返回 `recovery_execution`：workers（1或2）、experiments、
请求/实际生效线程数、wall_sec 和范围说明。双 worker 并行仅在 Gurobi 后端、
不超过118母线/128机组/16储能/24水库、至少4核且每解线程不超过核数一半时启用；
日内实验独立并行，跨日推进永远串行。并行协议的推导与预算见 performance.md。
界面可为所选历史日补算供需归因或含 LMP 完整链，并显示触发状态和恢复范围。

每阶段新增 `solver_timing`：Gurobi environment_sec / model_import_sec /
optimize_sec / result_extract_sec（含模型释放）；primal_start_sec 是独立初解成本。
其他后端为 null，presolve_sec/search_sec 未单独测量为 null。
这些是内部完成后计时，不是实时进度；optimize 仍含延迟更新、预处理、根节点和搜索。
推导、预注册预测与数值证据见 performance.md 的“原因分析触发与恢复定价隔离”。
条件 LMP 不保证唯一：同一 IEEE118 定价矩阵由现有 auto / dual_simplex 返回不同
最优对偶，最大价差6.64876 CNY/MWh，目标/残差均通过原门槛。规范化价格选取未实现；
不能由 SCED/物理承接一致推导价格逐 bit 一致，详见性能契约的失败门槛与复现证据。

## Yunnan Ancillary Coupling

独立辅助服务入口`run_yunnan_ancillary_market`复用本引擎，SCUC后固定u/稳定状态/一次调频，
进行小时AGC性能排序，再由SCED预留二次调频及选择水电允许区间，LMP固定二次分配与区间。
原入口仍消费外部预安排，不自动启用云南规则；配置不混入南方边界schema。
完整规则提取、公式、API/GUI字段、反例及IEEE118验证见[云南辅助服务契约](yunnan_ancillary_markets.md)。
原文档中“振动区未覆盖”适用于原始南方/周月引擎，不能扩大到新增辅助入口的静态允许区间；
后者也不覆盖水头依赖区间、跨区轨迹或AGC动态安全。
交流反馈重算每轮清除上一轮SCED/LMP可行状态，防止新一轮失败却继承旧的成功标志。

## GUI Navigation Audit

当前界面按模型拆为南方五页与通用 AC/DC 五页，原先混杂的八入口和五步条不再并存。
[操作教程](../../guides/market_simulation_workflow.zh.md)提供六条实际任务路线、按钮顺序、
成功标志和错误排查；页面“分步教程 / 操作手册 / 本页帮助”均可直达。

| 页面 | 输入及结果归属 | 拓扑更新 owner |
|---|---|---|
| 运行模拟 | 手工 operation 或独立 forecast 场景任务 | `marketOperation` |
| 边界与单日出清 | Southern 边界编辑与单日结果 | `marketBoundary` |
| 对比试验 | 独立 study，内含五环节视图 | `marketStudy` |
| 云南调频 | 独立能量/调频与本页账本 | `marketAncillary` |
| 实时市场 | 独立实时配置与滚动批次 | `marketRealtime` |
| 通用主体、预测、出清、安全、结算 | 工程电网和 generic 市场结果 | 工程画布 |

实现位于 `web/js/core/market_navigation.js`、`market_canvas.js`、各市场模块、
`web/js/app.js`、`web/index.html` 和 `web/css/style.css`。异步返回仅更新本页缓存，
激活页面时清除旧画布并恢复当前 owner；预测与手工运行共用运行页上下文。
导航不提交计算，不自动标记阶段完成，也不建立跨市场产品衔接。
初始窄屏收起拓扑，表格在本区域滚动；首次直达市场页不弹出工程建模引导。
调频/实时 404 返回中文服务版本提示，避免空响应触发英文 JSON 解析错误。

验证：八套直接 Node E2E 通过，名称及命令见[开发状态](../../overview/development_status.md)。
新增 `market_workflow_e2e.mjs` 检查十页 × 1440/768/390 px、十条刷新直达链接、
真实两节点出清、帮助文章、手机拓扑开关以及延迟返回隔离；证据在
`output/market-workflow/`。既有回归同步到独立页面，保留草稿、修订冲突、求解状态、
取消续算、曲线/Canvas 联动等断言。本轮未改数学、后端或持久化，也未重新编译 C++。

### GUI Request Activity and Rendering

`web/js/core/market_activity.js` 统一南方单日、运行、预测、试验、调频与实时页面的 JSON 请求。
以下反馈全部与数据来源对应，不表示求解器内部百分比或预计剩余时间：

| GUI 信息 | 来源与单位 | 有效范围 |
|---|---|---|
| 本次已耗时 | 浏览器 `performance.now()` 差值，秒 | 含排队、建模、计算、传输及解析，不等于 solver runtime |
| 场景 / 日 / 已完成日窗 | 发出的 `scenario/day` 与已返回 job 的完成计数 | GUI 转成 1-based；完成量不把失败当成功 |
| 服务端有计算在执行 | `GET /api/session/status` 的 `busy` 布尔值 | 仅会话占用，不能定位 SCUC/SCED 或证明收敛 |
| 已接收结果 | XHR ProgressEvent 的 `loaded/total` 字节，显示 MiB | 只有服务器返回长度时才显示总量，无伪造完成比例 |
| 响应已收到 / 连接中断 | HTTP/JSON 成功或网络错误 | 不覆盖求解结果状态；错误后不重试修改请求 |

每秒刷新耗时，状态请求间隔至少 2.5 秒且单次限 2.5 秒；仅有本页未结束请求时查询。
结束后停止定时器。重载后已有其他客户端计算仍由任务原 `busy` 提示，
本轮没有服务器任务推送、逐阶段日志或跨刷新计时持久化。
重复 GET 仅在同一修改代次的请求重叠时合并，不缓存过期结果；POST 从不合并或重试。
运行入口合并重复 load，预测只在首次进入、显式重载或边界修订变化时加载。
周计划总览/热力图与预测图在可见时才渲染，清空结果会取消待渲染回调。
日窗内模型和数值结果未作裁剪或修改，导出仍为原完整任务。

实测同一 135,439,395 字节 IEEE118 预测任务：首次打开原先 4 次相同下载
（541,757,580 字节），优化后 1 次（减少 75%）；运行步骤中的隐藏周计划
SVG 从 18 个降至 0。只读复现工具：
`node tools/market_validation/profile_market_gui.mjs --base http://127.0.0.1:8101`，
当前结果在 `output/market-activity/profile.json`。此测量不证明求解时间缩短；
每次完整响应仍约 135 MB，服务端历史结果分页/增量传输尚未实现。
`market_activity_e2e.mjs` 验证首次请求数、真实单日出清、延迟响应下计时/暂停、
会话心跳失败、连接中断恢复、桌面/手机状态可见性以及结果页延迟绘图。
其余回归结果见[开发状态](../../overview/development_status.md)。

## Fault/Inflow Study

“故障 / 来水对比试验”仅在南方模型的“对比试验”页面展示，五个分析环节共享
该试验的任务与场景选择。它调用南方规则滚动引擎，和通用静态工程市场的行为博弈、
N-1及实时结算任务分开保存。IEEE118仍是 `IEEE118-mixed-v1` 合成边界；
本试验不构成真实市场校准、正式结算或完整规则等价认证。

### 输入与状态契约

`GET/POST /api/session/market_study` 复用预测任务的 generate/step/cancel 生命周期，
但持有独立 session 数据、run_id 和取消状态，不覆盖 market_forecast 或 market_operation。
generate 请求 `{action, revision, config}`；step 请求 `{action, run_id, scenario, day}`，
其中 scenario/day 均为0-based。边界修订或过期run_id返回409；替换工程案例清空三类任务。
`GET ...?export=1` 保留基准边界和生成后的逐日配置；日初/日末状态在结果中保留。

| config字段 | 含义与边界 |
|---|---|
| mode | 可省略；若提供必须是 `study`，导出config可重新生成 |
| operation | 沿用运行模拟配置；horizon新增 `day`，另支持week/month；month从1日开始 |
| operation.posthoc_ac_audit | 布尔，是否对该日SCED执行98点交流事后复核；不修改计划 |
| inflow_scales / bid_scales | 非空、无重复的0..10倍数数组；报价倍数作用于发电及储能 |
| faults | 非空对象数组；与两个倍数数组做笛卡尔积，总数不超过64 |
| faults[].name | 非空唯一名称，不超过256 UTF-8字节 |
| faults[].generator_outages / branch_outages | authored稳定ID数组；重复/不存在ID拒绝 |
| faults[].first_day / last_day | 故障起止日，0-based，位于运行日历内 |
| faults[].first_slot / last_slot | 每个故障日重复使用的闭区间，0..95；区间外恢复基准状态 |

所有场景从相同基准初态独立出发，各自按96点末态携带UC、SOC、水位和梯级历史下泄。
故障转成 `boundary_overrides` 中设备的 `available=0`，不缩放容量、删除机组或网络约束。
来水/报价倍数叠加operation.days基准倍数，仍遵守每日first_slot/last_slot和单值报价口径。
线路与机组故障覆盖采用独立时段区间，不会把天然来水倍数意外限制在故障时段。
末日仍有D+1峰谷两点，故障若不延伸到次日则恢复该日基准可用性。
冲突覆盖和must-on/检修冲突显式拒绝；不自动修改作者约束以使场景可行。
恢复参照由共同的operation.days生成，不接受另设reference_days。
`explain=true` 时，使用既有单因素恢复重算；故障覆盖组整体恢复，属于条件敏感性，
不是唯一原因，也不是全周反事实。确定性组合不输出故障概率或置信区间。

### 分析与账本公式

`step_market_operation` 对有效计划添加 `days[d].analysis`；源函数为
`src/market/southern_market.cpp::analyze_southern_market_result`。
交流复核从SCED完整98点重建已有 `observe_ac` 控制量，执行一次 `audit_security`。
不生成纠正割，不把诊断缺额/富余注入交流模型；网损由平衡机补偿，并检查其P/Q上下限。
返回 `ac_audit.status=passed/failed/not_requested`、逐点收敛/违反量、网损和原有稳定ID键。
前96点为运行日，最后2点为预测。潮流收敛不等于安全，事后复核也不是全量N-1认证。

研究账本仅支持无external_schedules、无dc_links的封闭AC线性市场，其他结构返回
`unsupported`及原因。缺少有效LMP时返回unavailable。令h=0.25小时、λ为LMP、
P为SCED计划，所有求和只用前96点：

```text
L = sum_b,t lambda_b,t * (load_b,t - DR_b,t) * h
G = sum_g,t lambda_bus(g),t * P_g,t * h
S = sum_s,t lambda_bus(s),t * (Pdis_s,t + Pcharge_s,t) * h
D = sum_b,t lambda_b,t * (deficit_b,t - surplus_b,t) * h
R = sum_l,t (lambda_to(l),t - lambda_from(l),t) * flow_l,t * h
residual = L - G - S - D - R
DR_compensation = sum_d,t compensation_bid_d,t * reduction_d,t * h
```

充电功率Pcharge为负。R由有向线路潮流独立计算，不用收支差额补平；D为显式虚拟
诊断账户，不是向实际客户开票。DR补偿单列，不发明分摊规则。逐时最大残差和总残差
均须小于 `max(1e-3 CNY, 1e-8 * gross_absolute_energy_cashflows)`。
返回 `settlement.status=conditional/balance_failed`、periods和按实体类别+稳定ID索引的
accounts，电量MWh、收入CNY；储能净收入可为负，DR为削减补偿。
`formal_settlement_eligible` 恒为false：无合约、计量、辅助服务、实时偏差或成本分摊，
合成报价不是实际成本，不能把收入称为利润。

### 界面映射与曲线

| 数据源 | 展示与校验 |
|---|---|
| job.scenarios[].config / fault / inflow_scale / bid_scale | 场景、同日对照、故障时段；边界预览经服务器校验 |
| day.stages | SCUC/SCED/LMP真实状态；无有效解不绘成0 |
| day.nodes / lines / resources | 当前与对照的缺额、越限、节点中位价、消纳率、水电功率曲线 |
| day.analysis.ac_audit.periods | 交流违反项计数、中文物理量及MW/Mvar/MVA/pu单位 |
| day.analysis.settlement | 逐时账户现金流、所有资源账本、守恒残差、非正式结算标识 |
| day.counterfactuals | 单因素恢复变化；未执行时明确不能由曲线推断唯一原因 |

切换场景/日期/时段和点击对比曲线同步Canvas；Canvas时段选择和播放回写试验选择器。
“查看该场景全设备运行计划”投影到已有16指标热力图/CSV/设备曲线，不覆盖手工任务。
详情数值表限制滚动高度。单日节点电价图改为全节点价格范围、全部节点中位价和一个
可选择节点；取消前80条截断。中位价采用排序后中央两值平均，节点价格范围不是
输入不确定性的置信区间。对比曲线最多两条，不把18组曲线叠在一起。

### 推导、成本与验证

模型复用 `make_market_operation/step_market_operation`；场景编排在
`src/market/market_forecast.cpp::make_market_study`。预测：关闭explain时，K场景×D日
执行KD次完整SCUC→SCED→LMP，变量/约束不因编排减少；交流复核额外98KD次PF。
研究账本时间/空间 O(96×(节点+线路+资源数))，采用JSON/CPU顺序遍历；不承诺加速。
固定验证门槛：每个日窗输出96运行点和2预测点；全部场景初态相同；停运功率误差
小于1e-6 MW，来水覆盖误差小于1e-8 m3/s；独立账本测试小于1e-4 CNY，生产门槛如上。

Release `test_southern_market` 当前45用例/25804断言通过；新增 `[study]` 2/1874，
同范围ASan/UBSan通过（关闭泄漏检测）。解析case检查24小时100MW电量付款及零线路租金，
并主动破坏出力验证balance_failed；demo检查故障区间与完整来水倍数，避免相互混用。
注册 `market_study_e2e` 使用实际Plotly、IEEE118混合案例、故障×来水×报价组合，
检查逐日状态、账本、故障出力、Canvas和桌面/移动布局。最新结果与仍待闭环项目见
[开发状态](../../overview/development_status.md)，证据位于 `output/market-operation/fault-inflow-study/`。

初次18场景（3故障状态×3来水×2报价）实际完成18个日窗，用时102.803秒；每次
出清约5秒，98点交流事后复核额外执行，不包含在day.runtime_sec（出清耗时）内。
18组均有效出清，全部交流复核失败；无有功缺额/越限，不能据此认定交流安全。
最大账本残差5.93e-10 CNY，低于预定1e-4门槛。
正常枯水场景98点交流PF均收敛，但机组有功上限最多超107.10MW、无功下限超38.82Mvar、
线路首端容量超64.34MVA、电压下限差0.00640pu。此处是合成案例物理缺陷的暴露，
不改限额掩盖失败；Native在新混合案例的已知根LP失败仍未修复。

七日联合故障（线路113、机组1，第2–4日的41–44时段、来水0.5倍）也通过了逐日
状态严格相等、停运出力和配对恢复检查，保存为week.json。18日场景独立账本复核
另用节点/线路/资源向量重新计算，不依赖生产账本输出的residual字段。
单日/周复用统计输出中沿用的 `week_*` 字段在study模式表示所设日历范围；
概率模式仍固定7日，未改变原概率统计口径。
导出重放测试发现并修复raw基准缺少显式solver默认值的问题；现在首次生成即持久化
解析后的solver_options，重放快照完全一致，不改变求解约束。构建使用macos-release
及macos-asan-ubsan已有object/link规则；主仓库HEAD `8b93145bf4f5`，依赖HEAD
`e6c932e5f8a4`，两者均为脏树。未修改依赖pin，也未修改MIPSolvers源码。
本轮未执行study多场景整月或2000节点批量试验；较大任务的JSON内存和浏览器吞吐仍需测量。

## 四步运行流程与缩减研究案例

运行模拟入口现在按“选择算例 → 设置市场边界 → 运行仿真 → 结果与原因”
显示当前步骤。选择案例后进入边界页；手工模式确认每日边界后进入运行页；
预测模式生成七日边界成功后进入运行页；逐日计算结束或暂停后进入结果页。
完整条款、设备逐时覆盖、PTDF 和求解器细项折叠显示，原始字段和校验仍生效。
顶部步骤可返回编辑；选择新案例重置旧每日草稿，防止停运 ID 跨案例串用。

`POST /api/session/southern_market` 新增 `action=activsg2000_hydro`，
`thermal_limit` 为 0..544 整数，默认120；其他 action 不接受该字段。
仅合成研究导入可缩减火电：按原始 Pmax 降序、ID 升序选取，保留单机参数和
稳定 ID，删除其余机组；不缩放负荷、不聚合容量。新增资源 ID 仍按完整原始
机组集分配，避免不同缩减案例的水电与水库关联错位。
默认新案例为2000母线/3206支路、1320机组（120火电、720水电、240风、240光）、
180水库、80储能、120可控负荷；原544火电基准和原工程导入保留独立入口。
这是资源假设变化，不能称为原模型等价降阶，也不能直接用目标差异评价算法。

### 缩减与算法对比 RATIONALE

Model/algorithm: 相同98点稀疏 SCUC/SCED/LMP；仅改变合成案例火电集合。
Native 采用已有 NativeBranchAndCutAdapter，原生整数搜索与割，HiGHS LP内核；
显式关闭 strict full-MIP 委托和 auto root pipeline。连续定价仍使用HiGHS。
不是全原生LP算法，也不承诺原生算法更快。
Claim: 新案例火电数由544减至120，其余声明资源保持；默认目标gap从0改为0.01，
显式保存的旧gap不覆盖。只有通过原单位1e-6残差审计的解可进入下一阶段。
Cost model: 装配随T*G及稀疏网络系数规模增长，整数搜索最坏指数级。
Prediction: 删除424*98=41552个开机二进制声明；无固定耗时加速预测，因为
删机组同时改变可行域、拥塞与松弛强度。该变更是可配置研究假设而非等价优化。
Assumptions: 当前导入机组统一为合成火电报价，保留最大容量机组只是公开研究策略，
不是实际南方机组筛选规则；风光水资源及报价仍是合成数据。
References: 本文件紧凑式证明；`southern_boundary.cpp` 导入与
`southern_market.cpp::solve_scaled`；MIPSolvers `BCOptions`算法所有权契约。
Validation: 小型手算目标490000元、LMP200元/MWh；同一多资源demo及2000新案例
分别使用30s、120s每次优化预算，三后端同gap=.01；进程外120s、420s截止。
记录耗时、RSS、实际gap、残差、目标与失败状态；不同线程策略单列（Gurobi4，
本地适配器默认），不作为严格单线程速度排名。外部截止不是模型不可行证明。

复现入口：
```bash
node tests/run_southern_solver_comparison.mjs demo 30 120 -1 output/market-operation/local-demo
node tests/run_southern_solver_comparison.mjs 2000 120 420 120 output/market-operation/local-2000-reduced
```
驱动按后端串行启动独立进程，保留日志、输入规模、资源容量、Git HEAD、主机和
外部截止证据。`native`单次LP无时限，HiGHS定价LP仅有软时限（实测不硬截止），
不把MILP时限当总耗时保证。

## 逐时拓扑与 PTDF

对每个有效拓扑建立支路-母线关联矩阵A（起点+1、终点-1），
D=diag(baseMVA/(x*tap))，c=-D*shift(rad)。模型为
f=DA*theta+c，p=A^T*f，因此B=A^T*D*A、
H=D*A*B_red^{-1}，f=H*p+c-H*A^T*c。
每个孤岛各取一个参考母线，参考列为零；p在每个岛内必须平衡。
停运线路H行和固定项均为零，跨岛交易灵敏度不由单个参考母线跨岛平衡。

`southern_market_ptdf(boundary,period,branch_ids)`接受0..97时段和1..64条唯一线路。
一次稀疏LU分解后按需解所选行，不显式求逆；返回`bus_ids`列顺序、
`reference_bus_ids`、每列同岛`bus_reference_ids`、系数(MW/MW)、
`phase_shift_flow_mw`、线性方程残差、相同拓扑时段和不同拓扑数量。
一个快照中x/tap/shift是标量，因此其投运向量相同即可复用系数；线路限额、
负荷和报价变化不改变PTDF。不同快照必须同时检查端点、baseMVA、x、tap和shift，
不能只按投运向量跨快照缓存。当前查询只在一次调用内复用分解，未实现跨请求缓存。

API `POST /api/session/market_ptdf`接受`revision,config,day,period,branch_ids`，
先生成该日有效边界（含D+1代表点与停运覆盖），再查询PTDF；过期revision返回409。
GUI显示的是当前手工边界草稿，不冒充历史出清结果对应的PTDF。
生产出清继续使用逐时稀疏相角约束，与该仿射PTDF在岛内平衡下等价。
2000*3206的稠密H每种拓扑约51.3MB，若逐时展开约628M系数、仅double约5GB，
还不含优化器索引和复制，故不能假定全量PTDF会使MILP更快。

PTDF RATIONALE: 对称B的转置方程求指定H行，成本为一次稀疏分解及k次回代，
存储O(nnz(LU)+kN)。预测：三节点环网（非连续ID、tap=2、10度移相）按任意
相角构造的潮流应与H*p+f0相差<=1e-6MW；停线成岛后孤岛参考与零灵敏度正确。
`[ptdf]`自动化检查上述恒等式和非法查询；不是交流潮流或N-1安全认证。

### 当前验证证据

2026-09-06，本地M4 Max/arm64/16逻辑CPU/128GiB，缓存Release优化编译，
工作树含本次修改，HEAD由`comparison.json`记录。相同案例、gap=.01：

| 案例 / 后端 | 进程观察耗时 s | SCUC实际gap | 峰值RSS GiB | 结果 |
|---|---:|---:|---:|---|
| 多资源demo / HiGHS | 2.249 | 0.000815861 | 0.121 | 三阶段通过 |
| 多资源demo / Native | 29.153 | 不可用 | 2.219 | 限时无已验证解 |
| 多资源demo / Gurobi | 0.170 | 0 | 0.076 | 三阶段通过 |
| 2000节点120火电 / HiGHS | 380.485 | 不可用 | 15.526 | TimeLimit，审计拒绝返回向量 |
| 2000节点120火电 / Native | 56.178 | 不可用 | 13.410 | Root relaxation failed |
| 2000节点120火电 / Gurobi | 129.489 | 不可用 | 13.268 | TimeLimit，无解 |

进程观察包含启动/装载；demo内部pipeline墙钟HiGHS0.654s、Gurobi0.158s，
不要将进程启动差异当作优化器速度。demo成功阶段残差<=1.14e-13。
2000新案例实际2569364变量、145040二进制、10515386非零；相对旧案例
二进制减少41552，符合预测。减少火电后不能推论在120s内必然可解。
HiGHS的120s内部时限实际371.953s返回，说明该嵌入式适配流程时限非硬截止，
须保留外部进程预算；当前生产HTTP路径未实现进程隔离，仍有超时阻塞风险。
Native根松弛失败只说明本次参数/算法在该矩阵上的适用性未通过，不证明模型不可行。
原544火电Gurobi600s成功结果为不同案例及预算，不与此表声称加速比。

原单位复核拒绝HiGHS返回的残差285.41向量，`prices_valid=false`，其审计目标
0并非有效经济结果，报告和GUI不得作为出清目标使用。三种大模型均没有有效
ΔP、ΔPij或价格统计，不补零。性能目标仍未闭环，后续应定位根LP与初解耗时。

2000节点选取一条线路的HTTP PTDF查询0.609s、残差6.90e-12、单岛，98时段
同拓扑，保存`output/market-operation/ptdf-2000.json`。
优化构建Southern36案例/20953断言全部通过；重编译ASan/UBSan的
`[ptdf],[reduced_fleet],[local_solvers]`3案例/771断言通过（Gurobi关闭）。
四步手工与预测浏览器E2E均通过，覆盖实际加载120火电案例、停线PTDF、
七日/自然月、场景生成、出清、失败/取消/重载、Canvas定位及390px布局。
截图`output/market-operation/workflow-*.png`；新版服务`127.0.0.1:8087`。

## RATIONALE

Model/algorithm: 稀疏混合整数线性 SCUC，固定离散状态的 SCED，以及独立
罚因子和出力邻域的 LMP LP。网络使用逐时拓扑的无损 DC 潮流，其节点平衡
对偶与 PTDF 的平衡区乘子减线路/断面拥塞项等价。水库与储能使用守恒递推。

Claim: 每一已声明的边界必须进入变量界、目标或具名约束；输入完整快照与结果
绑定，拒绝未知字段和无效引用。改变边界可以重现对应最优解变化，不通过更改
报价来模拟负荷或检修变化。解的可行性与交流安全认证分别报告。

Cost model: 在 CPU 上以 Eigen 稀疏矩阵装配，基本规模 O(T(GK+B+L+S+H+D))；
启停历史/轨迹窗口另需 O(TG W)，群约束为 O(T sum(group membership))。
MILP 最坏指数复杂度，不承诺全规模求解时间。LP 由已有 HiGHS 适配器求解。

Prediction: 98 点单母线解析算例，100 MW、200 元/MWh 时，D 日电能费用为
480000 元；增加 1 MW 后运行日电能费用增加 4800 元，LMP 为 200 元/MWh。
目标总额另计显式代表点权重。存储往返效率 0.81 对应双向效率 0.9，
充入 10 MWh 后可回送 8.1 MWh。相同一小时内禁止通过相邻槽充放套利。

Assumptions: 确定性边界、线性报价、常耗水率/库面面积、直流恒定损耗率；
AC 潮流是额外认证。电压/无功和损耗不在电能量 MILP 中。
代表点时间及权重、启动历史、调频预出清容量结果均必须由输入明确给定。

References: 正式规则 2.4.1、2.4.2–11、2.6.2、2.6.3.1–21、
2.6.4.1–18、2.6.5.1–16、2.6.6；公式转写见
`chapters/southern_day_ahead_rules.tex`。

Validation: `cmake --build build/macos-release --target test_southern_market run_gui_server -j4`；
`ctest --test-dir build/macos-release --output-on-failure -R 'Southern|southern_market'`。
解析费用/功率/能量误差 <=1e-6；LP 价格微扰误差 <=1e-4 元/MWh；
未知字段、非法长度、重复 ID、冲突检修、无效上下限必须拒绝。
GUI 保存/重载/重置/情景比较及桌面/手机布局验证；ASan/UBSan 同目标验证。
测量若偏离预期超过 50%，先检查公式装配、单位、假设与推导，再改实现。

## 字段台账

手算校验已加入 `tests/test_southern_market.cpp` 的 `[hand_oracle]`：
可控负荷在发电边际成本 100、补偿 50 元/MWh 时削减 20 MW，运行日电量
削减 480 MWh；补偿升至 150 时削减为零。同一水库关联两台机组时，水量
释放按两台机组总出力计算，水位递推与 SI 单位守恒逐点核对。

### 大规模多资源扩展 RATIONALE

Model/algorithm: 保留 ACTIVSg2000 的 2000 母线、3206 支路和 544 台机组，
新增 240 风电、240 光伏、720 水电交易单元（共 1744），80 储能和 120 可中断负荷；
水电约占新增机组的 60%，用于复现水电主导的区域生产模拟规模。
网络及新增资源都是合成研究数据，不代表南方真实电网。原始数据按 Birchfield et al.,
IEEE TPWRS 32(4), 2017, doi:10.1109/TPWRS.2016.2616385 / CC BY 4.0 归属。

Claim: 需求响应满足 `0 <= r[d,t] <= available[d,t]*max_reduction[d,t]`，
`sum(t<96, 0.25*r[d,t]) <= max_day_reduction_mwh[d]`；目标增加
`sum(weight[t]*compensation[d,t]*r[d,t])`，节点实际负荷为预测值减 r。
多个单元同母线削减总量不得超过校正后负荷。LMP 固定 SCED 的需求响应计划；
因此报告的是该需求响应计划条件下的节点边际价。该可中断负荷模型是研究扩展，
并非附件 2.6 原式新增条款；无移峰回补或负荷侧备用重复计入。

Cost model: 新增 98D 个连续变量、98D 上界及 D 个能量约束，O(TD) CPU/存储。
全模型 O(T(GK+B+L+H+S+D)+TGW)，MILP 最坏指数；稀疏模型估计数百万变量/行。
每阶段结果提取后释放建模对象，最多保留一个完整阶段的行和矩阵，
预计建模对象峰值占用从三份降为一份（减少约 2/3，不含求解器和 JSON）。
紧约束明细最多保留 1000 项，同时保留全部约束的计数及最大残差，不影响可行性判定。

Prediction: 单母线 100 MW 负荷、200 元/MWh 发电，补偿 100 元/MWh、10 MW
可中断负荷时，全天削减 240 MWh，发电 2160 MWh；补偿 300 时削减为 0。
大算例结构必须精确达到上述计数，98 点原始输入预计小于 64 MiB 紧凑 JSON，
单次运行峰值目标小于 32 GiB；不预言指数复杂 MILP 能在时限内取得可行 incumbent。

Assumptions: 原始机组技术能力保留，报价/启停参数、时序及新增资源全为合成；
风光预测按容量系数限制，水电带独立库容/来水，储能有 SOC 守恒和日末目标。
保留交流母线并联电导/电纳，线性模型将电导按额定电压计入负荷，交流校核使用 V²。
大规模 benchmark 的 schedule_only 仅验证商业计划，不宣称交流安全认证。

References: 本文执行解释、正式规则 2.4 / 2.6；上述需求响应约束为此研究的明确扩展。
Validation: `build/macos-release/tests/test_southern_market`；
`/usr/bin/time -l build/macos-release/tests/run_southern_benchmark output/southern-market/large`；
结构计数精确匹配，解析能量/费用误差 <=1e-6，完整阶段最大残差 <=1e-6；
时限/无 incumbent 必须在报告中显示失败。GUI 大边界保存重载、分页和桌面/手机截图，
以及 ASan/UBSan 对应单元回归。超出预测 50% 时先复查实现、分配器/稀疏填充和输入假设。

公共数据使用版本化 JSON，C++ schema 是字段、类型、单位、范围、枚举和必填项的
唯一来源。C++ 校验后才允许求解/会话保存，GUI 根据同一 schema 生成编辑控件。
数组均使用显式稳定 ID 引用，位置只作为 JSON 编辑路径，不作为设备身份。

| JSON 类别 | 数值归属 / 单位 | GUI / 回传 / 验证 |
|---|---|---|
| periods | 96 个 0.25 h 日内槽 + 2 个显式峰谷代表点；费用权重、物理时长、连接时间独立 | 数值字段；长度/日内时长校验 |
| areas / buses | 统调与母线负荷 MW；按 2.4.1.2 比例校正，返回原始及有效值 | 可编辑序列；全零母线不能分摊非零统调负荷 |
| generators | 报价、逐时能力/状态、检修、启停历史/曲线、备用资格 | stable id + bus id；冲突拒绝 |
| branches / sections | 逐时输变电检修、正反向 MW 限值、断面成员 | 检修断开潮流方程，断面按有向支路求和 |
| external_schedules | 各类非市场主体及区外受送电 MW | 具名节点注入，不聚合丢失来源 |
| transfers / dc_hubs | 商业成分、物理关口、费用、D-2 日 MWh 下限、调整原因、DC 损耗与调节方向 | 输入和有效量均回传 |
| storage | 正放电/负充电 MW、MWh、往返效率、小时方向、循环数 | 每个单元独立；初值与日末目标独立 |
| reservoirs / groups | 水位 m、面积 m2、流量 m3/s、耗水率 m3/MWh、时滞、日 MWh | 上游历史和调度/物理界分别校验 |
| regulation | SCUC 组合上的调频预出清容量边界 MW | 来源必须明确，容量改限后进入 SCED；不得伪称复现另一份调频市场规则 |
| execution | M1–M4 与 M1'–M4'，A1–A7 执行解释、求解限额 | 全量快照及版本随结果返回 |

原始边界、校正边界、SCUC/SCED/LMP 各阶段状态、目标项、变量及具名约束残差分别
保留。情景比较只在同一数据版本和实体集合上进行；不把单次差分称为统计相关性或因果证明。

## 当前执行解释

`src/market/southern_boundary.cpp:southern_market_schema` 是完整可机读字段台账；
`validate_southern_market` 递归拒绝未知字段、缺字段、错误长度、NaN/无穷、
重复 ID、失效引用、冲突必开必停/检修、水库环路和缺失上游历史。
`src/market/southern_market.cpp:build_model` 装配约束；`stage_result` 独立代回
每个具名行及变量界，返回原单位残差。金额为人民币，所有结果数组为输入实体 ID 对应的
98 点顺序；D 日电量/费用另列，不把峰谷代表点混入 24 小时统计。

1. A1：运行日 96 点固定 15 分钟。D+1 峰谷的起点、持续时长和目标权重由输入指定；
   必须按起点排序。代表点之间的空白不积分电量和水量，爬坡/最短运行时间使用实际起点
   差。该假设只适用于代表点抽样研究，不能代替完整次日物理时序。
2. A2：区域集合、机组群和关口引用均使用稳定 ID。母线有功预测按省区比例校正，无功
   预测保持单独申报。上备用以资格乘容量减机组出力装配；下备用以机组出力减资格乘
   最小出力装配。该消元改变平衡行的乘子，LMP 必须恢复原行乘子：
   `lambda_original = lambda_transformed + dual_up - dual_down`，其中 HiGHS 的
   上界行 shadow 为 `-mu`。该变换有源码注释，不能把经过行变换的 dual 直接称原式电价。
3. A3：2.2.10 进一步给出绝对报价区间的衔接含义，因此将其无损转为递增价格的增量
   段宽，最多 10 段、每段至少覆盖申报可调范围的 1%，允许负价。稳定出力为技术最小出力加中标增量；
   启停轨迹状态与稳定状态互斥，轨迹功率直接进入分解。SCED 固定 SCUC 轨迹与组合。
   这是一项明确执行解释，并非声称自由下标原式本身具有唯一可执行含义。
4. A4：初始状态与已持续分钟数必须申报；温态/冷态门槛为整数分钟，等于门槛进入新状态。
   三种费用、三条启动轨迹、停机轨迹独立输入。事件为二元，零最大启动/停机次数有效。
   当前初始开机必须已完成启动轨迹，处于跨日启动过程的输入明确拒绝；该历史轨迹扩展
   仍是完整生产复现的未闭环项。未提供日前报价限价和分省特殊参数时不能将输入校验
   视为交易申报系统完整校验。
5. A5：`hard` 保留日优先电量硬下限；`penalized_shortfall` 显式加入 MWh 缺口和 M4
   罚项。后者为对目标/约束不一致的研究解释，不能标成原文硬约束。
6. A6：耗水率为 m3/MWh、区间流量与泄洪为 m3/s、面积为 m2。
   每点水位递推 `Z[t]=Z[t-1]+3600*duration_hr/area*(inflow+upstream-release)`；
   `release=P*water_m3_mwh/3600+spill`。时滞以所建模点序为单位；上游历史为从旧到新的
   下泄流量。库面面积和耗水率固定，未建立水头依赖效率。
7. A7：LMP 对负充电用有序的 `[(1+delta)Pch,(1-delta)Pch]`；不可定价资源固定。
   `power_neighborhood_only` 不在 LMP 复制 SOC/循环；`retain_energy_constraints`
   复制能量约束。`omit_unlisted` 不复制未单列的新能源和优先电量限制；
   `retain_sced` 复制。商业关口守恒仍保留。储能/DC 离散方向固定于 SCED 后求 LP 对偶。

SCUC 目标仅含规则列出的报价、最小技术出力费、三态启动费、跨省费及 M1–M4。
SCED/LMP 删除前两项承诺成本；LMP 使用独立 M1'–M4' 和费率。储能输入充电幅值
为非负、输出充电功率为负；小时方向限制使用四点联合约束，循环分子按效率加权。

调频阶段读取每个机组具备来源的预出清容量结果，以 `Pmin+reg_down`、
`Pmax-reg_up` 改限；非稳定/停机机组获分配时失败。本文附件没有调频市场的完整报价与
优化规则，本实现不声称已复现那份独立调频市场。需要基于 SCUC 在线组合生成容量结果的
外部调频服务，须在调用前提供相容结果；不相容结果不能自动改成零。

交流校核逐点重建检修后电网，检查电压、有向有功限值、两端视在功率、断面及同母线
机组聚合 P/Q 能力。网损及平衡机调整量单列；商业出清仍是无损 AC 模型。
发生越限时以 1e-3 MW 前向扰动获得局部灵敏度，生成切面并重跑整个链路；
只有重新计算的非线性潮流通过才可认证。这不是非凸 AC 可行域的全局分离 oracle，
不保证所有可行系统均能在迭代限内找到安全解。无潮流解或无有效切面时明确失败。
研究中选择 `schedule_only` 可得到线性计划和条件电价，但顶层 `feasible=false`，
`status=schedule_only`，不能误读为已经交流认证。

## HTTP 和 GUI

### 预测场景运行 RATIONALE

Model/algorithm: 7 日共同因子预测误差，保存基准 96 点日曲线和逐日确定性覆盖。
因素按 load/wind/solar/inflow/generator_bid/load_bid/line_limit 排序；日预测中心
为基准倍数，概率模式使用 Gaussian copula：z0=L*epsilon0，
zd=rho*z(d-1)+sqrt(1-rho^2)*L*epsilon_d，L*L'=R。Eigen 特征分解验证相关矩阵
半正定。u=Phi(z)，均匀分布用线性逆变换，三角分布用分段平方根逆变换；
限幅正态用 clamp(center+sigma*z,lower,upper)，其端点存在概率质量，不能称截断正态。
支持固定值；区间模式用随机分层覆盖上下界，不假定分布或输出概率。

Claim: 每个场景独立从同一初始状态逐日出清；场景内承接第 96 点状态。
DeltaP_i=deficit_i-surplus_i，统计异常用 deficit_i+surplus_i，避免节点正负抵消；
DeltaP_ij=max(0,Pij-Fmax,Fmin-Pij)，停运线路有效限额为零。阈值 1e-6 MW。
概率分母为已验证场景数，另报告总计划数/未知数及失败视为未知的概率上下界；
场景级独立样本使用 Wilson 95% 区间，不把同一路径 672 点当成独立样本。
均值及 nearest-rank P05/P50/P95 只针对有效值；区间模式只输出样本覆盖统计。
原因复核固定当日初态，恢复为当日预测中心；另输出约束族有效性信息、设备缺额/
线路限额证据和跨完整场景输入输出 Pearson 相关（零方差返回 null，非因果）。

Cost model: S 场景、D=7、K 因素，计算 O(S*D*(1+K)*C_day)，串行调用现有出清器；
采样 O(K^3+S*D*K^2)。每完成场景重新汇总历史，累计统计 O(S^2*D*T*(B+L))，
内存 O(S*D*T*(B+L))；当前实现不承诺大网大量场景的性能。
Prediction: 固定倍率退化为确定性结果；2 个解析路径负荷倍率 1 与 3，容量 200 MW，
周异常样本比例 1/2，周缺额均值 8400 MWh，逐点平均缺额 50 MW；一条失败路径
不补零，异常概率范围扩展为 [1/3,2/3]。均匀采样 4096 场景的均值误差 <0.02，
相关矩阵单位对角/对称/PSD 检查及同 seed 同配置复现。区间输出概率必须为 null。
Assumptions: 日尺度共同误差叠加模板；未拟合实际报价/预测历史，不覆盖节点级独立误差
或 15 分钟随机创新；每日预测曲线按倍数缩放，报价每天常数。区间采样不能证明最坏界。
References: Nelsen, An Introduction to Copulas (2006), Gaussian copula；
Wilson (1927), Probable inference, the law of succession, and statistical inference；
本契约滚动 RATIONALE 及 C++ 标准随机数库/Eigen 自伴矩阵分解。
Validation: 新增 test_market_forecast（生成、解析统计、失败分母、独立报价、恢复中心），
Release 与 ASan/UBSan；注册 GUI E2E（分布编辑→生成周边界→逐日出清→统计→原因），
并回归原运行模拟和南方出清测试。实测不符时先复查分母、恢复中心及概率口径。

### 预测场景接口与统计契约

运行模拟默认打开“预测场景”，手工周/月页仍可切换。先加载已保存南方边界，再设置
七日加末日预安排预测中心、分布/区间、场景数和相关性，点击“生成一周市场边界”。生成不自动
出清；场景下拉与八日表可检查实际倍数。点击“逐日出清全部场景”后每次请求计算
一个场景的一天（包括恢复重算），完成该周后进入下一个场景，日初状态不跨场景。
“采用手工周边界为底稿”保留每日停运、区间与倍数；预测倍数乘在该底稿上。
“恢复每日基准模板”清空每日覆盖。新预测配置需重新生成才生效。

| 输入/结果 | 后端来源与 API 字段 | GUI 与口径 |
|---|---|---|
| 基准预测 | 南方已保存边界 `base`、`config.operation.days` | 基准 96 点日曲线；八日底稿倍数及区间、停运计划，末日仅预测 |
| 因素顺序 | `factor_order` / `marginals[].factor` | 负荷、风、光、来水、发电/储能报价、可控负荷补偿、线路限额 |
| 分布 | `marginals[].distribution` | fixed / uniform / triangular / clipped_normal；区间模式仅 fixed / interval |
| 参数 | `center[8]/lower/upper/sigma` | 倍数 0–10；center 是固定值/三角众数/限幅正态位置和恢复参照，uniform 的均值仍为上下界中点 |
| 联合相关 | `correlation[7][7]`、`temporal_rho` | 潜在高斯相关矩阵，对称、对角 1、PSD；日相关 -0.95…0.95，初日平稳高斯。区间模式强制单位矩阵及 rho=0 |
| 规模/复现 | `sample_count/seed/sampler` | 1–512 场景，uint32 seed；结果保存实际场景，不依赖未来随机库完全一致 |
| 单独报价 | operation `generator_bid_scale/load_bid_scale` | 前者作用机组与储能，后者只作用可控负荷补偿；与既有全市场 `bid_scale` 相乘 |
| ΔPᵢ | `scenarios[].days[].nodes[].delta_p_mw` | deficit-surplus，MW；异常幅值统计 deficit+surplus，残差另列 |
| ΔPᵢⱼ | `lines[].delta_pij_mw` | 等于物理有功越限 `overload_mw`；界面限额乘投运状态 |
| 周异常概率 | `statistics.week_delta_p_peak_mw/week_delta_pij_peak_mw` | 每条完整周路径一次试验，报告 valid/total/unknown、有效样本比例、Wilson95 与未知路径概率界 |
| 时段统计 | `statistics.periods[672]` | 同一日期/时段跨已验证样本的均值、P05/P50/P95、异常比例及未知界；未执行点为 null |
| 设备统计 | `statistics.nodes/lines` | stable ID，完整场景的周峰值、周异常比例及积分；节点正负不能抵消 |
| 相关性 | `statistics.correlations` | 完整场景七日输入平均倍数 vs 周缺额/线路越限积分 Pearson r；少于 3 点或零方差为 null |
| 原因 | `counterfactuals[].sampled_value/reference_value/reduction_*` | 同日初态恢复预测中心；另保留停运恢复、容量/输送证据、SCED 生效约束，1000 行截断显式声明 |

新路由 `GET /api/session/market_forecast` 返回 `defaults/revision/run_id/busy/job`；
`POST` 接受 `generate {revision,config}`、`step {run_id,scenario,day}` 或
`cancel {run_id}`，action 及上述键均在同一 JSON 对象中。generate 校验后原子替换任务，
step 校验修订/场景/日期并使用全局 busy；非法参数 400、修订/进度冲突 409。
GET `?export=1` 附带完整基准边界，报告可独立追溯预测配置和已生成倍数；普通摘要
省略 base，各场景摘要省略 carry，但保留每个已出清日的 state_start/state_end。
任务是服务内存态，浏览器逐日推进；暂停/终止须等待当前日及恢复计算返回。
日内任务失败时，该路径停止推进，下一场景仍可开始；`completed` 表示遍历结束，
不表示所有场景都可行。统计通常在一条路径结束时更新，部分日窗仍可单独检查。

概率 `events/n_valid` 是有效样本条件估计；若有 n_unknown 个未知场景，基于原计划
N 的事件范围为 `[events/N,(events+n_unknown)/N]`。Wilson95 反映完整周独立样本的
抽样误差，不能消除模型错误或有效样本选择偏差。区间模式上述概率字段全部为 null，
表格中的样本比例与分位数仅用于所采样点的覆盖分析，不是置信区间或严格最坏界。
单因素恢复可离开原联合分布支持域，是机制敏感性实验；不能称为真实因果效应。

当前实现：`src/market/market_forecast.cpp`（生成/汇总/推进）、
`market_operation.cpp`（逐日模型与恢复）、`tests/run_gui_server.cpp`（会话 API）、
`web/js/core/market_forecast.js` 与 `market_operation.js`（预测编辑/统计/设备复核）。
以下为八日修正前证据。此前Release 和 ASan/UBSan `test_market_forecast` 均通过 **7 用例 / 12486 断言**，含
停运与必开冲突后该路径失败、其余路径继续的检查。
预测/实测：均匀[0,2] 的均值 1 → 0.981108，方差 1/3 → 0.342838（4096 抽样，
预设误差门槛 .02）；日相关 .6 → .579475（512 场景，门槛 .1）。解析双路径的周缺额
均值 8400 MWh、异常比例 .5、失败后概率界 [1/3,2/3] 均符合预设结果；线路 50/25 MW
限额产生 50/75 MW 越限，峰值均值 62.5 MW。独立报价费用为 49000 与 396410 元。
原南方 Release **21 / 1699** 回归通过。注册的 `market_forecast_e2e`、
`market_operation_e2e`、`southern_market_e2e`、`market_gui_e2e` 全部通过，含两种
预测口径、实际 GUI 编辑/导出/重载、原因恢复、参数冲突及 390px 图表重排。
没有开展真实预测分布拟合、真实报价重放或 2000 节点概率场景性能验证。

多资源演示证据：`output/market-forecast/demo-evidence.json`（包含基准和生成场景），
2026-09-07 起、seed=42、3 个周场景。七种因素三角分布 [0.85,1.15]、众数 1，
相邻日 rho=.5，负荷/风电潜变量相关 -.3；底稿第 2 日负荷倍数 5、第 3 日机组 2
停运且线路限额 .25。21 日窗全部完成；周缺额均值 10962.6607637 MWh，周节点
异常峰值均值 495.0653368 MW，线路越限峰值均值 952.3247118 MW。3/3 场景
出现异常，Wilson95 为 [.438503,1]；这是人为压力条件下的小样本演示，不能作为
真实风险概率。截图 `demo-config.png/demo-statistics.png/demo-causes.png/demo-mobile.png`。

### Gurobi 市场后端

`execution.solver`支持`highs`（旧快照默认）、`gurobi`和`native`，不自动替换；
Native 路径见本文 Native 根割与整数修复两节。
`execution.threads`默认0；非零仅接受Gurobi（最大128）。`time_limit_sec`与
`mip_gap`沿用原范围；Gurobi参数作用于每个SCUC/SCED/LMP优化调用，LP也限时。
时限不包含建模、许可证启动或全部恢复实验，仍不是整日硬预算。HiGHS路径中
SCUC MILP与LMP定价LP（含重复核验的剩余预算）接收时限，SCED连续LP不设时限。
运行模拟`config.solver_options={solver,time_limit_sec,mip_gap,threads}`可覆盖保存边界的
执行设置；配置缺省时从保存边界继承，规范化结果与预测场景导出保留实际选择。
GUI手工页和预测页分别编辑并重载这四项；日前边界页按schema编辑execution。
API `/api/session/market_operation.solver_capabilities`报告本地环境初始化结果，
模型特定许可证限制仍须实际求解确认。Gurobi不可用时返回错误，不回落HiGHS。

RATIONALE: `southern_market.cpp::solve`将同一Build模型交给Gurobi适配器，不改
98点、目标或约束；显式参数构造使LP复用稀疏装配，传递成本O(nnz+m+n)。
修正依赖适配器区间行展开后的Pi索引；仅最优LP返回对偶，限时调度需原约束审计，
无最优定价LP时价格不可用。原模型1e-6容差保持，Gurobi内部可行/整数容差设1e-8。
参考细则2.6、Gurobi C API参数和Status/SolCount/Pi文档、MIPSolvers的`docs/solvers.md`。
预测：100MW解析算例日费用480000元、节点价200元/MWh，误差1e-6；
多资源SCUC最优目标与HiGHS一致（绝对1e-4元加相对1e-5），不要求退化最优解逐变量相同。
带区间行LP手算目标10、原行对偶1/2；极短时限无定价Pi；失效选择和参数拒绝。
验证使用Release `test_southern_market '[gurobi]'`，全部Southern/forecast及注册四项GUI E2E。

阶段`solver/requested_solver/requested_time_limit_sec/requested_mip_gap/requested_threads`
随日结果和恢复结果返回；GUI展示实际后端。可行性/最优性分级沿用前述审计契约。
依赖变更在兄弟仓库MIPSolvers，需与本项目一起重建；不含可商业使用许可证的声明。
目前仅验证本地Gurobi13.0安装和小案例，未完成2000节点大规模求解基准。

实测（macOS arm64、Release，HySim HEAD 308ccc57、MIPSolvers HEAD e003dbb，
均加本次工作树）：Gurobi专项2用例41断言通过；预测/实测为100/100MW、
480000/480000元、200/200元/MWh，区间LP目标10/10和Pi 1/1、2/2，超短时限
返回TimeLimit且Pi为空。Southern全量30/11263、forecast 8/12503，四项GUI E2E
全部通过（34.89秒）；GUI验证真实Gurobi后端、参数保存/重载和预测场景全部阶段。
无Gurobi的ASan/UBSan构建`[solver_options]`为1/9通过，覆盖无库时失败、不推进carry；
不将其描述为Gurobi库的内存检查。两个仓库`git diff --check`与JS语法检查通过。
8085实机多资源周及固定预测周各完成7日；日1 SCUC/SCED目标481998.849107元、
残差<9e-14，配置2线程、每调用120秒、gap=.01。证据保存于
`output/market-operation/gurobi-{week,forecast}-evidence.json`，桌面/移动端截图
`gurobi-config-{desktop,mobile}.png`和`gurobi-results-desktop.png`。单次小模型SCUC
0.169秒仅为该次观测，不作2000节点性能预测；未测试商业分发或其他Gurobi版本。

### 次日联动契约

当前滚动入口每窗实际求解98点，执行、统计及状态承接仅使用当天96点。
`market_operation.cpp::daily_window` 从下一日覆盖、扰动后的96点统调负荷总和
选出峰谷，并按稳定实体ID提取同一来源时刻的所有 schema 固定长度98的时序字段。
这包括风光、来水、检修、备用、机组上下限、线路/断面、外送/非市场计划和
可控负荷补偿价。母线原始比例仍在拼接后按2.4.1.2校正。

RATIONALE:

- Model: `Bwin[d,0:96]=B[d,0:96]`，尾部为 `B[d+1,s_valley/s_peak]`；
  `s_valley=argmin sum_area L`、`s_peak=argmax sum_area L`，两点按实际时间排序。
  同值取首个；全天平坦时取0和95两个不同点。`start_minute=1440+15*s`。
- Claim: 仅改变次日边界会改变尾部输入，并可通过爬坡约束影响当日预安排；
  `carry` 永远来自第95点，不来自第97点。恢复实验固定次日抽样及当前日初状态。
- Cost: 两次既有边界覆盖、96点负荷扫描、每类实体有序ID索引及字段复制，
  O(98F + N log N) 时间及 O(98F) JSON 存储；MILP仍98点，不作求解加速声明。
- Prediction: 次日峰/谷180/60 MW精确进入两代表点；当天输入仍100 MW。
  必开机组爬坡2 MW/min、次日零点最小出力180 MW时，当天末两点至少120/150 MW，
  产生 `(20+50)*0.25=17.5 MWh` 富余；次日继承150 MW。
  100 MW负荷、20 MW可控负荷，D补偿9/D+1补偿90、发电200元/MWh时，
  98点费用 `24*(80*200+20*9)+0.5*(80*200+20*90)=397220` 元。
- Assumptions: 已有线性执行模型及空启停轨迹；峰谷按全区域统调负荷选取是显式
  执行约定，并非声称细则指定了这个唯一选点算法。时长/权重按基准同类代表点继承；
  点间未观测能量/水量不积分。重叠或超出D+1的代表区间由既有校验拒绝。
- References: 细则2.6的T=98、2.4.1.2、2.6.3.11；上述源码及`carry_from`。
- Validation: Release `test_southern_market '[southern_market]'`、`test_market_forecast`，
  注册的四项market浏览器E2E；手算误差门槛1e-6，离散映射逐项精确相等。

接口与GUI映射：

| 字段 | 语义与界面 |
|---|---|
| `config.days` | 周8日、月N+1日；最后一天仅预测，编辑器标记“仅预测”，统计仍7/N日 |
| `config.terminal_forecast_source` | `authored`或`baseline_template`；旧N日/空配置以基准补末日并在limitations明示，非真实预测 |
| `days[d].lookahead` / `preview.lookahead` | 来源日、目标/来源0-based时点、峰谷、MW、时长/权重；预览及结果表显示 |
| `preview.forecast_only` | 最后一日可预览前96点，不执行出清，也不提供该日自己的D+1预测 |
| `marginals[].center` | 8日中心；旧7日中心按第7日值补第8日并提示；采样器版本升为v2-eight-days |
| `stages.*.solution_quality/limit_reached` | 最优容差、限额可行、未证最优、无已验证解、已证不可行/无界、求解或审计失败；GUI展示原状态、gap、规模、耗时 |
| `statistics.complete_scenarios_with_limit/unproven` | 完整周中SCUC/SCED含限额解/未全部证明最优的场景数，GUI与导出同步 |

`southern_solver_status.hpp`识别适配器的精确状态标签，修正`Optimal`大小写误判；
最优证明必须先通过原约束/变量界/整数性审计，再满足optimal状态与gap<=1e-9。
限额解可以继续但保留质量；失败不推进状态、不当作零异常。正松弛及恢复差值均为
所求方案的诊断/条件敏感性，不能证明异常不可避免。

边界范围限制：所有单值物理参数、日终目标及发电/储能标量报价仍按D日整个日窗解释，
尚不支持尾部两点独立标量参数或报价；仅预测末日的标量设备覆盖显式拒绝。
概率模型仍是每日共同倍数，不是外部完整预测
材料接口。Gurobi接入见上节；进程隔离、整日硬预算及2000节点性能证据仍未完成。
`time_limit_sec`在HiGHS路径作用于SCUC MILP与LMP定价LP，SCED连续LP不限时；
Gurobi路径作用于每次优化调用（MILP/LP）；均不能解读为
含建模、定价与恢复实验的日总时限。

验证记录：macOS arm64，`macos-release`预设，HEAD `308ccc57`加当前工作树；
Southern 27用例/11214断言，forecast 8/12503全部通过；重建ASan/UBSan的
`test_southern_market '[lookahead]'` 3/438通过（`detect_leaks=0`、UBSan halt）。
四个注册浏览器测试全部通过（33.22秒）。预测/实测：代表点180/60→180/60 MW，
末状态150→150 MW，富余17.5→17.5 MWh，报价397220→397220元，均在1e-6内。
原旧测试396410元失败经公式复核，差额810元来自两个次日补偿报价恢复，非调整求解器
容差。实际多资源周回放完成7日，末日读取第8日550 MW；导出
`output/market-operation/lookahead-demo-evidence.json`。页面桌面/390px截图
`lookahead-live-{desktop,mobile}.png`；宽表局部滚动，整页无横向溢出。

### 周月运行模拟 RATIONALE

Model/algorithm: 顺序滚动日前 SCUC/SCED/LMP，日窗仍为 96+2 点；仅把第 96 点
实现状态传至次日，代表点不作为实现状态。机组连续开停分钟按 96 点末尾相同状态
长度递推；SOC/水位/下泄历史/直流调整方向按日末值承接。非空启停轨迹目前在运行
模拟入口拒绝，避免未实现的跨日轨迹历史。日末 SOC 目标每天仍是基准边界申报值。

Claim: 诊断模式以 `Pgen + Pnet + reduction + deficit - surplus = demand` 装配，
deficit/surplus 非负，缺额不超过母线负荷，目标加 `penalty*(deficit+surplus)*weight`。
原数值残差继续单列。线路越限为 max(0,flow-max,min-flow)，不把线路松弛或 AC
视在功率越限混同。严格南方入口默认不启用松弛。诊断价格受罚价影响，不作正常市场价。

Cost model: D 个日窗、K 个发生变化的边界因素，串行成本 O(D*(1+K)*C_day)，
每次最多一个模型；报告存储 O(D*T*(B+L+G))。反事实可关闭，以限制昂贵重复求解。
Prediction: 单母线容量 200 MW/负荷 100 MW，某日负荷变为 300 MW 时缺额 100 MW、
日缺供电量 2400 MWh；恢复该日负荷系数后归零。7 天为 672 个实现点，2028-02
月为 29 天/2784 点。原严格模型同场景不可行。物理/统计误差门槛仍为 1e-6 MW。
补充解析预测：100 MW 远端负荷/50 MW 线路限额，改为 25 MW 持续 1 小时，
越限由 50 变为 75 MW，恢复限额使越限积分减少 25 MW·h；150 MW 必开最小
出力/100 MW 负荷产生 50 MW 富余。水库时滞案例下游第一天水位 195.5 m，
次日日末 291.5 m（1 点时滞、上游 100 m3/s、面积 90000 m2、首历史值 50）。
Assumptions: 合成边界，确定性逐日实现，非全周联合最优；不提前知道未来全月水价值。
未出清日不能用零填充；失败/取消停止递推。默认模板逐日复用，GUI 逐日/指定时段
覆盖负荷、风光、来水、报价、线路限额及设备检修，并全量保存来源及配置。
References: 本契约 2.4/2.6 执行解释；新增节点松弛和滚动/配对实验为研究扩展。
Validation: 新增 C++ 运行模拟 tests 和注册 Playwright E2E，验证 7 日、真实月长、
日末状态、水库时滞、缺额/越限及配对恢复差值、非法输入、API 修订冲突和取消；
Release 与 ASan/UBSan。若与预测不符，先复查单位、日窗边界、罚项与残差再修改。

GUI/API 台账：`/api/session/market_operation` 保存 config/base/carry/days，
GET 返回进度、条款边界目录与逐日结果，POST start/step/cancel/preview；前端“运行模拟”页逐日推进，
服务端修订检查和 busy 状态保护任务。结果以 stable id 和 0-based day/slot 索引、
MW/MWh/CNY 单位存储。配对重算固定当日日初状态，只恢复一种当日边界因素；
不能将条件差值解读为统计因果，也不把生效约束直接命名为唯一原因。
手工任务 config 另接受可选 `reference_days`（同日历长度的每日边界数组）作为
恢复参照；预测与研究任务自行派生参照，显式拒绝另设 `reference_days`。

### 运行模拟接口与页面

#### 细则市场边界目录与每日覆盖

原文核对范围已扩展到 2.3（正文11–17页）、2.4（正文17–25页）；PDF页序加3。
2.3.10、2.4.1 的正式第20页已渲染核对。机器可读目录由
`southern_market_boundary_catalog()` 返回，并通过 GET
`/api/session/market_operation.boundary_catalog` 进入运行模拟页面。
目录按细则概念组织15类：统调负荷、母线负荷、跨省跨区送电下限及外来计划、
非市场出力、机组检修可用状态、输变电检修投退运、机组运行约束、新能源预测、
运行备用、设备/断面安全限额、必开必停及机组群、水库运用、水电优化调度、
清洁能源消纳、储能运行。字段 schema 直接引用 `southern_market_schema()`，
单位、整数状态、取值范围与服务端校验同源。实体数为零表示当前算例未配置，
不代表该条款不需要；目录的 `coverage=resolved_inputs` 只表示接收已换算边界。

报价单独归于2.5交易申报；可控负荷补偿、概率分布和区间采样、节点缺额罚变量
明确为研究扩展。原有8个倍数保留为扰动参数，不再被称为细则市场边界的全部内容。

每日 `config.days[d].boundary_overrides` 是可选的稀疏数组，每项为
`{table,id,field,first_slot,last_slot,value,reason}`。`id` 是该实体类别的稳定ID，
`first_slot/last_slot` 为0-based闭区间；时序覆盖仅接受0–95。
修改代表点须编辑下一预测日，直接覆盖96/97会被拒绝，防止静默丢失或双重来源。
数值/整数序列在区间内赋同一值；可用多段表达曲线。标量与文本必须指定0–97，
表示整个日窗。`reason` 非空，随任务配置、逐日边界及导出保留。
只准入目录列出的字段；设备关联、拓扑、机组初始状态、储能初始能量、
水库初始水位/下泄历史等仍通过基准模型和跨日承接管理，禁止每日任意重置。
同实体字段重叠覆盖、未知ID/字段、非法类型或范围、上下界冲突均拒绝新建任务。
每日日窗覆盖批量应用后才做模型关系校验，因此上下限可以在同一请求内成对修改。

RATIONALE（每日覆盖，非求解器改写）:

- Model: \(B_d^{raw}=S_d(O_d(B_0^{raw}))\)，再按2.4.1.2校正
  \(L_{i,t}^{eff}=L_{i,t}^{raw}L_{a,t}/\sum_{j\in a}L_{j,t}^{raw}\)。
  原始覆盖 `O` 优先于扰动 `S`；日初状态独立由上一日第95点承接。
- Claim: 设备级边界可进入同一SCUC/SCED路径；原始母线比例不会在任务创建时丢失。
  `make_market_operation` 校验但不以返回的有效边界替换原始基准。
- Cost: 赋值为O(ET)，T≤98；重叠检测用有序集合，为O(ET log(ET))。
  目前ID校验对各项构造类别索引，另有O(EN log N)代价；每天附加一次完整边界校验。
  不宣称大规模高吞吐；无求解器速度改进预测。
- Prediction: 总负荷200MW、节点原始比例1:3得到50/150MW；首节点改为3后得到
  100/100MW，下一日未覆盖仍50/150MW。100MW解析系统增加40MW非市场注入，
  常规机组出力降至60MW。数值容差1e-6MW。
- References: 原文2.3、2.4.1.2、2.4.11；`market_operation.cpp::apply`、
  `preview_market_operation_boundary`、`southern_boundary.cpp::validate_southern_market`。
- Validation: `test_southern_market '[boundary_rules]'` 与既有Southern/forecast全测试；
  已注册 `market_operation_e2e` 执行实际GUI编辑/校验/出清/重载和原子拒绝。

`POST /api/session/market_operation` 的 `preview` 请求接受
`{action:"preview",revision,config,day}`，返回 `preview.authored/effective/day/state_scope`。
该接口只预览使用申报初始状态的当天边界，不推进或替换任务；实际求解使用逐日承接。
报告 `days[d].nodes[].load_mw` 新增校正后的96点负荷（MW），Canvas直接显示该值，
不再用基准倍数猜测节点生效负荷。日前阶段仍显示 `effective_boundary`。

运行模拟的“设置”进入手工周/月底稿的当日设备级覆盖，提供范围、单位、实体ID、
调整依据和服务端预览。预测模式的“采用手工周边界为底稿”保留这些覆盖，抽样倍数
随后叠加；预测中心恢复实验保留同一确定性覆盖。在手工恢复实验中
`factor=boundary_overrides` 表示联合恢复全部设备级覆盖，不能当作每项独立贡献。

覆盖缺口必须保留：尚未完整实现水位库容/水头/滞时曲线、振动区、固定泄洪计划、
水位升降幅原始约束，完整D+1的96点新能源预测、审批披露与自动气象/燃料/供热换算。
水库当前为恒定库面/耗水率和固定时滞。滚动仅线性有功诊断；无功负荷、电压和
交流视在功率限额不作为本次每日可调字段，仍属于基准交流校核输入。
**目录定义、条款映射和已建模字段闭环，不等于正式细则全部物理/业务模型完全覆盖。**

实测与上述预测一致。Release Southern24例/2035断言，forecast7例/12486断言通过；
其中新增3例336断言涵盖目录全部准入字段的往返、原始/生效比例、非市场出力、
预测底稿及非法覆盖拒绝。GUI证据在 `output/market-operation/rule-boundary-*.png`。
重建ASan/UBSan通过新增3例336断言；4组市场浏览器回归全部通过。
预测场景的每日表另提供“预览”，按当前条款类别展示抽样后的原始/校正边界，
并标出设备级覆盖项数；未将全部边界缩减成概率倍数表。

`/xjtu/#market-operation` 或“电力市场 → 运行模拟”进入市场 Canvas 与运行结果分栏页面。
使用已保存南方边界，页面显示名称与 revision，支持直接加载三种市场算例，
完整机组/储能/可控负荷报价由“编辑完整边界与报价”进入原边界编辑器。
更换市场算例仍不改变 Canvas 工程系统。报价、预测及扩展水库数据均保留原来源声明。

#### 市场 Canvas 身份与时段契约

`web/js/core/market_canvas.js` 在主 `#canvasContainer` 内显示独立市场 SVG，
`app.js::setActiveModule` 在市场模块切换视图。原工程 `Canvas.state`、几何、
设备参数和撤销栈不作替换；离开市场恢复原视图。市场视图隐藏工程编辑工具，
`canvas.js::onKeyDown` 不再将市场按键发送给隐藏的工程设备。
此图是拓扑示意，不提供地理位置或交流安全证书。

| 画布值 | 数据源 / 身份 | 有效性与验证 |
|---|---|---|
| 节点与线路 | `/api/session/southern_market.boundary.buses/branches`，`buses:<id>` / `branches:<id>` | 独立南方交流节点域，不与工程 AC/DC 同号 ID 联结；拒绝同类重复 ID |
| 机组/储能/可控负荷/水库/直流联络 | 边界同名实体数组，`type:<id>` | 完整设备定位菜单；节点详情关联资源；原始基准报价及来源可展开 |
| ΔPᵢ | 滚动 `days[d].nodes[].deficit_mw[t] - surplus_mw[t]`；日前所选阶段 `buses[]` | MW，正缺额/负富余；不替代 `node_imbalance_mw` 数值残差 |
| ΔPᵢⱼ / 功率 / 限额 | 滚动 `days[d].lines[]`，日前阶段 `branches[]` 与 `effective_boundary.branches[]` | MW，有效上下限乘 available；有功限额超限不等于交流热稳超限 |
| 电价 | 滚动 `nodes[].lmp_per_mwh` / 日前有效 `lmp.buses[]` | 元/MWh；无有效价格显示不可用 |
| 日期/时段/场景 | `operationResultDay` / `operationSlot` / `forecastScenario` | Canvas 同步下拉与原控件双向绑定；日前 98 点、滚动 96 点；未来日期可预览，尚未出清为灰色 |
| 原因 | `days[d].periods`、`counterfactuals[].periods`、原有约束证据区 | 显示系统容量/限额证据、同日初状态配对恢复差；不称为所选节点/线路的唯一因果贡献 |

滚动着色仅当 `day.valid` 且报告、当前会话和 Canvas 的 boundary revision
完全相同；日前还要求所选阶段 `feasible`、`schedule_feasible`、非 stale。
未运行、失败、失效与缺失数值显示灰色/不可用，不能伪装零异常。
基准节点负荷乘当日倍数仅是已申报曲线预览，日前实际分配负荷取
`effective_boundary`。周/月摘要未提供的机组详细出力不会从初始值推测。

点击节点/线路或结果表 ID 定位同一稳定实体；“编辑基准边界”定位原编辑器，
编辑流程仍需原有保存/校验。跟随计算在每个日窗返回后跳到最新场景/日期；
播放以 700 ms 递进已选时段并可跨日，末端与离开市场时停止，不伪造求解进度。
平移、缩放、适应与键盘 Enter/Space 选择均可用。拓扑范围由“局部/扩展邻域”
选择器控制：局部为所选设备两跳内至多 24 节点，扩展为 BFS 至多 80 节点，
并标出可见/总数；完整设备菜单仍覆盖全部实体。
可视化只连真实支路，所选支路两端优先纳入，不声称全网同时绘制或真实地理布局。

已注册 `southern_market_e2e` / `market_operation_e2e` /
`market_forecast_e2e` 验证实际点击、同号工程设备隔离、Delete 不修改隐藏工程、
100 MW 缺额红/绿切换、限额收紧后的 50→75 MW 线路越限、时段播放、场景数值
匹配 API、基准设备编辑跳转、2000 节点末端定位和 390px 布局。
`market_gui_e2e` 同时回归。只改 GUI，没有重跑 C++ 数值/卫生器或大规模出清。

| 值 | API / 源码 | GUI / 单位与验证 |
|---|---|---|
| 周/月日期 | `config.horizon/start_date`，`make_market_operation` | 7 天或自然月；月起点必须为 1 日；闰年 2 月为 29 天 |
| 每日覆盖 | `config.days[d]` | 六个倍数 0–10、`first_slot/last_slot` 0–95（界面显示 1–96），停运机组/线路 stable ID 数组；拒绝重复/未知 ID |
| 预测倍数 | `load_scale/wind_scale/solar_scale/inflow_scale` | 同比覆盖指定区间负荷 P/Q、风光预测、本地来水 |
| 报价和线路 | `bid_scale/line_limit_scale` | 报价倍数作用于全天机组分段、储能充放和负荷补偿；限额倍数作用于区间有功上下限及额定 MVA |
| 物理缺額/富余 | `days[d].nodes[].deficit_mw/surplus_mw` | MW，独立于 `node_imbalance_mw` 数值残差；严格模式默认无松弛 |
| 线路越限 | `days[d].lines[].power_mw/min_mw/max_mw/overload_mw` | MW，原线路 ID 与端点，超过有功上下限的量；不是 AC 视在功率判据 |
| 周期积分 | `deficit_mwh/surplus_mwh/overload_mwh` | 前 96 点乘 0.25 h；最后一项是跨线路越限积分和，不是缺供电量 |
| 原因复核 | `counterfactuals[].factor/periods/reduction_*` | 同日初状态下恢复一种因素；差值为原场景减恢复场景，失败显示不可用 |
| 条件节点价格 | `nodes[].lmp_per_mwh`、`diagnostic_prices_valid` | 元/MWh；失败为 null，不补零；受罚价影响 |
| 状态与精度 | `stages`、`valid/status`、`state_start/state_end` | 逐日求解状态/目标/gap/残差；失败日不推进日初状态 |

请求示例（先加载并保存南方市场边界；revision/run_id 必须采用服务端返回值）：

```json
{"action":"start","revision":1,"config":{"horizon":"week","start_date":"2028-02-01","penalty_per_mwh":100000,"explain":true,"days":[]}}
```

`POST /api/session/market_operation` 的 `start` 创建并校验任务，`step` 接受
`{action:"step",run_id,day}` 计算一个日窗及配对恢复，`cancel` 接受
`{action:"cancel",run_id}`。`GET` 返回 `revision/run_id/boundary_name/busy/job`。
`job.base/carry` 保留于服务端，不在摘要响应中传输；报告 JSON 包含配置、日初/日末
状态、逐日节点/线路和配对结果。复现还需在南方边界页另行导出同一 revision 的基准 JSON。
当前任务仅保存在服务进程内存中，服务重启后不会恢复。

浏览器逐日发送 step；暂停等待当前日（含配对计算）返回后停止，重载后可继续。
终止也不强行打断求解器；已完成日保留，终止任务不可继续。run/day/revision 变化
返回 409；参数非法/终态推进返回 400。改变已保存南方边界使旧任务不可继续；
替换工程模型清除任务。界面可显示部分结果，但不能把未执行日期解释成零异常。

验证证据：Release `test_southern_market '[southern_market]'` 通过 21 用例 / 1699 断言，
重新构建的 `macos-asan-ubsan` 同样通过 21 / 1699，运行参数
`ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1`，无 sanitizer 报告。
包括 7 日/29 日全周期、100 MW 缺额/50 MW 富余、25 MW·h 限额恢复差值、
线路停运、共享水库/SOC 状态承接和跨日上游历史。`market_operation_e2e` 注册于
CTest，验证编辑、曲线全量时点、异常设备/恢复差值、月长、重载/继续/终止、revision
冲突以及桌面/390px 布局。截图与数值文件位于 `output/market-operation/`。
另实际运行多资源两节点 7 日演示（2026-09-07 起）：第 2 日负荷倍数 6，第 3 日
线路限额倍数 0.25 且机组 2 停运，第 4 日风/光 0.8、来水 0.9、报价 1.1。
七日全部取得可行诊断 SCED；第 2 日缺额 23227.5948004 MWh、线路越限积分和
22416 MW·h；第 3 日越限积分和 1494.8396392 MW·h。所有改变因素均取得有效
恢复重算。基准和报告为 `output/market-operation/demo-week-{boundary,evidence}.json`，
截图 `demo-{config,results,causes,mobile}.png`。负荷 6 倍是故意制造异常的压力试验，
不是实际负荷预测或发生概率。没有执行 2000 节点周月出清或概率联合采样；不得据此
宣称规模性能或正式规则等价。

### GUI 算例修复依据

RATIONALE: 算例构造沿用现有 98 点线性模型，不改优化方程。普通工程导入应保持
资产数量；资源增补仅由明确的研究算例调用。水电使用 price 报价且新能源字段为零，
每库挂接四台机组，库间串联使用现有 upstream/lag_slots。新增输入构造 O(T(G+H+S))。
小型演示预期 2 母线、8 台发电机组（2 火/1 风/1 光/4 水）、1 库、2 储能、2 可控负荷；
大边界预期 2000 母线、1744 机组（720 水）、180 库、80 储能、120 可控负荷。
参数均为合成研究输入，不承诺大规模最优解或真实系统代表性。验证命令：
`test_southern_market '[southern_market]'` 和 `node tests/e2e/southern_market_e2e.mjs`；
验收为上述精确计数、普通导入不增补、小型三阶段计划可行/残差 <=1e-6、
浏览器可由直接 URL 进入、加载/改报价/保存/运行/查看线路与节点结果。

字段链路：case_id -> 后端工厂 -> boundary.name/source/实体表 -> GUI 算例选择和摘要；
报价直接编辑 generators.segments / storage / controllable_loads；结果只读取后端阶段值。
`node_imbalance_mw` 是等式数值残差，不是允许缺电的松弛变量；线路超限按潮流与上下界
独立比较，不能用罚变量幅值替代。本段记录最初单日GUI修复；后续周/月、概率统计和
原因复核界面以本文运行模拟和预测契约为准。

数值复核发现演示 LMP 报告变量 9754 下界高于上界：SCED 允许的舍入误差使
充电功率略正，负充电邻域公式与硬上界 0 交集倒置。原 SCED 残差门槛保持 1e-6；
定价邻域中心先投影到已声明的物理功率区间（每变量 O(1)，修改量 <=已验收残差），
再构造邻域。预期小型演示由 lmp_failed 恢复有效价格，残差仍 <=1e-6；
大于门槛的不可行前序解仍在阶段审计拒绝，不以此修复真实不可行输入。

- `GET /api/session/southern_market`：schema、revision、boundary、baseline、latest。
- `POST /api/session/southern_market`：`{action,revision,boundary?}`，支持 `save`、
  `example`、`demo`、`activsg2000`、`from_system`、`pin_baseline`、`restore_baseline`。
  `demo` 为 2 母线多资源演示；`activsg2000` 从仓库 `data/case_ACTIVSg2000.m`
  构造独立市场边界，不替换工程 Canvas。普通 `from_system` 不自动增加资产。
  浏览器直达 `/xjtu/#southern-market`，或电力市场任一子页面自动打开工作区；
  算例下拉框与“加载市场算例”按钮显示名称和实际资源计数。
- `POST /api/session/run_southern_market`：仅接受 `{revision}`，对已保存完整快照求解。
  校验失败返回 400，版本冲突或忙返回 409；验证成功才原子替换状态。
- 运行中若工程系统被替换，旧结果返回 `stale=true`，不得写入新会话。
  工程系统替换清除边界及结果；南方规则运行不隐式触发工程 Canvas 参数同步，市场拓扑视图独立联动。
  `from_system` 支持显式 AC 母线/支路/负荷/机组及并联电导电纳；其它资产必须显式编制边界，拒绝丢弃。
  导入的报价和历史是有来源标签的合成研究数据，不是市场主体真实申报。
- GUI 的“边界工作区”直接消费 schema；边界类别/实体选择器、逐时序列、导入导出、
  保存重载和基准恢复均连通后端。结果页有三阶段状态/目标/残差、逐实体逐点结果、
  运行日 LMP 图、约束族和 AC 迭代详情。
  结果表每页 100 条，提供翻页访问全部记录；当前电价图仍最多显示前 80 个节点，
  完整电价通过节点表和结果导出访问。字段缺省时编辑器不补写新的可选字段。
- 基准与情景保存完整边界；比较返回 JSON Patch、费用/电量/机组曲线/电价差值。
  会话状态不跨进程持久化，导出 JSON 用于留存研究输入和结果。没有将两次差分包装成
  统计相关系数，也没有建立多年真实市场数据的外部认证。

## 公式—源码—测试索引

| 条款 | 生产装配 / 输出 | 独立数值检查 |
|---|---|---|
| 2.4.1 / 2.6.3.1 | `validate_southern_market` / `2.6.3.1/balance` | 100→101 MW 比例校正与 4800 元费用差 |
| 2.6.3.2–4 | `reserve`, `province`, `direct_dispatch` | 正负备用不足、一次调频 min 容量 |
| 2.6.3.5–8 | `state`, `phases`, `output`, `segment` | 检修/必开冲突、固定轨迹、非市场计划 |
| 2.6.3.9–10 | `power`, `energy`, `group_status` | 群日电量 1200 MWh |
| 2.6.3.11–13 | `up/down`, `initial`, `start_class`, `max_starts/stops` | 初始剩余停机、三态门槛、零启动数 |
| 2.6.3.14–15 / 2.6.6 | `line`, `section`, `stage_result` | 50 MW 拥塞、200/300 元电价、负荷微扰 |
| 2.6.3.16 | `energy`, `terminal`, `hour`, `cycles` | 0.81 往返效率、10 MWh 充电、独立 59 MWh 末值 |
| 2.6.3.17 | `dc`, `hub`, `no_reversal` | 单端 0.9 / 多段 0.81 效率、方向反转不可行 |
| 2.6.3.18–19 | `water`, `release`, `hydro_energy` | 水位 99.9 / 90.4 m、日电量上限不可行 |
| 2.6.3.20–21 | `quantity`, `minimum`, `gateway` | 新能源 20 MW 偏差、优先计划 80 MWh 缺口 |
| 2.6.2 / 2.6.4–5 | `run_southern_day_ahead_market` | 调频改限、独立定价、AC 越限反馈及迭代限失败 |

完整命令与最近结果记录在 [开发状态](../../overview/development_status.md)。
这些是合成解析与边界回归，不等于省级真实数据回放或发布机构认证；规则中歧义未取得
发布机构澄清前，不宣称“无条件与正式规则完全一致”。

## 四层正确性验证进度

概率场景分析的首个内部闭环已加入 `[scenario]` 测试：固定种子 `20260905`、
20 个等概率场景，逐场景扰动负荷并调用同一 `run_southern_day_ahead_market`。
测试检查概率和为 1、场景边界回传、最大残差以及负荷升高时发电量不下降；
当前 79 项断言通过。该测试验证场景驱动和结果语义，尚未代表 2000 节点
联合分布校准或统计相关性结论。

`[statistics]` 进一步联合扰动风、光、负荷、可控负荷补偿报价和资源报价，
逐场景统计有效出清、线路松弛事件、节点价格样本和新能源利用率样本；
当前 12/12 场景有效，16 项断言通过。场景统计已在预测任务层序列化
（`market_forecast_statistics`，含 Pearson 相关）；Spearman/PRCC 相关性
矩阵仍未实现。

1. **方程与守恒层：已执行。** `[hand_oracle]` 覆盖节点负荷削减、发电量、补偿费用、同库多机组下泄量和水位递推；手算断言 12 项全部通过。
2. **解析最优层：已执行。** 单母线单机组、储能、拥塞和水库算例均有确定的人工最优值或边界不可行值，测试位于 `tests/test_southern_market.cpp`。
3. **性质层：部分执行。** 已覆盖负荷微扰、线路/断面限额、备用不足、补偿价格阈值和水电日电量上限；随机性质测试、可行域包含关系和多流域交叉耦合测试仍待补齐。
4. **外部交叉验证层：待执行。** 计划将小型边界导出到独立 Python/HiGHS 或 SCIP 模型，并用 MATPOWER/pandapower 复核 AC 潮流。当前没有外部 oracle 结果，内部 HiGHS 回归不作为外部验证。同一模型在 HiGHS/Gurobi 双后端的目标一致性已由 `[gurobi]`、`[local_solvers]` 覆盖，但属于同模型不同求解器核对，不等同独立外部 oracle。

缺少真实报价时，报价字段均标记为 synthetic/研究输入；验证结论限定为方程、约束、物理守恒和给定实验报价下的优化性质，不解释为真实市场价格认证。
## 2000-bus exact formulation experiment

RATIONALE (before implementation): retain all 98 points and network/resource
constraints. For equal hot/warm/cold startup costs and three empty startup
curves, project out startup-class binaries and offline-history variables.
Reconstruct offline time chronologically from integer online states, selecting
the unique integer-minute threshold interval on a start. Their only objective
contribution is the common cost times `start`. This is an exact projection,
not a commitment heuristic. Noneligible units retain the original class model.

For binary adjacent online states, the transition equality plus
`start <= u`, `start <= 1-u_previous`, `stop <= u_previous`,
`stop <= 1-u` uniquely fixes continuous start/stop to their original binary
values. Reference: direct enumeration of the four pairs (0,0), (0,1), (1,0),
(1,1), and section 2.6.3.13 in this contract. The original exclusive row stays.
The exact reformulation does not change ramping, reservoirs, storage, network,
or rolling carry. `execution.formulation=reference` retains authored binaries;
`compact` is the default, including for saved boundaries without this field.

Cost model: sparse MILP on local arm64 / 128 GiB / 16 logical cores. For each
eligible unit/time remove 3 binary + 1 continuous columns, 1 equality and 9
inequalities; add 4 transition inequalities for every unit/time, and make its
2 event columns continuous in the solver (still audited as integral). For the
1744-unit / 98-point synthetic case, predict exactly 683648 fewer columns and
854560 fewer solver binaries. Matrix/row storage remains O(nnz); these counts
do not predict branch-and-bound runtime. Working hypothesis: peak RSS at least
10% lower; runtime improvement is measured, not guaranteed by projection.

Validation fixed before results: reference vs compact small-case objective
difference <= 1e-4 CNY, original-unit row/bound/integer and reconstructed history
residual <= 1e-6, startup-threshold and noneligible-unit regression tests.
Benchmark: `run_southern_market_benchmark 2000 output.json 120 [formulation]`,
Gurobi 13 / 4 threads / 1% requested gap, both forms on identical inputs; record
status, stage sizes/times, gap, residual, RSS and total wall time. A successful
scale run requires all SCUC/SCED/LMP stages audited and valid conditional prices;
it does not certify nonlinear AC security or week/month throughput.

The first matched 120-second runs measured 3543876 -> 2860228 columns,
1041152 -> 186592 binary columns, exactly the predicted differences. Peak RSS
27864072192 -> 19677478912 bytes (-29.4%, exceeding the >=10% reduction target);
assembly 7.220 -> 5.981 s; wall 161.928 -> 140.572 s. Both runs ended at SCUC
TimeLimit without an incumbent, so neither certifies a 2000-bus schedule.
Artifacts: `output/market-operation/performance-{baseline,compact}.json`.
The follow-up 600-second diagnostic isolates the root LP bottleneck. No runtime
speedup claim is inferred from the reduction in binary columns alone.

Contract ledger: `execution.formulation` is optional enum `compact|reference`,
normalized by `validate_southern_market`; all original stable IDs and units stay.
Stage JSON exposes `formulation`, `compact_units`, `variables`, `binary_variables`,
`nonzeros`, `assembly_sec`, `runtime_sec`, `audit_sec`, and
`reconstructed_max_residual`. `audit_sec` includes row/bound/integrality checks
and result generation; `runtime_sec` includes adapter loading and solving, so
it can exceed the optimizer time limit. Operation `days[].stages.<stage>` carries
these unchanged to the GUI performance table, even on a failed day. Missing old
fields render unavailable. Forecast day drill-down uses the same renderer.

The diagnostic timing extension adds stage `solve_wall_sec` (the complete
`solve()` call, including scaling and adapter work) and `solution_export_sec`
(SCUC/SCED name-to-value dispatch map; zero for LMP which has no such export).
Top-level `validation_sec` measures boundary validation only. Operation day
`execution_timing` adds `boundary_sec`, `validation_sec`, `summary_sec`, and
`analysis_sec`; recovery proofs add boundary/validation/runtime seconds.
Validation is contained in market runtime/main time; solve wall contains the
adapter runtime and must not be added to it. Failed stages may have no export
time. These are observed durations, not progress estimates or extra deadlines.
Day wall excludes final insertion/copy/HTTP serialization. Parallel recovery
worker durations cannot be summed as recovery wall. Reproduction, numerical
gates and parameter experiment results are in [performance](performance.md).

`execution.assembly_mode` optionally selects `cached` (default), `reference`, or
`verify`; it is independent of `execution.formulation`. `reference` retains the
map expression/string column lookup/Triplet assembly for the same mathematical
formulation. `cached` uses sorted contiguous expression terms and numeric
family/device/time column lookup, and exclusively borrows a stage template for
eligible small day-ahead workloads. Every coefficient, bound, RHS, cost and
audit expression is recomputed. Variable and row layout and every nonzero column
are checked before updating CSC values in place; a mismatch rebuilds the matrix.
This is storage/index/sparsity reuse, not a cached solution or solver model.

SCUC, SCED and LMP pools each retain at most two models. Realtime, ancillary and
systems exceeding 118 buses/128 generators/16 storage/24 reservoirs do not pool
templates; retained columns/rows/nonzeros have additional bounds in the
performance ledger. No references to another request's mutable model are shared.
Each stage emits `assembly_template.{mode,layout_reused,reused_matrices,
matrix_comparison}`. `verify` builds the reference too and requires exact equality
of CSC indices/values, RHS, costs, variable metadata, integer/audit sets, original
expressions/scales and recovery mappings before solving; mismatch throws an
error. `exact_match` certifies assembly identity, not AC security or optimality.
Verification performs extra work and is excluded from production speed claims.
Operation summaries/forecast days preserve this metadata, including interventions.

Local build evidence uses cached macOS Release `-O3 -DNDEBUG -std=c++20 -arch
arm64`, HySim HEAD 308ccc57 and MIPSolvers HEAD e003dbb1 plus their dirty trees.
Reconfigure is rejected by the existing Release dirty-dependency gate; incremental
object compilation/linking does not establish a clean reproducible Release.
No changes were discarded and the release guard was not weakened. The benchmark
target copies OpenDSS runtime dependencies and an opt-in CTest is registered by
`HACDCPF_ENABLE_MARKET_SCALE_TESTS=ON`; it requires a full-size Gurobi license.

Verification so far: `[compact]` passes 8299 assertions including 239/240 and
719/720 minute threshold ties, outage/restart, 98-point state/class/output parity
and all-stage objective parity. Full Southern passes 31 cases / 19562 assertions;
four registered GUI suites pass (32.96 s), including API/DOM performance fields.
These two counts are reproduced from the session record; no independent log of
them survives in tests/ or output/, so they are unverified historical numbers.

### Zero-Cost Commitment Projection

A second exact projection in `assemble_model` removes the commitment binary of
eligible zero-flexibility-cost units from the solver's integer set. Admission
requires the compact formulation, the SCUC stage, no real-time/ancillary
coupling, a hydro or renewable/wind/solar unit whose three startup costs are
zero and equal with empty startup curves (the compact class above), zero
`minimum_cost_per_hour`, zero `technical_min_mw`, an empty shutdown curve,
zero minimum up/down minutes, `max_starts`/`max_stops` >= T, a feasible
authored initial state, and all 98 points having zero `pmin_mw`, zero
regulation awards and ramp rates covering full capacity within each interval.
For such a unit every commitment only relaxes constraints and costs nothing,
so fixing `u` to its authored availability `available*(1-must_off)` is an exact
projection, not a heuristic: `u` is submitted as an implied-integer continuous
column with equal bounds, is absent from the solver binary set, and is still
audited for integrality (trivially satisfied by the equal bounds). SCED/LMP
inherit the state through the usual predecessor freeze; `derive_sced` clears
the projection set. Stage JSON reports `projected_commitment_units`, and
`model_size.commitment_by_kind` shows these units without declared binaries.
Equivalence proof: performance.md, Zero-Cost Commitment Projection; automated
coverage in `[compact][commitment_projection]`.

Second rationale, after profiling: Gurobi default concurrent root LP divides the
4-thread budget among primal/dual simplex and a 1-thread barrier. Observed root
factorization has 251.9 million nonzeros and 8.069e11 estimated operations; late
iterations take about 7 seconds. Select dedicated barrier for compact models
with >=1000000 columns; this cutoff separates the measured million-column
regime from small existing tests and is a tuning policy, not a mathematical
threshold. Model and acceptance gates do not change. Gurobi 13 parameter
reference `Method=2` allocates the requested threads to barrier. Hypothesis:
root LP wall time -30% versus default concurrent with 4 threads, fixed 600-second
budget and identical input; measure from native logs and retain any mismatch.
API: optional `execution.gurobi_method=auto|solver_default|barrier|dual_simplex`;
auto uses the size policy, explicit choices use Method -1/2/1. Nonauto selection
requires Gurobi. Result `lp_algorithm` records actual selected policy. Shared
adapter `GurobiOptions.method` defaults -1 and validates -1..5, per Gurobi's
documented LP method enum. This adds no model relaxation or heuristic schedule.

Third rationale: dedicated barrier completed in 156.33 s versus 250.21 s for
the concurrent run (-37.5%, predicted -30%), but crossover again restarted.
The root-LP prediction includes crossover and remains open. A conditioning
investigation identifies water balances with 9e-6 coefficients and ~100 m
absolute levels. Use the invertible coordinate
`E_h,t=(level_h,t-initial_level_h)*area_h/water_h` (MWh-equivalent stored water)
and multiply ONLY the solver water equality by `area_h/water_h`. This yields
unit coefficients for E transitions and duration-hour coefficients for unit
power; spill/inflow coefficients are `duration*3600/water`. This is algebraic
substitution, not a water/energy relaxation. Public levels, original water-row
residuals, and cross-day carry remain meters. The original Row expression is
retained for audits; scaled-row duals are multiplied by the same positive scale.
Reference: 2.6.3.18 water conservation and direct substitution above.

Complexity: O(H*T) additional scalar arithmetic, unchanged sparsity and variable
count, no dense transforms. For area1e8/water3600/duration.25, predicted water
state/output coefficients 1 / .25 instead of 1 / 9e-6, a 27778x reduction in the
water-row coefficient ratio. Hypothesis: zero crossover restarts and complete
root LP within 420 s on the 600-second 4-thread benchmark. Gate: small shared/
cascade reservoir objective parity <=1e-4, levels <=1e-6 m, original row residual
<=1e-6; no timing success claim until measured. Optional
`execution.reservoir_scaling=auto|original`; auto applies to compact models only.
Result records `reservoir_scaling=energy_coordinate|original` and all public
reservoir outputs retain existing SI units and stable IDs.

Absolute bound tolerances are not invariant to coordinate scaling. Both seed
and final audit explicitly reconstruct every reservoir level and check authored
meter bounds, in addition to internal water-energy bounds. Their violation
contributes to `max_residual`; this prevents large
water/area ratios from hiding meter violations behind an energy-unit tolerance.
This audit-only addition does not change the solved matrix or solver policy.

Primal-start rationale: a MILP incumbent is an upper bound and never removes
feasible integer schedules (standard branch-and-bound). For Gurobi compact SCUC
with >=1000000 columns and `execution.mip_start=auto`, try generator online=its
available upper bound and all other authored solver binaries at their lower
bounds, solve the FULL continuous completion LP with those bounds, and check
all original rows, bounds and authored integrality <=1e-6 before passing it as
`MIPModel.initial_solution`. Candidate infeasibility is permitted and never
reported as market infeasibility. Restore all authored bounds before MILP.
SCED tries its preceding SCUC vector with the same audit (regulation may reject
it). `mip_start=none` disables this policy. The candidate is a heuristic ONLY
for constructing an incumbent; no commitment is fixed in the final market MILP.

Cost: one continuous completion with O(nnz) adapter loading and O(n) bound/audit
bookkeeping, bounded optimization time min(60s,0.1*requested_time_limit),
subtracted from the main MILP allowance. Adapter loading/auditing are additional
wall time as before. Prediction: a verified incumbent before root search in the
synthetic 2000-bus case within this allowance; acceptance still requires final
SCUC/SCED audited feasibility and valid LMP. Seed status/time/residual are reported
even when unsuccessful. Small explicit `mip_start=enabled` fixtures exercise
successful and infeasible completion without requiring a large default test.

Observed seed completion took 38.95 s and passed original 1e-6 audit, but Gurobi
rejected the full vector for a scaled row violation 1.03e-7 against its 1e-8
gate. This is a tolerance/coordinate mismatch, not proof of market infeasibility.
Corrected submission: only rounded, audited solver integer columns are specified;
other columns use documented `GRB_UNDEFINED=1e101` to request Gurobi completion.
`primal_start.accepted` means application audit accepted the candidate for partial
submission; it does not claim Gurobi accepted it as an incumbent. `submission`
records `partial_integer_completion|none`. The final result always undergoes
the full original model audit again.

Continuous-barrier rationale: a primal/dual interior-point optimum already
provides the primal vector and row multipliers required by LMP and seed audits.
Neither consumer uses a simplex basis. Gurobi 13 `Crossover=0` omits basis
construction for pure LP while retaining X/Pi on an optimal barrier solve.
Set it only for explicitly selected barrier LP (including seed); MILP root
keeps Gurobi's crossover. Prediction: remove 100% of crossover work from those
LP calls (zero crossover log rows); unchanged <=1e-6 audit and analytic duals
1/2 on the ranged-row fixture. Cost and model stay unchanged, no extra LP.
The adapter option defaults -1; accepted enum -1..4 is validated. A limited LP
without an optimal solution still has no valid prices. This protocol is separate
from the MILP-root timing comparison and is not evidence of root speedup.

Measured full-chain evidence (before primal starts / no-crossover LP):
`performance-scaled-600.json` completed SCUC/SCED/LMP, wall 597.122 s, peak RSS
17838227456 bytes (16.613 GiB). Stage adapter seconds 389.005 / 151.307 / 26.476;
assembly 6.337 / 6.658 / 6.634 s; audit/result generation about1.91 s each.
SCUC gap2.2089188e-7, original residual1.7489076e-8; SCED gap0,
residual6.1162041e-8; LMP residual1.6412453e-7 with valid conditional prices.
SCUC `optimality_proven=false` preserves the nonzero gap even though the native
solver met the requested1% tolerance. SCUC objective101930871.400977 CNY.
There were zero SCUC crossover restarts and root relaxation finished in359.55 s,
meeting the coordinate prediction <=420s. The final portion overlapped another
4-thread diagnostic and small tests on the16-core machine; this single run is
not a controlled throughput distribution or a week/month benchmark.

Default concurrent600s and unscaled dedicated-barrier600s both reached limits
without incumbents (`performance-compact-600.json`, `performance-barrier-600.json`).
The dedicated barrier-only segment improved37.5%, but full root LP did not finish:
the original root timing hypothesis was not met. The observed cause is crossover
ill-conditioning/basis restarts; the coordinate derivation above fixes the measured
water scaling rather than dropping constraints. The old full-vector seed diagnostic
was manually interrupted after exposing its tolerance rejection; its log is not
a completed benchmark. Final partial-start / no-crossover execution is validated
separately by the live rolling API run and tests.

During this work the user committed both trees: current baselines are HySim
8b93145b and MIPSolvers e6c932e5 plus subsequent changes. Earlier A/B binaries
precede those commits. The cached build flags stayed the same. The final standard
Release configure remains refused while the sibling adapter changes are dirty.

Final optimized local regression: Southern33 cases/20182 assertions, forecast
8/12503; four market browser E2E suites34.96s. Fields for scaling, seed state and
LP algorithm pass API/DOM assertions, desktop/mobile layouts have bounded table
scrolling. Rebuilt ASan/UBSan with Gurobi OFF passes reservoir-coordinate1/597.
The full `[compact]` ASan run aborts on HiGHS `updateActivityUbChange`,
`HighsDomain.cpp:1705`, while solving the reference startup/outage fixture. No
memory report was produced and this unresolved debug assertion is not waived.
Gurobi itself is not sanitized by this configuration.

Final live rolling API evidence: port8086, ACTIVSg2000 augmented boundary,
`POST /api/session/market_operation` start then a single step, week starting
2026-09-07, Gurobi4 threads,600s/optimization allowance,1% gap, explanations OFF.
Day1 completed (`completed_days=1`, job remains resumable at1/7) in680.365s of
market pipeline time. The true D+1 flat-load rule selects slots0/95 at minutes
1440/2865, not the earlier fixed-template representatives. All2000 node and3206
line series are available, prices valid, daily deficit/line-excess integrals0.
This feasible synthetic boundary does not imply violations cannot occur.

Final SCUC/SCED/LMP adapter times493.130/145.502/13.364s; SCUC includes27.097s
candidate completion/audit, SCED reuses and audits its predecessor in0.552s.
Native log confirms partial integer seed completion and actual incumbent load.
SCUC gap8.8286485e-7; SCED/LMP gap0; original residuals7.182621e-9 /
8.113882e-7 /4.603633e-8. LMP barrier takes12.05s with zero crossover work,
meeting the continuous-LP prediction. SCUC/SCED objective102034568.866174 CNY;
LMP objective101751466.300885 CNY. Reconstructed startup residuals0.
Export: `output/market-operation/performance-2000-gui-day.json`.
This is one rolling day, not a completed7-day benchmark. The driving Node fetch
hit its300s headers timeout; the backend continued and final persisted state was
retrieved using GET, without submitting a duplicate solve. Whole-day process
budgets and asynchronous stage progress remain future work.

Live browser verification selects `buses:2000`, changes slot0 to48 and observes
updated Canvas detail with valid results; no page overflow at390px. Screenshots:
`performance-2000-{desktop,mobile,canvas}.png`. The performance tables use ordered
SCUC/SCED/LMP rows and scientific notation for nonzero gap below1e-4, avoiding
rounding an unproven gap to0. Meter-bound audit was added after this large run;
it leaves the solved matrix unchanged and is covered by the final small tests.
The port8086 service intentionally retains the completed in-memory large result;
the top-level executable is updated for subsequent starts.
Post-audit rebuild passes `[compact],[gurobi]`5/8958 and the operation/forecast
browser suites23.15s. The earlier full33/20182 regression predates only the
explicit meter-bound audit and gap display/order correction.

### Solver Selection Readiness and Failure Diagnosis

The forecast solver editor could mount before the operation capability GET
completed. Its subsequent `load(true)` preserved the draft and skipped editor
reconstruction, leaving an enabled, empty select. Changing a distribution rebuilt
the editor and masked the race. `web/js/core/market_operation.js` now shares the
pending capability request and hydrates every mounted solver select on arrival.
Loading disables the select; execution rejects unloaded/unavailable selections.
Hydration retains the authored solver and other inputs; a saved unavailable
solver is shown explicitly without silently substituting another backend.
`market_forecast.js` uses a permissive draft read for rendering and validates the
selection when submitting. Failure details show requested/actual backend,
requested time/GAP, elapsed solver time and original-model audit rejection.

Both `market_forecast_e2e.mjs` and `market_operation_e2e.mjs` pass against the
optimized server. The forecast regression holds capability GETs until the editor
mounts, checks loading rejection, then checks both lists hydrate without changing
distributions. It exercises native/HiGHS selection, real Gurobi forecast execution
with two threads and 30 s allowance, and saved-selection reload. The operation
suite also verifies failure text with a synthetic limited-day display fixture;
that fixture is UI evidence, not a solver benchmark. Desktop and 390 px live
screenshots are `output/market-operation/solver-select-fixed-{desktop,mobile}.png`.
Frontend cache keys changed; no numerical solver code changed for this fix.

Independent diagnostic reruns use the reduced synthetic 2000-bus case, 120
thermal /720 hydro /240 wind /240 solar units,180 reservoirs,80 storage and120
controllable loads. Each daily SCUC still has98 points,2569364 variables,
145040 binaries,3333454 rows and10515386 nonzeros. The 1% requested gap is a
termination tolerance once a feasible incumbent and usable bound exist; it does
not guarantee an incumbent, or prove infeasibility when a limit is reached.

Reproduction (sequential isolated processes,120 s optimizer allowance and420 s
external watchdog):

```bash
MIPSOLVERS_GUROBI_VERBOSE=1 MIPSOLVERS_BC_TIMELINE=1 node tests/run_southern_solver_comparison.mjs 2000 120 420 120 output/market-operation/failure-diagnosis gurobi,native,highs
MIPSOLVERS_HIGHS_LP_KERNEL_TRACE=1 MIPSOLVERS_BC_TIMELINE=1 node tests/run_southern_solver_comparison.mjs 2000 120 420 120 output/market-operation/failure-native-kernel native
```

Gurobi took129.123 s process wall time. Its12 s seed LP expired without an
accepted seed; the remaining108 s MILP budget expired during root barrier
(25 iterations, root relaxation time limit, solution count0). The presolved
model retained1458401 columns and140799 binaries; barrier factorization reported
2.272e8 factor nonzeros. This is root-relaxation budget exhaustion, not a
positive infeasibility certificate. Detailed log: `failure-diagnosis/2000-gurobi.log`.

Native took56.998 s in the three-backend run and58.845 s in the trace rerun.
The latter records `run=1 status=IterationLimit iter=25000`: the native B&C root
uses the embedded HiGHS LP, whose simplex cap is
`max(max_lp_iter*50,10000)` with default `max_lp_iter=500`.
Source: sibling `include/mipsolvers/engine/bc/options.hpp`,
`src/engine/solver/native/milp/bc/legacy/bc_relaxation.cpp`, and root status
propagation in `src/engine/solver/native/milp/bc/legacy/bc_run/02_root_relaxation_a.inc`.
The upper layer reduces this to `Root relaxation failed`.
It is an iteration-limit failure, not evidence of numerical breakdown or
infeasibility. Detailed log: `failure-native-kernel/2000-native.log`.

A two-second macOS stack sample of the HiGHS run, about127 s after process
start, shows160/164 main-thread samples in root `ziRound` /
`calculateRowValuesQuad`, with the worker in analytic-center IPM factorization.
Sibling `highs/mip/HighsPrimalHeuristics.cpp::ziRound` recomputes row activities
for each fractional integer inside a loop without an internal time-limit check.
This supports non-preemptive root-heuristic work as an overrun contributor;
the short sample is not a full-runtime attribution. Evidence:
`failure-diagnosis/highs-stack.txt`. Production HTTP still lacks a hard process
deadline. Changing root iteration budgets, initial-solution strategies or HiGHS
heuristic scheduling requires separate numerical validation; none is claimed
implemented by this GUI correction. All original-model feasibility gates remain.

The HiGHS rerun completed naturally before the420 s watchdog:418.318 s process
wall,409.506 s SCUC adapter runtime for120 s requested, `TimeLimit`, returned
vector original-model maximum violation285.41 (rejected), invalid prices.
The three-backend driver completed and wrote `failure-diagnosis/comparison.json`;
all three SCUC runs failed to obtain a verified solution. The diagnostic native
trace overlapped part of the HiGHS run, so these times are diagnostic observations,
not a controlled solver speed ranking. Syntax checks and `git diff --check` pass.
### Model Size and Bound Redundancy Audit

`inspect_southern_market_model(boundary)` validates and assembles the current
98-point SCUC, returning `status=model_inspected`, `solved=false`, formulation,
runtime and `model_size`. It never invokes a solver and does not certify a
schedule, prices or feasible boundaries. Benchmark mode `inspect` is a successful
inspection exit, distinct from the default full-chain `solve` exit semantics.
Each actual SCUC/SCED/LMP result also contains `model_size`; rolling day
`stages.<stage>.model_size` propagates it to the manual and forecast result view.
The GUI's expandable SCUC size table is available even after solver failure;
old results without the field do not fabricate counts. Counts are dimensionless,
grouped by variable family / rule section and generator kind, not device positions.

Rationale fixed before implementation: read-only assembly census and box support
function `U = c + sum(a_i >= 0 ? a_i*u_i : a_i*l_i)` for `a*x+c <= rhs`.
If a conservatively rounded `U <= rhs`, the inequality follows from the authored
column bounds. Scan cost is O(columns + nonzeros), with a small fixed set of
family counters; no factorization or optimization is performed. Acceptance:
large-case totals exactly match prior solver input counts, family sums reconcile,
the hand example keeps objective490000 CNY / LMP200 CNY/MWh, and adding a positive
minimum output removes its `-p <= 0` box certificate. Runtime reduction is not
predicted because the solver matrix is unchanged.

Implementation: `src/market/southern_market.cpp::model_size_report` rounds each
nonzero product and accumulation toward +infinity with `nextafter`. Exact zero
terms remain zero; infinite or sentinel-sized bounds are inconclusive. No
feasibility tolerance is used to declare redundancy. Counts are sufficient-only:
roundoff padding may exclude an exactly redundant row, and dependencies among
rows are deliberately not inferred. Fixed columns remain counted as authored
columns; declared binary totals include fixed binary columns still submitted
before solver presolve. `constraints_removed=0` is explicit. These candidates
may already be removed by the backend's presolve and are not an independent
promise of speedup. Final residual/dual audits and all original rows are intact.

Reproduction using the cached optimized macOS arm64 build (`-O3 -DNDEBUG`,
16 logical CPUs,128GiB RAM; HySim8b93145b / MIPSolvers e6c932e5 plus recorded dirty
worktrees):

```bash
build/macos-release/tests/run_southern_market_benchmark 2000 output/market-operation/model-size-120.json 120 compact auto gurobi 120 inspect
build/macos-release/tests/run_southern_market_benchmark 2000 output/market-operation/model-size-544.json 120 compact auto gurobi -1 inspect
build/macos-release/tests/test_southern_market '[model_size]'
ASAN_OPTIONS=detect_leaks=0 build/macos-asan-ubsan/tests/test_southern_market '[model_size]'
```

Measured counts match the prior solve artifacts exactly:

| Synthetic thermal fleet | Total units | Columns | Declared binaries | Rows | Inspection process wall |
|---|---:|---:|---:|---:|---:|
| 544 thermal | 1744 | 2860228 | 186592 | 3957582 | 7.985 s |
| 120 thermal | 1320 | 2569364 | 145040 | 3333454 | 6.892 s |

Thermal unit reduction77.94% becomes only10.17% fewer columns,22.27% fewer
declared binaries and15.77% fewer rows. The retained120-thermal case has:

- 1530564 network/diagnostic columns:196000 angles,314188 line flows,
  628376 signed line slacks and392000 node deficit/surplus columns (59.57%).
- 952560 generator columns,39200 storage columns,35280 reservoir columns and
  11760 interruptible-load columns. Water state is already per shared reservoir,
  not redundantly replicated for each of its generators.
- Commitment binaries:11760 thermal,70560 hydro,23520 wind,23520 solar;
  storage contributes another15680 direction binaries.2842 thermal commitment
  columns are fixed by authored availability but included before presolve.
- 778800 startup/state rows (2.6.3.13);628376 line-limit inequalities;
  314286 network-flow/reference equalities;196000 bus balances;
  258720 ramp inequalities; other rules are itemized in the JSON artifact.
- 213331 box-certified redundant inequalities (6.40% of all rows), including
  120442 output-envelope rows,47040 renewable rows and17820 release rows.

Current customization is partial: empty device tables skip their loops; zero
minimum up/down times create no window rows; empty startup curves/equal startup
costs enable the existing exact class/history projection; outaged branches omit
their flow equation but retain fixed-zero flow/slack columns and limit rows.
However, all generator kinds currently receive the generic state, transition,
output, primary-reserve and ramp template. In particular, the synthetic wind,
solar and hydro rows inherit the baseline state/reserve parameters; a resource
label alone does not establish that its commitment can be removed. All current
large-case minimum up/down times are zero, so no 2.6.3.12 window rows appear.
This synthetic boundary must not be described as actual Southern plant data.

Next admissible reductions require separate implementation and validation:
fixed-column substitution with objective-constant/result recovery; certified
redundant-row omission with dual-row mapping; trajectory-free stable-state
substitution; renewable commitment projection only after checking costs,
minimum output, reserve participation, group status and intertemporal coupling.
Hydro commitment is not generically removable: shared/cascade water balance,
reservoir limits and plant operating restrictions remain part of the boundary.
Removing lines merely because they were uncongested in one prior scenario is
invalid for uncertainty studies; screened rows need full-network checking and
reinsertion. None of these further reductions is implemented by this census.

Validation: rebuilt Release Southern37 cases/20970 assertions; focused ASan/UBSan
1 case/17 assertions (Gurobi OFF) pass. Manual and forecast browser suites pass,
including API count propagation, expanded size details and390px overflow checks.
The unchanged hand oracle meets objective and price acceptance. The normal
Release reconfigure remains guarded by dirty sibling dependencies; cached
compile/link scripts were used. Recreating the generated archive was necessary
before `ar qc` to avoid appending duplicate objects. No source was reverted.
### IEEE 118 System Validation and Forecast Resolution

The intermediate-scale fixture uses `external_data/matpower/case118.m`:118 buses,
186 branches and54 generator records. `make_southern_market_ieee118(system)`
preserves the original AC IDs and pi-branch electrical parameters and adds1 wind,
1 solar,4 hydro generators,2 shared two-unit reservoirs with one upstream link,
2 storage devices and2 compensated loads. All offers, startup history, added
assets and symmetric200 MW /200 MVA branch limits are explicitly synthetic.
The source generator records include synchronous condensers; treating their
MATPOWER Pmax as dispatchable thermal capability is a stated test assumption,
not a claim about IEEE physical plant classification. No real Southern bids
or network security certification follows from this fixture.

MATPOWER also creates Transformer2W metadata alongside the exact AC branches.
The fixture checks source_branch_idx, endpoints, service state, fixed tap,
phase shift and impedance consistency before collapsing that metadata alias in
a local copy. It retains the exact original branch tap/r/x/shift and bus shunts;
unknown/unlinked or controllable/modified transformers are rejected. The general
engineering market import remains strict. Initial system tests exposed this
previously unsupported import path before any optimization took place.

GUI/API contract: Southern POST action `ieee118` loads this fixture with normal
revision/busy checks and baseline invalidation. `operationCase` and `southernCase`
both offer the same backend case; inherited day drafts reset through existing
load logic. The benchmark accepts case `118`; optional thermal_limit applies
only to case2000. New `market_ieee118_e2e` is registered with a1800s test timeout.

Pre-execution rationale: retain the current fixed96+2 SCUC/SCED/LMP equations
and validate the middle scale using original-model residual <=1e-6, node/line
stable identities, MW sums and quarter-hour energy integrals. Assembly remains
O(T*(network+devices)+nnz) on this sparse fixture; no runtime speedup threshold
is asserted for the added case. Seven coupled daily windows exercise baseline,
4x load,5% line limits, temporary all-line outages, low wind/solar/inflow,
changed generator/load bids and recovery. Under stressed cases, positive
diagnostic deficit / line excess is expected; all-network outages must have
zero outaged-branch flow. These are synthetic stress tests, not probabilities.

Time-resolution study: calculation remains15 minutes /96+2. A one-slot4x load
pulse is block-averaged to15/30/60-minute forecast resolution, separately for
bus/area loads, wind/solar forecasts and inflow, then expanded onto the original
15-minute grid. For block size k, `p_bar = sum(p_i)/k`, so
`sum(p_bar*0.25 h) = sum(p_i*0.25 h)` on the96-point operating day. Prediction:
equal operating-day forecast energy and exactly identical assembled column/
nonzero counts, but lower peak forecast load; deficit, overload and prices may
change because the underlying boundary shape changes. Next-day representatives
are selected from each resulting next-day forecast through the existing rolling
rule; their omitted hours are not used to certify day energy preservation.

This does NOT implement30/60-minute calculation steps. For a hypothetical
unchanged two-point lookahead, daily point counts would be50 and26, versus98
(about51.0% and26.5% for terms linear in T). Actual speed does not scale linearly
with counts. True variable-step support needs synchronized min-up/down minutes,
ramp elapsed minutes, objective/energy weights, water/SOC integration, travel
lags, storage hourly restrictions, lookahead selection, carry and GUI statistics.
Current validator rejects a0.5h operating interval; a regression enforces that
unsupported edits are not silently accepted. Coarse dispatch would also require
15-minute replay before comparing peaks or claiming rule-equivalent results.

Reproduction:

```bash
build/macos-release/tests/test_southern_market '[ieee118]'
node tests/run_southern_solver_comparison.mjs 118 30 120 -1 output/market-operation/ieee118-solvers gurobi,highs,native
node tests/e2e/market_ieee118_e2e.mjs --server build/macos-release/tests/run_gui_server
```

Measured validation (cached macOS arm64 Release,16 logical CPUs,128GiB,
HySim8b93145b / sibling e6c932e5 plus the current worktrees):

- IEEE118 model132300 columns,6272 declared binaries,157906 rows,481939 nonzeros.
  The seven-day Gurobi2-thread run took25.332s summed pipeline time
  (3.336--3.968s/day); all stage original residuals <=1.88e-10 and conditional
  prices valid. Baseline deficit/overload0; high-load day6055.140MWh deficit and
  1045.980MW*h summed line excess; restricted-line day948.164MW*h line excess;
  all-line outage day1543MWh deficit with zero outaged-line flow.
- Gurobi4-thread30s benchmark completed4.253s process wall, SCUC gap0,
  objective17413231.016942CNY. HiGHS completed9.168s, SCUC gap8.90e-6,
  objective17413386.028540CNY; relative objective difference8.90e-6, within the
  requested1% tolerance. Both full chains passed1e-6 original residual gates.
  Native took44.157s, `Time limit reached`, no verified solution or prices.
  These diagnostics overlapped system tests, so times are observations rather
  than a controlled speed ranking. Logs: `output/market-operation/ieee118-solvers/`.
- Forecast study met the fixed prediction:104989.5MWh daily load energy and
  identical132300 columns/481939 nonzeros in all three cases. Results:

| Forecast resolution (calculation always15 min) | Peak load MW | Peak deficit MW | Peak single-line excess MW | Summed line excess MW*h | Pipeline s |
|---|---:|---:|---:|---:|---:|
| 15 min | 16968 | 6051.213 | 350.000 | 261.771 | 3.932 |
| 30 min | 10605 | 0 | 320.260 | 930.197 | 4.035 |
| 60 min | 7423.5 | 0 | 22.000 | 22.000 | 4.002 |

Thus averaging can hide short-duration shortage while changing network-risk
integrals nonmonotonically. These are deterministic pulse experiments, not
probability estimates, and prices include diagnostic penalties. SCUC objective
includes the two lookahead representatives and must not be labeled operating-day
settlement cost. Saved `boundary.json`, three `forecast-*-boundary.json`,
`week.json`, `forecast-*-day.json`, `system-report.json` and desktop/mobile images
under `output/market-operation/ieee118/` make inputs and results inspectable.

Rebuilt Release Southern38 cases/22477 assertions and ASan/UBSan IEEE1181/1507
(Gurobi OFF) pass. The new IEEE118 E2E and existing operation/forecast browser
suites pass, with frontend console/overflow checks. Initial import runs failed
before solve on redundant fixed-transformer metadata; the checked alias mapping
above resolves that path without removing branches. New CTest registration is
in source; normal configure is still blocked by the dirty sibling guard, so
browser scripts were run directly. Port8089 runs the rebuilt server with a
completed IEEE118 stress-test week, while8088 and earlier tasks are preserved.
### Certified Inequality Omission

`execution.row_presolve=none|enabled` defaults to `none`. Enabled requires
compact formulation and omits only certified bound-redundant inequalities
from the SCUC/SCED solver matrices. LMP retains all rows. The GUI schema editor
exposes this execution option; rolling/forecast windows inherit it from the
saved boundary. The new benchmark final argument selects the same option.

Pre-implementation rationale: for each authored inequality `a*x+c <= rhs`,
evaluate the support of the variable box with outward-rounded products and sums,
`U=c+sum(a_i>=0 ? a_i*ub_i : a_i*lb_i)`. Only `U<=rhs` certifies omission;
infinite/sentinel bounds are inconclusive and no tolerance is added to rhs.
The projected feasible set and objective are unchanged because the omitted row
already follows from column bounds. Retained variables and all authored rows
remain available for the original1e-6 residual audit, pricing, physical output
and primal-start checks. Cost O(nonzeros+rows), memory O(rows) for a mask and
submitted-to-authored row indices. Prediction before coding:7022 fewer SCUC
rows on IEEE118 and213331 on the reduced2000 fixture. No guaranteed runtime
speedup follows because downstream presolve may already eliminate these rows.

Source: `Build::bound_redundant`, `Build::finish`, `solve` and
`model_size_report` in `src/market/southern_market.cpp`. The row certificate is
computed once and shared with the census. Solvers receive only retained rows;
returned multipliers are lifted to authored order, inserting zero for omitted
inequalities, then existing water-row scaling is undone. LMP skips omission to
retain the original pricing-row formulation. Changing solver presolve can still
select another equally optimal discrete schedule, so its conditional LMP problem
can differ; preserving LMP rows does not guarantee identical prices across
different predecessor schedules.

Result contract: existing `equalities`, `inequalities`, `nonzeros` and family
counts describe the authored compact/reference model. `model_size` additionally
returns `submitted_equalities`, `submitted_inequalities`, `submitted_nonzeros`,
actual `row_presolve` and `constraints_removed`. Its original `nonzeros` remains
the family sum. The operation GUI displays submitted totals below the original
model census. Fixed columns are not removed by this implementation. Security,
water/cascade,98-point timing, objective penalties and acceptance tolerances
are unchanged.

Verification protocol: `[row_presolve]` compares enabled/none at zero gap on the
analytic and multi-resource fixtures, checks original objectives for SCUC/SCED,
all original residuals and restored startup rows, actual row omission, and the
unchanged single-bus490000CNY /200CNY/MWh oracle. LMP requires a valid optimal
continuous solve and original residuals; identical multi-resource conditional
objectives are not required when the optimal predecessor is nonunique. An initial
overly strong multi-resource LMP equality assertion failed (457922.372 vs
457957.572CNY) while SCUC/SCED objectives matched; the test now states that
conditionality explicitly instead of claiming identical prices. Invalid modes
and enabled/reference combinations are rejected. Original residuals include
all omitted inequalities, not only rows sent to the solver.

Performance protocol: `tests/run_southern_row_presolve_comparison.mjs` executes
three alternating enabled/none pairs per Gurobi and HiGHS, IEEE118,30s/GAP.01,
Gurobi4 threads and default HiGHS threads,120s process watchdog. It requires
valid full chains, original residual <=1e-6, exactly7022 SCUC rows omitted,
LMP omission0 and SCUC/SCED objectives within the requested1% tolerance.
Standalone reproduction:

```bash
node tests/run_southern_row_presolve_comparison.mjs output/market-operation/row-presolve-isolated
build/macos-release/tests/test_southern_market '[row_presolve]'
ASAN_OPTIONS=detect_leaks=0 build/macos-asan-ubsan/tests/test_southern_market '[row_presolve]'
node tests/e2e/market_ieee118_e2e.mjs --server build/macos-release/tests/run_gui_server --row-presolve
build/macos-release/tests/run_southern_market_benchmark 2000 output/market-operation/row-presolve/2000-inspect.json 120 compact auto gurobi 120 inspect enabled
```

Measured2000 input reduction matches prediction exactly:3333454 to3120123 rows,
10515386 to9950617 nonzeros, unchanged2569364 columns/145040 binary declarations.
This is assembly evidence, not a completed2000 solve. First exploratory IEEE118
median process times none/enabled: Gurobi3.315/3.287s, HiGHS7.346/7.150s.
Some first-pass timings overlapped compilation/regressions; do not treat them
as controlled speedups. Small observed gains support keeping default `none`.

Final sequential rerun after compilation/regressions completed, no agent-owned
concurrent solve: three-pair median none/enabled Gurobi3.345/3.330s (0.45%),
HiGHS7.358/7.163s (2.65%). All pairs passed the prescribed protocol; max original
residual was6.06e-9. Gurobi SCED omitted30210 rows, HiGHS30812, reflecting their
different fixed predecessor states. Only the7022 SCUC prediction was fixed
before the run and it matches exactly. Evidence:
`output/market-operation/row-presolve-isolated/comparison.json`. Three local
pairs do not establish a general speed guarantee; backend presolve limits the
incremental benefit, so default remains `none`.

Release Southern39 cases/22538 assertions and focused ASan/UBSan1/61 pass.
Southern editor E2E verifies enabled save/reload and actual omission; operation,
forecast and enabled IEEE118 system E2E pass. Seven stress-day deficit/surplus/
line-excess integrals match the saved preceding default results within1e-6.
The source registers enabled IEEE118 E2E separately; cached configure remains
guarded by sibling dirty state and scripts were executed directly. Top-level
run_gui_server is updated; port8090 is the new service and earlier tasks remain.

## Native Root Cut Experiments

`execution.native_root_cuts=default|enhanced` is an optional Native-only
experiment. Missing means `default`; enhanced with HiGHS/Gurobi is rejected.
It is also accepted by rolling/forecast `operation.solver_options` (direct
rolling JSON uses `solver_options`). The boundary schema editor and both
operation advanced solver editors persist it. Changing the GUI backend to
HiGHS/Gurobi resets this explicitly backend-specific control to default.
Capability entry `native.root_cut_profiles` advertises support. New static
frontend assets hide/omit the field against older server binaries lacking this
capability, so existing local sessions can still submit their older contract.

The market calls the same public `solve_milp_bc` used by
`NativeBranchAndCutAdapter`, preserving `BCStats` before identical result
conversion. LP/pricing stages retain the existing LP adapter. No sibling code
is changed. Default requests10 root rounds and20 cuts/round; enhanced requests
20 and100, enables `root_cut_audit_force_separation`, and raises
`cut_budget_xlarge_row_threshold` from35000 to100000. These are requested
budgets, not measured rounds or guaranteed admissions. The existing size caps,
extra-large-model exclusion above the selected threshold, row audits, density/efficacy filtering, nonmoving
row rejection, deadline checks and stall termination still apply. The existing
SCUC auto-tuning and separate early root-source passes also remain; these two
numbers are not a bound on all separators in the complete solve. Enhanced is
experimental and does not assert that additional cuts are valid merely because
they preserve one incumbent. Validity relies on the existing separators; their
finite/norm/violation checks are additional numerical guards.

Admission audit: the first paired run with only20/100 and forced separation
still produced zero cuts because IEEE118 has80241 presolved rows, above35000.
This is a violated admission assumption, not evidence of separator execution.
The revised experiment admits up to100000 rows, retaining the very-large
adaptive cap of3 rounds/30 cuts and all numerical checks. Expected LP-bound
improvement remains below1CNY; at most three conventional cut reoptimizations
are permitted in this size band. This threshold also changes the upstream
`is_xlarge_root` heuristic classification; timings compare full profiles and
cannot isolate the causal effect of cuts alone. Larger roots still require a
separate performance and memory study. The original gated runs are retained
under `native-root-cuts-admission-gated/`; final profile runs use
`native-root-cuts/`.

Rationale fixed before implementation: for a minimization MILP with integer
feasible set X and LP relaxation P, valid cuts H satisfy X subset H and
`z_LP(P) <= z_LP(P intersect H) <= z_IP`. If the initial LP objective already
matches a verified integer optimum, increasing cut budgets has no significant
objective-bound room; fractional LP states can still exist through degeneracy.
Per-round cost is separation plus O(nnz+nnz(cuts)) sparse rebuild and LP
reoptimization; increasing rows can increase sparse factor fill. A speed gain
requires those costs to be smaller than avoided tree/heuristic work. Baseline
IEEE118 trace had root LP1.084s, objective17413231.016941711,796 fractional
integer variables, zero early source cuts, then feasibility-jump work; total
39.794s solver/40.070s wall for30s request, no verified incumbent. The objective
matches the prior Gurobi optimum. Prediction: enhanced objective-bound lift
below1CNY, no speedup asserted. Default remains unchanged.

References are the relaxation inequality above and the sibling implementation
`include/mipsolvers/engine/bc/options.hpp`,
`src/engine/solver/native/milp/bc/legacy/bc_run/05_root_relaxation_d.inc`
(size/fractionality admission), `06_root_heuristics_a.inc` (stall/nonmoving
checks), and `native_adapters.cpp` (the original MIP dispatch). Do not add hard
capacity covers `sum(Pmax*u) >= load` to diagnostic market runs: admitted
deficit variables make that inequality invalid. A capacity-derived inequality
must retain permitted slack, imports, storage and controllable-load effects.
Cuts and cutoff-scoped conflicts are not reused across changed daily boundaries.

Each stage emits `native_diagnostics`, null on non-Native-MIP paths. It contains
profile and requested budgets, root row count (null if not collected), selected
row admission threshold, backend collection scope and availability,
`cuts_added` over the full search, `root_source_cuts_added` for the upstream
early source collector (NOT every root cut), rejected nonmoving rows, nodes,
LP solves and incumbent updates. Bounds use CNY with the market constant cost
restored. Infinite/sentinel bounds and unavailable counters are null; a solver
incumbent is distinct from the market's original-row feasibility gate. No root
completion flag or exact measured round count is invented: upstream BCStats
does not supply them. Logs remain necessary to distinguish admission skips,
root-LP failure and pre-separation heuristic exhaustion. Rolling day summaries
retain this object, and GUI shows full-search cut counts, LP/node counts,
incumbent updates and available bounds even for unsuccessful days.

Fixed validation protocol: `[root_cuts]` checks default/invalid/backend mismatch
and both profiles against analytic490000CNY /200CNY/MWh with all original
residuals<=1e-6. Release full Southern regression plus focused ASan/UBSan,
Southern editor save/reload and a real Native enhanced rolling day in operation
E2E check the full contract. The manual comparison driver uses two alternating
default/enhanced pairs on IEEE118,30s/GAP0.01, unchanged compact15-minute96+2
model, row omission off,120s external watchdog, sequential runs after build and
regressions. Failures are retained as failures, not zero imbalance or prices.
Every successful stage must pass original residual<=1e-6. Preserve cut counts,
bounds, incumbent presence, logs, requested and actual time; this is an
applicability experiment, not a claim of guaranteed acceleration.

```bash
build/macos-release/tests/test_southern_market '[root_cuts]'
ASAN_OPTIONS=detect_leaks=0 build/macos-asan-ubsan/tests/test_southern_market '[root_cuts]'
node tests/e2e/southern_market_e2e.mjs
node tests/e2e/market_operation_e2e.mjs --server build/macos-release/tests/run_gui_server
node tests/run_southern_root_cut_comparison.mjs output/market-operation/native-root-cuts
```

Final measured evidence (Apple M4 Max,16 logical CPUs,arm64/macOS, cached
macos-release; dirty HySim8b93145b / sibling e6c932e5; sequential after builds
and tests) is retained in `output/market-operation/native-root-cuts/comparison.json`:

| Order | Profile | Solver seconds | Process seconds | Added cuts | Incumbent |
|---|---|---:|---:|---:|---|
| 1 | default | 36.537 | 38.592 | 0 | none |
| 2 | enhanced | 36.679 | 36.978 | 0 | none |
| 3 | enhanced | 37.056 | 37.357 | 0 | none |
| 4 | default | 37.520 | 37.822 | 0 | none |

All runs report80241 root rows,5 LP solves,0 explored nodes,0 incumbent updates
and the same17413231.01694171CNY best bound. Measured bound improvement0CNY
matches the predicted<1CNY. Early source logs return zero candidates/admissions;
the conventional loop also leaves zero admitted cuts in the final counters.
No per-family or completed-round count is inferred from these zeros. Later
trace reaches feasibility_jump with no incumbent. All four stop at the time
limit without verified schedules or prices. The timings do not show improved
solution throughput; profile classification changes and small timing differences
are not evidence that cuts accelerated a solved instance. These are pre-repair
measurements; the subsequent Native Fixed-Integer Repair Debug section resolves
the IEEE118 incumbent failure.2000-bus Native solve performance has not been
fixed or rebenchmarked by the root-cut profile change.

Release Southern40 cases/22567 assertions pass; focused ASan/UBSan1/29 passes
with Gurobi disabled. Analytic objectives/prices and original-row feasibility
pass for both profiles. Southern schema editor save/reload, operation week/month
plus actual enhanced Native day, forecast controls/regression, mobile/desktop
and legacy-capability omission checks pass. An asynchronous E2E interception
teardown race was corrected by awaiting unrouteAll before browser disposal;
the final operation process exits0. The final metadata-only capability addition
was verified in Release/browser after the numeric sanitizer run. No new
nonzero-cut validity coverage or2000-bus throughput claim follows from these
zero-cut118 observations. New server8091 retains the IEEE118 diagnostic case;
earlier local server sessions remain untouched.

The8091 live rolling day uses its resolved D+1 forecast representatives, so it
is not the benchmark's identical98-point boundary. Its enhanced run reports
80239 root rows,7 LP solves,2 rejected nonmoving rows,0 admitted cuts and no
incumbent after36.979s solver time. Best bound17414384.32825787CNY must not be
compared as a cut lift against the separate single-day benchmark objective.
`native-root-cuts/live-rolling.json` retains that resolved day, and
`live-{desktop,mobile}.png` shows the real failure diagnostics. The failed day
does not supply a valid dispatch, imbalance statistic or price observation.
With no valid periods the GUI now states that valid statistics are unavailable
and hides/purges empty plots; it does not show zero totals or a default2000
date axis. Partial-job totals explicitly refer only to valid periods.

## Native Fixed-Integer Repair Debug

The IEEE118 no-incumbent failure was traced into sibling MIPSolvers, not a
missing market constraint reduction. For fixed integer preferences z, domain
propagation produces bounds l(z),u(z); the repair subproblem remains
`min c'x : A*x<=b, Aeq*x=beq, l(z)<=x<=u(z)`. Changing its LP kernel preserves
the feasible set and objective. Every accepted integer candidate still passes
the existing original-row, bound and integrality audits. Failed repair cannot
prove infeasibility of the original market. No market constraint, penalty,
calculation step or 96+2 structure was removed by this repair.

Source branches in sibling `bc/legacy/bc_run/06_root_heuristics_a.inc` now:

- Dispatch fixed-integer repair to the selected HiGHS LP kernel. Previously a
  size-triggered `use_ipm_root` flag selected an unbudgeted cached NativeIPM
  even when the root/node LP kernel was HiGHS. Baseline sampling found
  1368/1465 main-thread samples in NativeIPMLPAdapter and about2.2GB footprint.
- Pass a single remaining-time snapshot minus the existing finalization
  reserve to that LP solve. Optional repair/feasibility-jump entry, attempt and
  flip loops check the deadline; the flip check is in `07_root_heuristics_b.inc`.
- Mark root polishing complete when `auto_highs_root_pipeline=false`, and skip
  optional incumbent fixed-point processing when audited incumbent U and root
  bound L already satisfy `max(0,U-L)/max(1,abs(U)) <= gap_tol`. The latter
  shortcut requires no explicit tree-exhaustion request. The full automatic
  HiGHS root pipeline retains its own polishing rules. Gap acceptance is not
  a claim of exact equality or full tree exhaustion.

The cost model is O(nnz+n) sparse-copy traffic/memory plus the existing HiGHS LP
solve. The pre-change prediction was repair below3s, full market chain below15s
(at least50% reduction from roughly37s), and a verified incumbent within the
30s request, assuming a feasible continuous completion of integer preferences.
First measurement (`native-repair-debug/fixed.json` and `.log`) gave0.397s
repair and an incumbent at2.435s, but45.103s overall. This mismatch exposed the
polishing/GAP control-flow defect above; it was not grounds to weaken model
constraints. With both fixes, `fixed-gap.json` gave6.401s overall/3.532s SCUC,
objective17413261.53763228CNY, gap1.75273e-6 and valid SCED/prices. The original
below15s criterion remains in force. This first timing overlapped compilation;
controlled measurements are retained separately in `native-repair-fixed/`.

Validation uses `tests/test_southern_market.cpp [native_repair]`: IEEE118 with
both profiles,30s/GAP0.01, full original/reconstructed residuals<=1e-6,98-point
dispatch, valid prices and objective within1% of the independently verified
Gurobi17413231.016942CNY solution. `[root_cuts]` retains the analytic objective
490000CNY and LMP200CNY/MWh. The new registered `market_ieee118_native_e2e` runs
the existing seven-day stress and forecast-resolution protocol with explicit
Native B&C ownership; its60s per-solve budget is distinct from the30s timing
experiment. Native artifacts are under `output/market-operation/ieee118-native/`.

The stress week's first-day SCUC objective171841311.522024CNY agrees with the
stored Gurobi171841311.5221077CNY. Both include the next day's16968MW peak as
lookahead slot97; the single-day fixture has a different future boundary.
Daily imbalance/line-excess integrals still cover only the executed96 slots.
Thus these objective differences do not demonstrate a solver or accounting
error. Synthetic forecasts/offers and schedule-only AC scope remain declared.

Reproduction (macOS arm64, Apple M4 Max, cached builds; dirty HySim8b93145b and
MIPSolvers e6c932e5 with the described source changes):

```bash
build/macos-release/tests/test_southern_market
build/macos-release/_deps/mipsolvers_build/test_branch_and_cut
node tests/run_southern_root_cut_comparison.mjs output/market-operation/native-repair-fixed
node tests/e2e/market_ieee118_e2e.mjs --server build/macos-release/tests/run_gui_server --solver native
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 HACDCPF_TEST_NATIVE_REPAIR_SECONDS=180 build/macos-asan-ubsan/tests/test_southern_market '[native_repair],[root_cuts]'
```

The sanitizer cache uses `-g -fsanitize=address,undefined` without optimization;
30s runs timed out without a verified schedule, including an isolated rerun.
The test-only environment override above changes the instrumentation budget,
not the production default or Release performance criterion. Cooperative
deadlines cannot preempt a factorization/domain traversal, and SCED/LMP plus
assembly/audit are additional work; a SCUC budget is not a whole-chain hard
deadline.2000-node Native performance and general nonzero-cut acceleration
remain unvalidated by this IEEE118 repair.

Current-source Release Southern regression passes41 cases/22597 assertions;
sibling `test_branch_and_cut` passes35/490. Operation and forecast GUI/API
suites pass, including Native profile selection and legacy capability handling.
The Native IEEE118 E2E passes seven stress days plus three forecast-resolution
runs with valid schedules/prices and original residuals<=1e-6. Its day times
5.366/5.344/5.708/21.952/5.226/6.146/5.242s overlapped other tests and are not
controlled throughput measurements. Full-network outage is the slowest day.

Build provenance: normal cache regeneration is blocked by the existing dirty
sibling dependency guard. Changed translation units were compiled through
cached target build rules, the generated branch-and-cut object was forced to
rebuild after `.inc` edits, the static archive was recreated and executables
relinked. The sibling regression target's stale snapshot source path was
replaced only in the compile invocation with the current sibling source;
its temporary link invocation used the installed GCC16 runtime paths. This is
a verified dirty-worktree experiment, not a clean pinned-dependency build or
full CTest baseline.

Updated executable `build/macos-release/run_gui_server` is served on8092;
older services remain running. Real browser clicks selected Native/enhanced,
30s/GAP0.01 and a4x-load stress at slots40..43 (zero-based), then paused after
day1/7. Live chain5.626s, SCUC root-gap closure, deficit6055.140450913252MWh,
line-excess integral1045.97991772318MW*h and valid prices were verified. This
demonstration disables single-factor counterfactual re-solves; it does not
claim a new causal experiment. `native-repair-fixed/live-rolling.json` and
desktop/mobile screenshots retain actual results, charts and Canvas selection.
The390px mobile document has no horizontal overflow. Live timing overlapped
sanitizer work and is separate from the controlled comparison.

Final controlled alternating runs (after all builds, test solves and browser
checks completed; same host/revisions and30s/GAP0.01 protocol):

| Order | Profile | Full chain s | SCUC s | Process s | Schedule / Prices |
|---|---|---:|---:|---:|---|
| 1 | default | 6.2894 | 3.5182 | 6.404 | valid / valid |
| 2 | enhanced | 6.3867 | 3.5848 | 6.495 | valid / valid |
| 3 | enhanced | 6.2885 | 3.4847 | 6.394 | valid / valid |
| 4 | default | 6.2757 | 3.4864 | 6.387 | valid / valid |

All satisfy the unchanged below15s pipeline prediction, produce SCUC objective
17413261.53763229CNY and gap1.7527268e-6, and pass original/reconstructed audit
at1e-6. Max original SCUC residual9.8500033e-7, SCED3.4259e-11, LMP4.9170e-12;
the recovered-constraint residual is at most1e-7. Diagnostics show1 incumbent
update,3 SCUC LP solves,0 explored nodes and0 added cuts in each profile. Bound
17413231.01694174CNY remains unchanged. Compared with the pre-fix37-second
no-incumbent runs, this is a successful repair and earlier certified exit, not
evidence for stronger cuts. The enhanced profile supplies no measured bound
lift or meaningful speed advantage on this fixture.

ASan/UBSan180s-per-solve validation passes2 cases/62 assertions, covering both
IEEE118 profiles plus the analytic root-cut test, with no sanitizer report and
leak detection disabled. Its integer repair LP is about10s, compared with the
subsecond Release repair, and SCUC about94..96s; the unoptimized instrumentation
cost explains why the30s test remains non-green. `native-repair-fixed/asan-30.log`
preserves that failure, `asan-180.log` the explicit extended-budget pass. This
does not expand the Release performance claim to unoptimized builds.

## Mixed IEEE118

`make_southern_market_ieee118_mixed` in `src/market/southern_boundary.cpp` adds
version `IEEE118-mixed-v1`. It is available as global built-in `market_ieee118`
and Southern/operation action `ieee118_mixed`; old `ieee118` / benchmark `118`
remain unchanged performance controls. The benchmark accepts `118-mixed`.
GET `/api/session/southern_market` advertises `case_profiles`; static editors
disable the new option when connected to a pre-feature binary.

The original118 buses,186 pi branches (including fixed taps/shifts),54 unit
IDs/buses/Pmax/P-Q limits remain. Sort original unit IDs and assign two hydro
then one thermal:36 hydro/6566MW and18 thermal/3400.2MW. Add6 wind/480MW and
6 solar/600MW,6 storage (each20MW/80MWh) and6 compensated loads (each20MW,
80MWh/day). Each of12 reservoirs owns3 original hydro units; four independent
three-reservoir chains use one-slot travel lag. The synthetic200MW/MVA branch
limits remain explicit. Fuel classification is a research assumption, including
the original synchronous-condenser records; these are not identified real
hydro plants or Southern market quotes.

Model/data rationale: replacing a synthetic fleet is not an equivalent solver
reduction. New counts66 units/12 reservoirs/6 storage/6 loads are structural
acceptance criteria, not a throughput prediction. Data assembly scales with
T*(B+G+H); network sparsity and118-bus topology stay fixed. Original hourly
demand is multiplied by `1+0.15*sin(2*pi*(hour-7)/24)`, whose96-point mean is1,
preserving101808MWh/day. Three incremental bid segments per dispatchable unit
use capacity fractions0.40/0.35/0.25; hydro base prices35..47CNY/MWh and
thermal base prices186..339CNY/MWh are synthetic. Thermal minimum on/off times
are60min, startup costs500CNY, rampPmax/30MW/min. Hydro rampPmax/5MW/min.
Wind uses phase-shifted daily cosine profiles; solar is zero outside6..18h.
Reservoirs use1e7m2 surface,100m initial level,95..105m limits and3600m3/MWh;
root inflow0.5*local Pmax, downstream local inflow0.15*local Pmax in m3/s.
Hydro daily energy ceiling is0.75*24*local Pmax. These are explicit editable
boundary assumptions; changing them changes the feasible set.

Independent validation in `market_ieee118_resources_e2e.mjs` re-derives:

- Shared turbine release: `q_release=q_spill+sum(Pg)*water_m3_mwh/3600`.
- Water continuity: `level[t]=level[t-1]+dt*3600/area*(local_inflow+delayed_parent_release-release)`;
  the first point uses authored initial level and upstream release history.
- Storage energy: `E[t]=E[t-1]-dt*(Pdis/eta+eta*Pcharge)` with negative
  charging power, eta0.9; no simultaneous charge/discharge and terminal energy.
- Controllable-load availability/power/day-energy limits; actual resource
  outage causes zero dispatch/response, not merely a changed label.

`make_ieee118_market_system` supplies a companion engineering snapshot for
the global built-in. It splits flexible-load demand out of bus Pd without
double counting, uses the initial renewable forecast as static capability,
and carries generator types/costs, storage and flexible-load records. The
generic market does not optimize AC-storage SOC, compensated interruption or
reservoirs; `market_simulation.cpp` now explicitly reports all three relevant
limitations. The Southern JSON boundary owns those optimization contracts.
Global case load installs both; replacing/synchronizing the engineering model
still invalidates Southern results through the existing session lifecycle.

GUI regression exposed a genuine field-loss defect: generator Canvas
import/export omitted `fuel_type`, `min_up_time_hr`, `min_dn_time_hr`,
`max_startups_per_day`, `max_shutdowns_per_day`. Both directions now preserve
them. The fuel editor includes Wind/Solar/Unknown and every backend fuel enum;
the mixed generic E2E asserts36 Hydro/18 Coal/6 Wind/6 Solar and retained
thermal minimum times after the Canvas round trip.

Rolling reports now retain `job.days[d].resources` for valid days. Each row
uses the authored boundary `id` and `name`; array index0..95 denotes the
realized day's15-minute slot, excluding both lookahead points. Generator
rows expose `power_mw` and `online`; storage exposes `discharge_mw`, negative
`charge_mw` and end-of-slot `energy_mwh`; reservoirs expose end-of-slot
`level_m`, `spill_m3_s` and `release_m3_s`; controllable loads expose
`reduction_mw`. Units are MW, MWh, m and m3/s as named; online is the unit's
commitment indicator. The Canvas reads these arrays at the selected slot,
including charging sign and shared-reservoir identity. Older reports without
resources still show unavailable details rather than invented zero dispatch.
This is result projection only, with no constraint or objective changes.
Additional payload scales as96*(2G+3S+3H+D) values/day;2000-bus monthly memory
performance is not certified by this118-bus test. The final seven-day browser
regression checks row counts,96-point arrays and all four resource selections;
Southern42/22870 and operation GUI/API regressions pass after the projection.

### Validation Coverage

| Function | IEEE118 evidence | Interpretation |
|---|---|---|
| Built-in / editing / Canvas | Structural273 assertions; mixed browser load and selection | Stable IDs, source capability and demand-energy checks |
| SCUC -> SCED -> LMP | Mixed full-day and seven-day stress tests | Original residual<=1e-6;98 optimization points,96 realized points |
| Hydro/storage/DR | Independent equations and resource-outage experiment | Base wind/solar/hydro/thermal all used; peak activates storage and DR |
| Weekly boundaries/topology |7 days including load4x, reduced limits, outages, dry/renewable loss and bid changes | Node/line sums, signed-limit violations and carry validated |
| Monthly operation | February2027,28 completed days | Every day valid; state_start equals previous state_end |
| Joint uncertainty |2 independently sampled weekly scenarios,14 solved days, all7 marginals, correlated wind/solar | Overload probability independently recounted;2 samples are regression coverage, not statistical adequacy |
| Forecast resolution |15/30/60min inputs on unchanged15min calculation | Equal input energy; altered peaks/results, not a changed calculation step |
| AC security | Southern required/strict run returns ac_security_failed, prices invalid | Certification failure preserved; diagnostic slacks cannot be combined with required AC certification |
| Generic behavior/N-1/settlement/game | Separate engineering-snapshot GUI protocol | N-1/AC diagnostics and failed-baseline rejection are tested; financial runs explicitly disable AC certification |
| Southern real-time settlement / full AC/DC / nonlinear hydro | Not covered by this fixture | Not implemented as one unified multi-resource settlement chain; no full-rule equivalence claim |
| Forecast price/utilization aggregation | Existing forecast API lacks these aggregates | Single-day utilization is computed independently in test report; not presented as a production forecast-distribution feature |

Measured Gurobi resource evidence (`output/market-operation/ieee118-mixed-resources/`):
base hydro91986.555929MWh, thermal55.418073MWh, wind5184MWh, solar4582.025998MWh;
renewable utilization1.0. Base storage and DR are optimally unused. A4x-load
stress at slots40..43 activates148.148148MWh charging,120MWh discharging and
120MWh compensated interruption. Disabling wind/solar/storage/DR makes all
four corresponding outputs zero; absent renewable availability has null
utilization, not a fabricated ratio. The two probabilistic weeks have no
observed overload; Wilson95 upper bound is0.65762, not evidence of negligible
system risk. Full28-day month and7-day stress results are retained separately.

Reproduction using the rebuilt cached macos-release server:

```bash
build/macos-release/tests/test_southern_market '[ieee118_mixed]'
node tests/e2e/market_ieee118_e2e.mjs --server build/macos-release/tests/run_gui_server --mixed
node tests/e2e/market_ieee118_resources_e2e.mjs --server build/macos-release/tests/run_gui_server
node tests/e2e/market_gui_e2e.mjs --server build/macos-release/tests/run_gui_server --builtin market_ieee118
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 build/macos-asan-ubsan/tests/test_southern_market '[ieee118_mixed]'
```

Three new CTest entries register these system protocols; direct invocation
remains necessary in the dirty-dependency guarded cache. Focused structural
ASan/UBSan passes1/273, leak detection disabled; this is not a full sanitized
month or new sanitized solver regression. Full Southern Release passes42
cases/22870 assertions, retaining the original IEEE118 Native performance
test. Numeric run times overlap other regression work and are not controlled
cross-solver speed rankings.

Final generic GUI test passes behavior editing, N-1 diagnostics, rejected
AC-invalid real-time baseline, conditional linear two-settlement ledger and
finite-candidate repeated game. A stationary strategy with no best-response
profit improvement is valid convergence; movement is not required on every
new fixture. Original case9 still tests actual strategic movement. Recorded
generic statuses are DA `n1_security_failed`, conditional RT `converged`, game
`converged` in1 round, cashflow residual2.0009e-11CNY. N-1 branch113/183 outages
cause2.5653/78.6702MWh load shedding; these are meaningful security failures,
not missing result arrays. Current-source generic C++ market regression passes
22/845. Its cached link file had obsolete GCC15 paths and HFactor duplicate
archives; a temporary link invocation reused the current Southern test's
libraries with the freshly compiled generic test object. Original generated
files and pinned dependency policy remain unchanged.

New fixture backend diagnostics (60s/GAP0.01, overlapping other test activity):
Gurobi5.856s and HiGHS14.956s full chain, both verified objective
4212673.15329467CNY; original SCUC residuals1.08e-10/1.25e-11. Native4.777s
returns `Root relaxation failed`; traced rerun confirms both root attempts hit
`IterationLimit iter=25000`, with0 incumbents and unavailable prices. This is
not proof of model infeasibility and does not contradict the old-fixture Native
repair result. Logs under `ieee118-mixed/` and the retained trace capture the
remaining Native iteration-budget applicability gap; no algorithm/constraint
relaxation was made to turn this failure into a success.

Initial mixed-fixture GUI validation used `http://127.0.0.1:8094/xjtu/#market-operation` with the
mixed built-in loaded. Live first stress day uses Gurobi60s/GAP0.01 and pauses
after1/7 for inspection/resume. Earlier services remain available; new controls
are capability gated to prevent submitting this action to old backends.
Latest live run takes5.062s and reports8010.076869MWh deficit under the authored
4x peak-load stress. Desktop/mobile screenshots and `live-operation.json` in
`ieee118-mixed/` preserve the result; four resource selections, slot-driven
topology changes and390px horizontal-overflow checks pass. The served binary
was replaced atomically after macOS killed an in-place-overwritten executable;
the replacement process and live browser workflow were then verified.

## Weekly Plan Results

`market_operation.cpp::summarize` now projects every valid realized day into
the weekly/monthly plan. `southern_market.cpp` exports `primary_reserve_mw`
from its existing `primary` decision variable; neither formulation nor solve
parameters change. `web/js/core/market_weekly_plan.js` supplies six graphical
overviews (generation, commitment, reserve, line loading, renewable consumption,
reservoir levels), a40-device-per-page heatmap and a selected-device curve.
Tables are collapsed by default. Every device remains selectable and CSV
exports every filtered device and all calendar slots, independent of pagination.
Weekly curves contain672 points; monthly curves follow the calendar length.
No D+1 representative point is mixed into the realized timeline.

### Result Mapping

All resource keys use boundary-authored stable IDs, qualified by component
table. Array indices0..95 are15-minute realized slots. Generator `kind`, `bus`
and `area` accompany its series. The `online` field is fixed commitment;
`power_mw` is the SCED power base point, not a separate reserve award.

SCED reserve contribution follows2.6.3.2--4 exactly. With generator output P,
commitment u, eligibility flags alpha and precleared regulation R:

```text
up_g   = alpha_up * (Pmax - R_up) * u - P
down_g = P - alpha_down * (Pmin + R_down) * u
up_area   = sum(up_g) - sum(storage_discharge + storage_charge)
down_area = sum(down_g) + sum(storage_discharge + storage_charge)
up_required   = reserve_up + network_reserve_reduction
down_required = reserve_down - load_side_down_reserve
margin = contribution - required
```

`reserve_up_contribution_mw` / `reserve_down_contribution_mw` deliberately
retain negative contributions, including output of ineligible generators.
They are not independently awarded reserves or ramp/deliverability-certified
capacity. `primary_reserve_mw` is the actual SCED primary-response variable.
`resources.areas` carries `reserve_up_mw`, `reserve_down_mw`, each corresponding
`*_required_mw` and `*_margin_mw`. All are MW. Overview sums of area quantities
do not imply that reserves can be transferred between areas.

For renewable generators, `renewable_available_mw` is
`min(Pmax, forecast)*available*(1-must_off)` from the effective daily boundary;
`curtailment_mw=max(0,available_power-P)` and
`utilization_percent=100*P/available_power` when the denominator is positive.
Other generator kinds and zero availability have null utilization. Daily and
whole-period utilization use the ratio of summed energy (0.25h per slot),
not an average of percentages. Missing or invalid samples are excluded and
identified by the completed-day coverage. Storage charging remains negative.
Water levels and storage energies are end-of-slot values.

`lines.loading_percent=100*abs(P)/directional_limit`, where the limit is
`max_mw` for nonnegative P and `-min_mw` for negative P. Outages, a nonpositive
directional limit or a signed interval not spanning zero return null. This is
active-power limit utilization, not AC apparent-power loading; the existing
`overload_mw` / `delta_pij_mw` remain the authoritative violation quantities.

The GUI shows gaps for pending/failed/stale days and missing fields in older
reports, never a fabricated zero. It supports type/search filters,16 metrics,
full-calendar CSV, device/period selection, and Canvas navigation by qualified
stable ID. Clicking the detailed heatmap or curve updates the result-day/slot
controls. A new browser without saved mode preferences opens an existing
manual-week result when no forecast job exists; explicit user mode preferences
remain authoritative. On first entry, an existing daily result defaults to
workflow step4 unless the session has a saved step.
The automatic mode check runs after forecast controls are initialized. Moving
it before editor construction initially blocked `forecastSolver` creation when
operation capabilities were delayed; the gated-capabilities forecast E2E
caught this regression, and the reordered initialization passes that test.

### Validation and Cost

Pre-change prediction: zero change in optimization variables, constraints,
nonzeros and objective; report projection scales with96*(G+L+A*(G+S)) per day,
linear in device count for bounded area count. Added generator/area/line arrays
cost96*(6G+6A+L) values per day. The heatmap renders at most40*96*days cells;
the complete export still covers all devices. Overview curves aggregate all
devices; reservoir overview currently plots all reservoirs. No2000-bus monthly
browser-memory/performance certification is claimed.

Measured: all7 mixed-IEEE118 stress-day objectives have exactly zero difference
from the previous retained log, and variable/nonzero counts match on every day.
The analytic100MW example independently requires20MW primary reserve and
100MW up/down contributions; Release and ASan/UBSan pass1 case/963 assertions.
Full Southern Release passes43 cases/23930 assertions, including directional
loading and outage null checks. Sanitizer scope is the analytic projection
case, with leak detection disabled; it is not a sanitized full-week run.

The registered mixed IEEE118 E2E now checks all resource counts, full672-point
curves, effective-boundary reserve/renewable formulas, loading, CSV row counts,
pending/stale gaps, all six overview plots, type filters and Canvas navigation.
New numerical projection/plot assertions use an absolute1e-6 threshold.
It passes7 rolling stress days plus15/30/60-minute forecast inputs with fixed
15-minute calculation. Operation and forecast GUI/API regressions also pass.
The final live browser check covers an actual heatmap mouse click, a fresh
session opening the saved week, desktop/mobile rendering, and full heatmap
container height (the inherited300px chart height initially clipped rows and
is now overridden by the calculated heatmap height).

Reproduce the new checks:

```bash
build/macos-release/tests/test_southern_market '[weekly_plan]'
node tests/e2e/market_ieee118_e2e.mjs --server build/macos-release/tests/run_gui_server --mixed
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 build/macos-asan-ubsan/tests/test_southern_market '[weekly_plan]'
```

The updated top-level executable serves a complete seven-day mixed-case stress
plan at `http://127.0.0.1:8095/xjtu/#market-operation`. Previous services remain
available. Numerical/CSV artifacts are in `ieee118-mixed-week/`; live report,
screenshots and the repeatable browser inspection are in `ieee118-mixed/`
(`live-weekly-operation.json`, `weekly-*.png`, `live-weekly-plan.mjs`).
The guarded cached build was used; no new full configure/CTest claim is made.

## Market Canvas Layout

`web/js/core/market_canvas.js` reuses the vendored ELK0.9.3 layered layout with
orthogonal edge routing. The former numeric-ID grid and always-visible line
annotations caused long crossings and tiny overlapping labels. The default
view now includes at most24 buses within two hops of the selected device;
the explicit extended view uses breadth-first expansion up to80 buses. All
authored buses and resources remain in the device picker. Counts identify the
visible/total buses; `+N` denotes physical connections beyond the displayed
neighborhood. This is a schematic view filter, not electrical aggregation,
network deletion, geographic coordinates or a new market-model approximation.
Boundary total counts are published immediately, separately from asynchronous
layout readiness (`aria-busy` and `data-layout`).

ELK reserves64x48 layout units per compact node or138x88 when numeric labels
are enabled. Each branch has its own fixed-side port, fanned within the bus
symbol's vertical extent; shared ports initially produced5 duplicate paths
in the128-line extended test and are no longer used. Distinct parallel routes
retain the authored branch IDs. Default graphics show bus IDs and boundary
connection counts; numeric bus and resource labels are opt-in. Empty resource
labels are omitted. Branch values remain available through tooltips and the
selected-device inspector, so normal lines have no overlapping inline text.
Existing imbalance/overload colors, outage dashes and result-validity gating
remain authoritative. Reservoir selection highlights all of its represented
generator buses; selecting a branch includes both endpoints.

Layout is asynchronous and guarded by a monotonically increasing request ID.
Selecting another external device, or returning to an already visible device,
invalidates an obsolete pending layout. The12-entry boundary-local layout
cache and same-revision reload check retain deterministic positions and the
viewport. Selecting a visible device or changing time does not relayout or
reset zoom. Fit restores the current graph bounds; recenter explicitly builds
the neighborhood of the selected device. A new boundary revision invalidates
the cache. A missing/failed ELK layout is reported as failed, without silently
claiming successful topology rendering.

The Canvas uses separate toolbar, bounded topology, legend and scrolling
inspector tracks. Mobile allocates720px to this workspace; short desktop
viewports scroll the left workspace instead of overlapping its regions.
`market_canvas_layout_e2e` is registered and passes on a newly spawned mixed
IEEE118 server and on the live seven-day result at8095. Assertions cover
nonoverlapping node/label boxes, distinct parallel routes, local/extended
coverage, asynchronous selection cancellation, stable zoom/time selection,
branch endpoints, shared-reservoir highlighting, and desktop/mobile geometry.
Existing operation GUI/API and Southern editor regressions pass, including
actual line clicks, changing overload values and2000-bus identity navigation.
These tests do not establish a crossing-free embedding or2000-bus full-graph
rendering performance. No C++ model, solver, result schema or numerical
baseline changed in this presentation revision.

```bash
node tests/e2e/market_canvas_layout_e2e.mjs --server build/macos-release/tests/run_gui_server
node tests/e2e/market_canvas_layout_e2e.mjs --base http://127.0.0.1:8095
```

The second command reads the existing mixed-case session without changing its
boundary or solved days. Screenshots and geometry evidence are retained under
`output/market-operation/canvas-layout/`. The existing8095 service serves the
updated static assets; its seven-day results are preserved. CTest registration
was updated in source and the script was invoked directly in the guarded cache.
