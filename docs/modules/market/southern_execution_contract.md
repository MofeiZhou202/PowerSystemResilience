# 南方区域日前执行模型与边界契约

本文件记录 `southern_market` 的执行解释、字段归属和验证门槛。
规范基准为正式附件第 2.4、2.6 节；原式和歧义见
[逐条比较](southern_rules_comparison.md)。执行解释不是发布机构勘误。

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

`execution.solver`支持`highs`（旧快照默认）和`gurobi`，不自动替换。
`execution.threads`默认0；非零仅接受Gurobi（最大128）。`time_limit_sec`与
`mip_gap`沿用原范围；Gurobi参数作用于每个SCUC/SCED/LMP优化调用，LP也限时。
时限不包含建模、许可证启动或全部恢复实验，仍不是整日硬预算。HiGHS仍仅MILP限时。
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
`time_limit_sec`在HiGHS路径仅用于MILP，Gurobi路径用于MILP/LP；均不能解读为
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
平移、缩放、适应与键盘 Enter/Space 选择均可用。大于 80 节点时显示所选设备
的至多 80 节点 BFS 邻域并标出可见/总数；完整设备菜单仍覆盖全部实体。
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
当前 12/12 场景有效，16 项断言通过。统计量下一步将序列化为场景报告并
加入 Spearman/PRCC 相关性矩阵。

1. **方程与守恒层：已执行。** `[hand_oracle]` 覆盖节点负荷削减、发电量、补偿费用、同库多机组下泄量和水位递推；手算断言 12 项全部通过。
2. **解析最优层：已执行。** 单母线单机组、储能、拥塞和水库算例均有确定的人工最优值或边界不可行值，测试位于 `tests/test_southern_market.cpp`。
3. **性质层：部分执行。** 已覆盖负荷微扰、线路/断面限额、备用不足、补偿价格阈值和水电日电量上限；随机性质测试、可行域包含关系和多流域交叉耦合测试仍待补齐。
4. **外部交叉验证层：待执行。** 计划将小型边界导出到独立 Python/HiGHS 或 SCIP 模型，并用 MATPOWER/pandapower 复核 AC 潮流。当前没有外部 oracle 结果，内部 HiGHS 回归不作为外部验证。

缺少真实报价时，报价字段均标记为 synthetic/研究输入；验证结论限定为方程、约束、物理守恒和给定实验报价下的优化性质，不解释为真实市场价格认证。
# 2000-bus exact formulation experiment

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
