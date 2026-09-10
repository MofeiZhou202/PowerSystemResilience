# 智能仿真标签与价格代理

本文是离线工具的实现契约与实验记录。研究问题见[边界优化数学模型](../../theory/market_boundary_optimization_model.md)；生产出清仍遵循[南方执行契约](southern_execution_contract.md)。v1在固定 IEEE118 系统/边界/周初状态下建立合成场景响应代理；v2加入四类压力输入和线路限额扰动，系统与初态保持不变。两轮均未执行 Bayesian/CMA-ES 边界优化。

下一轮补样的[理论设计](../../theory/market_surrogate_sampling_design.md)单独记录，包含组内辨识、过渡定位、配对边界动作与独立留出样本量；其§9对应本模块的前置辨识工具，正式序贯补样尚未执行。

最新研究范围已明确为[固定各站周水电电量、优化7日分配以改善新能源消纳](../../theory/hydro_renewable_allocation.md)。在线快速分析、离线数小时学习是新专项要求；当前线限额代理不能直接预测水电配额动作，需新增动作输入、电量/水量审计和独立验证，尚未部署专项服务。

## 1. 实验协议和理论依据

协议在训练前固定：同一保存边界，24 个新周（seed 20260910–20260933），8 日预测、7 日逐日滚动，每日执行前 96 个 15 分钟点。18 周训练（前18个seed）、6 周留出测试（后6个seed）；节点和时段不拆分成独立训练样本。原输入 seed 20250905 单独运行一次 `full` 原因分析链作为校准周，其样本不进入训练或测试。

完整主链始终执行 SCUC→SCED→LMP。`recovery_pricing=full` 只控制原因分析的反事实重算，不能用它判断主链是否定价。24个新周设置 `explain=false`，避免每个训练样本执行与目标标签无关的反事实；校准周设置 `explain=true, explain_trigger=always, recovery_pricing=full`。二者的主链均有完整LMP，校准周还复核全部反事实价格。

本地独立服务器每个进程拥有自己的session，每个场景内日窗顺序执行；最多两个进程，每个SCUC使用Gurobi 2线程、120 s/阶段、请求gap 0.01。许可证/后端失败记录为错误，不自动降级或抹去失败样本。使用既有本地Release二进制，哈希写入manifest，不宣称本轮进行了干净源码构建。

输入保持保存任务的原始拓扑、资产、报价、储能/水库初态与分布。风、光、负荷、来水、发电/储能报价、可控负荷补偿为[0.9,1.1]逐日共同倍数；线路倍率固定为1；保存任务的时间相关参数为0。分布是合成研究设定，未拟合真实预测误差。

理论与成本模型：每个周评价7个真实日窗，成本约为 $7(T_{UC}+T_{ED}+T_{LMP})+T_{IO}$；并发不改变单周状态递推。历史主链约38 s/周，仅用作预测，24周串行约912 s；双进程预期降低总wall，实际与预测均须记录。标签提取对节点价格为 $O(N\log N)$ 排序，$N=118\times672=79296$；缺额、方向越限和新能源能量为线性求和。岭回归解 $\min_\beta\|X\beta-y\|^2+\alpha\|\beta\|^2$；树模型采用Extra-Trees。预测24个周的全部指标预期平均低于0.1 s/周（不含文件读取），只衡量指标预测，不能与真实出清能力等同。

依据：Hoerl & Kennard (1970), Ridge Regression；Geurts, Ernst & Wehenkel (2006), Extremely Randomized Trees；Rockafellar & Uryasev (2000), Optimization of Conditional Value-at-Risk。源码公式对应 `build_market_label_dataset.py::price_summary/extract` 和 `train_price_surrogate.py::candidates/train`。

## 2. 标签契约（schema_version=3）

输入特征是全部8日×7因素，共56维；保存完整base、归一化config、采样器与求解参数。`sample_id`来自物理base、初态、全部8日输入、起始日期和诊断罚价，不来自文件路径；求解器线程和装配方式等数值控制不改变物理样本身份。重复性能回放不得跨训练/测试分组。同名系统也必须检查`case_hash`。

`nodes`是数组，价格来自 `nodes[i].lmp_per_mwh[0:96]`，稳定节点ID随结果保存。本轮复算每周节点P95/P99、系统节点—时段等权P95/P99及均值、最大值、CVaR99。等权系统指标是所定义的节点—时间分布，不是负荷加权电费或真实市场统一结算价格。

经验分位数为 $Q_q=z_{(\lceil qN\rceil)}$，与 `market_forecast.cpp::quantile` 同口径。CVaR用

$$
\operatorname{CVaR}_{.99}=Q_{.99}+\frac{\sum_i(z_i-Q_{.99})_+}{.01N}.
$$

该式保留离散分布尾部的分数质量。尖峰阈值为独立校准周的系统P95，在查看新周训练/测试结果前按协议确定；之后所有样本用同一阈值。`spike_fraction`是价格超过阈值的节点—时段占比；`spike_duration_hr`按任一节点发生尖峰的执行时段求和，每时段0.25 h；不把这些时间频率叫作独立周事件概率。

缺额按节点`deficit_mw`积分，必须与每日`deficit_mwh`一致。当前是诊断松弛缺额，不等同于已执行失供，也不证明不可避免。线路按方向上下限独立复算 $o=\max(0,f-f_{max},f_{min}-f)$；停运线检查零流，零容量不做除法。`line_overload_integral_mwh`为有功越限积分，不是失供电量，也不是AC MVA越限。逐节点和逐线路积分均保留稳定ID。

新能源读取 `renewable_available_mw`、`power_mw`、`curtailment_mw`，乘0.25 h再求和；分母为0返回null。其可用量是当前场景的有效预测边界，不是独立实测潜在发电量。优化新能源边界时必须固定外生评价分母，防止通过削低预测虚增利用率。

日SCUC目标覆盖98点，七日相加只能命名`scuc_98point_objective_sum`。不能称为实际周成本，不能把SCUC、SCED、定价LP目标累加。第一版代理不训练此目标。

标签准入需七日完成、每次96个执行点、节点/线路/机组集合完整无重复、跨日state_end与下一日state_start相同；SCUC/SCED/LMP残差≤1e-6、实际gap不超过请求gap、价格标志有效、定价最优且独立重复定价一致性检查通过。失败行保留issues，目标为null/空，不用零填充。所有标签明确`local_linear_diagnostic_oracle`、`ac_certified=false`、`n1_certified=false`。满足本协议不代表物理/监管认证。

旧提取器v1/v2存在三类错误：将nodes数组当对象使有效LMP变空；按文件路径识别样本使重复回放虚增数量；把SCUC的98点目标合计叫作周成本。旧 `output/market-intelligence/labels*.jsonl` 中schema缺失/v2的文件不得用于训练。v3要求含base的完整导出，旧缺base文件显式拒绝。

## 3. 模型、验证与准入

每个目标单独比较：训练均值基线、标准化岭回归(alpha=1/10/100)、256棵Extra-Trees(min_samples_leaf=1/2)。按18个训练周的固定3折(seed917)交叉验证MAE选择；6个测试周仅做一次留出评估，不参与阈值、模型或超参数选择。最终模型只拟合18个训练周，测试集保持独立。

目标为价格均值/P95/P99/CVaR99、尖峰占比/时长，以及缺额、有功越限积分、新能源利用率/弃电量。训练集无变化（物理量范围≤1e-6，利用率≤1e-9）的目标记`no_training_variation`，不报告预测准确率或伪造分类能力。预测不裁剪来掩盖误差；概率/比例越界记录门槛失败。

预声明单指标门槛：测试MAE至少比训练均值基线减少10%，且比例输出不越界，才标记`predeclared_10_percent_gate=true`。不满足仍保存模型和负结果，但不能作为可信优化目标。六个测试周不足以验证罕见事件、跨系统泛化或统计置信区间；CVaR99这里只是单周节点—时段经验尾部标签。

预测接口检查相同case_hash、特征集合、采样支撑区间；越界直接要求Oracle。即使在区间内，也只是研究预测，没有校准的不确定性或安全保证。每条输出均带`oracle_verification_required=true`。当前b固定，只有ξ响应数据；尚不能用于学习任意边界动作、输出完整LMP矩阵或证明最优边界。

## 4. 复现与产物

在仓库根运行：

```bash
python3 -m venv output/market-intelligence/venv
output/market-intelligence/venv/bin/python -m pip install -r tools/market_intelligence/requirements.txt
python3 tools/market_intelligence/run_price_oracle.py --output output/market-intelligence/pilot-full --weeks 24 --workers 2
python3 tools/market_intelligence/run_price_oracle.py --output output/market-intelligence/calibration-full --weeks 1 --workers 1 --seed 20250905 --recovery
output/market-intelligence/venv/bin/python tools/market_intelligence/train_price_surrogate.py train --dataset output/market-intelligence/pilot-full --calibration output/market-intelligence/calibration-full/week-000/job.json.gz --output output/market-intelligence/price-surrogate-v1
python3 -m unittest discover -s tests -p test_market_intelligence.py -v
```

完整原始结果保存在每周`input.json.gz/job.json.gz/manifest.json/server.log`。成功周可按输入与二进制身份跳过；失败样本重跑，不能把部分周当完整缓存。服务器仅绑定本机动态端口，结束时退出自己的进程。工具为离线CLI，无新增GUI/HTTP路由。

模型目录保存`labels.jsonl`、`model.joblib`、`report.json`、`report.md`、`test_predictions.json`，其中逐周LMP原向量可从source_job溯源。`predict`子命令读取含case_hash和features的JSON，输出研究预测及各目标测试门槛。

数值验证和本轮边界见[开发状态](../../overview/development_status.md)；最终指标以模型目录中的完整报告为准。

## 5. 首轮数值结果与偏差分析

24个不同seed周的168个主定价阶段全部有效且重复对偶一致；共1,903,104个执行节点—时段价格，双进程wall 608.433 s。独立校准周178.430 s，7个主链及42个反事实定价全部通过；固定尖峰阈值59.61200124564096 currency/MWh。

| 标签 | 选中模型 | 留出MAE | 训练均值基线MAE | MAE改善 | 10%门槛 |
|---|---|---:|---:|---:|---|
| 平均价格 | Extra-Trees, leaf=1 | 11.078 | 15.005 | 26.2% | 通过 |
| P95 | Extra-Trees, leaf=2 | 50.912 | 81.611 | 37.6% | 通过 |
| P99 | Extra-Trees, leaf=2 | 55.774 | 83.112 | 32.9% | 通过 |
| CVaR99 | Extra-Trees, leaf=2 | 48.190 | 65.280 | 26.2% | 通过 |
| 尖峰占比 | Ridge, alpha=1 | 0.08217 | 0.07184 | -14.4% | 未通过 |
| 尖峰时长/h | Extra-Trees, leaf=2 | 11.541 | 12.542 | 8.0% | 未通过 |

价格MAE单位currency/MWh；占比MAE 0.08217相当于8.217个百分点。负荷缺额与线路越限全零，不训练风险模型。新能源利用率和弃电量变化很小，CV选择均值基线，未通过改善门槛。训练耗时3.987 s，全部已拟合目标预热批量预测0.001646 s/周（预计<0.1 s）；不包含文件读取和真实求解，不称为出清器加速倍数。

相对38 s/周历史假设，24周串行估计912 s；实测双进程608.433 s。该比较跨工作负载与并发设置，不是同输入串行/并行配对测速，不能宣称1.5倍算法加速。

准确性偏差按理论技能顺序核查：解析/变异测试和真实价格、物理积分核查没有发现公式实现偏离；数值计算/存储时间不解释统计误差。失效的是低维平滑与足量样本的假设：只有18个训练周却有56个输入维度，出清离散切换使价格分位数和阈值事件非光滑，普通±10%样本没有失供/越限变化。训练MSE或一次MAE改善不保证跨周尾部精度。P99留出R2=0.006，CVaR99 R2=-0.262，即使MAE过门，也不能称为高准确尾部预测。六周样本不足以确定误差原因的统计显著性，以上为证据支持的解释，不是已证唯一原因。

本轮保留失败门槛，未在测试后调参或改变阈值。下一阶段需预先固定新的压力/边界动作训练设计及新测试周，再评估可行域边缘和尖峰；当前模型只供筛选研究，所有候选仍需Oracle。本次56维输入是完整已给定情景的条件响应，不是证明日前能够获知后续真实路径。

模型重新加载后的预测与留出报告逐值一致；CLI已实测拒绝不同case_hash和采样区间外输入。证据为模型目录`inference-validation.json`，正常调用输入/输出为`example-input.json/example-prediction.json`。

## 6. 第二轮压力试验预注册

在本轮新标签及测试结果产生前固定以下协议，保留v1模型、6周测试和门槛。新32周seed=20261000..20261031，按index%4轮换普通/供需紧张/线路受限/新能源富余；每组前6周训练、后2周留出。复用v1前18个训练周，旧6个测试周不再用于选择模型。尖峰阈值保持59.61200124564096，来源仍为独立校准周。

| 组 | 负荷倍率 | 风光倍率 | 来水倍率 | 线路倍率 |
|---|---|---|---|---|
| 普通 | 0.9–1.1 | 0.9–1.1 | 0.9–1.1 | 1 |
| 供需紧张 | 2.2–2.6 | 0.3–0.7 | 0.2–0.5 | 1 |
| 线路受限 | 1.0–1.2 | 0.9–1.1 | 0.9–1.1 | 0.05–0.20 |
| 新能源富余 | 0.1–0.3 | 1.5–2.0 | 0.9–1.1 | 1 |

其他已存在因素保持原设定，每日独立均匀扰动（rho=0），8日输入/7日执行。压低线路限额是合成运行边界试验，不是修改物理额定值的许可；各组出现次数不表示真实风险概率。总装机约11,046 MW、基准负荷峰值约4,878 MW，故负荷2.2–2.6且新能源/来水减少可触及供给不足；限额缩小和低负荷分别探测网络瓶颈和消纳约束。预期至少出现可验证的正缺额/越限/弃电标签，若未出现则记录试验设计未覆盖，不调低事件阈值。

模型候选使用原56维、或追加每个因素执行7日均值/标准差/min/max/最大日变动/第8日值的42个输入统计量。统计量不读取已出清状态或目标。它们编码周能量尺度及最坏日条件，在少样本下减少模型自行重建统计关系的难度；仍不宣称充分统计量。比较训练均值、Ridge(alpha=1/10/100，仅无界价格目标)和256棵Extra-Trees(leaf=1/2/4，两种特征)；有界比例/时长仅用均值/树，避免事后截断掩盖错误。训练3折按场景组分层，保证同一周不拆分；各目标按训练折MAE选择。新8周只报告一次，含全体和各组误差。各组仅2周，不能用总体改善冒充任一组准确。

保持10%相对均值MAE改善门槛；额外报告R2及预测物理边界。二分类风险为缺额/越限/弃电积分>1e-6；缺额/越限为诊断事件，非实际事故。比较频率基线和Extra-Trees，训练折log loss选择，固定0.5分类阈值；报告Brier、log loss、混淆矩阵和漏判率。测试每类少于5个时不作可靠分类准入声明。

成本预测以首轮24周608 s为参考，新32周双进程约811 s（14分钟），压力整数模型可能更慢。预测汇总指标<0.1 s/周；数值训练为小规模CPU树/岭回归，Oracle成本占主导。完整真实SCUC/SCED/LMP和原标签准入门槛保持，任何超时/无价格/失败样本保留为未认证，不伪装零风险。失效模式、实测耗时与预测差异另记在本节结果中。

第二轮复现命令（隔离输出目录）：

```bash
python3 tools/market_intelligence/run_price_oracle.py --output output/market-intelligence/stress-full-v2 --weeks 32 --workers 2 --seed 20261000 --design stress
output/market-intelligence/venv/bin/python tools/market_intelligence/train_stress_surrogate.py --dataset output/market-intelligence/stress-full-v2 --output output/market-intelligence/price-surrogate-v2
output/market-intelligence/venv/bin/python -m unittest discover -s tests -p test_market_stress_surrogate.py -v
```

第二轮训练器将所有尝试的标签与失败原因保存到`all_labels.jsonl/failures.json`；可用目标的评估是完整合格周条件统计，失败样本不赋零值。一旦产生正式`report.json`，工具拒绝覆盖同一目录的评估结果。`model.joblib/report.json/report.md/test_predictions.json`均留档。v2预测使用`--model .../model.joblib --predict-input .../example-input.json`；支撑域是四组区间的并集，不能把它们的整体包围盒当作已采样区域。

标签质量门额外拒绝NaN/Inf/负残差或负gap元数据，避免比较运算对NaN误放行。旧有效标签含义与schema_version=3保持；这类异常记录只会被拒绝。

## 7. 第二轮结果与事后诊断

32个新周全部完成，224个主定价阶段均有效且重复定价一致，标签准入32/32，失败0周。训练集为旧18周加新24周，测试为新8周（每组2周）。压力设计成功产生正缺额、正有功越限和弃电标签，但均为线性诊断Oracle结果，不是AC/N-1认证或真实市场风险概率。

事后基线审计的数学定义如下。对目标j和预定义场景组g，仅用训练周计算

$$
\bar y_{g,j}=\frac{1}{|T_g|}\sum_{i\in T_g}y_{i,j},\qquad
E_j^{\rm group}=\frac{1}{|V|}\sum_{i\in V}|y_{i,j}-\bar y_{g(i),j}|.
$$

此均值是组内平方损失常数解；这里只将其作为简单对照报告MAE，不声称它是MAE的最优常数。组别来自预先固定的输入采样区间，不读取测试目标；四组差异可能使全局均值基线过弱。`audit_stratum_baseline.py`只复算已冻结预测的误差，不重新拟合、选模型或修改预声明门槛。成本为O((训练周数+测试周数)×目标数)，本地CPU线性求和；不是性能优化，速度改善预测不适用。复现预期为与首次独立手算的10项MAE差≤1e-8，同时原模型、标签、report.json和test_predictions.json哈希不变；该容差仅检验复算，不是预测精度准入。以下为事后诊断，不能改称预注册检验。


**结论：全局均值门槛通过不等于具备可用的边界优化精度。** 10项指标均通过原定全局均值相对改善10%门槛，但与按预定义场景类型计算的训练均值相比，8项更差；另外两项（缺额、利用率）仅改善4.9%、8.1%。后者是事后诊断，不替换原报告的预声明门槛。当前模型不能准入Bayesian/CMA-ES边界优化。

| 目标 | v2留出MAE | 分组训练均值MAE | 相对分组基线改善 |
|---|---:|---:|---:|
| price.mean | 569.164 | 227.735 | -149.9% |
| price.p95 | 519.347 | 211.104 | -146.0% |
| price.p99 | 697.138 | 167.797 | -315.5% |
| price.cvar99 | 477.054 | 156.343 | -205.1% |
| price.spike_fraction | 0.0308447 | 0.024568 | -25.5% |
| price.spike_duration_hr | 4.45044 | 2.5599 | -73.9% |
| metrics.load_loss_mwh | 4548.95 | 4781.37 | 4.9% |
| metrics.line_overload_integral_mwh | 6432.69 | 2011.87 | -219.7% |
| metrics.renewable_utilization | 0.0208126 | 0.0226355 | 8.1% |
| metrics.renewable_curtailment_mwh | 2618.36 | 2445.65 | -7.1% |

价格水平MAE单位为currency/MWh；缺额、越限积分及弃电量为MWh；占比/利用率为0–1；时长为h。新8周尖峰占比MAE为3.084个百分点、时长MAE为4.450 h。混合压力样本含100000 currency/MWh诊断罚价尺度，全局均值基线MAE很大，容易产生夸大的相对改善观感。分组误差与R2完整保留在report.json，各组仅2周，不能据此证明稳定组内泛化。

相同2个新普通测试周上，冻结v1与v2平均价格MAE从10.081升至25.347，弃电量MAE从0.212升至159.898 MWh。v2没有全面替代v1。缺额事件分类为TN=6、TP=2；越限与弃电均为TN=4、TP=4，三项FP/FN均为0，但各类样本不足，可靠分类准入仍为false，事件分数未经概率校准。

偏差复核：标签解析、能量与价格一致性、特征单元测试及冻结预测重载未发现公式偏离；岭回归训练折又经独立增广最小二乘核查（见下）。计算速度不能解释预测误差。证据支持的主要假设问题是普通训练24周、每个压力组仅6周，组间距离大，树模型易学习分组而组内变化仍欠拟合。此为诊断解释，尚未证明唯一成因。下一轮应先固定组内/过渡区域采样、分组基线与误差门槛，再采集全新测试周；本轮没有用这8周重新选模型或调参。

Oracle总wall为1088.214 s（18分08秒），对比预期811 s，慢34.2%。普通/紧张/线路受限/富余各组平均单周wall为47.618/99.761/84.344/35.741 s；沿用普通周单价的成本假设低估压力求解负担，尚无阶段级配对证据将差值归因于某一种求解器操作。训练16.880 s，预热回归指标批量推断0.009370 s/周，满足预期<0.1 s/周；不含I/O、冷加载、分类器或真实出清，不能称为已验证的出清加速倍数。

训练日志出现NumPy/scikit-learn矩阵乘法的divide/overflow/invalid警告。`audit_ridge_numerics.py`仅用训练集，对4个价格目标×3个alpha×3折共36次Ridge拟合独立求解增广最小二乘，最大预测差2.765e-10，CV MAE差≤1.001e-11，满足预设1e-5绝对容差。最终10项回归均选Extra-Trees；全部输出有限。警告的库层根因未证实，不宣称已修复。

验证包括7项标签解析/变异/身份测试、4项压力特征/分组/支撑域测试；推断重载逐值一致，错误case、域外与缺失特征均拒绝。事后基线复算与首次独立计算差≤1e-8，冻结模型、标签、原始报告和测试预测哈希不变。没有为本轮Python工具重建C++或重跑全套C++测试。


复核命令（不重新训练）：

```bash
output/market-intelligence/venv/bin/python tools/market_intelligence/validate_stress_model.py output/market-intelligence/price-surrogate-v2
output/market-intelligence/venv/bin/python tools/market_intelligence/audit_ridge_numerics.py output/market-intelligence/price-surrogate-v2
python3 tools/market_intelligence/audit_stratum_baseline.py output/market-intelligence/price-surrogate-v2
```

数值证据分别为模型目录的`inference-validation.json`、`ridge-numerical-audit.json`和`stratum-baseline-audit.json`；原始Oracle位于`output/market-intelligence/stress-full-v2/`。`provenance.json`保存运行manifest中的HEAD与最终源码/产物哈希；运行期间仓库HEAD从5041bcdb变为ef54df20，Oracle始终使用同一既有二进制哈希，不能将当前HEAD解释为干净重建的二进制来源。

## 8. 前置辨识与开发学习曲线（preflight-v3）

用户选择先给出误差—成本曲线，工程误差门槛暂不指定。[采样理论§9](../../theory/market_surrogate_sampling_design.md)在Oracle产生新标签前固定了12家族×2日序×2线路动作及2次重复，共50个周评价。家族是开发划分单位，不把配对和重复计作独立样本。不使用旧测试周，也未设置新的最终盲测。

`run_identification_oracle.py`生成并冻结design.json；复用`run_price_oracle.py::run_one`的独立HTTP进程和真实7日出清链，新增可选实验配置及生成输入校验回调。原pilot/stress调用方式不变。全部fixed边际显式填写8日轨迹，生成后必须检查实际days/reference_days等于设计、无额外bid_scale/停运/局部时窗修改，核对通过后才开始出清。`--prepare-only`只冻结设计，后续运行拒绝与已有设计或二进制身份不符的配置。

`analyze_identification.py`逐周保留schema3标签，检查相同系统和初态、实际方向限额、配对的非线路轨迹；日序反转下比较low5和orderless42不变性。temporal17的时序分量是六种外生因素各两个DCT-II系数：sqrt(2/7)Σ_d x_d cos(πk(d+1/2)/7)，k=1,2。全56维保留完整8日输入。反转对的标签差给出低维确定性模型在该二点上的MAE/MSE下界；不将两种形状宣称为完整条件分布。

3折按家族划分，4/6/8个嵌套训练家族共16/24/32个周评价；模型为固定均值/中位数与三种表示的Extra-Trees/Ridge。增广最小二乘实现Ridge，所有缩放仅用训练折。保存逐点预测、各折MAE/RMSE、物理输出界、配对动作差值误差及零变化基线；成本按实际训练周worker-seconds累加。相邻规模的MAE下降/新增worker-minute仅是开发经验斜率，不外推精度保证。

复现命令：

```bash
output/market-intelligence/venv/bin/python tools/market_intelligence/run_identification_oracle.py --prepare-only
output/market-intelligence/venv/bin/python tools/market_intelligence/run_identification_oracle.py
output/market-intelligence/venv/bin/python tools/market_intelligence/analyze_identification.py
output/market-intelligence/venv/bin/python tools/market_intelligence/validate_identification_report.py output/market-intelligence/identification-analysis-v3
output/market-intelligence/venv/bin/python tools/market_intelligence/audit_identification_cost.py
output/market-intelligence/venv/bin/python tools/market_intelligence/analyze_identification.py --report-only
output/market-intelligence/venv/bin/python -m unittest discover -s tests -p test_market_identification.py -v
output/market-intelligence/venv/bin/python -m pip install -r tools/market_intelligence/requirements-plot.txt
output/market-intelligence/venv/bin/python tools/market_intelligence/plot_identification.py output/market-intelligence/identification-analysis-v3 --style-kernel /Users/tianyangzhao/.codex/skills/figure-style/kernel.py
```

绘图依赖固定matplotlib3.9.4，`--style-kernel`指向本地已安装figure-style技能的kernel.py；不影响市场求解器依赖。图中细线为三个开发折、粗线为其平均，不是置信区间；输出PNG/PDF和几何检查记录，并进行图像目检。新增7项测试覆盖配对输入、日序统计不变、回放输入变异拒绝、时序坐标、家族隔离、误差下界和岭回归闭式缩减恒等式。

### 8.1 实测结果与适用结论

50次周评价全部完成，350个日主定价有效且重复对偶一致，50/50通过schema3标签准入。非重复物理样本48个、独立设计家族12个。24组边界配对均核验实际及参考非线路输入相同、初态相同，方向限额执行复算最大误差0 MW。low5加截距的标准化设计矩阵秩6/6、条件数2.022；仅证明局部线性设计不退化。24组反转配对在low5与规范化求和顺序的orderless42上逐值相同；14组日序配对、14组动作配对的导出机组online序列发生变化。

同低维坐标二点平均MAE下界为价格均值615.679 currency/MWh、缺额2621.765 MWh、越限积分7914.845 MWh、尖峰占比0.005566（0.557个百分点）；最大单对分别4153.763、23009.863、78237.422、0.055514。两次相同输入重复的全部10项标签差为0，但不构成全域数值稳定保证。下界是所选反转二点上的经验约束，不是目标市场分布上的不可约误差估计。它证明周水平与无序统计不足；并不证明temporal17已经充分。

| 训练家族数 | 每折训练周数 | 训练成本/worker-min | full56树尖峰占比MAE/百分点 | full56树缺额MAE/MWh | full56树越限MAE/MWh |
|---|---:|---:|---:|---:|---:|
|4|16|27.988|22.657|94985.4|165605.3|
|6|24|44.441|16.097|118133.7|125100.9|
|8|32|56.758|12.264|83589.2|97185.2|

成本为三折训练子集实际worker-time之平均，不含开发周与重复。相同开发家族下部分指标随样本增加改善，但缺额非单调，弃电量反而变差；不能用三点曲线外推达到任意误差需要的总周数。8家族树模型缺额MAE：low5=71306.0、temporal17=58193.8、full56=83589.2 MWh；时序表示有局部收益却非全面占优，价格及越限等仍表现不同。Ridge出现负能量/越界比例等结果已在bounds_ok中保留，未事后裁剪或认作合格模型。

动作辨识尚未准入：full56树在8家族的缺额Δ预测MAE=4407.793 MWh，零变化基线=133.945 MWh；越限Δ误差20941.351 MWh，仅略好于零变化基线21272.746 MWh。总量模型不能直接当可信边界敏感度代理。48个非重复评价中47个尖峰时长饱和到168 h，另一个164.25 h；时长的小MAE不是过渡精度证据。固定阈值没有改变。这批Q的分布不同于v2，不作跨轮MAE直接优劣比较。

Oracle双进程wall2712.082 s（45分12秒），相对预测1500 s偏高80.8%；曲线拟合17.818 s满足预期<120 s。按理论偏差顺序核查，输入/公式/家族划分和独立误差复算均通过；48个非重复周平均worker=106.422 s，v2为66.866 s。SCUC/SCED/LMP平均solve_wall_sec从26.944/21.884/11.119变为44.977/40.264/14.247 s。每周worker与operation runtime差均约4 s，差异主要在出清阶段，支持旧成本分布不可直接迁移的解释；这是不同输入队列比较，不证明某个求解算法退化，阶段计时字段也不重复相加。

`validate_identification_report.py`从11520条预测独立复算720组开发指标，最大差5.821e-11（容差1e-8），并检查120个低信息预测误差下界、家族隔离及训练基线。两张10面板曲线PNG/PDF通过文本/边界几何检查及整图、分面目检；曲线只表示开发误差，不是盲测置信区间。18项相关单元测试通过，文档锚点检查0失败；本轮未改生产C++、未重建或重跑全套C++测试。

当前结论：保留完整轨迹，避免继续只用周水平加样；后续应比较更充分的时序/状态代理和直接Δo模型，围绕缺额/拥塞/未饱和尖峰切换安排独立配对家族。须先根据本曲线确定工程容限与预算，再固定序贯补样协议。当前只完成前置辨识，没有部署新代理、启动主动补样或Bayesian/CMA-ES。
