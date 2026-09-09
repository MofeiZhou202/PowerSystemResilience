# 智能仿真标签与第一版价格代理

本文是离线工具的实现契约与实验记录。研究问题见[边界优化数学模型](../../theory/market_boundary_optimization_model.md)；生产出清仍遵循[南方执行契约](southern_execution_contract.md)。本轮仅建立固定 IEEE118 系统/边界/周初状态下的合成场景响应代理，没有执行 Bayesian/CMA-ES 边界优化。

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
