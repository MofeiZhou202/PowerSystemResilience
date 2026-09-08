# AEMO 数据与市场模块的可证伪验证

## 编码前验证协议

目标是主动寻找模型和流程反例，不是从有限测试归纳出全部正确。
证据分三层：AEMO原始数据内部一致性；将其价量/时间序列显式迁移到本项目规则后验证
数学不变量；项目已有规则/流程回归。跨市场制度不同，三层不能互相替代。

理论依据：南方执行契约中的节点平衡、线路松弛、SI水量平衡、储能效率、98点及实时滚动；
AEMO数据字典BIDDAYOFFER_D/BIDPEROFFER_D、DISPATCHREGIONSUM、DISPATCHLOAD、
DISPATCHINTERCONNECTORRES。来源适用日与原始申报日分开，实测价格不当作报价。

预设实验与定量验收：

1. 官方七天5分钟区域数据须按时间戳/区域/干预标志唯一连接；应有2016时点×5区域。
   保留负价、干预与软约束记录，检验区域`发电-可调负荷-净外送-需求`、
   风光合计及5分钟到15分钟的电量守恒。区域数值容差0.05MW（公开字段舍入），
   聚合电量相对容差1e-10；若额外项或字段语义不适用，先核对字典，不能硬凑恒等式。
2. 实测报价和区域需求映射到可手算两母线系统，分别验证网络拥塞、限额放松、线路停运、
   缺额/富余、负价与需求削减。已知可行域包含关系保证最优目标单调，
   但不能无条件要求非凸模型价格或每条线路潮流单调。
3. 两机四个自由时段，后94点固定停机，完整枚举256种开机组合。无爬坡约束且线性报价时
   每个固定组合的经济分配可直接按价格填充；包含启动费和最小开机时长。
   SCUC最优目标须与全枚举相符，相对误差<=1e-6，不以恰好相同并列开机表为条件。
4. 固定开机情况下做报价正比例缩放、记录排列与稳定ID重命名、reference/compact及
   HiGHS/native/Gurobi对照。检查目标/可行性和价格缩放，不要求退化解逐机完全相同。
5. 风光水火储/可控负荷场景逐设备、逐时点重新计算节点功率、线路软越限、共享水库与梯级
   延迟、SOC递推及电量费用；不直接相信结果自带的residual或feasible。
   容差为功率/流量1e-5，水位1e-7m，能量1e-5MWh，费用相对1e-6。
6. 日前滚动检查96点执行、2点下一日代表与跨日状态；实时检查24×5min与8×15min、
   仅前3点推进状态、失败/过期请求不推进、封存报价不允许篡改。使用官方时序作为事后
   边界，不能标为申报时真实可见的预测。
7. 调频以AEMO报价/价格作为显式缩放的研究输入，核对本项目云南规则的固定UC/一次调频、
   双向能力和安全区间；FCAS容量报价不等于云南调频里程报价，不能直接移植金额结算。
   结算以独立价×量×小时、节点现金守恒和完整计量门控核验，未取得真实合同/计量不验证实际账单。
8. 故意对结果注入1MW、1MWh、0.01m错误，以及漏设备/ID重复、NaN、98点误计96点等错误。
   每个预设变异都必须被独立校验器发现；检查能发现某类错误，不等于校验器完整。

成本模型：流式MMS解析O(记录数)、独立物理审计O(时点×设备数)，小UC枚举O(256×4×2)；
使用本项目既有求解器，不新增替代市场出清引擎。既有三套C++市场回归补足未映射的HVDC、
N-1、博弈、概率场景及双结算；这些仍标为合成/规则证据。

所有场景、失败证据、实际二进制哈希和覆盖矩阵保存在 `output/market-validation/`。
原始文件位于 `external_data/market_validation/` 与上一轮 `external_data/market_bids/`。
真实水库/梯级物理参数、完整节点网络、合同、AGC秒级计量和黑启动恢复过程如无公开可核验数据，
必须列为未获得AEMO实证覆盖，不能以补造参数实现“全覆盖”。

## 实际结论与反例

2026-09-07执行：新增实验108项检查通过，包含12项主动破坏结果的变异检查；
独立校验器9项、既有报价解析/数学9项Python测试通过。现有Release二进制的
三套C++回归共89用例、41310断言通过，四套GUI/API E2E通过。
**检查通过包括预期失败被正确识别，不表示所有出清、安全和结算结果通过。**
本轮未修改生产求解器，没有重新编译C++，也没有取得完整NEM模型或南方实际报价。

已获得三类实质反例：

1. **线性出清可行不能推出交流安全。** 两节点多资源系统的正常/断线×零/基准来水
   四个场景均有线性计划，但事后AC校核全部失败。正常拓扑98点潮流收敛，仍有98次送端、
   98次受端MVA热限越界；两个来水条件分别有21/26次线路有功上限越界。
   断线场景前96个实际点未收敛；第97/98个展望点恢复拓扑，仍存在热限越界。
   这不能表述为四个场景AC安全通过，也不能把非收敛直接解释成已证明物理不可行。
   `observe_ac` 的非收敛输出缺少详细失败原因，某些点的residual=0来自默认字段，
   必须结合`converged=false`读取，不能当作零潮流残差证书。四个场景均禁止正式结算，
   仅保留条件性研究账本。结果见`fault_inflow_study.json`。
2. **电能量最优UC不能保证后续调频容量充足。** 原始混合场景的能源UC关闭部分火电，
   10至15小时不足以满足调频需求及单主体50%上限，返回`ancillary_capacity_shortfall`，
   没有伪造SCED或有效价格。另设“常规AGC提供机组保持在线”反事实后可行。
   后者增加了人为must-on边界，不是原场景求解器性能改进，也不是证明顺序出清总可行。
   两套输入/失败/成功结果分别保留在`ancillary_capacity_shortfall.json`、
   `ancillary_fixed_provider_day.json`。
3. **不能无条件把公开FCAS的ACTUALAVAILABILITY当作中标量上界。**
   163584条非干预UNIT_SOLUTION记录形成1635840个产品对照，其中25个超过0.05MW容差，
   最大100MW。25条均出现INITIALMW与TOTALCLEARED跨充放电方向或从零进入负出力，
   但该关联不是已验证的因果解释。例：BLYTHB1，09-01 06:50，RAISEREG=17MW，
   ACTUALAVAILABILITY=0，INITIALMW=45.5，TOTALCLEARED=-20；对应AVAILABILITY=74MW。
   官方字典`Elec20_1.htm`只明确后者为梯形调整后的可用量；当前证据尚不足以统一
   双向设备、方向及初始/目标点的语义。所有原始行保存在`official_data_audit.json`，
   该项状态为`unresolved_field_semantics`，不计入`identities_pass`。

故障/来水四场景的线性ΔPij均为零；这恰好说明“ΔPij=0”不排除交流MVA越限。
另设专门两母线压力实验，独立验证正的ΔP/ΔPij，避免只在全零结果上验证统计程序。

## 官方数据核验

数据来源、原件和SHA256见[源文件说明](../../../external_data/market_validation/README.md)
及同目录`source_manifest.json`。官方报价延续上一轮七日样本；新增加七日DispatchIS
和一个市场日Next_Day_Dispatch。保存MMS原时间戳，不当作中国市场本地时间。
区域序列为288个5分钟点/日；单位调度档案按AEMO市场日跨自然日，不能直接按日期截断连接。

| 检验 | 样本/结果 | 解释 |
|---|---|---|
| 时间与区域连接 | 2016时点、10080区域时点 | 非干预RUNNO=1，时间戳/区域唯一连接 |
| 发电−可调负荷−净外送−需求 | 最大0.010000000002MW | 小于预设0.05MW公开舍入容差 |
| 风光分项与合计 | 最大9.09e-13MW | 同时核验UIGF与已出清风光 |
| 5分钟平均到15分钟的电量 | 最大相对误差3.147e-15 | 小于1e-10，未将功率直接相加 |
| 通用约束记录 | 1953615条，34条正VIOLATIONDEGREE | 保留软约束记录，不删除异常样本 |
| CASE_SOLUTION | 180时点TOTALGENERICVIOLATION>1e-6 | 与约束行计数是不同统计口径 |
| SOLUTIONSTATUS | 0有1084点，1有932点 | 官方非零标志含稀缺/过剩/约束违反，不能等同求解无解 |
| 区域负RRP | 1546区域时点 | 负价保留，RRP不是节点LMP或主体报价 |
| 单位调度/FCAS | 568个正午DUID，25个未解释对照差异 | 区域恒等式通过不覆盖这一项 |

全部原始边界并非来自AEMO：节点网络、水库面积/耗水率/梯级、储能效率和响应负荷是
明确标注的合成物理参数。公开区域需求/风光UIGF形状与报价被迁移到本项目南方规则，
不能将本次结果与AEMO出清逐点相等当作验收条件。没有完整NEM约束及损耗模型，
本次不做NEMDE调度或区域价格复现。

## 方程与独立数值证据

`tools/market_validation/independent_audit.py`使用稳定设备ID重新关联输入输出，拒绝缺设备、
重复ID、非有限值和错误长度。它不调用生产模型的残差计算函数。当前允许使用结果的
`effective_boundary`，故边界规范化等价性仍依赖既有边界回归，不能声称这部分也独立验证。

独立复算使用以下符号和单位：时长h，功率MW，能量MWh，流量m3/s，水位m。
储能`charge_mw<=0`；线路按from→to方向，节点ΔP由缺额与富余两非负变量表达。

```text
节点：sum(Pg) + Pdis + Pch + DR - Load - sum_out(F) + sum_in(F)
      + deficit - surplus = 0
线路：slack+ = max(0, F - Fmax)，slack- = max(0, Fmin - F)
      停运线路F=0；ΔPij = slack+ + slack-
储能：E[t] = E[t-1] - dt_h*(Pdis/eta + Pch*eta)，eta=sqrt(roundtrip_efficiency)
共享水库：release = spill + sum(Pg)*water_m3_mwh/3600
梯级水位：H[t] = H[t-1] + 3600*dt_h/area_m2 * (inflow + upstream[t-lag] - release)
日能量：sum(t=0..95, P[t]*dt_h)，两展望点不计入已实现日电量
日能量报价费用：sum(t=0..95, weight_h[t]*integral(0..P[t], bid(q)dq))
小时实时价：(四个已执行15分钟时段的首个定价点价格之和)/4
现金：负荷付款 = 发电收入 + 储能收入 + 诊断松弛收入 + 线路租金
```

现金公式限当前无损、无额外税费/合同的线性试验；报价费用复算限Pmin=0且无启停轨迹，
不含启动、空载、调频和费用分摊。UC枚举另行显式计入启动费、空载费及最小开机时长。
独立校验器尚不全面覆盖爬坡、最小出力、备用、断面和DC模型，DC/外部计划/交易显式拒绝。
这些不能因本轮审计通过而获得完整证明。

| 实验 | 实测结果 |
|---|---|
| 两机4个自由时段全枚举 | 256组合中17可行，最优目标132.58706884519836 |
| HiGHS/native/Gurobi UC | 三后端目标均与上述全局枚举值一致；小样本不能证明大系统性能 |
| 网络限额30→150MW | 最优目标3264.189225→1130.225263，符合可行域包含关系 |
| 节点负荷微扰与价格 | 费用有限差分2.3576999993，对应LMP=2.3577 |
| 断线且本地机组移除 | 严格出清失败；诊断模式首点缺额ΔP=87.74011797875MW |
| 保持线路但限额30MW | 首点越限ΔPij=57.74011797875MW，可与缺额区分 |
| 报价×1.5、ID重命名/排列 | 固定UC等价关系通过，reference/compact及三后端对照通过 |
| 混合资源独立节点平衡 | 最大1.706e-12MW（包含正常/干旱/实时/调频可行场景） |
| 储能SOC递推 | 最大1.635e-12MWh；充放电符号、互斥及终值检查通过 |
| 两个共享水库、一级延迟梯级 | 水位最大1.422e-14m，放水最大2.843e-14m3/s |
| 日报价积分 | 最大相对误差1.231e-16；前96点发电能量误差0 |
| 七日滚动 | 每天96执行点+2个D+1代表点；下一日负荷来源逐点核对 |
| 实时四轮 | 每轮24×5min出清、8×15min定价、仅前3点推进；功率/SOC/水位承接误差0 |
| 两节点整小时价格 | 四个已执行季度重算误差0；空记录、重复季度和错误价格均被拒绝 |
| 七日研究账本 | 逐价格×量×时长重算误差0，现金恒等式残差约7.1e-15 |
| 联合场景任务 | 2场景×7日完成，种子重复一致；零缺额均值重算一致 |
| 12种结果变异 | 1MW发电/线路、1MWh SOC、0.01m水位、1m3/s放水及结构/总量/状态造假全部被捕获 |

表中经济量采用明确缩放的人工货币单位；API字段名`cny`不代表本次已换汇。
能源样本p1=0.5833、p2=2.3577来自正容量GEN真实段价/100。
小UC需求序列为[20,120,20,83.46264384]MW：区域极值及末点被重排成压力序列，
并非按原时间重放。调频初始映射/100部分超出云南3至8元区间，原记录保留规则回退，
可行反事实另用`round(min(8,3+original_price/100),1)`；不把FCAS容量价等同里程价。

## 全功能覆盖矩阵

| 功能 | 本轮AEMO相关证据 | 规则/流程证据及未覆盖部分 |
|---|---|---|
| 市场行为与报价 | 七日GEN/LOAD申报及上一轮27次配对出清 | 共同倍数不能覆盖排序/符号改变；未校准南方行为、水价值和主体策略 |
| 预测/市场边界 | 实测区域需求、风光UIGF时间形状、七日观测均值范围 | 2场景14日验证任务/统计；其他随机因子固定，不能声称联合预测误差分布已校准 |
| 日前SCUC | 源报价+压力需求，256组合全枚举，三后端 | `southern_market.cpp`及59例Southern回归；本轮非2000节点性能验证 |
| SCED与LMP | 拥塞两母线解析、报价缩放、有限差分、ID置换 | 费用/价格仅在定义模型内；不与NEM RRP混比 |
| 风光水火储/可控负荷 | 风光/负荷形状与真实价段，独立守恒 | 水库/梯级/储能/DR为合成参数；没有真实水位、效率、用水和可控负荷计量 |
| 周滚动与状态 | AEMO七日形状，96+2来源及费用 | 第8日为明确持久性假设；周案例为两母线，水/SOC跨轮另由实时及既有测试覆盖 |
| 实时市场 | AEMO5分钟负荷输入，24/8点、四次推进、小时价 | `southern_real_time.hpp`、Southern回归及IEEE118 GUI；未复现NEM实时规则 |
| 安全校核与故障归因 | 正ΔP/ΔPij反例，4故障/来水AC失败证据 | 事后AC不修正计划、不穷举N-1；断线非收敛详细根因仍不足 |
| 辅助服务 | 正午RAISEREG/LOWERREG真实报价迁移；容量不足/可行反事实 | `yunnan_ancillary.hpp`固定UC/一次调频、双向备用、安全区间及日内封存；FCAS语义差异未闭环 |
| 结算与更正 | 七日价量现金独立复算 | `yunnan_rules_workflow.hpp`的计量、日/月账本、更正/重复门控由合成E2E验证；无真实合同和AGC遥测账单 |
| 通用混合AC/DC市场 | 本轮未取得对应AEMO网络实证 | 22例通用市场回归、GUI混合出清；不能借南方AC样本认证DC物理 |
| 重复博弈 | 无主体持仓/所有权等识别数据 | 既有测试/GUI仅流程；`maximum_rounds_reached`不是纳什均衡 |
| 黑启动 | 无AEMO恢复过程数据 | 当前仅云南黑启动规则提取，没有动态恢复出清实现，不能标记为功能验证通过 |

独立审计与场景入口见`tools/market_validation/{aemo_data,independent_audit,run_aemo_validation}.py`。
周/概率实现为`src/market/{market_operation,market_forecast}.cpp`；API生产入口为
`tests/run_gui_server.cpp`。本轮GUI回归发现导航测试仍期待6个页面，已按生产8页面
修正`tests/e2e/market_gui_e2e.mjs`，并实跑通过；不是修改GUI来迎合旧测试。

## 复现与构建边界

```bash
# 首次运行；已有extracted.json时不必重复上一轮报价提取
python3 tools/market_validation/aemo_data.py --download
python3 tools/market_bid_empirical.py --parse-only
python3 tools/market_validation/run_aemo_validation.py
python3 -m unittest discover -s tests -p 'test_market_independent_audit.py' -v
python3 -m unittest discover -s tests -p 'test_market_bid_empirical.py' -v

build/macos-release/tests/test_southern_market -r compact -o output/market-validation/southern-regression.txt
build/macos-release/tests/test_market_simulation -r compact -o output/market-validation/generic-regression.txt
build/macos-release/tests/test_market_forecast -r compact -o output/market-validation/forecast-regression.txt
node tests/e2e/southern_realtime_e2e.mjs > output/market-validation/realtime-gui.txt 2>&1
node tests/e2e/yunnan_ancillary_e2e.mjs > output/market-validation/ancillary-gui.txt 2>&1
node tests/e2e/market_forecast_e2e.mjs > output/market-validation/forecast-gui.txt 2>&1
node tests/e2e/market_gui_e2e.mjs > output/market-validation/generic-gui.txt 2>&1
python3 tools/market_validation/summarize_aemo_validation.py
```

`run_aemo_validation.py`读取上一轮`output/market-bids/extracted.json`；正式运行需要真实
可用的HiGHS/native/Gurobi后端和本地Gurobi许可，不会将后端不可用静默算作通过。
运行器使用独立临时端口并在finally终止自己的服务器，保留现有GUI会话。
所有输入、结果、容差、反例及哈希留在`output/market-validation/`，
汇总为`validation_summary.json`，运行环境/源状态为`campaign_manifest.json`。

本轮使用已存在的macos-release二进制；兄弟MIPSolvers源码与固定依赖版本不一致，
正常CMake重新生成仍受已有保护阻止，未更改pin。89例C++通过只认证记录哈希的现有二进制，
不宣称干净检出重构建通过，也未在本轮重跑sanitizer。Python测试已在CMake注册，
实际由Python直接执行，未将其表述为本轮CTest通过。
实时/调频E2E实用IEEE118多资源案例，实时两窗口约0.812/0.862s，
调频约21.57s；这不能外推2000节点的求解性能。通用GUI的日前结果为
`n1_security_failed`、博弈为`maximum_rounds_reached`，E2E通过仅说明状态/流程如实呈现。

后续研究准入仍需解决：非收敛诊断与AC安全修正、顺序能源/调频的可行性反馈、
双向FCAS字段解释、完整网络与可见信息集、真实水库/合同/计量数据。
在这些证据缺口闭合前，不能宣称“全部功能经AEMO实证验证正确”。
