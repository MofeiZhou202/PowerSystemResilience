# 暴雨内涝与雷击：实现契约

适用：弹性仿真平台，2026-09-27。实现与验证状态见[开发状态](../../overview/development_status.md)。
这是参考书第二章灾种的第一版简化研究模型；未安装、调用或复制 SWMM、OpenETran。
参考书 PDF 页序与开源候选见[调研](multi_hazard_sources.md)。

## 用户流程

首页进入 33 节点演示（默认加载 `dist33_weather_mixed`）→ 场景生成与选择 → 灾害类型选“暴雨内涝”或“雷暴雷击” →
设置参数并生成 → 选择代表场景 → 快速恢复 → 计算并输出指标。

参数默认值、单位和范围由 `GET /api/edition` 的 `scenario_hazards` 提供，
仅 Resilience Edition 扩展该 profile 字段。Web 根据 schema 生成表单，基础参数直接显示，
高级参数折叠；仅提交用户改动的值，可恢复当前灾种默认值。
切换灾种保留各自参数，本地分析版本保存完整配置。
场景生成时域仍为 48 h；候选数为代表场景数的十倍、最多 200。
新灾种只有一个参数组，不使用台风强度等级、台风目录或路径。

结果显示雨量/积水或地闪密度过程、架空线/电缆/绝缘子/变压器归因、暴露与故障的 AC/DC 支路数、暂时跳闸/保护停运/永久损坏数、设备与支路稳定 ID、安全开工和恢复时间。`fault_count` 是本场景停运的支路组件数；每个在役支路最多计一次，不是雷击次数、停电用户数或损坏母线数。
快速恢复仅开放“考虑移动储能调度”开关；故障和时间来自所选场景，其余参数使用平台默认值。旧分析版本重新载入时，历史隐藏参数覆盖不进入新请求。
计算摘要、模型参数和详细假设仍在页面最后的折叠区。
当前没有灾害空间分布地图，因为模型不具备地理强度场，不能把绘图坐标当作地理坐标。

## 数学模型与有效范围

### 暴雨内涝

给定降雨开始 s、历时 T、总量 P、峰位置比例 r、形状参数 b 和 n。
局部时间 x=t-s，峰时间 x_p=rT。峰前令 τ=(x_p-x)/r，峰后令 τ=(x-x_p)/(1-r)。

```text
w(τ) = [b + (1-n)τ] / (b+τ)^(1+n)
R_k = P * a * w(τ_k) / Σ[w(τ_j) Δt]              (mm/h)
D_(k+1) = max(0, D_k + (C R_k - q) Δt)           (mm)
D_cable,k = 1[D_k >= h_entry]                       (电缆附件暴露筛选)
P/P0 = (1-H/44330)^5.25
R_w = c ρ (A+0.02)^(-0.44)
U_flash = (a exp(R_w)+b)(P/P0)^n                   (书中绝缘子式)
W_oil = a1 N A^0.949 (P/P0)^n1 + b1
W_paper = a2 N exp(1.323 A) (P/P0)^n2 + b2
```

使用 5 min 子步、区间中点取雨强。`a` 是一个候选共享的均匀强度倍率，
在 `[1-v,1+v]` 抽样；归一化保证实际总雨量为 `P*a`。
这是 Chicago **形状**的给定总雨量雨型，不含城市 IDF 参数拟合或重现期计算。
输出降雨为小时均值，积水为小时内最大值。

`A` 是 5 min 子步雨强（mm/min），`ρ` 是雨水电阻率，`H` 是海拔，`N` 是分钟数。
绝缘子使用书中式 (2.47–2.50) 的**相对标定**：把 1 mm/min 时的湿闪电压设为设备字段
`weather_insulator_wet_ref_kv`，只保留雨强与气压变化；低于交流相对地运行电压时记一次雨闪暂时跳闸。
[IEEE Std 4 工作组湿试验资料](https://www.ewh.ieee.org/soc/pes/switchgear/presentations/tp_files/2016-2_HVTT-IEEE_STD_4_Presentation.pdf)
给出试验降雨 1–2 mm/min 和收集水电阻率 100±15 Ω·m；平台只借用 100 Ω·m
作为演示雨水参数，没有把实验室湿试验条件当作现场故障频率。
油/纸受潮按书中式 (2.53–2.54) 对每个子步的增量积分，跨越演示判据时记“变压器绝缘受潮保护停运”。
这相当于允许已经声明有进水途径的变压器在降雨期间受潮；密封正常设备不参与。
相关系数与判据未由书中给出，平台值均为可见的**演示假设**，不是现场绝缘击穿预测。
[美国垦务局 FIST 3-31 变压器诊断手册](https://www.usbr.gov/power/data/fist/fist3_31/fist3-31.pdf)
在 60 °C 下给出的低于 69 kV、服役变压器油含水 35 ppm 及纸含水 2.5% 数值属于维护/干燥参考，
并非雨中瞬时跳闸阈值；本平台的 60 ppm/3% 仅是高于这些参考量级的合成保护决策值。

电缆本体不因浸水直接故障。仅显式给出 `weather_cable_entry_height_m` 的易受淹电缆附件，
在积水达到入口高度时作预防性保护停运；安全开工不早于降雨结束及退水至入口高度以下，
随后增加演示修复时长。此为将[慕尼黑电网洪涝研究的“临界水位→设备失效”判据](https://www.frontiersin.org/journals/earth-science/articles/10.3389/feart.2021.572925/full)
迁移到**有明确进水路径的附件**，并非该论文提供了电缆失效概率或具体入口高度。
[IEEE 2021 电缆接头受潮实验](https://ieeexplore.ieee.org/document/9683947)指出受潮是中压交联电缆接头常见缺陷，
未及时处理会发展为早期故障；[IEEE 2014 电缆失效研究](https://ieeexplore.ieee.org/document/6869395/)分别分析了受淹接地箱和接头绝缘击穿。
这些研究支持“聚焦薄弱附件、先停运检修”的选择，但没有给出本算例的通用进水高度或瞬时故障概率。
[Eindhoven/IEEE 电缆进水实验](https://research.tue.nl/en/publications/fault-development-on-water-ingress-in-damaged-underground-low-vol/)
研究的是受损低压电缆，不能推断完好电缆一浸水就立即故障。
零排水且附件仍浸水时显式拒绝，因为固定停运窗口无法表示无穷期修复。

算例 `dist33_weather_mixed` 以原 33 节点 DER 算例为底稿，约三分之一 AC 线路标为电缆，
其余标为架空线；1 号 AC 支路作为 9001 号变压器的电气等值支路，DC 线标为电缆。
入口高度 0.05/0.10 m、弱绝缘子湿闪参考 5.5 kV 和变压器受潮属性是合成资产信息。
变压器停运打开已关联的支路，求解网络不额外添加并联路径。其他算例的未知 AC 类型默认归架空、
未知 DC 类型默认归电缆，但若缺少附件入口或绝缘子参考耐压，就不会生成对应暴雨故障。

不包含地形汇流、排水管网、河道洪水、真实站房进水或道路阻断；不宣称得到城市洪涝空间预报。

### 雷暴雷击

```text
I ~ Lognormal(ln(I50), σ²)
p_flash = 1 - Φ(ln(Icrit/I50)/σ)
A = L_km * W_m / 1000                            (km²)
λ = N_g * a * A * p_flash                        (1/h)
P(至少一次跳闸 | 本次雷暴) = 1-exp(-λT)
首次跳闸等待时间 ~ Exponential(λ)
```

`N_g` 是每平方千米每小时的地闪数；不是年雷击密度。
`W` 是等效收集宽度，`Icrit` 是等效闪络阈值。缺失或无效线路长度显式采用用户可配置的
`fallback_length_km`，结果返回采用假设的支路数量。模型用电流分布阈值积分，未显式求解雷电波形。

仅明确分类为架空线的 AC 支路进入雷击抽样；电缆与变压器等值支路不参与。
首个跳闸事件按 `permanent_fraction` 分类：永久损坏等待雷暴结束，再加修复作业时长；
暂时跳闸经过给定等效停运时间自动解除。默认永久损坏概率 0.2 是研究演示值，
不是把参考书某地案例的 19.5% 推广为通用常数。
每支路最多一次停运，不模拟反复重合闸失败或暂时故障后再次永久损坏。
不包含绕击/反击细分、接地阻抗暂态、防雷器动态、秒级重合闸或 OpenETran 电磁暂态。

### 共用近似与随机性

- 仅在役且有明示暴露参数的设备；同号 AC/DC 支路索引通过域区分。重复稳定身份拒绝，先按域和稳定 ID 排序再抽样，
  重排组件向量不改变结果。
- 雷击给定强度倍率后各架空线抽样独立；暴雨使用同一零维积水过程，设备按确定性阈值筛选，不输出伪造的电缆失效概率。
- 开始向下取整到小时，恢复向上取整；故障窗口保守覆盖物理事件。
  暂时跳闸最小等效时长 1 h，保守取整可能覆盖两个小时区间。
- `repair_hr`/`repair_duration_hr` 沿用既有接口，但本模式含等待时间；
  实际作业时间另见 `hands_on_repair_hr`。
- 风光和负荷沿用既有随机基础时序，尚未加入这两类天气专属降额。
- 概率是设定灾害条件下的样本权重，不是年风险。已有指标逐项检查输入，不能因新增灾种而伪造年化指标。
- 每次请求最多 200 个候选、候选×支路不超过 200000，投影子步工作量不超过 100000000；
  超出显式报错，要求减少候选数或模型规模。

## HTTP、源码和界面字段

生产路由：`POST /api/session/generate_scenarios`。

```json
{
  "regular": {"enabled": false},
  "reliability": {"enabled": false},
  "resilience": {
    "hazard_type": "rainstorm",
    "rainstorm": {"total_mm": 260, "drainage_mm_hr": 15},
    "candidates_per_intensity": 10,
    "default_cluster_count": 1
  }
}
```

雷击改为 `"hazard_type":"lightning"` 和 `"lightning":{"density_km2_hr":5}`。
省略 `hazard_type` 兼容旧台风请求。非数值、未知参数、越界、事件超出观测时域均返回 HTTP 400。
默认值不是 JavaScript 常量，以下源码表是唯一默认/范围来源：
`src/scenario_generation/weather_hazards.cpp` 的 `rain_fields`、`lightning_fields`。

| 参数组 | 字段（单位）与默认值 |
|---|---|
| 暴雨基础 | `start_hr=4`、`duration_hr=6`、`total_mm=180`、`drainage_mm_hr=15`、`repair_hr=6` |
| 暴雨高级 | `peak_fraction=0.4`、`shape_b_hr=0.5`、`shape_n=0.7`、`runoff=0.85`、`severity_variation=0.2`；雨水电阻率 100 Ω·m、海拔 500 m、绝缘子拟合系数 0.01/0.5、初始油/纸含水 15 ppm/1%、油/纸受潮系数 0.025/0.00025、保护停运判据 60 ppm/3% |
| 雷击基础 | `start_hr=4`、`duration_hr=6`、`density_km2_hr=2`、`critical_current_ka=50`、`permanent_fraction=0.2`、`repair_hr=4` |
| 雷击高级 | `collection_width_m=100`、`fallback_length_km=1`、`median_current_ka=30`、`log_current_sigma=0.6`、`transient_duration_hr=1`、`severity_variation=0.2` |

字段名中 hr/h 表示小时、mm 毫米、m 米、km 千米、kA 千安；比例与对数标准差无量纲。

| 契约 | C++/HTTP | Web 与验证 |
|---|---|---|
| 灾种与参数 | `ResilienceScenarioOptions.weather`；`resilience.hazard_type/rainstorm/lightning` | `scenarioConfig`；`data-portal-hazard-parameter`；版本保存 |
| 代表场景 | 保留 `resilience.intensities[].clusters[]` 结构；新灾种 `intensity` 为灾种 ID，不是台风等级 | 复用代表场景选择及恢复绑定 |
| 过程 | `representative.resilience_event.hazard_evidence.profiles[]`，每项含 time_hr、values、unit、label | 图表与峰值表，缺图形库时仍有数据摘要 |
| 损伤 | `faults[]` 为求解输入，新增 `equipment_type/equipment_index/failure_cause`；`hazard_evidence.fault_effects[]/asset_risks[]/affected_equipment` 为溯源 | 域/支路与受灾设备稳定 ID、架空/电缆/绝缘子/变压器数量、物理时刻、开工与停运时长 |
| 模型范围 | `hazard_evidence.model_scope/model_limitations/parameters/seed/severity_scale` | 页尾折叠摘要；不生成假台风风速（null）或假路径（空数组） |
| 恢复约束 | `DistributionResilienceOptions.respect_fault_windows`；恢复请求同名布尔字段 | 新灾种选择后自动传 true；快速恢复仅开放 MESS 开关 |

主实现：`include/hacdcpf/analysis/weather_hazards.hpp`、对应 `.cpp`；
生成/聚类集成：`src/scenario_generation/scenario_generation.cpp`；
profile：`src/server/edition_profile.cpp`；HTTP：`tests/run_gui_server.cpp`；
UI：`web/js/core/resilience_portal.js`、`web/js/app.js`。

## 分阶段恢复的时间窗模式

既有 RA 分阶段路径在灾后按恢复收益贪心选择修复线路，可能改写给定修复时长。
新模式在每个时刻用 `start <= t < start+duration` 得到不可用边集合，再求解重构，
允许暂时跳闸在灾中解除，禁止永久故障在安全等待结束前被“贪心修复”。
保留原始故障序列时长，活动/已恢复故障计数按同一时间窗计算。
当 `F(t)=∅`（该小时已无活动故障）时，令支路状态回到建模初始拓扑 `β(t)=β₀`、拓扑切负荷 `shed(t)=0`。这样前一小时为了隔离故障打开的支路不会在故障结束后持续断开；恢复比例按该小时供电量除以需求量重新计算。该归零行为属于 RA 拓扑模型的正常基线假设，不是潮流、电压或供电容量可行性的证明。
结果 scope 增加 `:authored-fault-windows`，声明这是外部给定修复计划，不是抢修队优化。
旧场景默认 false，原恢复策略保留。

指标默认“灾害结束=最后一次故障开始”保持不变。
物理雨/雷暴结束、安全开工、故障恢复分别记录，不能混作指标默认值。
Web 对新灾种使用默认 48 h 观测时域，不为覆盖全部维修完成而自动拉长。
超过观测窗的停运保持未恢复状态；原台风路径的自动延长规则保持不变。
本模式不提高 RA 的潮流/电压约束认证等级；原有近似和求解器限制仍有效。

## 验证入口与数值检查

- `test_weather_hazards`：schema 默认值回写、越界/未知/非数值拒绝；总雨量守恒；零产流；
  零排水无有限修复的拒绝；同号 AC/DC 与顺序不变性；设备归因、变压器受潮、仅架空线雷击；暂时/永久分类；聚类权重与标准时序。
- 可复算雨量检查：总雨量 200 mm、倍率 1、产流 0.8、排水 0，末端水深 0.160 m；
  此例设置较高安全阈值，仅校验质量守恒。
- 可复算雷击检查：线长 2 km、宽度 100 m、地闪密度 1、阈值等于电流中值、持续 6 h，
  `λ=0.1/h`，跳闸概率 `1-exp(-0.6)=0.4511883639`；1000 个种子的抽样频率与其比较。
- `test_resilience_assessment [weather]`：长故障窗口 `[1,7)` 与暂时跳闸 `[3,4)`，
  8 个小时的活动故障数必须为 `0,1,1,2,1,1,1,0`；不得把长故障提前修复。
- `tools/resilience_edition_e2e.py`：真实混合算例生成→恢复→ENS；逐故障核对时间窗、
  schema 有效值、默认灾害结束、非法请求。
- 已注册 `weather_scenario_gui_e2e`：真实后台与浏览器灾种选择、参数编辑/切换/重置、
  桌面和移动布局、选择场景→恢复→指标及本地版本；还验证最后一小时活动故障为 0 时供电率回到 1，以及设备数量和单一 MESS 开关。证据在忽略的 `build/weather-gui/`。
- 当前 Release 定向数值证据：`test_weather_hazards` 133 条断言/8 组用例；恢复天气用例 25 条断言/2 组。
  真实 Web 选中的暴雨场景为 8 条支路停运（6 条保护停运、2 条雨闪暂时跳闸），雷击场景为
  9 条架空线停运（7 条暂时跳闸、2 条永久损坏）；两者观测末端供电率均为 1，
  每个场景有 25 项指标成功计算。此为固定种子/演示参数的回归结果，不是地方故障率估计。
  执行命令、其他覆盖和未闭环范围以[开发状态](../../overview/development_status.md)为准；
  尚无 SWMM/OpenETran 交叉验证、真实灾害数据校准或全库回归。
