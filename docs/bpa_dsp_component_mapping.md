# BPA/DSP 元件卡片 ↔ HySim 元件模型对照与 LCC 集成

> 实现契约类文档。对照依据：DSP dat 卡片技术手册（外部资料，LaTeX 六章）、
> HySim 元件模型（`include/hacdcpf/model/`）与 BPA/DSP 导入器
>（`include/hacdcpf/io/bpa_io.hpp`、`src/io/bpa_io.cpp`）。
> 与代码冲突时以代码与注册测试（`tests/test_bpa_io.cpp`、`tests/test_bpa_dsp_compare.cpp`）为准。

## 1. 结论：不新增"特高压直流系统"类别，新增 `LCCConverter`

DSP 的直流卡片语义本身就是两层：**换流节点卡**（BD/BM/BA，换流器元件）与
**直流线路卡**（LD/LM/LY/LZ，直流网络元件）。±800/±1100 kV 特高压直流在
网络层只是更高电压等级的 `DCBus`/`DCBranch`（纯电阻稳态模型，`base_kv`
上限 1000 kV），与交流/直流双域架构完全一致，**不需要第三个顶层系统容器**。
新增并列容器会冲击 validation、projection、SolverData 装配与结果归因全链路。

真正的缺口在换流器层：DSP 的 LCC 是电流源特性（α/γ 角控制、无功由换流器
方程内生决定、依赖换流变分接头维持角裕度），与 `VSCConverter` 的受控电压源/
功率注入特性物理本质不同。因此本仓库在
`include/hacdcpf/model/converter_components.hpp` 新增 **`LCCConverter`**
（`station_role`、桥数 `n_bridges`、`alpha_min/stop`、`gamma_min`、阀压降、
换相电抗 `x_comm_pu`@系统 MVA 基准、控制模式 `ConstantPower/ConstantCurrent/
ConstantAlpha/ConstantGamma` 及对应 setpoint），挂于
`HybridPowerSystem::lcc_converters`，复用现有 `DCBus`/`DCBranch` 网络层。

准稳态外特性（与 DSP dat 卡片手册第 4 章一致）：

- `Ud0 = (3√2/π)·nb·E`（E 为阀侧空载线电压）
- 整流：`Ud = Ud0·cosα − (3/π)·nb·Xc·Id`；逆变侧按 γ 角同式
- 无功由换流器方程内生：`Q = P·tanφ`，`cosφ ≈ Ud/Ud0`
- 约束：`αmin ≤ α ≤ αstop`、逆变 `γ ≥ γmin`

已知边界（写入 `LCCConverter::model_limitations`）：换相重叠角 μ 未显式迭代，
平波电抗仅动态用；R 卡分接控制只进入统一 Newton 潮流的外层迭代，尚未作为
最优潮流的联合决策变量。R 卡缺失、范围无效或 T 支路无法匹配时，潮流会保留
T 卡固定 tap 并报告限制。

## 2. 逐元件映射表

图例：✅ 完全对应 / 🔶 部分对应 / ❌ 缺失 / ➕ HySim 独有

### 2.1 交流部分

| DSP 卡片 | HySim 元件 | 对应度 | 说明 |
|---|---|---|---|
| B（PQ 节点） | `ACBus`(PQ) + `Load` + `Shunt` | ✅ | Pload/Qload→负荷；Pshunt/Qshunt→`gs/bs` |
| BS（平衡） | `ACBus`(Slack) + `Generator` | ✅ | |
| BE（PV） | `ACBus`(PV) + `Generator` | ✅ | |
| BQ（无功受限 PV） | `ACBus`(PV) + `Generator.qmax/qmin` | ✅ | 导入为 PV；NR 求解支持 Q 限值转 PQ |
| L（线路） | `ACBranch` | 🔶 | R/X、B/2、额定电流→rate_a、并联回数均有；**G/2 对地电导不建模**（导入告警） |
| T（双绕组变） | `ACBranch`(带 tap) / `Transformer2W` | 🔶 | 励磁 G/B 不建模（告警）；DSP 数值扰动表明本案例的 T 卡 R/X 直接按系统基准进入支路 |
| R（LTC 调压） | 既有 `ACBranch` + `LCCConverter` tap 控制元数据 | 🔶 | R 是 T 支路的控制记录，**不新建第二台变压器或第二条电气支路**；匹配 LCC 换流变时保留范围并由统一 PF 外环调节 |
| BX/X（可投切并联） | `Shunt` | 🔶 | 固定并联可表示；无可投切并联稳态控制 |
| L+（线路高抗） | `Shunt` | 🔶 | 只能挂母线 |
| RZ（串补） | — | ❌ | 无串联补偿元件 |
| FS/FT/FU（FACTS） | — | ❌ | 无 SVC/STATCOM/TCSC/UPFC 稳态元件 |
| A/I/Z（区域/联络） | `ACBus.area/zone` | 🔶 | 字段在；区域交换功率（I 卡）无对象 |
| WA/WD/WS（风机） | `RenewableGen` | 🔶 | 潮流口径可映射，卡级导入未实现 |

### 2.2 直流部分

| DSP 卡片 | HySim 元件 | 对应度 | 说明 |
|---|---|---|---|
| **BD**（LCC 换流节点） | **`LCCConverter`** | ✅ | 桥数、αmin/αstop、阀压降、桥额定电流、直流额定电压全字段解析 |
| **LD**（直流线路+运行方式） | `DCBranch` + 两台 `LCCConverter` 的 setpoint | ✅ | R、额定电流、Psch、Vdc 整定、αN、γN、功率控制点标志均解析；L/C 仅动态用，跳过 |
| BA/LY/DC/BB（分层接入） | — | ❌ | 导入器显式跳过；Pwrflow.exe 不支持分层（仅 pfnt 需狗，§6.1）；结构上需 LCCConverter 串联端口小改（§6.4 判定 b） |
| BM/LM（LCC-MTDC） | `DCBus`+`DCBranch`+`LCCConverter` | 🔶 | 网络层可组（§6.4 判定 a）；DSP 接受（MULTI TERMINAL DC loader），列位部分实证（§6.3）；导入器跳过，角色分配未实现 |
| **BZ/BZ+**（VSC 站） | `VSCConverter` + `DCBus` | ✅ | 已解析（列位实证确定，见 §5）：定功率站 → `PQ_MODE`，定直流电压站（BZ+ 标志 1）→ `VDC_Q` 硬下垂 |
| **LZ**（柔直线路） | `DCBranch` | ✅ | 已解析：稳态取每极电阻 R（38-43 列）/极数；In 进 `rate_a_mva`；L/C/平波电抗仅动态用，跳过 |

### 2.3 HySim 独有（DSP dat 无对应）

➕ 三相族 7 类（`ThreePhaseACBus` 等）、`Transformer3W`、`FlexibleLoad`/
`AsymmetricLoad`、`AsynchronousMotor`、`Storage`/`MobileStorage`、
`DCStorage`/`DCLoad`/`PVArrayDC`/`StaticGeneratorDC`、`DCDCConverter`、
`EnergyRouter(+Port)`、`ChargingStation`/`Charger`、`Switch`/`CircuitBreaker`/
`DCCircuitBreaker`（含保护档案）、`VirtualPowerPlant`/`Microgrid`。

## 3. bpa_io 卡片支持矩阵与已知近似

已支持：`B/BS/BE/BQ`（含 zone、Vsch、Q 限值）、`L`、`T`、`BD`、`LD`、
与 LCC 换流变匹配的 `R`、`BZ`/`BZ+`/`LZ`（柔直，见 §5），以及控制卡中的
`MVA_BASE=/CASEID=/PROJECT=`。普通交流 LTC 的完整 R 系列控制不在本次范围内。

显式跳过（进 ImportReport）：`BA/LY/DC/BB/BM/LM`（实证结论见 §6），
其余未识别两字母码按 UnknownField 跳过；`(...)` 控制段在 `(END)` 终止。

已知近似/偏差（对比 DSP 解时的系统性来源）：

1. **T 卡阻抗基准**：DSP 对 CIGRE 的单字段扰动表明 T.X 直接按 100 MVA
   系统基准进入显式支路；`.00417/.00833/.01666` 对应的 T 支路无功损耗约为
   128.28/258.11/514.68 Mvar。LCC 每桥换相电抗按
   `Xc_pu = T.x_pu/(n_parallel*n_bridges)` 保存，基准为系统 MVA；CIGRE 为
   `0.004165 pu@100 MVA = 1.96125685 Ω`（217 kV 阀侧）。T 卡的 1800 MVA
   仍保留为设备额定容量，不能用来把 `.00833` 换算成 0.2179 Ω。
2. **R 卡与 T 卡是一对“设备+控制记录”**：导入器把 R 的可调绕组、上下限和
   档位数绑定到既有 T 支路，不增加支路。统一 PF 每次内层 Newton 收敛后按实际
   `Ud/Id`、LD 的 αN/γN（逆变端还包括 VDGA 的直流电压目标）反解阀侧目标电压，
   对 tap 限幅/取档/阻尼后重建 Ybus 并热启动下一轮。
3. **G/2 与励磁支路不建模**（仅告警），对超高压长线路的充电功率有微小影响。
4. `VscApprox` 兼容路径保留（`BpaImportOptions::lcc_model`），仅供回归对照，
   默认已切换为 `LccQuasiSteady`。
5. **优化边界**：PF 外环不是 OPF 的 tap 联合优化。OPF 仍把 LCC 控制定值和
   换流变 tap 视为固定输入；换相重叠角 μ 也未显式迭代。

## 4. 与 DSP 潮流的集成对比（四案例）

测试目标 `test_bpa_dsp_compare`（`tests/test_bpa_dsp_compare.cpp`），流程：
`parse_bpa_dat`（data/dsp/*.dat）→ `solve_power_flow` → 对比 DSP 参考解
（`data/dsp/*NEW.SOL` 母线电压；2DC/cigre 换流器数值取自 DSP `.pf` 日志，
硬编码于测试中并注明来源）。

参考解来源：39/IEEE90 取自 DSP 官方 Samples 自带 SOL；2DC/cigre 由
`DSP Bin/Pwrflow.exe` 无 GUI 批处理生成（平启动 + PQ 自动转 NR，5 次迭代收敛；
命令行复现的 39 解与样本 SOL 逐字节一致，证明批处理等价于 GUI）。

下表是 DSP 参考值以及 R 卡外环完成后的回归验收目标。新 tap 实现完成一致重建和
聚焦测试前，不把“目标”表述为已经通过：

| 量 | DSP / 2DC 目标 | DSP / CIGRE 目标 | 说明 |
|---|---|---|---|
| 整流/逆变 P_ac (MW) | -1500.00 / +1410.00 | -1500.00 / +1410.00 | 直流线路损耗不计作 LCC 阀侧有功损耗 |
| I_d (kA) | 3.000 | 3.000 | `Psch/Udr` |
| U_d 整流/逆变 (kV) | 500 / 470 | 500 / 470 | VDGA 的伴随电压目标必须闭合 |
| 直流线损 (MW) | 90.00 | 90.00 | `R·I² = 10·3²` |
| α / γ (°) | 15.00 / 17.00 | 15.00 / 17.00 | PAAL/VDGA 的角度目标 |
| `ACBranch.tap` 整流/逆变 (pu) | 549.25/500 / 574.48/500 | 547.97/525 / 571.25/525 | 分别约 1.09850/1.14896 与 1.04375/1.08810 |
| 阀侧 LCC Q 吸收 (Mvar) | 526.54 / 530.91 | 526.54 / 530.90 | `LCCTransfer.q_ac_mvar` 采用注入号规，结果应为负值 |
| T 支路漏抗 Q 损耗 (Mvar) | 258.15 / 258.15 | 258.11 / 258.11 | 与阀侧 LCC Q 分开统计 |
| 一次交流系统总 Q 吸收 (Mvar) | 784.70 / 789.06 | 784.65 / 789.01 | 阀侧 Q + T 支路 Q 损耗 |

PF 外环每一轮都先用当前 tap 完整求解 AC/DC 状态，再由该轮实际 `Ud/Id` 反解
满足 αN/γN 的阀侧电压。整流端的 PAAL 由内层定 P 与外层 α 共同闭合；逆变端
ConstantGamma 在未限流时角度残差恒为零，因此还必须使用 VDGA 的直流电压目标，
不能重复用 γ 残差冒充第二个自由度。所有更新都作用于原 T 支路；R 本身没有独立
潮流、损耗或画布线路。

## 5. BZ/BZ+/LZ（柔直 VSC）卡：实证列位格式与集成对比

dat 卡片手册只给了 LZ 的字段表，BZ/BZ+ 标注"待补充"。下列列位（字节列，
1 基闭区间，GBK 双字节名按原始字节域解析）由**实证**确定：以真实电网生产
dat（含 28 张 BZ、9 张 LZ 卡）为格式假设，编写单字段扰动算例用
DSP 2.1.47 `Pwrflow.exe` 试跑，观察 `.pf` 报告"柔性直流换流器"段、直流线
路潮流段与报错信息逐字段确认（约 25 组对照运行；更新日志提供旁证，如
"LZ卡没有对应BZ卡，提示错误后终止计算"、"BZ+ 卡直流平衡站指定电压为零的
错误提示"）。

### 5.1 BZ（换流站节点卡）

| 列 | 含义 | 实证依据 |
|---|---|---|
| 1-2 | `BZ` | |
| 3 / 4-6 | 修改码 / 所有者 | 惯例 |
| 7-14 | 换流站名（= 交流母线名，可直接无 B 卡） | DSP 按 8 字节定列匹配，错位即"找不到节点" |
| 15-18 | 交流基准电压 (kV) | |
| 19-20 | 分区 | |
| 21-25 | 额定容量 Sn (MVA)（信息位；改动稳态结果不变） | 扰动实验 |
| 34-37 | 某 pu 电抗（0.06-0.11；**稳态无影响**，疑动态用，未导入） | 扰动 ×2 稳态结果不变 |
| 41-43 / 45-49 | 含义未定（样本 286/400/336 与 15000/12000/8000；稳态均无影响，疑额定电流/短路容量） | 扰动实验 |
| 51-55 | **Rc 换流器电阻 (pu，换流器容量基准)**；DSP 按 \|P\|·Rc 记换流损耗 → 映射 `eta = 1 − Rc` | 扰动 ×2，损耗精确 ×2 |
| 59-60 | 含义未定（样本 "1."，稳态无影响） | 扰动实验 |
| 64 | **极数 n**；直流网络看到的线阻 = R/n（2→1 极线损精确 ×2） | 扰动实验 |
| 67-70 | 直流额定电压 (kV)（信息位；BZ+ 的 36-39 列为生效值） | 扰动实验 |
| 71+ | 可选尾随字段（含义未定，动态嫌疑） | — |

### 5.2 BZ+（延续卡；须跟在同名 BZ 之后，否则 DSP 终止，本库记 Error）

| 列 | 含义 | 实证依据 |
|---|---|---|
| 1-3 | `BZ+` | |
| 7-14 / 15-18 | 站名 / 交流基准电压 | |
| 20-24 | **P 整定 (MW)**；**负荷号规**：正 = 从交流吸收（整流），负 = 注入（逆变） | 翻转 P 符号，DSP 支路潮流方向随之翻转 |
| 26-29 | **Q 整定 (Mvar)**；正 = 吸收无功 | Q −50→+100 时母线电压下降（1.051→0.984） |
| 34 | **控制模式标志**：1 = 定直流电压站（模式 1，Udc 由 41-44 列整定，P 自由）；空/2/3/4 = 定功率站（模式 2，P/Q 按整定） | DSP 换流器表"模式"列；两站同置 1 时潮流归零、同置 2 时无平衡站结果失真 |
| 36-39 | **直流额定电压 UdcN (kV)**；必填 >0，同一直流网络各站须一致（不一致/空缺 DSP 报错"直流额定电压<=0kV"） | 置空报错；单站改动报错；两站同改通过 |
| 41-44 | **直流电压整定值 Udc_set (kV)**（模式 1 站；解精确等于该值） | 改 310/290，Udc 解精确跟随 |
| 53-58 | **Xc 换流电抗 (pu)**；只产生换流器内部无功损耗，不到交流端口（故不导入潮流模型） | 置 0 后内部无功损耗归零 |
| 60-63 / 66-69 | 交流/直流电压参考（信息位；66-69 列缩放换流器阀侧额定电压，稳态结果不受影响） | 扰动实验 |

### 5.3 LZ（柔直线路卡；两端须为已声明 BZ 站，否则 DSP 终止，本库记 Rejected 告警并跳过）

| 列 | 含义 | 实证依据 |
|---|---|---|
| 1-2 | `LZ`；3 / 4-6 修改码 / 所有者 | |
| 7-14 / 15-18 | 节点 1 名 / 基准电压（= 该节点自身声明电压，交流侧 kV，信息位） | 错位即"找不到该柔直线路末端节点" |
| 20-27 / 28-31 | 节点 2 名 / 基准电压 | 同上 |
| 34-37 | 额定电流 In (A)（信息位，进 `rate_a_mva`） | |
| 38-43 | **电阻 R (Ω/极)**；稳态 R_eff = R/极数 | 改 ".01"→".99" 线损按 I²R 精确跟随；置空时 DSP 默认 0.01 Ω |
| 44-53 | L (mH) / C (μF) / 平波电抗（**仅动态**，跳过） | 扰动稳态结果不变 |

### 5.4 导入映射与 vsc2 端到端对比

映射：BZ 站 → `VSCConverter` + `DCBus`（base = UdcN）；BZ+ 标志 1 →
`VDC_Q`（`v_dc_set_pu = Udc_set/UdcN`，k_vdc = 1e5 硬下垂，配合
`plan_dc_island_references` 的唯一电压源锁定保证不被模式切换降级）；
其余 → `PQ_MODE`；`p_set = −P卡`、`q_set = −Q卡`（卡为负荷号规）；
`eta = 1 − Rc`；LZ → `DCBranch`（`r_pu = (R/n)·S_base/UdcN²`）。

测试算例 `data/dsp/vsc2.dat`（两端柔直：VSCA 定功率逆变 1470 MW，
VSCB 定直压 300 kV 整流），DSP 参考解 `vsc2NEW.SOL` / `vsc2.pf`。
`test_bpa_dsp_compare` 实测（统一 Newton，8 次迭代收敛）：

| 量 | DSP | HySim | 偏差 |
|---|---|---|---|
| VSCA 母线电压 (pu) / 相角 (°) | 1.144266 / 20.8068 | 1.14427 / 20.8069 | ~4e-6 pu |
| VSCB 母线电压 (pu) / 相角 (°) | 1.051377 / −24.7554 | 1.0516 / −24.7345 | 2.4e-4 pu / 0.021° |
| VSCA P_ac / Q_ac (MW/Mvar) | +1470.00 / −50.00 | +1470.00 / −50.00 | 精确（控制量） |
| VSCA 直流侧 P (MW) / 损耗 | 1480.36 / 10.363 | 1480.29 / 10.29 | 0.005% |
| VSCB P_ac (MW) | −1491.69 | −1490.77 | 0.06% |
| VSCB 直流侧 P (MW) / 损耗 | 1481.25 / 10.442 | 1480.41 / 10.36 | 0.06% |
| Udc B / A (kV) | 300.0000 / 299.9753 | 299.978 / 299.953 | ≤0.03 kV（硬下垂残差） |
| I_dc (kA) | 4.935 | 4.935 | <0.2% |

未决事项：BZ 卡 34-37 / 41-43 / 45-49 / 59-60 列与 71 列后尾随字段含义未
定（实证均为稳态惰性，疑动态/保护参数）；LZ 的 L/C/平波电抗列位边界未
逐一校核（动态用，不导入）；BZ+ 60-63/66-69 列为信息位。DSP 对 Xc 内
部无功损耗建模在换流器内部（不到交流端口），HySim 潮流模型无此内部电
抗，不影响端口量对比。

## 6. BA/LY/DC/BB（分层接入 LCCDC）与 BM/LM（多端 LCC-MTDC）：实证结论与架构判定

调查方法同 §5（单字段扰动 + 报错信息迭代，DSP 2.1.47 `Pwrflow.exe`，约 30 组对照运行），
外加二进制字符串/加载器清单取证。核心结论先行：**可运行的 Pwrflow.exe 不支持分层接入
（BA/LY/DC），该功能只存在于需加密狗的 bin2/pfnt.exe；多端直流（BM/LM）则被
Pwrflow.exe 完整接受**。未实证字段一律明确标注。

### 6.1 BA/LY/DC（分层接入）：Pwrflow.exe 实证为不可用

证据链：

1. **加载器清单不含 LCCDC**。Pwrflow.exe 二进制中全部 `LOADING ...` 阶段字符串枚举
   只有 `TWO TERMINAL DC DATA`（BD/LD）与 `MULTI TERMINAL DC DATA`（BM/LM）及
   VSC 相关读取器；二进制全文检索"分层"/"LCCDC"（GBK）命中 0 次。
2. **BA 卡被排序器/查重识别但无加载器认领**。BA 指向已存在交流母线时报
   `ERROR 重复节点：NORTH 500.`（证明 7-14 列节点名、15-18 列基准电压被读取，
   BA 属建节点卡）；但指向新节点时，无论卡片位置（BD 后/LD 后/L 段后/数据末尾）
   与字段布局（8 种变体）如何，一律在变压器数据阶段报
   `BPATRF: THE FOLLOWING CARD IS IN THE WRONG LOCATION.` 并致命终止
   （排序表把 BA 排在 BS 与 BF 之间的母线段，但功率流各加载器均不认领）。
3. **pfnt.exe（bin2）实现了 LCCDC 但需加密狗**（无狗直接崩溃，exit=24）。其
   字符串与手册语义一一对应，可作为手册第 4 章语义的旁证：
   "分层接入直流仅支持直流系统控制方式'整流侧定功率定电压'"、
   "整流/逆变侧层控模式不正确，仅支持定比例"、"整流端高低侧功率分配比例之和
   不等于100%"、"直流系统控制卡整流/逆变端高/低侧节点不存在"、
   "的整流/逆变端类型与控制卡中的不一致"、报告段"LCCDC直流数据列表"。

因此手册 BA/LY/DC 字段表的**语义**（Ud = Ud,H + Ud,L、Id 各层相同、kp,H+kp,L=100%、
DC 卡四端节点+系统类型+控制方式+定比例层控+Psch/Vsch）有手册与 pfnt 字符串双重依据，
但**列位未实证**（无法用可运行程序做数值确认），本库不据此实现解析。

### 6.2 BB（混合直流无源节点）：被 VSC/混合直流读取器认领

BB 试验卡被 `RDVSCDC` 处理并报 `RDVSCDC ERROR: THIS BZ BUS IS ISOLATED: RECTH 217.`，
即本版 DSP 把 BB 当作挂入 VSC 直流网络上下文的无源节点（与手册"混合直流无源节点"
定位一致），而不是分层接入卡。其列位（7-14 名、15-18 kV 已确认被读取；类型/层型/
Ppercent 列位）未实证。

### 6.3 BM/LM（多端直流）：Pwrflow.exe 接受，列位部分实证

- **BM 卡**：由 `LOADING MULTI TERMINAL DC DATA` 认领解析。已确认：7-14 列节点名、
  15-18 列基准电压、19-20 列分区（报错均按此定位节点）；前段与 BD 同构（桥数、SR、
  αmin/αstop、阀压降、桥额定电流、一次侧节点/电压，参照 §2.2 BD 行）。一致性检查
  按直流系统依次校验："约束条件数不合理"→"定电压节点数不合理"→"功率平衡节点数
  不合理"，即**每个多端系统须恰有一个定电压（平衡）站，其余站定功率**；无源联络
  节点须在换流器类型处填 M（"如果是多端直流的无源联络节点，应该填写标志位M"）；
  另有"直流平衡站指定电压为零"检查。
- **LM 卡**：端点必须是已声明的 BM 节点（否则报
  `找不到直流枢纽节点：RECTH 217.`）；前段布局与 LD 一致（7-14/15-18 节点1、
  20-27/28-31 节点2），In/R 列位参照 LD（34-37/38-41，未单独扰动确认）。
- **BM 尾段列位（部分实证，单布局证据，未最终固定）**：在布局"63 列换流器类型 R/I、
  64-68 αN/γN、69-73 γmin、78-82 Psch、83-87 DcVsch、88-92 直流额定电压"下，
  三端试验系统（整流定功率 1500 MW + 逆变定电压 470 kV + 逆变定功率 -500 MW）
  通过了定电压/功率平衡一致性检查并推进到换流节点-交流关联校验；该校验把一次侧
  节点名按约左移 1 列读出（报"找不到直流的换流节点：.NORTH 500"），提示 BM 的
  一次侧字段边界与 BD（51-58/59-62 列）可能差 1 列（桥电流额定值或为 46-49 四列）。
  迭代到此中止，以上尾段列位标注"**部分实证/未确认**"。
- **DSP 内部规格化卡影像**（运行日志 BLFSIM_dc.msg，两端直流实证）：换流器按
  "直流节点→GROUND"建模（如 `BD RECTFIER217.GROUND ...`），控制模式码
  PAAL（定功率定α）/VDGA（定电压定γ）。本库把代码保存在
  `external_control_code` 中，并用 `tap_control_modelled` 表示是否已找到同一换流站的
  T 支路和有效 R 范围。它不替代 `LCCControlMode`：内层仍选择定 P/定 γ 外特性，
  PF 外层再消费 α/γ 与伴随直流电压目标。

### 6.4 架构判定：(a) 多端可表达 / (b) 分层需小改 / 不需要新顶层类别

**(a) 多端直流（BM/LM）——现有 `LCCConverter + DCBus + DCBranch` 原样可表达。**
证据：`DCBranch` 是任意两直流节点间的纯电阻（`from_bus`/`to_bus`），DC 网络是一般
节点式网络（手册 BM 卡的 Y_dc 节点法同构）；DSP 的一致性检查要求"一个定电压/平衡站
+ 其余定功率站"。现有两端 BD/LD+R 导入已经把 `ConstantPower`/`ConstantGamma`
内层特性与 PAAL/VDGA 的分接头外环组合起来；逆变端还使用伴随的直流电压目标闭合
VDGA。这里尚未覆盖的是 BM/LM 的卡片解析、站角色分配、R 记录绑定和多端复合控制，
而不是两端 PF 缺少 tap 状态。即使补齐这些导入语义，OPF 仍需另行把 tap 作为联合决策
变量；多端网络拓扑本身无需新顶层类别。

**(b) 分层接入（BA/LY/DC）——原样不能表达，需给 `LCCConverter` 加直流串联端口
（小改，非新元件类别）。** 证据：
- 物理结构（手册 + pfnt 字符串）：高/低两层换流器直流侧串联，Ud = Ud,H + Ud,L，
  Id 相同，存在中点节点 KCL；两层分别接不同交流母线。
- HySim 现状（代码证据）：`LCCConverter` 只有单 `dc_bus`
  （converter_components.hpp:145）；`lcc_eval_at_state` 以 `vdc[dc_pos]*base` 求
  ud_kv（lcc_model.cpp:185）；`lcc_dc_injection` 返回标量、只注入单一节点
  （jacobian_builder.cpp:196-198、pf_injection_assembly.cpp:83）；DC 雅可比只取
  对角元 `dc_vdc_diag_nz`（jacobian_builder.cpp:364-374）；`DCBus` 电压一律对地
  参考。
- 串联对（高端 +Id 取自利点 P、−Id 注入中点 M，端口电压 V_P−V_M）无法用"每节点
  对地注入"表达——需要换流器产生 DC 雅可比非对角耦合项，现有 pattern 不会生成。
  把两台定功率 LCC 并挂在同一极母线上虽能使极级电流/功率正确
  （(kp,H+kp,L)·Psch/U = Id），但每层看到的端口电压是全极电压而非 kp·U，
  层间量（各层 α、Q_ac）会系统性失真，故不算 (a)。
- 最小扩展：`LCCConverter` 增加可选第二直流端 `dc_bus_low`（默认 0 = 对地，行为
  与现状完全一致）；ud 取两端电压差；DC 残差在两端注入 ±p_dc；雅可比补两端导数
  （含非对角）；validation 增加 dc_bus_low 合法性检查。改动集中在 lcc_model.cpp
  与 jacobian_builder/pf_injection_assembly 的 LCC 注入点，不触及 AC 侧与
  projection。导入映射：BA → 两台 LCCConverter（H 层 dc_bus=极母线、
  dc_bus_low=中点；L 层 dc_bus=中点），整流层 ConstantPower(kp·Psch)、逆变层按
  定比例换算；LY → 极间 DCBranch；DC 卡 → 系统级整定来源。

**(c) 不需要新增"特高压直流"顶层类别——§1 结论成立并被本次实证强化。** 分层接入是
换流器阀组级的串联结构，不是新的网络类型；±800/±1100 kV 在网络层只是更高 `base_kv`
的 `DCBus`/`DCBranch`。DSP 侧佐证：所有直流形态（两端/多端/柔直/分层）都以"节点卡+
线路卡"挂在同一个直流网络上（内部影像 bus→GROUND、BM 卡节点法 Y_dc），pfnt 对分层
也只加一张 DC 控制卡做系统级整定，并无独立网络层。
