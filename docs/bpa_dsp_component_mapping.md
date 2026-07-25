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
换相电抗 `x_comm_pu`@换流变 Sn 基准、控制模式 `ConstantPower/ConstantCurrent/
ConstantAlpha/ConstantGamma` 及对应 setpoint），挂于
`HybridPowerSystem::lcc_converters`，复用现有 `DCBus`/`DCBranch` 网络层。

准稳态外特性（与 DSP dat 卡片手册第 4 章一致）：

- `Ud0 = (3√2/π)·nb·E`（E 为阀侧空载线电压）
- 整流：`Ud = Ud0·cosα − (3/π)·nb·Xc·Id`；逆变侧按 γ 角同式
- 无功由换流器方程内生：`Q = P·tanφ`，`cosφ ≈ Ud/Ud0`
- 约束：`αmin ≤ α ≤ αstop`、逆变 `γ ≥ γmin`

已知边界（写入 `LCCConverter::model_limitations`）：换相重叠角 μ 未迭代、
换流变分接头固定（R/LTC 卡不解析，DSP 用分接头把 α 维持在 αN）、
平波电抗仅动态用。因此与 DSP 解对比时，α 可偏离整定值 αN。

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
| T（双绕组变） | `ACBranch`(带 tap) / `Transformer2W` | 🔶 | 励磁 G/B 不建模（告警）；T 卡 R/X 按系统基准读入（DSP 手册口径为 Sn 基准，见 §3 已知偏差） |
| R（LTC 调压） | `RegulatorControl`（模型层） | 🔶 | **bpa_io 整体跳过 R 卡**，变比固定在 T 卡值；DSP 侧 LTC 会调分接头 |
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
| BA/LY/DC/BB（分层接入） | — | ❌ | 导入器显式跳过（可用两台 LCCConverter 串联表达，未实现） |
| BM/LM（LCC-MTDC） | `DCBus`+`DCBranch`+`LCCConverter` | 🔶 | 网络层可组；多端控制角色分配未实现，导入器跳过 |
| **BZ/BZ+**（VSC 站） | `VSCConverter` | 🔶 | 模型能力完全覆盖（定 Udc/P/Q/Vac）；**bpa_io 未解析 BZ 卡** |
| **LZ**（柔直线路） | `DCBranch` | 🔶 | 稳态结构对应；导入器跳过 |

### 2.3 HySim 独有（DSP dat 无对应）

➕ 三相族 7 类（`ThreePhaseACBus` 等）、`Transformer3W`、`FlexibleLoad`/
`AsymmetricLoad`、`AsynchronousMotor`、`Storage`/`MobileStorage`、
`DCStorage`/`DCLoad`/`PVArrayDC`/`StaticGeneratorDC`、`DCDCConverter`、
`EnergyRouter(+Port)`、`ChargingStation`/`Charger`、`Switch`/`CircuitBreaker`/
`DCCircuitBreaker`（含保护档案）、`VirtualPowerPlant`/`Microgrid`。

## 3. bpa_io 卡片支持矩阵与已知近似

已支持：`B/BS/BE/BQ`（含 zone、Vsch、Q 限值）、`L`、`T`、`BD`、`LD`、
控制卡中的 `MVA_BASE=/CASEID=/PROJECT=`。

显式跳过（进 ImportReport）：`R`（LTC）、`BA/LY/DC/BB/LZ/BM/LM`，
其余未识别两字母码按 UnknownField 跳过；`(...)` 控制段在 `(END)` 终止。

已知近似/偏差（对比 DSP 解时的系统性来源）：

1. **T 卡阻抗基准**：DSP 手册口径 T 卡 R/X 以变压器 Sn 为基准；bpa_io 按
   系统基准标幺读入（历史行为，39/IEEE90 样本数值恰为系统基准故无影响）。
   LCC 路径的换相电抗已按手册正确口径取 Sn 基准（`x_comm_pu`）。
2. **R 卡跳过**：DSP 开 LTC 时会把换流变分接头调到维持 α=αN
   （2DC 参考解中变比被调到 217/549.25）；本库固定分接头，α 自由漂移，
   阀侧交流电压随之不同——这是直流案例母线电压偏差的主因。
3. **G/2 与励磁支路不建模**（仅告警），对超高压长线路的充电功率有微小影响。
4. `VscApprox` 兼容路径保留（`BpaImportOptions::lcc_model`），仅供回归对照，
   默认已切换为 `LccQuasiSteady`。

## 4. 与 DSP 潮流的集成对比（四案例）

测试目标 `test_bpa_dsp_compare`（`tests/test_bpa_dsp_compare.cpp`），流程：
`parse_bpa_dat`（data/dsp/*.dat）→ `solve_power_flow` → 对比 DSP 参考解
（`data/dsp/*NEW.SOL` 母线电压；2DC/cigre 换流器数值取自 DSP `.pf` 日志，
硬编码于测试中并注明来源）。

参考解来源：39/IEEE90 取自 DSP 官方 Samples 自带 SOL；2DC/cigre 由
`DSP Bin/Pwrflow.exe` 无 GUI 批处理生成（平启动 + PQ 自动转 NR，5 次迭代收敛；
命令行复现的 39 解与样本 SOL 逐字节一致，证明批处理等价于 GUI）。

结果摘要（求解器接入后实测，统一 Newton + LCC 准稳态，6/5 次迭代收敛）：

| 量 | DSP | 2DC 实测 | cigre 实测 | 说明 |
|---|---|---|---|---|
| 整流 P_ac (MW) | -1500.00 | -1500.00 | -1500.00 | 定功率控制，精确 |
| 逆变 P_ac (MW) | +1410.00 | +1410.00 | +1410.00 | 精确 |
| I_d (kA) | 3.000 | 3.000 | 3.000 | 额定电流（电流指令）处限幅 |
| U_d 整流/逆变 (kV) | 500 / 470 | 500.0 / 470.0 | 500.0 / 470.0 | 精确 |
| 直流线损 (MW) | 90.00 | 90.00 | 90.00 | R·I² = 10·3² |
| α 整流 (°) | 15.00 | 28.55 | 23.89 | 固定分接头，反算值（DSP 靠 LTC 维持 αN） |
| γ 逆变 (°) | 17.00 | 33.14 | 29.10 | 反算值，> γ_min，换相裕度保持 |
| 整流 Q_ac (Mvar) | -784.7 | -825.0 | -676.0 | Q=P·tanφ，cosφ=U_d/U_d0，U_d0 因固定分接头偏高 |
| 逆变 Q_ac (Mvar) | -789.1 | -928.7 | -792.1 | 同上 |
| 全母线 max\|ΔV\| (pu) | — | 7.9e-3 | 2.6e-4 | 2DC 偏差集中在换流母线（无功差），cigre 系统坚强 |
| 全母线 max\|Δδ\| (°) | — | 0.15 | <0.001 | |
| 39 / IEEE90 | SOL | ≤2e-3 / ≤0.5° | ≤2.6e-3 / ≤0.4° | 纯交流，与 VSC 时代一致 |

机理：固定分接头使 U_d0 比 DSP 调压后的值高 ~10-15%，逆变 CEA 特性在该
U_d0 下要求的电流远超额定，额定电流限幅（= LD 卡电流指令 Psch/U_dr =
3 kA）成为起作用约束，直流工作点精确落在 DSP 的 500/470 kV、3 kA 上；
α/γ 两个由分接头吸收的自由度以反算值报告（写入 `LCCTransfer` 并附
`[LCC-PHYS-03]` 诊断），母线电压偏差主要来自换流器无功差
（Q = P·tanφ，cosφ = U_d/U_d0）。
