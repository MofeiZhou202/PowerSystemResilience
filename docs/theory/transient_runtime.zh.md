# 暂态运行时契约

> 本文档为 [transient_runtime.md](transient_runtime.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

更新日期：2026-08-24

暂态模块是一个相量域的机电暂态仿真。其实现位于 `include/hacdcpf/dynamics`
与 `src/dynamics`。历史性的数学背景保留在
`archive/theory/dynamics_electromechanical_transient_design.md`；该归档设计
不定义当前行为。本文档负责定义运行时契约。

## 求解路径

`POST /api/session/run_transient` 构建一个 `DynamicSystem`，可选地执行潮流
初始化和动态修整（dynamic trimming），应用预定事件，对模型进行积分，并
记录选定的快照。响应包含初始化诊断、求解器统计信息、母线电压/频率矩阵、
设备序列、已应用事件记录、警告，以及可选的模态/CSV 数据。

对 `MassMatrixDae`，只有请求的潮流初始化已收敛且快状态残差满足
`dynamic_trim_tol`，才允许进入时间积分。全局一致初值 Newton 阶段会把代数网络
容差临时收紧为“用户值”和 `0.1*dynamic_trim_tol` 中较小者，结束后恢复运行期值，
避免较松的网络求解噪声阻止较严的微分状态认证。默认 Anderson--Picard 网络预算为
10 次并保留提前退出；Newton 回退从 Picard 之前的电压种子重新启动。

混合 AC/DC 初始化与稳态潮流共享同一 DC/DC 端口功率方程。Power 模式保留一阶功率
状态；Voltage/Droop 模式在机电时间尺度作快内环代数约化。`DERAADynamic` 实现
PSD/WECC 的 7 状态（`Freq_Flag=0`）与 10 状态（`Freq_Flag=1`）链，包括 P/Q 电流
优先级、频率死区/下垂、IEEE 421.5 方向性非抗饱和、功率指令与电流斜率限制。
其频率量测输入为当前动态状态计算的系统 COI 频率；直接保护继电器仍使用本地 CT/PT
频率，两者不能混用。

对 `MassMatrixDae`，同时设置 `enable_der_protection=true` 与
`localize_der_protection_events=true`，会对实现
`DynamicDevice::previewProtection` 的保护执行回滚/二分事件定位。当前可执行范围
包括 IEEE 1547 设备保护，以及直接挂接到 `DynamicSystem` 的定时限电压继电器和
平衡正序 Zone-1 表观导纳继电器；继电器动作目前覆盖 `ACLoadScale` 与
`ACBranchTrip`，并复用预定事件的网络重置路径。
请求可配置 `protection_event_time_tol_s`、
`protection_event_cluster_window_s`、
`protection_event_max_localization_iters` 与
`post_event_algebraic_residual_tol`；响应返回是否实际定位、试积分次数、事件簇数、
最大时间括号、最大事件簇跨度（s）和最大事件后代数残差。超过迭代或残差门限时仿真失败，不返回
看似合理的轨迹。

定位容差与聚类窗口相互独立。最早保护动作物理时刻为 $t_e$ 时，求解器打开固定的
`[t_e, t_e + protection_event_cluster_window_s]` 前向窗口，并显式积分到其右边界。
窗口内动作保留各自物理时间戳，但属于同一事件簇；中间动作不会滚动延长窗口。
每条保护事件记录带零基 `protection_cluster_id`；预定或其他非保护事件为 `-1`。

直接继电器使用本地相量域 CT/PT 测量链。对零阶保持的本地正序输入 $z$，复数互感器
状态按 `z_m(t+dt)=z+(z_m(t)-z)exp(-dt/T)` 精确推进。本地频率按
`f_n+unwrap(angle(V_PT,k)-angle(V_PT,k-1))/(2*pi*dt)` 计算，并可再经过一阶精确
滤波；PT 电压低于 `frequency_min_voltage_pu` 时频率无效并闭锁频率元件。电压恢复后，
必须经过一个首尾 PT 电压均有效的完整区间才解除闭锁，不能重新启用跌落前的陈旧频率。
距离元件
使用 `|I_CT|/|V_PT|`。系统 COI 频率不再作为直接继电器输入。
相角估计要求每个接受区间满足 `|Delta angle| < pi`。线性 CT/PT 等效模型不覆盖
饱和、磁滞、变比误差、抗混叠或数字继电器采样固件。

服务器会缓存序列化后的富空间（rich-space）结果。GUI 通过以下方式获取单个
采样帧：

```http
GET /api/session/transient/frame?index=N
```

## 一帧可以显示什么

- 原创（authored）AC 和 DC 母线电压；
- 原创 AC 母线频率；
- 实际记录到的动态设备 P/Q、电流、控制器/状态量以及在运指标；
- 仅当存在 SOC（荷电状态，state-of-charge）序列时才显示 SOC；
- 已应用事件的位置及其 active/applied 状态。

## 禁止的推断

暂态帧不得使用静态潮流公式计算支路 P/Q。只有在动态求解器为该采样点记录
了端子电流或端子 P/Q 之后，才允许显示动态支路箭头。在此之前，该帧返回：

```json
{
  "capabilities": { "branch_power": false },
  "geo_ac_branches": [],
  "geo_dc_branches": [],
  "ac_circuit_breaker_flows": []
}
```

## 验证边界

GridLAB-D/OpenDSS 对比仅验证其明确共享的模型范围。平衡快照对比并不能为
动态逆变器、电机、直流链路（DC-link）、储能或事件方程提供认证。电压健康
检查失败与初始化不收敛仍然是可见的错误；回放不得把失败的运行重新标记为
已收敛。

当前定位范围是数值保证，不是形式化认证。分块积分器仍采用接受步末保护语义并
返回警告；rich-model builder、JSON/HTTP/GUI 继电器构造、反时限积分、多区距离
保护、擦边/芝诺事件、EMT 开关和不确定性可达性不在覆盖范围。由于网络只提供相量，
`EmtInstantaneous` 测量会在构造时显式拒绝。Zone-1 仅实现 COSMIC 的平衡正序、无变比线路子集，不是通用距离保护
包。继电器守卫在接受步端点之间线性插值；若越限脉冲在一个步内发生并恢复、且
两个端点均未越限，当前方法不会发现，必须减小基础步长。

外部验证使用 MATLAB R2025b 运行作者 COSMIC 仓库的固定干净提交
`6acc77e4d3f17925f1f4b79a93652eef0d1314cc`。仓库自带 `sim_case9.m` 已复现：
`10.0 s` 切除支路 6，`10.5 s` 母线 5 UVLS 动作，切负荷 `31.25 MW`（25%）。
但按论文图 2 的支路 7 初始切除、`0.92 pu` UVLS 阈值及 `0.5 s` 延时重建时，
母线 5 最低电压虽为 `0.896065 pu`，支路 6 最大距离拾取比却仅为
`0.3098124214893234 < 1`，没有 `10.5 s` 距离动作或 `10.7 s` UVLS。由此只能
判定论文配置或代码版本未完整包含在该公开提交中；这是明确的交叉验证失败，不能
写成 HySim 已匹配。复现命令见 `tools/transient_validation/README.md`。

经典 GUI `power_system.json` 混合算例现已成为注册的 `MassMatrixDae` 初始化回归。
其中 Droop DC/DC 通过共享的稳态/动态方程提供直流电压参考；不得用伪造直流源或渲染
回退掩盖控制缺失。

DER_A 已用本机 PowerSimulationsDynamics 源码对初始化、两个频率标志分支、非抗饱和、
功率指令与电流斜率逐式建立 `1e-12` 方程门。但尚不宣称 PSD Test 42 轨迹等价：当前
检出的 PSD 测试环境在算例运行前即因 SciML 包集无法预编译而失败
（`LinearVerbosity`/扩展方法覆盖冲突）。

保护动作现在按语义 `component_type`、组件编号以及（若给出）母线联合匹配；
因此 PV、VSC 与同步发电机可以复用编号而不会交叉跳闸。不存在的支路、母线负荷
或故障目标会使本次运行失败，且不会写入 `applied_event_records`。拓扑保护动作后，
MassMatrixDae 在同一物理时刻执行保护闭包（最多 32 次迭代），每次动作后重新求解
代数约束；超过上限会显式报告 chronology 失败。保护事件使用独立配置的固定前向
窗口聚类并报告最大物理时刻跨度。直接电压、频率和距离继电器均使用本地 PT/CT
测量状态；EMT 瞬时波形测量仍明确不支持。
