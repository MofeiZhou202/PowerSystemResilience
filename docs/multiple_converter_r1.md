# AC/DC 变换器 AC 侧构网与 VSC 七类控制模式补充文档

# 理论补充与工程实现建议

---

## 0. 文档目的

本文档用于补充《混合 AC/DC 多变换器控制校核与统一潮流求解系统》中的 AC/DC 变换器控制建模部分，重点完善以下内容：

1. AC/DC 变换器 AC 侧 PV 控制模式；
2. AC/DC 变换器 AC 侧构网模式；
3. AC/DC 变换器 AC 侧构网与 DC 侧构网的互斥规则；
4. VSC 七类典型控制模式与统一设备方程框架的映射；
5. AC/DC 变换器在 AC 侧 PV、AC 侧构网、DC 侧构网、DC 下垂等模式下的方程闭合条件；
6. 对现有数据结构、控制角色解析、方程生成器和校核规则的补充建议。

核心结论是：

> AC/DC 变换器可以支持 AC 侧 PV 控制，也可以支持 AC 侧构网控制；但普通两端口 AC/DC 变换器不能同时在 AC 侧和 DC 侧构网。VSC 七类典型控制模式可以在统一设备方程框架下覆盖其稳态代数形式，其中 AC 侧构网模式需要显式增加角度或频率参考建模，并与 DC 侧构网互斥。

---

## 0.1 当前代码实现状态

本文档是理论补充和工程实现建议；截至当前代码，核心稳态框架已经落到以下位置：

- `include/hacdcpf/model/converter_components.hpp`：`VSCConverter` 已包含 `p_is_hard_constraint` / `p_schedule_mw` / `p_initial_mw`、`v_ac_angle_set_deg`、AC/DC 电流限值、调制比限值、`r_conv_ac_pu`、AC/DC 构网标志、双侧构网能量缓冲门控、协调组/主从/参与因子字段；`DCDCConverter` 已包含拓扑、占空比上下限和变比字段。
- `include/hacdcpf/model/device_control_role.hpp`：`resolve_device_control_role()` 将 VSC 控制枚举解析为七类控制角色，明确每种模式控制哪些量、释放哪些量、提供哪些 AC/DC 岛参考。
- `include/hacdcpf/power_flow/converter_coordination.hpp` 与 `src/power_flow/converter_coordination.cpp`：执行 AC_PV、AC 构网、DC 构网、双侧构网、DC 岛多电压源、主从/参与因子等协调检查，并把 `ConverterCoordinationReport` 写入 `PowerFlowResult::diagnostics`。
- `include/hacdcpf/power_flow/power_flow_result.hpp`：`PowerFlowResult` 返回 `vsc_transfers`、`dcdc_transfers`、`er_port_transfers`、`effective_converters` 和 `converter_model_scope`。DC/DC 结果包含 duty、voltage ratio、duty_defined、duty_feasible。
- `include/hacdcpf/optimal_power_flow/opf_result.hpp`：AC/DC OPF 结果也返回 `converter_model_scope`，用于说明当前求解路径实际约束了哪些换流器物理/控制特性。

当前稳态潮流已支持 Mode 1 `AC_GRID_FORMING`、Mode 2 `AC_PV`、Mode 3 `PQ_MODE`、Mode 6 `DC_V_DROOP_AC_V` 以及 VDC_Q/VDC_VAC 的 DC 电压控制角色。需要注意：

- `AC_PV` 固定 AC 有功和 AC 电压幅值，释放 AC 无功，但不提供 AC 角度参考。
- `AC_GRID_FORMING` 固定 AC 角度和电压幅值，释放 AC P/Q，必须有 DC 侧功率/电压支撑。
- 普通两端口 VSC 不能同时 AC 侧和 DC 侧构网；只有显式 `allow_dual_side_grid_forming && has_energy_buffer` 时才允许进入高级双侧构网情形。
- 潮流会报告 converter model scope。Newton 潮流当前建模 VSC 损耗、AC 侧导通损耗、VDC 控制、DC/DC 损耗和多源协调，但 VSC 容量圆、电流限值、调制限值以及 DC/DC duty 限值主要以 OPF 约束或 post-solve 诊断形式出现。使用结果时应读取 `converter_model_scope.validity`，不要只根据模式名推断。

---

# 1. AC/DC 变换器 AC 侧 PV 控制

---

## 1.1 AC 侧 PV 控制的定义

AC/DC 变换器 AC 侧 PV 控制是指：

```text
AC 侧有功功率固定；
AC 侧电压幅值固定；
AC 侧无功功率作为未知量；
AC 侧相角不由该设备固定。
```

其控制方程为：

$$
P_{k,ac} - P_{k,ac}^{set} = 0
$$

$$
V_i^{ac} - V_{k,ac}^{set} = 0
$$

其中：

- $$P_{k,ac}$$ 为 AC/DC 变换器向 AC 网络注入的有功功率；
- $$V_i^{ac}$$ 为 AC 侧母线电压幅值；
- $$P_{k,ac}^{set}$$ 为 AC 侧有功设定值；
- $$V_{k,ac}^{set}$$ 为 AC 侧电压幅值设定值。

功率守恒方程为：

$$
P_{k,ac} + P_{k,dc} + P_{k,loss} = 0
$$

其中：

$$
P_{k,loss} \geq 0
$$

此时，$$Q_{k,ac}$$ 不应作为给定量，而应作为未知量参与 AC 无功平衡方程。

---

## 1.2 AC 侧 PV 控制不是 AC 侧构网

AC 侧 PV 控制与 AC 侧构网有本质区别。

AC 侧 PV 控制给定：

$$
P_{k,ac} = P_{k,ac}^{set}
$$

$$
V_i^{ac} = V_{k,ac}^{set}
$$

但不固定 AC 相角：

$$
\theta_i
$$

因此 AC_PV 设备不能为 AC 岛提供相角参考。

如果一个 AC 岛只有 AC_PV 型 AC/DC 变换器，而没有 slack bus、外部电网、同步发电机或 AC grid-forming 设备，则该 AC 岛仍然缺少角度参考，应报：

```text
ACISLAND-REF-01:
AC island has no angle reference.
```

---

## 1.3 AC 侧 PV 控制与传统 PV 节点的区别

传统 AC PV 节点通常表示：

| 给定量 | 未知量 |
|---|---|
| $$P_i, V_i$$ | $$Q_i, \theta_i$$ |

但 AC/DC 变换器 AC 侧 PV 控制不应简单地将 AC bus 设置为传统 PV bus。

原因包括：

1. 一个 AC bus 上可能挂接多个设备；
2. AC/DC 的 $$Q_{k,ac}$$ 是设备注入，不一定等于整个母线的无功注入；
3. AC/DC 还连接 DC 网络，需要满足 DC 侧功率平衡；
4. AC/DC 存在损耗方程；
5. AC/DC 的有功给定需要 DC 侧具备吸收或提供功率的能力；
6. AC_PV 不提供 AC 角度参考。

因此建议建模为：

```text
AC bus 只描述网络状态变量；
AC/DC 设备通过控制方程约束 P_ac 和 V_ac；
Q_ac 作为设备未知注入进入统一方程；
P_dc 由损耗方程和 DC 侧平衡共同决定。
```

---

## 1.4 AC 侧 PV 控制的方程集合

对于 AC/DC 变换器 $$k$$，连接 AC bus $$i$$ 与 DC bus $$j$$，AC_PV 模式下建议注册以下未知量：

```text
theta_i
V_i_ac
P_k_ac
Q_k_ac
P_k_dc
必要时 P_k_loss
```

其核心等式包括：

### AC 有功控制

$$
F_{P,k}^{acpv}
=
P_{k,ac} - P_{k,ac}^{set}
=
0
$$

### AC 电压幅值控制

$$
F_{V,k}^{acpv}
=
V_i^{ac} - V_{k,ac}^{set}
=
0
$$

### AC/DC 损耗与功率守恒

$$
F_{loss,k}^{acdc}
=
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

### AC 网络平衡

$$
F_{P,i}^{ac}
=
P_i^{inj}
-
P_i^{calc}(V^{ac},\theta)
=
0
$$

$$
F_{Q,i}^{ac}
=
Q_i^{inj}
-
Q_i^{calc}(V^{ac},\theta)
=
0
$$

### DC 网络平衡

$$
F_{P,j}^{dc}
=
P_j^{inj}
-
P_j^{calc}(V^{dc})
=
0
$$

---

## 1.5 AC_PV 控制合法性规则

建议新增如下规则。

```text
ACDC-CTRL-03:
AC_PV mode requires AC active-power setpoint and AC voltage setpoint, with AC reactive power released as unknown.
```

若 AC_PV 模式下同时硬指定无功功率：

$$
Q_{k,ac} - Q_{k,ac}^{set} = 0
$$

则报：

```text
ACDC-CTRL-04:
AC_PV mode cannot simultaneously impose AC reactive power as hard constraint.
```

若 AC_PV 被错误用于提供 AC 岛角度参考，则报：

```text
ACDC-CTRL-05:
AC_PV mode does not provide AC angle reference.
```

---

# 2. AC/DC 变换器 AC 侧构网模式

---

## 2.1 AC 侧构网的定义

AC/DC 变换器 AC 侧构网是指该设备在 AC 端口形成 AC 电压参考，至少应包含：

```text
AC 电压幅值参考；
AC 相角参考或频率参考；
有功平衡能力；
无功支撑能力。
```

在静态潮流模型中，AC 侧刚性构网通常表示为：

$$
\theta_i - \theta_{k}^{set} = 0
$$

$$
V_i^{ac} - V_{k,ac}^{set} = 0
$$

其中：

- $$\theta_i$$ 为 AC bus $$i$$ 的电压相角；
- $$V_i^{ac}$$ 为 AC bus $$i$$ 的电压幅值；
- $$\theta_k^{set}$$ 为 AC 侧构网角度参考；
- $$V_{k,ac}^{set}$$ 为 AC 侧构网电压幅值参考。

---

## 2.2 AC 侧构网与 AC_PV 的区别

AC_PV 控制为：

$$
P_{k,ac} = P_{k,ac}^{set}
$$

$$
V_i^{ac} = V_{k,ac}^{set}
$$

AC 侧构网控制为：

$$
\theta_i = \theta_k^{set}
$$

$$
V_i^{ac} = V_{k,ac}^{set}
$$

二者对比如下：

| 控制类型 | 控制 $$P_{ac}$$ | 控制 $$Q_{ac}$$ | 控制 $$V_{ac}$$ | 控制 $$\theta$$ | 是否 AC 构网 |
|---|---:|---:|---:|---:|---:|
| AC_PQ | 是 | 是 | 否 | 否 | 否 |
| AC_PV | 是 | 否 | 是 | 否 | 否 |
| AC_GRID_FORMING | 否 | 否或 droop | 是 | 是 | 是 |
| DC_V_AC_Q | 否 | 是 | 否 | 否 | DC 侧构网 |
| DC_V_AC_V | 否 | 否 | 是 | 否 | DC 侧构网 + AC 电压控制 |

因此必须明确：

```text
AC_PV 不等于 AC 侧构网；
AC 电压幅值控制不等于 AC 侧构网；
AC 侧构网必须提供相角或频率参考。
```

---

## 2.3 AC 侧构网的功率自由度

AC 侧构网时，AC/DC 通常不应再硬指定 AC 侧有功：

$$
P_{k,ac} - P_{k,ac}^{set} = 0
$$

也不应硬指定 DC 侧有功：

$$
P_{k,dc} - P_{k,dc}^{set} = 0
$$

因为 AC 侧构网设备承担 AC 岛功率不平衡，其有功注入应由：

1. AC 网络潮流；
2. DC 侧供能能力；
3. AC/DC 损耗方程；
4. 设备功率限值；
5. 能量支撑机制；

共同决定。

因此若：

$$
z_k^{ac,gfm} = 1
$$

则应满足：

$$
z_k^{P_{ac}} = 0
$$

$$
z_k^{P_{dc}} = 0
$$

若违反，应报：

```text
ACDC-GFM-04:
AC grid-forming AC/DC converter cannot also impose AC or DC active power as hard constraint.
```

---

## 2.4 AC 侧构网的无功自由度

AC 侧构网通常包含 AC 电压幅值控制：

$$
V_i^{ac} - V_{k,ac}^{set} = 0
$$

此时无功功率 $$Q_{k,ac}$$ 通常是未知量，用于支撑 AC 电压。

因此不应同时硬指定：

$$
Q_{k,ac} - Q_{k,ac}^{set} = 0
$$

若同时刚性控制 AC 电压和 AC 无功，则可能过约束。

对应规则：

```text
ACDC-GFM-07:
AC grid-forming AC/DC cannot rigidly control AC voltage and AC reactive power simultaneously.
```

如果采用 Q-V droop，则可使用：

$$
Q_{k,ac}
-
Q_k^0
-
K_k^{ac,V}
\left(
V_k^{ac,set} - V_i^{ac}
\right)
=
0
$$

其中：

$$
K_k^{ac,V} > 0
$$

此时不应再加入刚性电压控制方程，除非采用特殊的二次控制或 active-set 模式。

---

## 2.5 AC 侧构网必须定义角度或频率参考

AC 侧构网必须至少定义以下之一：

### 相角参考

$$
\theta_i - \theta_k^{set} = 0
$$

### 频率参考

$$
\omega_i - \omega_k^{set} = 0
$$

若稳态潮流中不显式建模频率，则必须通过相角参考体现 AC 构网能力。

若设备声明为 AC 侧构网，但没有角度或频率参考，应报：

```text
ACDC-GFM-09:
AC-side grid-forming mode requires AC angle or frequency reference definition.
```

---

## 2.6 AC 侧构网的 DC 侧能量支撑条件

AC/DC 在 AC 侧构网时，其 AC 侧有功输出必须来自 DC 侧或内部能量缓冲。

因此，若：

$$
z_k^{ac,gfm} = 1
$$

则该设备 DC 侧所属的 DC 功率耦合域必须存在至少一种能量支撑机制。

可定义支撑源集合：

$$
\mathcal{E}_{d}^{support}
=
\mathcal{V}_{d}^{rigid}
\cup
\mathcal{V}_{d}^{droop}
\cup
\mathcal{V}_{d}^{part}
\cup
\mathcal{S}_{d}^{storage}
\cup
\mathcal{G}_{d}^{external}
\cup
\mathcal{R}_{d}^{dcdc}
$$

要求：

$$
\left|
\mathcal{E}_{d}^{support}
\right|
\geq 1
$$

否则报：

```text
ACDC-GFM-05:
AC-side grid-forming AC/DC requires a DC-side energy source or voltage/power balancing mechanism.
```

---

# 3. AC/DC 变换器 AC 侧构网与 DC 侧构网互斥

---

## 3.1 基本互斥原则

普通两端口 AC/DC 变换器不能同时在 AC 侧和 DC 侧构网。

定义：

- $$z_k^{ac,gfm} = 1$$ 表示 AC 侧构网；
- $$z_k^{dc,gfm} = 1$$ 表示 DC 侧构网。

普通 AC/DC 应满足：

$$
z_k^{ac,gfm} + z_k^{dc,gfm} \leq 1
$$

若：

$$
z_k^{ac,gfm} = 1
$$

且：

$$
z_k^{dc,gfm} = 1
$$

则应报：

```text
ACDC-GFM-03:
Ordinary AC/DC converter cannot be grid-forming on both AC and DC sides simultaneously.
```

---

## 3.2 不能双侧构网的理论原因

普通 AC/DC 变换器满足功率守恒：

$$
P_{k,ac} + P_{k,dc} + P_{k,loss} = 0
$$

这说明 AC 侧与 DC 侧的有功功率不是独立自由度。

如果同时 AC 侧构网和 DC 侧构网，则可能同时强加：

$$
\theta_i - \theta_i^{set} = 0
$$

$$
V_i^{ac} - V_i^{ac,set} = 0
$$

$$
V_j^{dc} - V_j^{dc,set} = 0
$$

这会导致：

1. AC 侧功率不平衡要求设备调节 $$P_{k,ac}$$；
2. DC 侧电压控制要求设备调节 $$P_{k,dc}$$；
3. 但 $$P_{k,ac}$$ 与 $$P_{k,dc}$$ 又通过损耗方程耦合；
4. 若无内部储能或能量缓冲，则两侧不能独立吸收功率扰动；
5. 方程容易过约束；
6. 动态上 DC-link 能量不可闭合。

因此，普通 AC/DC 双侧构网不是普通潮流模型中的合法控制模式。

---

## 3.3 双侧构网的高级模式例外

若必须支持 AC/DC 双侧构网，则必须进入高级模式，并显式声明：

1. 内部储能；
2. DC-link 能量状态；
3. AC 侧频率或相角控制；
4. DC 侧电压控制；
5. AC/DC 有功不平衡由谁承担；
6. 能量状态动态方程；
7. 触限后的优先级；
8. 储能 SOC 或能量上下限；
9. 功率限值；
10. 是否允许短时能量偏差。

能量状态可写为：

$$
\frac{dE_k}{dt}
=
-
\left(
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
\right)
$$

若稳态要求：

$$
\frac{dE_k}{dt} = 0
$$

则仍有：

$$
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

如果允许短时能量缓冲，则必须满足：

$$
E_k^{min} \leq E_k \leq E_k^{max}
$$

否则应报：

```text
ACDC-GFM-02:
AC/DC dual-side grid-forming requires explicit energy buffer or balancing mechanism.
```

---

# 4. VSC 七类典型控制模式映射

---

## 4.1 七类模式概述

常见 VSC 控制模式可整理为：

| 模式 | d-axis 控制 | q-axis 控制 | 典型应用 |
|---|---|---|---|
| Mode 1 | Constant $$\delta_s$$ | Constant $$V_s$$ | Virtual synchronous generator / Synchronous generator emulation |
| Mode 2 | Constant $$P_s$$ | Constant $$V_s$$ | Grid-connected inverter / PV and wind power systems |
| Mode 3 | Constant $$P_s$$ | Constant $$Q_s$$ | Grid-connected / PQ point control |
| Mode 4 | Constant $$U_{dc}$$ | Constant $$Q_s$$ | DC-AC interface |
| Mode 5 | Constant $$U_{dc}$$ | Constant $$V_s$$ | Islanded microgrid / Black start |
| Mode 6 | Droop $$U_{dc}$$ | Constant $$V_s$$ | Multi-VSC parallel system |
| Mode 7 | Droop $$U_{dc}$$ | Constant $$Q_s$$ | Multi-VSC parallel / Reactive power control |

---

## 4.2 Mode 1：Constant $$\delta_s$$ + Constant $$V_s$$

### 控制含义

该模式控制：

$$
\theta_i - \theta_i^{set} = 0
$$

$$
V_i^{ac} - V_i^{ac,set} = 0
$$

其中 $$\delta_s$$ 可理解为 AC 侧电压相角或功角参考。

### 对应统一模型

```text
AC_GRID_FORMING
```

或在存在 droop 时：

```text
AC_GRID_FORMING_DROOP
```

### 方程

$$
F_{\theta,k}^{acgfm}
=
\theta_i - \theta_k^{set}
=
0
$$

$$
F_{V,k}^{acgfm}
=
V_i^{ac} - V_k^{ac,set}
=
0
$$

$$
F_{loss,k}^{acdc}
=
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

### 覆盖情况

原模型若只支持 DC 侧构网，则只能部分覆盖。

补充 AC 侧构网后可以覆盖其稳态代数形式。

### 限制

若启用 Mode 1，则普通 AC/DC 不得同时 DC 侧构网：

$$
z_k^{ac,gfm} + z_k^{dc,gfm} \leq 1
$$

---

## 4.3 Mode 2：Constant $$P_s$$ + Constant $$V_s$$

### 控制含义

该模式控制：

$$
P_{k,ac} - P_{k,ac}^{set} = 0
$$

$$
V_i^{ac} - V_{k,ac}^{set} = 0
$$

属于 AC 侧 PV 控制。

### 对应统一模型

```text
AC_PV
```

### 方程

$$
P_{k,ac} - P_{k,ac}^{set} = 0
$$

$$
V_i^{ac} - V_{k,ac}^{set} = 0
$$

$$
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

### 覆盖情况

支持。

### 注意事项

AC_PV 不提供角度参考。所在 AC 岛仍需外部 slack 或 AC 构网源。

---

## 4.4 Mode 3：Constant $$P_s$$ + Constant $$Q_s$$

### 控制含义

该模式控制：

$$
P_{k,ac} - P_{k,ac}^{set} = 0
$$

$$
Q_{k,ac} - Q_{k,ac}^{set} = 0
$$

属于 AC 侧 PQ 控制。

### 对应统一模型

```text
AC_PQ
```

### 方程

$$
P_{k,ac} - P_{k,ac}^{set} = 0
$$

$$
Q_{k,ac} - Q_{k,ac}^{set} = 0
$$

$$
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

### 覆盖情况

支持。

---

## 4.5 Mode 4：Constant $$U_{dc}$$ + Constant $$Q_s$$

### 控制含义

该模式控制：

$$
V_j^{dc} - V_{k,dc}^{set} = 0
$$

$$
Q_{k,ac} - Q_{k,ac}^{set} = 0
$$

属于 DC 侧刚性构网 + AC 侧无功控制。

### 对应统一模型

```text
DC_V_GRID_FORMING_AC_Q
```

### 方程

$$
V_j^{dc} - V_{k,dc}^{set} = 0
$$

$$
Q_{k,ac} - Q_{k,ac}^{set} = 0
$$

$$
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

### 覆盖情况

支持。

### 限制

此时不得同时硬指定 AC 有功或 DC 有功：

$$
z_k^{dc,gfm} + z_k^{P_{ac}} \leq 1
$$

$$
z_k^{dc,gfm} + z_k^{P_{dc}} \leq 1
$$

若违反：

```text
ACDC-GFM-01:
DC grid-forming AC/DC converter cannot also impose AC or DC active power as hard constraint.
```

---

## 4.6 Mode 5：Constant $$U_{dc}$$ + Constant $$V_s$$

### 控制含义

该模式控制：

$$
V_j^{dc} - V_{k,dc}^{set} = 0
$$

$$
V_i^{ac} - V_{k,ac}^{set} = 0
$$

属于 DC 侧刚性构网 + AC 侧电压幅值控制。

### 对应统一模型

```text
DC_V_GRID_FORMING_AC_V
```

### 方程

$$
V_j^{dc} - V_{k,dc}^{set} = 0
$$

$$
V_i^{ac} - V_{k,ac}^{set} = 0
$$

$$
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

### 覆盖情况

支持，但需要澄清语义。

### 关键边界

Constant $$U_{dc}$$ + Constant $$V_s$$ 不自动等价于 AC/DC 双侧构网。

因为 AC 侧构网还要求控制：

$$
\theta_i
$$

或：

$$
\omega_i
$$

若 Mode 5 在工程语义中用于黑启动或孤岛微网，并隐含 AC 频率/相角形成能力，则它已经接近 AC/DC 双侧构网。此时普通 AC/DC 不应默认允许，除非显式建模能量缓冲和高级控制。

---

## 4.7 Mode 6：Droop $$U_{dc}$$ + Constant $$V_s$$

### 控制含义

该模式采用 DC 电压下垂：

$$
P_{k,dc}
-
P_k^0
-
K_k^{dc}
\left(
V_{k,dc}^{set}
-
V_j^{dc}
\right)
=
0
$$

并控制 AC 侧电压幅值：

$$
V_i^{ac} - V_{k,ac}^{set} = 0
$$

### 对应统一模型

```text
DC_V_DROOP_AC_V
```

### 方程

$$
P_{k,dc}
-
P_k^0
-
K_k^{dc}
\left(
V_{k,dc}^{set}
-
V_j^{dc}
\right)
=
0
$$

$$
V_i^{ac} - V_{k,ac}^{set} = 0
$$

$$
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

### 覆盖情况

支持。

### 多源协调要求

若 DC 岛存在多个 droop 源，应满足：

$$
\sum_k K_k^{dc} > 0
$$

否则：

```text
DCISLAND-DROOP-01:
DC droop sources exist but total droop gain is zero.
```

---

## 4.8 Mode 7：Droop $$U_{dc}$$ + Constant $$Q_s$$

### 控制含义

该模式采用 DC 电压下垂：

$$
P_{k,dc}
-
P_k^0
-
K_k^{dc}
\left(
V_{k,dc}^{set}
-
V_j^{dc}
\right)
=
0
$$

并控制 AC 无功：

$$
Q_{k,ac} - Q_{k,ac}^{set} = 0
$$

### 对应统一模型

```text
DC_V_DROOP_AC_Q
```

### 方程

$$
P_{k,dc}
-
P_k^0
-
K_k^{dc}
\left(
V_{k,dc}^{set}
-
V_j^{dc}
\right)
=
0
$$

$$
Q_{k,ac} - Q_{k,ac}^{set} = 0
$$

$$
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

### 覆盖情况

支持。

---

# 5. VSC 七类模式覆盖性总结

---

## 5.1 稳态潮流层面的覆盖情况

| 模式 | 控制方式 | 统一模型模式 | 覆盖情况 |
|---|---|---|---|
| Mode 1 | Constant $$\delta_s$$ + Constant $$V_s$$ | `AC_GRID_FORMING` | 补充 AC 侧构网后支持 |
| Mode 2 | Constant $$P_s$$ + Constant $$V_s$$ | `AC_PV` | 支持 |
| Mode 3 | Constant $$P_s$$ + Constant $$Q_s$$ | `AC_PQ` | 支持 |
| Mode 4 | Constant $$U_{dc}$$ + Constant $$Q_s$$ | `DC_V_GRID_FORMING_AC_Q` | 支持 |
| Mode 5 | Constant $$U_{dc}$$ + Constant $$V_s$$ | `DC_V_GRID_FORMING_AC_V` | 支持，但不能自动解释为 AC 构网 |
| Mode 6 | Droop $$U_{dc}$$ + Constant $$V_s$$ | `DC_V_DROOP_AC_V` | 支持 |
| Mode 7 | Droop $$U_{dc}$$ + Constant $$Q_s$$ | `DC_V_DROOP_AC_Q` | 支持 |

---

## 5.2 动态控制层面的覆盖边界

当前统一潮流模型主要覆盖稳态代数约束。

以下内容不属于普通潮流模型直接覆盖范围：

1. 虚拟同步机惯量；
2. 阻尼控制；
3. PLL 动态；
4. 电流内环动态；
5. DC-link 电容动态；
6. 黑启动过程；
7. 频率动态；
8. 故障穿越；
9. 开关级调制过程；
10. 保护动作序列。

因此应明确：

```text
统一 AC/DC 潮流模型可以覆盖 VSC 七类模式的稳态代数形式；
若需覆盖完整控制动态，应扩展为动态仿真模型或微分-代数方程模型。
```

---

# 6. 推荐控制模式枚举

---

## 6.1 AC/DC 控制模式建议

建议将 AC/DC 控制模式扩展为：

```cpp
enum class ACDCControlMode {
    AC_PQ,
    AC_PV,

    DC_P_AC_Q,
    DC_P_AC_V,

    DC_V_GRID_FORMING_AC_Q,
    DC_V_GRID_FORMING_AC_V,

    DC_V_DROOP_AC_Q,
    DC_V_DROOP_AC_V,

    AC_GRID_FORMING_DC_FREE,
    AC_GRID_FORMING_DC_P,
    AC_GRID_FORMING_DROOP,

    GRID_FORMING_AC_DC_ADVANCED
};
```

其中：

- `AC_PQ` 对应 Mode 3；
- `AC_PV` 对应 Mode 2；
- `DC_V_GRID_FORMING_AC_Q` 对应 Mode 4；
- `DC_V_GRID_FORMING_AC_V` 对应 Mode 5；
- `DC_V_DROOP_AC_V` 对应 Mode 6；
- `DC_V_DROOP_AC_Q` 对应 Mode 7；
- `AC_GRID_FORMING_DC_FREE` 或 `AC_GRID_FORMING_DROOP` 对应 Mode 1；
- `GRID_FORMING_AC_DC_ADVANCED` 仅用于显式能量缓冲的高级双侧构网。

---

## 6.2 构网侧枚举

建议增加：

```cpp
enum class ACDCGridFormingSide {
    NONE,
    AC_SIDE,
    DC_SIDE,
    DUAL_SIDE_ADVANCED
};
```

普通工程模式下：

```text
AC_SIDE 与 DC_SIDE 互斥；
DUAL_SIDE_ADVANCED 需要显式能量缓冲。
```

---

# 7. DeviceControlRole 补充

建议将 AC/DC 控制解析为统一角色。

```cpp
struct DeviceControlRole {
    bool controls_ac_p = false;
    bool controls_ac_q = false;
    bool controls_ac_v = false;
    bool controls_ac_angle = false;
    bool controls_ac_frequency = false;

    bool controls_dc_p = false;
    bool controls_dc_v_rigid = false;
    bool controls_dc_v_droop = false;

    bool is_ac_grid_forming = false;
    bool is_dc_grid_forming = false;

    bool provides_ac_angle_reference = false;
    bool provides_ac_voltage_reference = false;
    bool provides_dc_v_reference = false;

    bool ac_p_is_free = false;
    bool ac_q_is_free = false;
    bool dc_p_is_free = false;

    bool participates_ac_active_balance = false;
    bool participates_ac_reactive_balance = false;
    bool participates_dc_active_balance = false;

    bool participates_active_balance = false;
    double participation_factor = 0.0;

    double fixed_p_mw = 0.0;
    double fixed_q_mvar = 0.0;

    double flex_up_mw = 0.0;
    double flex_down_mw = 0.0;

    bool has_energy_buffer = false;
    bool allow_dual_side_grid_forming = false;

    std::string coordination_group_id;
};
```

核心互斥检查：

```cpp
if (role.is_ac_grid_forming && role.is_dc_grid_forming) {
    if (!role.allow_dual_side_grid_forming || !role.has_energy_buffer) {
        fatal("ACDC-GFM-03",
              "Ordinary AC/DC converter cannot be grid-forming on both AC and DC sides simultaneously.");
    }
}
```

---

# 8. 数据结构补充建议

---

## 8.1 AC/DC 控制数据结构

建议 AC/DC 变换器控制数据结构扩展为：

```json
{
  "type": "ACDC",
  "index": 0,
  "bus_ac": 1,
  "bus_dc": 1,

  "control": {
    "grid_forming": {
      "ac_side": {
        "enabled": false,
        "type": "RIGID",
        "v_ac_set_pu": null,
        "theta_set_rad": null,
        "frequency_set_hz": null,
        "p_f_droop_enabled": false,
        "p_f_droop_k_mw_hz": null,
        "q_v_droop_enabled": false,
        "q_v_droop_k_mvar_pu": null
      },
      "dc_side": {
        "enabled": false,
        "type": "RIGID",
        "v_dc_set_pu": null,
        "droop_enabled": false,
        "droop_k_mw_pu": null
      },
      "allow_dual_side_grid_forming": false,
      "energy_buffer_required": true
    },

    "active_power": {
      "ac_p_mode": "FREE",
      "dc_p_mode": "FREE",
      "p_set_mw": null,
      "p_schedule_mw": 0.0,
      "p_initial_mw": 0.0,
      "p_is_hard_constraint": false
    },

    "reactive_power": {
      "ac_q_mode": "FREE",
      "q_set_mvar": null,
      "pf_set": null
    },

    "ac_voltage_control": {
      "enabled": false,
      "v_set_pu": null,
      "droop_k_mvar_pu": null
    }
  },

  "physical": {
    "topology": "TWO_LEVEL_VSC",
    "s_rated_mva": 0.1,
    "p_min_mw": -0.1,
    "p_max_mw": 0.1,
    "q_min_mvar": -0.05,
    "q_max_mvar": 0.05,
    "m_min": 0.0,
    "m_max": 1.0,
    "modulation_model": {
      "type": "linear_gain",
      "k_m": 0.612,
      "voltage_definition": "AC_LL_RMS_TO_DC_POLE_TO_POLE"
    },
    "i_ac_max_ka": null,
    "i_dc_max_ka": null,
    "loss_model": {
      "type": "constant_efficiency",
      "eta_ac2dc": 0.98,
      "eta_dc2ac": 0.98
    },
    "energy_buffer": {
      "enabled": false,
      "e_min_mj": null,
      "e_max_mj": null,
      "soc_min": null,
      "soc_max": null,
      "p_charge_max_mw": null,
      "p_discharge_max_mw": null
    }
  },

  "initial": {
    "p_ac_mw": 0.0,
    "q_ac_mvar": 0.0,
    "p_dc_mw": 0.0,
    "m": 0.8
  }
}
```

---

## 8.2 AC_PV 模式数据示例

```json
{
  "type": "ACDC",
  "index": 1,
  "bus_ac": 10,
  "bus_dc": 3,
  "control": {
    "grid_forming": {
      "ac_side": {
        "enabled": false
      },
      "dc_side": {
        "enabled": false
      }
    },
    "active_power": {
      "ac_p_mode": "P_SET",
      "p_set_mw": 1.0,
      "p_is_hard_constraint": true
    },
    "reactive_power": {
      "ac_q_mode": "FREE",
      "q_set_mvar": null
    },
    "ac_voltage_control": {
      "enabled": true,
      "v_set_pu": 1.0
    }
  }
}
```

该配置表示：

```text
AC 侧有功固定；
AC 侧电压固定；
AC 无功释放；
不提供 AC 角度参考；
不提供 DC 电压参考。
```

---

## 8.3 AC 侧构网模式数据示例

```json
{
  "type": "ACDC",
  "index": 2,
  "bus_ac": 20,
  "bus_dc": 5,
  "control": {
    "grid_forming": {
      "ac_side": {
        "enabled": true,
        "type": "RIGID",
        "v_ac_set_pu": 1.0,
        "theta_set_rad": 0.0,
        "frequency_set_hz": 50.0
      },
      "dc_side": {
        "enabled": false
      },
      "allow_dual_side_grid_forming": false
    },
    "active_power": {
      "ac_p_mode": "FREE",
      "dc_p_mode": "FREE",
      "p_set_mw": null,
      "p_schedule_mw": 0.0,
      "p_initial_mw": 0.0,
      "p_is_hard_constraint": false
    },
    "reactive_power": {
      "ac_q_mode": "FREE",
      "q_set_mvar": null
    }
  }
}
```

该配置表示：

```text
AC 侧构网；
控制 AC 电压幅值和相角；
AC 有功、无功均由网络平衡决定；
DC 侧必须有能量支撑；
不允许同时 DC 侧构网。
```

---

# 9. 方程生成器补充

---

## 9.1 AC/DC 方程生成逻辑

建议方程生成器按照以下逻辑处理 AC/DC 变换器。

```cpp
// 1. AC/DC 双侧构网互斥检查
if (role.is_ac_grid_forming && role.is_dc_grid_forming) {
    if (!role.allow_dual_side_grid_forming || !role.has_energy_buffer) {
        fatal("ACDC-GFM-03",
              "Ordinary AC/DC converter cannot be grid-forming on both AC and DC sides simultaneously.");
    }
}

// 2. AC 侧构网方程
if (role.is_ac_grid_forming) {
    if (!role.controls_ac_angle && !role.controls_ac_frequency) {
        fatal("ACDC-GFM-09",
              "AC-side grid-forming mode requires AC angle or frequency reference definition.");
    }

    add_equation(Theta(bus_ac) - theta_ac_set);
    add_equation(Vac(bus_ac) - vac_set);

    if (role.controls_ac_p || role.controls_dc_p) {
        fatal("ACDC-GFM-04",
              "AC grid-forming AC/DC converter cannot impose AC or DC active power as hard constraint.");
    }

    if (role.controls_ac_q) {
        fatal("ACDC-GFM-07",
              "AC grid-forming AC/DC converter cannot rigidly control AC voltage and AC reactive power simultaneously.");
    }
}

// 3. DC 侧刚性构网方程
if (role.controls_dc_v_rigid) {
    add_equation(Vdc(bus_dc) - vdc_set);

    if (role.controls_ac_p || role.controls_dc_p) {
        fatal("ACDC-GFM-01",
              "DC grid-forming AC/DC converter cannot impose AC or DC active power as hard constraint.");
    }
}

// 4. DC 侧 droop 方程
if (role.controls_dc_v_droop) {
    add_equation(Pdc(conv) - P0 - k_dc * (vdc_set - Vdc(bus_dc)));
}

// 5. AC_PV 方程
if (role.controls_ac_p && role.controls_ac_v && !role.is_ac_grid_forming) {
    add_equation(Pac(conv) - pac_set);
    add_equation(Vac(bus_ac) - vac_set);

    if (role.controls_ac_q) {
        fatal("ACDC-CTRL-04",
              "AC_PV mode cannot simultaneously impose AC reactive power as hard constraint.");
    }
}

// 6. AC_PQ 方程
if (role.controls_ac_p && role.controls_ac_q && !role.controls_ac_v) {
    add_equation(Pac(conv) - pac_set);
    add_equation(Qac(conv) - q_set);
}

// 7. 固定 AC 无功
if (role.controls_ac_q && !role.is_ac_grid_forming) {
    add_equation(Qac(conv) - q_set);
}

// 8. 非构网 AC 电压控制
if (role.controls_ac_v && !role.is_ac_grid_forming) {
    add_equation(Vac(bus_ac) - vac_set);
}

// 9. 固定 DC 有功
if (role.controls_dc_p) {
    add_equation(Pdc(conv) - pdc_set);
}

// 10. AC/DC 功率守恒
add_equation(Pac(conv) + Pdc(conv) + Ploss(conv));
```

---

# 10. 系统级校核补充

---

## 10.1 AC_PV 设备的 AC 岛校核

AC_PV 设备不能提供 AC 岛角度参考。

因此在 AC 岛校核中：

```cpp
if (device.mode == AC_PV) {
    island.has_voltage_control = true;
    island.has_angle_reference = island.has_angle_reference || false;
}
```

如果某 AC 岛只有 AC_PV 设备而没有 AC slack 或 AC grid-forming 源，则报：

```text
ACISLAND-REF-01:
AC island has no angle reference.
```

---

## 10.2 AC 侧构网设备的 AC 岛校核

若 AC/DC 是 AC 侧构网：

```cpp
if (role.is_ac_grid_forming) {
    island.has_angle_reference = true;
    island.has_voltage_reference = true;
    island.has_active_power_balancing_source = true;
    island.has_reactive_power_support = true;
}
```

若同一 AC 岛中存在多个刚性 AC 角度参考，且无协调机制，则报：

```text
ACISLAND-REF-02:
Multiple rigid AC angle references without coordination.
```

若多个 AC 侧构网源采用 P-f droop 或虚拟同步控制，则必须显式给出协调方程。

---

## 10.3 AC 侧构网的 DC 侧支撑校核

若 AC/DC 在 AC 侧构网，应检查其 DC 侧支撑。

```cpp
if (role.is_ac_grid_forming) {
    auto dc_domain = find_dc_power_coupling_domain(conv.bus_dc);

    if (!dc_domain.has_voltage_reference &&
        !dc_domain.has_droop_source &&
        !dc_domain.has_external_grid &&
        !dc_domain.has_storage &&
        !dc_domain.has_dispatchable_source &&
        !dc_domain.has_import_path_from_other_supported_domain &&
        !role.has_energy_buffer) {

        fatal("ACDC-GFM-05",
              "AC-side grid-forming AC/DC requires a DC-side energy source or voltage/power balancing mechanism.");
    }
}
```

---

## 10.4 DC 侧构网的 AC 侧支撑校核

若 AC/DC 在 DC 侧构网，应检查其 AC 侧有功支撑。

```cpp
if (role.is_dc_grid_forming) {
    auto ac_island = find_ac_island(conv.bus_ac);

    if (!ac_island.has_angle_reference &&
        !ac_island.has_slack &&
        !ac_island.has_grid_forming_source &&
        !ac_island.has_dispatchable_generator &&
        !ac_island.has_active_power_balancing_source &&
        !role.has_energy_buffer) {

        fatal("ACDC-GFM-06",
              "DC-side grid-forming AC/DC requires an AC-side energy source or active-power balancing mechanism.");
    }
}
```

对应规则：

```text
ACDC-GFM-06:
DC-side grid-forming AC/DC requires an AC-side energy source or active-power balancing mechanism.
```

---

# 11. 新增规则编号

---

## 11.1 AC/DC 控制规则

```text
ACDC-CTRL-03:
AC_PV mode requires AC active-power setpoint and AC voltage setpoint, with AC reactive power released as unknown.

ACDC-CTRL-04:
AC_PV mode cannot simultaneously impose AC reactive power as hard constraint.

ACDC-CTRL-05:
AC_PV mode does not provide AC angle reference.
```

---

## 11.2 AC/DC 构网规则

```text
ACDC-GFM-03:
Ordinary AC/DC converter cannot be grid-forming on both AC and DC sides simultaneously.

ACDC-GFM-04:
AC grid-forming AC/DC converter cannot also impose AC or DC active power as hard constraint.

ACDC-GFM-05:
AC-side grid-forming AC/DC requires a DC-side energy source or voltage/power balancing mechanism.

ACDC-GFM-06:
DC-side grid-forming AC/DC requires an AC-side energy source or active-power balancing mechanism.

ACDC-GFM-07:
AC grid-forming AC/DC cannot rigidly control AC voltage and AC reactive power simultaneously.

ACDC-GFM-08:
Multiple AC-side grid-forming AC/DC converters require explicit synchronization or droop coordination.

ACDC-GFM-09:
AC-side grid-forming mode requires AC angle or frequency reference definition.

ACDC-GFM-10:
AC-side grid-forming mode cannot be reduced to ordinary PV control.
```

---

# 12. 推荐校核流程补充

建议将 AC/DC 控制相关校核流程扩展为：

```text
1. 读取系统数据。
2. 统一单位与基准。
3. 构建 AC 导电岛。
4. 构建 DC 导电岛。
5. 构建 DC/DC 功率耦合图。
6. 解析 AC/DC 控制模式。
7. 判断 AC/DC 是否为 AC_PQ、AC_PV、DC_V_AC_Q、DC_V_AC_V、DC_DROOP、AC_GFM 等角色。
8. 解析 DeviceControlRole。
9. 检查 AC/DC 是否同时 AC 侧与 DC 侧构网。
10. 若双侧构网，检查是否显式建模能量缓冲和高级协调机制。
11. 检查 AC_PV 是否错误地同时指定 Q。
12. 检查 AC_PV 是否被错误用作 AC 角度参考。
13. 检查 AC 侧构网是否定义角度或频率参考。
14. 检查 AC 侧构网是否硬指定 AC/DC 有功。
15. 检查 AC 侧构网是否硬指定 AC 无功。
16. 检查 DC 侧构网是否硬指定 AC/DC 有功。
17. 检查 AC 侧构网设备的 DC 侧能量支撑。
18. 检查 DC 侧构网设备的 AC 侧有功支撑。
19. 执行 AC 岛角度参考和平衡能力校核。
20. 执行 DC 岛电压参考和平衡能力校核。
21. 注册潮流变量。
22. 生成控制方程、网络方程、损耗方程和物理方程。
23. 检查方程数与变量数。
24. 检查结构秩。
25. 求解统一潮流或分块潮流。
26. 检查限值。
27. 若触限，执行模式切换。
28. 输出报告。
```

---

# 13. 工程实现优先级建议

---

## 13.1 短期必须实现

1. 支持 `AC_PV`；
2. 明确 `AC_PV` 不提供 AC 角度参考；
3. 增加 `AC_GRID_FORMING` 语义；
4. 增加 AC 侧构网与 DC 侧构网互斥规则；
5. 增加 AC 侧构网角度或频率参考校核；
6. 增加 AC 侧构网的 DC 侧能量支撑校核；
7. 增加 DC 侧构网的 AC 侧有功支撑校核；
8. 将 VSC 七类模式映射到统一控制角色。

---

## 13.2 中期建议实现

1. 多 AC 构网源协调校核；
2. AC 侧 P-f droop；
3. AC 侧 Q-V droop；
4. DC 侧 droop 多源分担；
5. AC/DC 限值触发后的模式切换；
6. AC_PV 触 Q 限后切换为 AC_PQ；
7. AC 构网源触 P/Q 限后的控制优先级处理。

---

## 13.3 长期建议实现

1. AC/DC 双侧构网高级模式；
2. DC-link 能量动态；
3. 内部储能 SOC；
4. VSG 动态；
5. 虚拟惯量和阻尼；
6. 动态频率变量；
7. 微分-代数方程模型；
8. 黑启动过程仿真；
9. 故障穿越与保护逻辑；
10. OPF 中的构网约束建模。

---

# 14. 最终结论

---

## 14.1 AC_PV 控制已经可以支持

AC/DC 变换器在 AC 侧为 PV 控制时，可通过以下方程支持：

$$
P_{k,ac} - P_{k,ac}^{set} = 0
$$

$$
V_i^{ac} - V_{k,ac}^{set} = 0
$$

$$
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

其中：

$$
Q_{k,ac}
$$

为未知量，由 AC 无功平衡决定。

但 AC_PV 不提供 AC 角度参考。

---

## 14.2 VSC 七类模式可以覆盖其稳态形式

七类 VSC 模式在统一框架下的对应关系为：

| 模式 | 对应统一模型 |
|---|---|
| Constant $$\delta_s$$ + Constant $$V_s$$ | `AC_GRID_FORMING` |
| Constant $$P_s$$ + Constant $$V_s$$ | `AC_PV` |
| Constant $$P_s$$ + Constant $$Q_s$$ | `AC_PQ` |
| Constant $$U_{dc}$$ + Constant $$Q_s$$ | `DC_V_GRID_FORMING_AC_Q` |
| Constant $$U_{dc}$$ + Constant $$V_s$$ | `DC_V_GRID_FORMING_AC_V` |
| Droop $$U_{dc}$$ + Constant $$V_s$$ | `DC_V_DROOP_AC_V` |
| Droop $$U_{dc}$$ + Constant $$Q_s$$ | `DC_V_DROOP_AC_Q` |

---

## 14.3 普通 AC/DC 不能同时 AC 侧和 DC 侧构网

普通两端口 AC/DC 变换器必须满足：

$$
z_k^{ac,gfm} + z_k^{dc,gfm} \leq 1
$$

若同时启用 AC 侧构网和 DC 侧构网，则应报：

```text
ACDC-GFM-03:
Ordinary AC/DC converter cannot be grid-forming on both AC and DC sides simultaneously.
```

除非显式建模内部储能、DC-link 能量动态和高级协调控制。

---

## 14.4 一句话总结

**AC/DC 变换器可以支持 AC_PV、AC_PQ、DC 侧构网、DC 侧下垂和 AC 侧构网等多种 VSC 控制模式；其中 AC_PV 不是 AC 侧构网，AC 侧构网必须提供相角或频率参考。普通 AC/DC 只能选择 AC 侧构网或 DC 侧构网之一，不能双侧同时构网；VSC 七类典型控制模式可以在统一设备方程框架下覆盖其稳态代数形式，但动态意义上的 VSG、黑启动和双侧构网必须进一步引入能量状态和动态控制模型。**
