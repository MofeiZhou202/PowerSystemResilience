# 混合 AC/DC 多变换器控制校核与统一潮流求解系统  
# 理论文档与工程实践建议

---

## 0. 文档目的

本文档面向混合 AC/DC 电力系统、直流微电网、柔性直流配电网、多变换器耦合系统的建模、控制校核与潮流求解器重构。

目标是建立一套严谨、可实现、可扩展的统一框架，用于支持：

1. 多个 DC 电压源并存；
2. AC/DC 变换器 DC 侧构网；
3. DC/DC 端口电压控制；
4. AC/DC、DC/DC 设备物理约束；
5. Q 节点、未知 P 注入节点、droop 节点等扩展潮流对象；
6. 控制模式合法性校核；
7. 系统级方程闭合性校核；
8. 设备级可行性校核；
9. 统一 AC/DC 潮流迭代；
10. 后续高级控制、储能、PV、能量路由器、多端口装置扩展。

核心思想是：

> 不再把变换器简单映射为传统 PQ/PV/Slack 节点，而是将其建模为带控制方程、物理约束和端口注入的设备，由统一潮流方程系统求解。

---

# 1. 总体建模原则

## 1.1 分层架构

建议将系统分为五层，而不是简单的规则检查器。

```text
设备物理层
    ↓
设备控制层
    ↓
网络拓扑与电气岛层
    ↓
方程生成与结构校核层
    ↓
统一潮流求解层
```

各层职责如下。

| 层级 | 主要职责 |
|---|---|
| 设备物理层 | 描述 AC/DC、DC/DC、储能、PV、负荷、外部电网的额定容量、电流、电压、调制比、占空比、效率、损耗、功率方向等约束 |
| 设备控制层 | 描述设备控制目标，例如 P 控制、Q 控制、Vdc 控制、Vac 控制、下垂控制、主从控制、参与因子控制、构网控制 |
| 网络拓扑与电气岛层 | 构建 AC 岛、DC 导电岛、DC 电压控制岛、变换器耦合图 |
| 方程生成与结构校核层 | 动态注册未知量，动态生成网络方程、设备方程、控制方程，执行方程数量、结构秩、自由度闭合校核 |
| 统一潮流求解层 | 求解 AC 潮流、DC 潮流、变换器耦合、损耗方程、下垂方程和限值切换 |

---

## 1.2 核心原则

### 原则 1：节点状态与设备控制分离

节点只应描述网络状态变量，例如：

- AC 节点电压幅值；
- AC 节点相角；
- DC 节点电压；
- 节点注入功率汇总。

设备控制不应强行写入 bus type。

也就是说：

```text
PQ/PV/Slack 不应作为所有控制行为的唯一抽象。
```

变换器、储能、PV、STATCOM、负荷等设备应通过设备方程向节点注入功率。

---

### 原则 2：控制自由度与物理可行性分离

控制模式合法不代表设备物理可行。

例如：

- AC/DC 在 DC_V_AC_Q 模式下控制逻辑可能合法；
- 但如果 DC 电压太低，调制比超限，则物理不可行。

因此必须分两类校核：

```text
控制自由度校核
设备物理可行性校核
```

---

### 原则 3：等式方程与不等式限值分离

统一潮流方程应写成：

$$
F(x, m) = 0
$$

其中 $$x$$ 是连续未知量，$$m$$ 是当前离散运行模式。

设备限值应写成：

$$
g(x, m) \leq 0
$$

不应把所有物理限值都塞入等式方程。

例如：

- 损耗方程是等式；
- Buck 电压变比可以是等式；
- 占空比上下限是不等式；
- 电流限值是不等式；
- 容量圆是不等式。

---

### 原则 4：所有自由注入必须有闭合方程

如果某设备的有功或无功作为未知量进入潮流，则必须存在额外方程决定它。

例如 Q_NODE：

```text
Q 固定，P 未知。
```

则 P 必须由 DC 侧平衡、损耗方程、droop 方程、参与因子方程或其他设备方程决定。

否则系统欠定。

---

### 原则 5：多电压源并存不是错误，但必须有数学闭合机制

DC 岛允许多个电压形成源，但必须明确其协调模型：

- 单主源；
- 多刚性电压源；
- 下垂控制；
- 主从控制；
- 参与因子控制；
- 二次控制；
- 能量管理层调度。

不能简单规定：

```text
一个 DC 岛只能有一个 V 节点。
```

但也不能简单允许：

```text
多个 DC_V 节点无条件合法。
```

---

# 2. 符号、方向与单位约定

这是工程实现中最容易出错的部分，必须首先统一。

---

## 2.1 端口功率正方向

本文建议统一采用：

```text
端口功率 P_port > 0 表示设备向所在网络注入有功功率。
端口功率 P_port < 0 表示设备从所在网络吸收有功功率。
```

例如 AC/DC 变换器连接 AC bus 和 DC bus：

- $$P_{ac} > 0$$ 表示 AC/DC 向 AC 网络注入有功；
- $$P_{dc} > 0$$ 表示 AC/DC 向 DC 网络注入有功；
- 损耗 $$P_{loss} \geq 0$$。

功率守恒为：

$$
P_{ac} + P_{dc} + P_{loss} = 0
$$

这意味着如果变换器从 AC 侧吸收功率、向 DC 侧送功率，则：

$$
P_{ac} < 0
$$

$$
P_{dc} > 0
$$

并且：

$$
|P_{ac}| = P_{dc} + P_{loss}
$$

---

## 2.2 DC/DC 端口功率方向

对于 DC/DC，定义：

- $$P_{in}^{port}$$：输入端口向其所在 DC 网络注入功率；
- $$P_{out}^{port}$$：输出端口向其所在 DC 网络注入功率。

统一损耗方程：

$$
P_{in}^{port} + P_{out}^{port} + P_{loss} = 0
$$

其中：

$$
P_{loss} \geq 0
$$

注意：

```text
这里的 in/out 是设备端口命名，不代表功率方向一定从 in 到 out。
```

如果设备正向从输入端吸收功率、向输出端注入功率，则：

$$
P_{in}^{port} < 0
$$

$$
P_{out}^{port} > 0
$$

---

## 2.3 效率模型与功率方向

如果使用恒定效率模型，必须避免反向运行时出现负损耗。

定义设备通过功率：

$$
P_{through} \geq 0
$$

正向运行时：

$$
P_{abs,in} = -P_{in}^{port}
$$

$$
P_{inj,out} = P_{out}^{port}
$$

恒定效率：

$$
P_{inj,out} = \eta P_{abs,in}
$$

损耗：

$$
P_{loss} = P_{abs,in} - P_{inj,out}
$$

即：

$$
P_{loss} = (1-\eta)P_{abs,in}
$$

反向运行时应重新定义：

$$
P_{abs,out} = -P_{out}^{port}
$$

$$
P_{inj,in} = P_{in}^{port}
$$

并有：

$$
P_{inj,in} = \eta_{rev} P_{abs,out}
$$

因此实现中应显式区分：

```text
FORWARD
REVERSE
BIDIRECTIONAL
```

禁止简单使用一个公式在所有方向上套用。

---

## 2.4 AC 电压单位

AC 侧电流公式如果使用三相线电压 RMS：

$$
I_{ac} =
\frac{\sqrt{P_{ac}^2 + Q_{ac}^2}}
{\sqrt{3}V_{ac,ll}}
$$

其中：

- $$P_{ac}$$ 使用 MW；
- $$Q_{ac}$$ 使用 Mvar；
- $$V_{ac,ll}$$ 使用 kV；
- 则 $$I_{ac}$$ 的单位为 kA。

工程公式为：

$$
I_{ka} =
\frac{S_{MVA}}
{\sqrt{3}V_{kV}}
$$

如果内部使用标幺值，则：

$$
I_{pu} =
\frac{S_{pu}}
{V_{pu}}
$$

必须避免 MW、kV、A 混用。

---

## 2.5 AC/DC 调制比基准

VSC 调制关系不应写死，应使用通用形式：

$$
V_{ac}^{model} = K_m m V_{dc}
$$

其中：

- $$m$$ 是调制比；
- $$K_m$$ 是与拓扑、PWM 方式、电压定义有关的系数；
- $$V_{ac}^{model}$$ 必须明确是相电压 RMS、线电压 RMS、基波峰值还是其他定义；
- $$V_{dc}$$ 必须明确是极间电压、极对地电压还是 pole-to-pole 电压。

调制比限值：

$$
m^{min} \leq m \leq m^{max}
$$

可行性要求：

$$
m =
\frac{V_{ac}^{model}}{K_m V_{dc}}
$$

并满足：

$$
m^{min}
\leq
\frac{V_{ac}^{model}}{K_m V_{dc}}
\leq
m^{max}
$$

---

# 3. 设备物理层

---

# 3.1 AC/DC 变换器物理模型

## 3.1.1 基本变量

对于 AC/DC 变换器 $$k$$，连接 AC bus $$i$$ 和 DC bus $$j$$。

变量包括：

| 变量 | 含义 |
|---|---|
| $$V_{i}^{ac}$$ | AC 侧电压幅值 |
| $$\theta_i$$ | AC 侧相角 |
| $$V_{j}^{dc}$$ | DC 侧电压 |
| $$P_{k,ac}$$ | AC 端口有功注入 |
| $$Q_{k,ac}$$ | AC 端口无功注入 |
| $$P_{k,dc}$$ | DC 端口有功注入 |
| $$P_{k,loss}$$ | 损耗 |
| $$m_k$$ | 调制比 |
| $$I_{k,ac}$$ | AC 侧电流 |
| $$I_{k,dc}$$ | DC 侧电流 |

---

## 3.1.2 功率守恒

统一端口注入方向下：

$$
P_{k,ac} + P_{k,dc} + P_{k,loss} = 0
$$

其中：

$$
P_{k,loss} \geq 0
$$

---

## 3.1.3 损耗模型

可支持多种模型。

### 恒定效率模型

当功率方向为 AC 到 DC：

$$
P_{k,ac} < 0
$$

$$
P_{k,dc} > 0
$$

则：

$$
P_{k,dc} = \eta_{ac2dc}(-P_{k,ac})
$$

损耗：

$$
P_{k,loss} = (1-\eta_{ac2dc})(-P_{k,ac})
$$

当功率方向为 DC 到 AC：

$$
P_{k,dc} < 0
$$

$$
P_{k,ac} > 0
$$

则：

$$
P_{k,ac} = \eta_{dc2ac}(-P_{k,dc})
$$

损耗：

$$
P_{k,loss} = (1-\eta_{dc2ac})(-P_{k,dc})
$$

### 二次损耗模型

也可使用：

$$
P_{loss} = a + b|I| + cI^2
$$

或：

$$
P_{loss} = a + b|P| + cP^2
$$

要求：

$$
P_{loss} \geq 0
$$

若损耗模型给出负损耗，应报错。

---

## 3.1.4 调制比约束

采用通用模型：

$$
V_{ac}^{model} = K_m m_k V_{dc}
$$

若该关系作为潮流等式使用，则加入：

$$
F_{mod,k} =
V_{ac}^{model} - K_m m_k V_{dc}
= 0
$$

调制比不等式：

$$
m_k^{min} \leq m_k \leq m_k^{max}
$$

等价可行性校核：

$$
m_k^{min}
\leq
\frac{V_{ac}^{model}}{K_m V_{dc}}
\leq
m_k^{max}
$$

若违反，则报：

```text
ACDC-PHYS-01:
Modulation index out of range.
```

若由于 DC 电压太低导致无法支撑 AC 电压，则报：

```text
ACDC-PHYS-05:
DC voltage is insufficient for requested AC voltage under modulation limit.
```

---

## 3.1.5 AC 电流约束

三相 AC 侧电流：

$$
I_{k,ac}
=
\frac{
\sqrt{P_{k,ac}^2 + Q_{k,ac}^2}
}{
\sqrt{3}V_{k,ac,ll}
}
$$

约束：

$$
I_{k,ac} \leq I_{k,ac}^{max}
$$

若违反：

```text
ACDC-PHYS-02:
AC current exceeds converter current limit.
```

---

## 3.1.6 DC 电流约束

DC 侧电流：

$$
I_{k,dc}
=
\frac{|P_{k,dc}|}{V_{k,dc}}
$$

约束：

$$
I_{k,dc} \leq I_{k,dc}^{max}
$$

若违反：

```text
ACDC-PHYS-03:
DC current exceeds converter current limit.
```

---

## 3.1.7 容量约束

视在功率：

$$
S_k =
\sqrt{P_{k,ac}^2 + Q_{k,ac}^2}
$$

约束：

$$
P_{k,ac}^2 + Q_{k,ac}^2 \leq S_{k,rated}^2
$$

有功限值：

$$
P_k^{min} \leq P_{k,ac} \leq P_k^{max}
$$

无功限值：

$$
Q_k^{min} \leq Q_{k,ac} \leq Q_k^{max}
$$

若违反：

```text
ACDC-PHYS-04:
Apparent power exceeds converter rating.
```

---

# 3.2 DC/DC 变换器物理模型

## 3.2.1 基本变量

对于 DC/DC 变换器 $$m$$，连接 DC bus $$a$$ 与 DC bus $$b$$。

变量包括：

| 变量 | 含义 |
|---|---|
| $$V_{m,in}$$ | 输入端口电压 |
| $$V_{m,out}$$ | 输出端口电压 |
| $$P_{m,in}^{port}$$ | 输入端口功率注入 |
| $$P_{m,out}^{port}$$ | 输出端口功率注入 |
| $$I_{m,in}$$ | 输入电流 |
| $$I_{m,out}$$ | 输出电流 |
| $$D_m$$ | 占空比 |
| $$P_{m,loss}$$ | 损耗 |
| $$\eta_m$$ | 效率 |

---

## 3.2.2 功率守恒

$$
P_{m,in}^{port}
+
P_{m,out}^{port}
+
P_{m,loss}
=
0
$$

其中：

$$
P_{m,loss} \geq 0
$$

若损耗模型给出负损耗：

```text
DCDC-PHYS-05:
Converter loss model yields negative loss.
```

---

## 3.2.3 Buck 模型

理想 CCM Buck：

$$
V_{out} = D V_{in}
$$

占空比：

$$
D =
\frac{V_{out}}{V_{in}}
$$

可行性要求：

$$
D^{min}
\leq
\frac{V_{out}}{V_{in}}
\leq
D^{max}
$$

如果作为等式潮流模型：

$$
F_{buck}
=
V_{out} - D V_{in}
=
0
$$

---

## 3.2.4 Boost 模型

理想 CCM Boost：

$$
V_{out}
=
\frac{V_{in}}{1-D}
$$

占空比：

$$
D =
1 -
\frac{V_{in}}{V_{out}}
$$

可行性要求：

$$
D^{min}
\leq
1 -
\frac{V_{in}}{V_{out}}
\leq
D^{max}
$$

等式形式：

$$
F_{boost}
=
V_{out}(1-D) - V_{in}
=
0
$$

---

## 3.2.5 Buck-Boost 模型

非反相 Buck-Boost 近似：

$$
\frac{V_{out}}{V_{in}}
=
\frac{D}{1-D}
$$

占空比：

$$
D =
\frac{V_{out}}
{V_{in}+V_{out}}
$$

可行性要求：

$$
D^{min}
\leq
\frac{V_{out}}
{V_{in}+V_{out}}
\leq
D^{max}
$$

等式形式：

$$
F_{bb}
=
V_{out}(1-D) - D V_{in}
=
0
$$

---

## 3.2.6 隔离型 DC/DC 模型

对于带变压器变比 $$n$$ 的隔离型 DC/DC：

$$
V_{out}
=
n M(D) V_{in}
$$

其中 $$M(D)$$ 是调制增益。

可行性约束：

$$
M^{min}
\leq
\frac{V_{out}}{n V_{in}}
\leq
M^{max}
$$

如果显式建模调制：

$$
F_{iso}
=
V_{out} - nM(D)V_{in}
=
0
$$

---

## 3.2.7 电流约束

输入电流：

$$
I_{m,in}
=
\frac{|P_{m,in}^{port}|}{V_{m,in}}
$$

输出电流：

$$
I_{m,out}
=
\frac{|P_{m,out}^{port}|}{V_{m,out}}
$$

约束：

$$
I_{m,in} \leq I_{m,in}^{max}
$$

$$
I_{m,out} \leq I_{m,out}^{max}
$$

若违反：

```text
DCDC-PHYS-02:
Input or output current exceeds limit.
```

---

## 3.2.8 功率方向与双向能力

如果设备单向，只允许输入端吸收、输出端注入，则要求：

$$
P_{in}^{port} \leq 0
$$

$$
P_{out}^{port} \geq 0
$$

若反向，则违反：

```text
DCDC-PHYS-04:
Power direction violates unidirectional converter setting.
```

---

## 3.2.9 模型适用边界

Buck、Boost、Buck-Boost 公式默认：

1. 连续导通模式；
2. 稳态平均模型；
3. 忽略开关压降；
4. 忽略电感电阻；
5. 忽略死区；
6. 忽略轻载 DCM；
7. 忽略控制环动态；
8. 忽略热约束动态。

因此这些模型适合：

```text
潮流级可行性校核
稳态控制方程
工程预筛选
```

不应宣称其等同于精确开关级仿真。

---

# 4. 设备控制层

---

# 4.1 AC/DC 控制模式

建议 AC/DC 控制不要只用一个字符串，而应拆成：

```text
DC 侧控制
AC 有功控制
AC 无功/电压控制
构网能力
功率自由度
```

---

## 4.1.1 推荐 AC/DC 模式

| 模式 | DC 侧 | AC 有功 | AC 无功/电压 | 说明 |
|---|---|---|---|---|
| `AC_PQ` | DC 侧等效定功率 | 固定 P | 固定 Q | DC 侧表现为定功率设备 |
| `AC_PV` | DC 侧等效定功率 | 固定 P | 控 Vac | Q 为未知 |
| `DC_P_AC_Q` | 固定 DC 有功 | 由损耗确定 | 固定 Q | DC 侧给定 P |
| `DC_P_AC_V` | 固定 DC 有功 | 由损耗确定 | 控 Vac | Q 为未知 |
| `DC_V_GRID_FORMING_AC_Q` | DC 刚性构网 | P 自由 | 固定 Q | DC 岛电压参考 |
| `DC_V_GRID_FORMING_AC_V` | DC 刚性构网 | P 自由 | 控 Vac | 同时参与 AC 电压控制 |
| `DC_V_DROOP_AC_Q` | DC 下垂构网 | P 由下垂决定 | 固定 Q | 多源协调 |
| `DC_V_DROOP_AC_V` | DC 下垂构网 | P 由下垂决定 | 控 Vac | 多源协调 |
| `GRID_FORMING_AC_DC` | AC/DC 双侧构网 | 高级模式 | 高级模式 | 需要能量缓冲或严格闭合机制 |

---

## 4.1.2 AC/DC 有功自由度规则

定义布尔变量：

- $$z^{P_{ac}}$$：AC 有功硬给定；
- $$z^{P_{dc}}$$：DC 有功硬给定；
- $$z^{V_{dc}}$$：DC 电压刚性控制；
- $$z^{droop}_{dc}$$：DC 电压下垂控制；
- $$z^{bal}$$：参与系统功率平衡。

最基本规则：

$$
z^{P_{ac}} + z^{P_{dc}} + z^{V_{dc}} \leq 1
$$

但更严谨地说，应满足：

```text
有功通道不能同时被多个互斥硬约束占用。
```

### 错误规则

```text
ACDC-CTRL-01:
AC/DC cannot simultaneously impose incompatible active-power-side constraints.
```

典型冲突包括：

1. 同时硬指定 AC 有功和 DC 有功；
2. 同时硬指定 AC 有功和 DC 电压构网；
3. 同时硬指定 DC 有功和 DC 电压构网；
4. 同时刚性 DC_V 和 DC droop；
5. 同时参与因子平衡和硬 P 控制，除非参与因子只是调度层参考。

---

## 4.1.3 DC 侧构网规则

如果 AC/DC 在 DC 侧构网：

$$
z^{V_{dc}} = 1
$$

则：

$$
z^{P_{ac}} = 0
$$

$$
z^{P_{dc}} = 0
$$

也就是说：

```text
AC/DC DC 侧构网时，AC 有功和 DC 有功不能作为硬约束。
```

对应规则：

```text
ACDC-GFM-01:
DC grid-forming AC/DC converter cannot also impose AC or DC active power as hard constraint.
```

此时应使用：

```json
{
  "ac_p_mode": "FREE",
  "dc_p_mode": "FREE",
  "dc_mode": "GRID_FORMING"
}
```

而不是：

```json
{
  "dc_mode": "GRID_FORMING",
  "p_set_mw": 1.0
}
```

如果存在 `p_schedule_mw`，它只能作为调度值或初值，不得作为硬约束。

---

## 4.1.4 AC 无功/电压控制规则

定义：

- $$z^{Q_{ac}}$$：AC 无功硬给定；
- $$z^{V_{ac}}$$：AC 电压控制；
- $$z^{pf}$$：功率因数控制。

基本规则：

$$
z^{Q_{ac}} + z^{V_{ac}} + z^{pf} \leq 1
$$

对应错误：

```text
ACDC-CTRL-02:
AC/DC cannot simultaneously control AC reactive power, AC voltage and power factor.
```

例如：

```text
Q_SET + VAC_CONTROL
```

通常过约束，除非其中一个是软目标或二次控制目标。

---

## 4.1.5 AC/DC DC 构网潮流方程

对于 AC/DC 变换器 $$k$$，连接 AC bus $$i$$ 和 DC bus $$j$$。

### DC 刚性构网

$$
V_j^{dc} - V_{k,dc}^{set} = 0
$$

### DC 下垂构网

$$
P_{k,dc}
-
P_k^0
-
K_k^{dc}
\left(
V_{k,dc}^{set} - V_j^{dc}
\right)
=
0
$$

其中：

$$
K_k^{dc} > 0
$$

在端口功率注入为正的约定下，当：

$$
V_j^{dc} < V_{k,dc}^{set}
$$

则：

$$
P_{k,dc} > P_k^0
$$

表示设备向 DC 网络增加注入功率，符号合理。

### AC 侧固定 Q

$$
Q_{k,ac} - Q_{k,ac}^{set} = 0
$$

### AC/DC 损耗方程

$$
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

此时 $$P_{k,ac}$$ 是未知量，不应硬指定。

---

## 4.1.6 AC/DC 双侧构网模式限制

`GRID_FORMING_AC_DC` 是高风险高级模式，不建议初期默认支持。

若支持，必须显式声明：

1. 是否有内部储能；
2. 是否有 DC-link 能量缓冲；
3. AC 侧是否控制频率或相角；
4. DC 侧是否控制电压；
5. 有功不平衡由谁承担；
6. 触限后优先保护 AC 还是 DC；
7. 是否使用 P-f droop、P-Vdc droop 或联合 droop。

否则应报：

```text
ACDC-GFM-02:
AC/DC dual-side grid-forming requires explicit energy buffer or balancing mechanism.
```

---

# 4.2 DC/DC 控制模式

---

## 4.2.1 DC/DC 控制对象拆分

不应只写：

```cpp
DCDCControlMode::Voltage
```

必须拆成：

```cpp
DCDCControlQuantity::Voltage | Power | Current | Droop
DCDCControlledPort::Input | Output
```

建议数据结构：

```json
{
  "control": {
    "controlled_port": "OUT",
    "quantity": "VOLTAGE",
    "v_set_pu": 1.0,
    "p_ref_mw": null,
    "p_ref_semantics": "SCHEDULE",
    "droop_enabled": false,
    "droop_k_mw_pu": null
  }
}
```

---

## 4.2.2 DC/DC 电压控制规则

### 控制输出电压

若 DC/DC 控制输出端电压：

$$
V_{out} - V_{out}^{set} = 0
$$

则输入侧 DC 岛必须有电压形成源或电压参考。

否则：

```text
DCDC-CTRL-03:
If DC/DC controls output voltage, input-side DC island must have a voltage-forming source.
```

原因是 DC/DC 需要从输入侧吸收功率来支撑输出侧电压和负荷。

---

### 控制输入电压

若 DC/DC 控制输入端电压：

$$
V_{in} - V_{in}^{set} = 0
$$

则输出侧 DC 岛必须有电压形成源或功率平衡机制。

否则：

```text
DCDC-CTRL-04:
If DC/DC controls input voltage, output-side DC island must have a voltage-forming source.
```

---

## 4.2.3 DC/DC 不能刚性控制两端电压

普通双端口 DC/DC 不能同时刚性控制输入端和输出端电压：

$$
z^{V_{in}} + z^{V_{out}} \leq 1
$$

否则：

```text
DCDC-CTRL-01:
Ordinary DC/DC cannot rigidly control both terminal voltages.
```

除非：

1. 设备是特殊多端口能量路由器；
2. 有内部储能；
3. 有额外自由度；
4. 控制之一是软目标。

---

## 4.2.4 DC/DC 不能同时指定两端功率

普通 DC/DC 不能同时硬指定两端功率：

$$
z^{P_{in}} + z^{P_{out}} \leq 1
$$

因为两端功率还必须满足损耗方程。

否则：

```text
DCDC-CTRL-02:
Ordinary DC/DC cannot specify both terminal powers.
```

---

## 4.2.5 功率控制 DC/DC 的参考要求

若 DC/DC 控制传输功率：

$$
P_{transfer} - P_{set} = 0
$$

则两端 DC 岛都必须有电压参考或等效电压形成机制。

否则功率注入无法被网络吸收或供应。

错误：

```text
DCDC-CTRL-05:
Power-controlled DC/DC requires voltage references on both sides.
```

---

# 5. 网络拓扑与电气岛层

---

# 5.1 AC 岛

AC 岛由以下元件连接形成：

- AC 线路；
- 变压器；
- 闭合开关；
- 母联；
- 其他导电 AC 连接。

AC/DC 变换器不应把 AC 和 DC 合并为同一岛。

每个 AC 岛必须有：

1. 相角参考；
2. 电压幅值参考，若需要；
3. 足够的有功平衡能力；
4. 足够的无功支撑能力。

---

## 5.1.1 AC 岛角度参考

每个 AC 岛至少需要一个相角参考或 AC grid-forming 源。

否则 AC 潮流相角整体漂移。

错误：

```text
ACISLAND-REF-01:
AC island has no angle reference.
```

如果存在多个刚性角度参考且无协调：

```text
ACISLAND-REF-02:
Multiple rigid AC angle references without coordination.
```

---

# 5.2 DC 导电岛

DC 导电岛由以下元件连接形成：

- DC 线路；
- DC 母线；
- DC 开关；
- 低阻抗 DC 支路；
- 显式导电连接。

重要规则：

```text
普通 DC/DC 变换器不应作为 DC 导电岛拓扑边。
```

也就是说，不能因为 DC/DC 连接了两个 DC bus，就把两个 DC bus 合并为一个 DC 电压岛。

应删除类似逻辑：

```cpp
for (const auto& dcdc : sys.dc.dcdc_converters) {
    adj[f].push_back(t);
    adj[t].push_back(f);
}
```

改为：

```cpp
// DC/DC converters are power-electronic interfaces.
// They should not merge two DC conductive islands.
```

---

# 5.3 DC 电压控制岛

DC 电压控制岛是控制意义上的概念。

一个 DC 导电岛内部可能有：

- AC/DC DC 构网源；
- DC/DC 受控端口；
- 储能 DC_V；
- 外部 DC 电网；
- 理想 DC 电压源；
- 多个 droop 源；
- 多个参与因子源。

应区分：

```text
DcConductiveIsland
DcVoltageControlIsland
DcPowerCouplingGraph
```

---

# 5.4 DC/DC 耦合图

虽然 DC/DC 不合并 DC 导电岛，但它会在功率层面耦合两个 DC 岛。

建议构建：

```text
DC power coupling graph
```

其中：

- 节点是 DC 导电岛；
- 边是 DC/DC 变换器；
- AC/DC 可以作为 DC 岛与 AC 岛之间的耦合设备。

这样可以分析：

1. 哪个岛有电压参考；
2. 哪个岛依赖 DC/DC 供电；
3. 功率是否可从有源岛传递到无源岛；
4. 控制方向是否合理。

---

# 6. DC 多电压源协调理论

---

# 6.1 基本集合

对于 DC 导电岛 $$\mathcal{I}_d$$，定义：

刚性 DC 电压源集合：

$$
\mathcal{V}_d^{rigid}
$$

下垂 DC 电压源集合：

$$
\mathcal{V}_d^{droop}
$$

主从控制源集合：

$$
\mathcal{V}_d^{ms}
$$

参与因子源集合：

$$
\mathcal{V}_d^{part}
$$

等效电压形成源集合：

$$
\mathcal{V}_d^{eff}
=
\mathcal{V}_d^{rigid}
\cup
\mathcal{V}_d^{droop}
\cup
\mathcal{V}_d^{ms}
\cup
\mathcal{V}_d^{part}
$$

基本必要条件：

$$
|\mathcal{V}_d^{eff}| \geq 1
$$

如果不满足：

```text
DCISLAND-REF-01:
DC island has no voltage-forming source.
```

但这只是必要条件，不是充分条件。

---

# 6.2 DC 岛可解性条件

一个 DC 岛要可求解，至少需要：

1. 有电压参考或等效电压形成机制；
2. 固定功率不平衡可由灵活源承担；
3. 多个电压源之间无设定冲突；
4. 协调机制方程闭合；
5. 所有参与平衡源未超限；
6. DC 网络方程存在物理解；
7. 恒功率负载不超过网络传输能力；
8. 设备损耗和限值可满足。

可写成：

$$
N_{vref}^{effective} \geq 1
$$

并要求平衡功率满足：

$$
P_{flex}^{min}
\leq
-\sum P_{fixed}
\leq
P_{flex}^{max}
$$

否则：

```text
DCISLAND-BALANCE-01:
Fixed DC power imbalance has no flexible balancing source.
```

或：

```text
DCISLAND-BALANCE-02:
Required balancing power exceeds available flexible range.
```

---

# 6.3 单刚性 DC 电压源

若：

$$
|\mathcal{V}_d^{rigid}| = 1
$$

且无其他冲突，则合法。

该源承担 DC 岛功率不平衡。

方程：

$$
V_i^{dc} - V_i^{set} = 0
$$

其功率注入由 DC 网络平衡求出。

需要校核：

$$
P_i^{min} \leq P_i \leq P_i^{max}
$$

---

# 6.4 多刚性 DC 电压源

多刚性 DC 电压源不能简单判定为错误，也不能简单判定为合法。

需要区分物理位置、网络阻抗、设定值和是否额外指定功率分配。

---

## 6.4.1 多刚性源设定值冲突

如果两个刚性源连接到同一零阻抗节点或等效同一节点，且：

$$
V_1^{set} \neq V_2^{set}
$$

则必然冲突。

错误：

```text
DCISLAND-MULTIV-01:
Multiple rigid DC voltage sources have inconsistent voltage setpoints at electrically identical or zero-impedance nodes.
```

---

## 6.4.2 多刚性源位于不同 DC 节点

如果多个刚性电压源位于不同 DC 节点，中间存在非零电阻网络，则数学上可以固定多个节点电压。

DC 网络方程为：

$$
P_i =
V_i
\sum_{j}
G_{ij}
(V_i - V_j)
$$

若 $$V_i$$ 被固定，则 $$P_i$$ 由网络方程决定。

这种情况不是天然错误。

但需要检查：

1. 其注入功率是否超过设备限值；
2. 电压设定是否导致异常大电流；
3. 是否存在额外参与因子方程导致过约束；
4. 是否存在零阻抗路径导致电压冲突。

---

## 6.4.3 多刚性源不应再强加参与因子

如果多个源均刚性控制本地电压：

$$
V_i = V_i^{set}
$$

则其功率注入由网络电阻和其他节点功率决定。

此时不应再额外指定：

$$
\Delta P_i = \alpha_i \Delta P
$$

否则可能过约束。

因此规则应为：

```text
Rigid voltage control and participation-factor active-power sharing are generally mutually exclusive unless the model explicitly releases some voltage constraints or introduces a common secondary-control variable.
```

对应错误：

```text
DCISLAND-PARTICIPATION-02:
Rigid Vdc constraints and participation power-sharing equations over-constrain the DC island.
```

---

# 6.5 多 DC_V 下垂源

下垂源允许并联。

对每个下垂源 $$e$$：

$$
P_e =
P_e^0
+
K_e^{dc}
\left(
V_e^{set} - V_i^{dc}
\right)
$$

要求：

$$
K_e^{dc} > 0
$$

总下垂刚度：

$$
K_d^{total}
=
\sum_{e \in \mathcal{V}_d^{droop}}
K_e^{dc}
$$

要求：

$$
K_d^{total} > 0
$$

否则：

```text
DCISLAND-DROOP-01:
DC droop sources exist but total droop gain is zero.
```

---

## 6.5.1 下垂源触限处理

若某个 droop 源触及：

$$
P_e = P_e^{max}
$$

或：

$$
P_e = P_e^{min}
$$

则该源应退出 droop 分担，转为限值模式：

$$
P_e = P_e^{lim}
$$

剩余 droop 源重新分担。

若剩余 droop 源无法平衡，则：

```text
DCISLAND-DROOP-02:
Remaining DC droop sources cannot balance island after one or more sources hit limits.
```

---

# 6.6 主从控制

主从控制要求：

1. 一个 master；
2. 若干 slave；
3. master 提供电压参考；
4. slave 不应独立刚性控制电压；
5. slave 通过功率参考、下垂或参与因子响应。

数学条件：

$$
\sum_{e \in \mathcal{V}_d^{ms}}
z_e^{master}
=
1
$$

否则：

```text
DCISLAND-MS-01:
Exactly one master voltage source is required in a master-slave DC voltage group.
```

---

# 6.7 参与因子分配

参与因子适合描述多个灵活源共同承担功率不平衡。

引入公共平衡变量 $$\lambda_d$$：

$$
P_e =
P_e^0
+
\alpha_e \lambda_d
$$

要求：

$$
\sum_e \alpha_e = 1
$$

$$
0 \leq \alpha_e \leq 1
$$

并检查：

$$
P_e^{min}
\leq
P_e^0 + \alpha_e \lambda_d
\leq
P_e^{max}
$$

如果参与因子不满足归一：

```text
DCISLAND-PARTICIPATION-01:
Participation factors must sum to one.
```

如果参与源超限：

```text
DCISLAND-PARTICIPATION-03:
Participation-based balancing exceeds one or more source limits.
```

---

# 7. 统一 AC/DC 潮流方程理论

---

# 7.1 变量不是固定向量，而是动态变量池

不应无条件注册所有变量。

应定义候选变量池：

$$
x_{pool}
=
\left[
\theta^{ac},
V^{ac},
V^{dc},
P^{conv},
Q^{conv},
P^{dcdc},
D,
m,
P^{slack},
Q^{slack},
\lambda,
P^{free},
Q^{free}
\right]^T
$$

但具体变量是否进入 $$x$$，取决于：

1. 节点类型；
2. 设备控制模式；
3. 设备物理模型；
4. 是否采用显式损耗；
5. 是否采用显式调制比；
6. 是否采用显式占空比；
7. 是否有自由注入；
8. 是否有下垂或参与因子。

最终未知量为：

$$
x =
\text{RegisteredVariables}(system, mode)
$$

---

# 7.2 等式方程

统一等式方程为：

$$
F(x,m)=0
$$

其中：

$$
F(x,m)
=
\begin{bmatrix}
F_P^{ac} \\
F_Q^{ac} \\
F_P^{dc} \\
F_{loss}^{acdc} \\
F_{loss}^{dcdc} \\
F_{control} \\
F_{constitutive} \\
F_{slack} \\
F_{droop} \\
F_{participation}
\end{bmatrix}
$$

注意这里不包含普通不等式限值。

---

## 7.2.1 AC 有功平衡

对 AC bus $$i$$：

$$
F_{P,i}^{ac}
=
P_i^{inj}
-
P_i^{calc}(V^{ac},\theta)
=
0
$$

其中：

$$
P_i^{inj}
$$

由挂接在该节点的设备注入汇总而来。

---

## 7.2.2 AC 无功平衡

$$
F_{Q,i}^{ac}
=
Q_i^{inj}
-
Q_i^{calc}(V^{ac},\theta)
=
0
$$

---

## 7.2.3 DC 功率平衡

对于 DC bus $$j$$：

$$
F_{P,j}^{dc}
=
P_j^{inj}
-
P_j^{calc}(V^{dc})
=
0
$$

电阻型 DC 网络中：

$$
P_j^{calc}
=
V_j
\sum_l
G_{jl}
(V_j - V_l)
$$

---

## 7.2.4 AC/DC 损耗方程

$$
F_k^{acdc}
=
P_{k,ac}
+
P_{k,dc}
+
P_{k,loss}
=
0
$$

---

## 7.2.5 DC/DC 损耗方程

$$
F_m^{dcdc}
=
P_{m,in}^{port}
+
P_{m,out}^{port}
+
P_{m,loss}
=
0
$$

---

## 7.2.6 控制方程

根据设备控制生成。

### 固定有功

$$
P - P^{set} = 0
$$

### 固定无功

$$
Q - Q^{set} = 0
$$

### 固定 DC 电压

$$
V^{dc} - V^{dc,set} = 0
$$

### 固定 AC 电压

$$
V^{ac} - V^{ac,set} = 0
$$

### DC 下垂

$$
P
-
P^0
-
K^{dc}
(V^{set}-V^{dc})
=
0
$$

### 参与因子

$$
P_e - P_e^0 - \alpha_e \lambda = 0
$$

---

# 7.3 不等式限值

统一不等式：

$$
g(x,m) \leq 0
$$

典型包括：

### 电压限值

$$
V^{min} - V \leq 0
$$

$$
V - V^{max} \leq 0
$$

### 功率限值

$$
P^{min} - P \leq 0
$$

$$
P - P^{max} \leq 0
$$

### 无功限值

$$
Q^{min} - Q \leq 0
$$

$$
Q - Q^{max} \leq 0
$$

### 容量限值

$$
P^2 + Q^2 - S^2 \leq 0
$$

### 电流限值

$$
I - I^{max} \leq 0
$$

### 调制比限值

$$
m^{min} - m \leq 0
$$

$$
m - m^{max} \leq 0
$$

### 占空比限值

$$
D^{min} - D \leq 0
$$

$$
D - D^{max} \leq 0
$$

---

# 7.4 方程-变量闭合校核

在求解前必须检查：

$$
N_{eq} = N_{var}
$$

若不满足：

```text
PF-EQ-COUNT-01:
Number of equations and variables mismatch.
```

即使数量相等，也必须检查结构秩。

如果结构秩不足：

```text
PF-JAC-STRUCT-01:
Structural rank deficiency detected before numerical solve.
```

如果数值雅可比奇异：

```text
PF-JAC-RANK-01:
Numerical Jacobian rank deficiency detected.
```

---

# 7.5 自由注入闭合

若某节点或设备有自由有功：

$$
P^{free}
$$

必须有外部方程决定它。

否则：

```text
ACNODE-FREEP-01:
Free active power injection has no external equation to determine it.
```

对于 Q_NODE：

```text
ACNODE-Q-01:
Q node has free active power but no external equation to determine it.
```

---

# 8. 扩展节点与设备注入类型

---

# 8.1 传统 AC 节点仍可保留

| 类型 | 给定量 | 未知量 |
|---|---|---|
| PQ | $$P,Q$$ | $$V,\theta$$ |
| PV | $$P,V$$ | $$Q,\theta$$ |
| Slack | $$V,\theta$$ | $$P,Q$$ |

但这些只是统一方程系统的特殊情况。

---

# 8.2 不建议节点类型爆炸

不要把所有控制行为都塞入 bus type。

建议节点只描述网络状态，设备描述控制行为。

可以保留概念标签：

| 概念 | 含义 |
|---|---|
| Q_NODE | 固定 Q，P 由外部设备方程决定 |
| P_NODE | 固定 P，Q 由外部设备方程决定 |
| FREE_PQ_NODE | P、Q 均由设备方程决定 |
| DROOP_NODE | P 或 Q 由下垂方程决定 |
| CONVERTER_INJECTION_NODE | 注入来自变换器 |
| LIMITED_PV_NODE | PV 触限后切换为 PQ |

但代码实现应以设备方程为主。

---

# 8.3 Q_NODE 数学处理

Q_NODE 的典型情况：

$$
Q_i = Q_i^{set}
$$

但：

$$
P_i
$$

未知。

例如 AC/DC 的 `DC_V_GRID_FORMING_AC_Q`：

$$
Q_{k,ac} = Q_{k,ac}^{set}
$$

$$
V_j^{dc} = V_j^{dc,set}
$$

$$
P_{k,ac}
+
P_{k,dc}
+
P_{loss}
=
0
$$

$$
P_{k,dc}
$$

由 DC 平衡决定，进而通过损耗方程得到：

$$
P_{k,ac}
$$

因此 AC 侧表现为：

```text
固定 Q + 未知 P 注入
```

若缺少决定 $$P_{k,ac}$$ 的方程，则系统欠定。

---

# 9. 系统级可求解性校核

---

# 9.1 AC 岛校核

每个 AC 岛必须检查：

1. 是否有角度参考；
2. 是否有电压参考或电压控制能力；
3. 是否有有功平衡源；
4. 是否有无功支撑能力；
5. 多个 slack 是否冲突；
6. AC/DC 自由注入是否闭合；
7. PV 节点触 Q 限后是否仍可解。

错误示例：

```text
ACISLAND-REF-01:
AC island has no angle reference.
```

```text
ACISLAND-BALANCE-01:
AC island has no active power balancing source.
```

```text
ACISLAND-QSUPPORT-01:
AC island has insufficient reactive power support.
```

---

# 9.2 DC 岛校核

每个 DC 岛必须检查：

1. 是否有 DC 电压形成源；
2. 多电压源是否冲突；
3. droop 增益是否有效；
4. 主从组是否唯一 master；
5. 参与因子是否归一；
6. 固定功率是否可平衡；
7. 设备功率是否超限；
8. DC 网络是否可能无解；
9. DC/DC 控制方向是否合理；
10. 恒功率负载是否超过传输能力。

---

## 9.2.1 DC 恒功率负载可解性

DC 网络存在恒功率负载时，不能只看总功率平衡。

两节点示例：

电源电压 $$V_s$$，线路电阻 $$R$$，负载端电压 $$V_l$$，负载功率 $$P$$：

$$
P =
\frac{V_l(V_s - V_l)}{R}
$$

该函数最大值出现在：

$$
V_l = \frac{V_s}{2}
$$

最大功率：

$$
P_{max}
=
\frac{V_s^2}{4R}
$$

如果：

$$
P > P_{max}
$$

则无稳态解。

因此建议加入粗校核：

```text
DCISLAND-SOLVABILITY-01:
DC constant-power load may exceed maximum transferable power under voltage constraints.
```

---

# 10. 限值切换理论

---

# 10.1 限值切换不是简单报错

部分限值触发后，可以切换控制模式继续求解。

例如：

| 原模式 | 触限 | 切换后 |
|---|---|---|
| AC PV | Q 超限 | PQ |
| AC/DC DC_V_AC_Q | P 超限 | P 限值模式或不可行 |
| AC/DC DC_V_AC_V | Q 超限 | DC_V_AC_Q 或不可行 |
| DC/DC V_CONTROL | P 超限 | P_LIMITED |
| 储能 DC_V | SOC 到边界 | P_LIMITED 或退出平衡 |
| 多 droop | 某源触 P 限 | 剩余源重新分担 |

---

# 10.2 Active-set 形式

当前模式定义为 $$m$$。

如果某约束触限：

$$
g_i(x,m) = 0
$$

则该约束变为激活约束，可能替代原控制方程。

例如 PV 节点：

正常：

$$
V - V^{set} = 0
$$

若 Q 超上限：

$$
Q - Q^{max} = 0
$$

同时释放电压控制。

---

# 10.3 防止模式振荡

必须加入滞环或锁定机制。

例如：

```text
若 Q > Qmax，则 PV 切 PQ。
只有当 Q < Qmax - margin 后，才允许恢复 PV。
```

否则在迭代中可能出现：

```text
PV -> PQ -> PV -> PQ -> ...
```

导致不收敛。

---

# 11. 统一潮流求解策略

---

# 11.1 长期推荐：联立 Newton-Raphson

统一方程：

$$
F(x^r)=0
$$

Newton 迭代：

$$
J(x^r)\Delta x = -F(x^r)
$$

$$
x^{r+1} = x^r + \Delta x
$$

其中：

$$
J =
\frac{\partial F}{\partial x}
$$

收敛判据：

$$
\|F(x)\|_{\infty} < \epsilon_F
$$

$$
\|\Delta x\|_{\infty} < \epsilon_x
$$

---

# 11.2 必须进行变量缩放

统一潮流中变量量纲不同：

| 变量 | 量纲 |
|---|---|
| 相角 | rad |
| 电压 | p.u. |
| 功率 | p.u. 或 MW |
| 占空比 | 无量纲 |
| 调制比 | 无量纲 |
| 电流 | p.u. 或 kA |

建议内部全部 p.u. 化。

残差缩放：

$$
\tilde{F}_i =
\frac{F_i}{F_i^{scale}}
$$

避免雅可比病态。

---

# 11.3 短期可用：分块迭代

如果短期无法实现联立 Newton，可采用分块迭代。

```text
1. 初始化 AC 电压、DC 电压、变换器功率。
2. 根据设备控制角色生成初始注入。
3. 求解 DC 岛功率平衡。
4. 根据 AC/DC 损耗更新 AC 注入。
5. 求解 AC 潮流。
6. 根据 AC 电压检查 VSC 调制比、电流、容量。
7. 求解 DC/DC 端口功率和占空比。
8. 更新 droop、参与因子、限值状态。
9. 检查残差。
10. 若未收敛，带松弛更新并继续迭代。
```

松弛更新：

$$
x^{r+1}
=
\alpha x_{new}
+
(1-\alpha)x^r
$$

其中：

$$
0 < \alpha \leq 1
$$

---

# 11.4 推荐实现路线

建议分三层求解能力：

| 层级 | 能力 |
|---|---|
| Level 1 | 规则校核 + 设备物理预校核 + 分块潮流 |
| Level 2 | 分块迭代 + droop + 限值切换 |
| Level 3 | 联立 Newton + 动态方程生成 + active-set |

---

# 12. 数据结构建议

---

# 12.1 AC/DC 数据结构

建议：

```json
{
  "type": "ACDC",
  "index": 0,
  "bus_ac": 1,
  "bus_dc": 1,

  "control": {
    "dc_mode": "GRID_FORMING",
    "dc_voltage_control": {
      "enabled": true,
      "type": "RIGID",
      "v_set_pu": 1.0,
      "droop_k_mw_pu": null,
      "participation_factor": null,
      "group_id": "dc_v_group_1"
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
      "ac_q_mode": "Q_SET",
      "q_set_mvar": 0.0,
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

# 12.2 DC/DC 数据结构

```json
{
  "type": "DCDC",
  "index": 0,
  "bus_in": 1,
  "bus_out": 2,

  "control": {
    "controlled_port": "OUT",
    "quantity": "VOLTAGE",
    "v_set_pu": 1.0,
    "p_ref_mw": null,
    "p_ref_semantics": "SCHEDULE",
    "droop_enabled": false,
    "droop_k_mw_pu": null
  },

  "physical": {
    "topology": "BUCK",
    "bidirectional": true,
    "d_min": 0.05,
    "d_max": 0.95,
    "p_min_mw": -0.1,
    "p_max_mw": 0.1,
    "i_in_max_ka": null,
    "i_out_max_ka": null,
    "voltage_ratio_min": null,
    "voltage_ratio_max": null,
    "loss_model": {
      "type": "constant_efficiency",
      "eta_forward": 0.97,
      "eta_reverse": 0.97
    }
  },

  "initial": {
    "p_in_mw": 0.0,
    "p_out_mw": 0.0,
    "duty": 0.5
  }
}
```

---

# 12.3 DeviceControlRole

建议内部解析成统一角色：

```cpp
struct DeviceControlRole {
    bool controls_ac_p = false;
    bool controls_ac_q = false;
    bool controls_ac_v = false;
    bool controls_ac_angle = false;

    bool controls_dc_p = false;
    bool controls_dc_v_rigid = false;
    bool controls_dc_v_droop = false;

    bool is_ac_grid_forming = false;
    bool is_dc_grid_forming = false;

    bool provides_ac_angle_reference = false;
    bool provides_dc_v_reference = false;

    bool ac_p_is_free = false;
    bool ac_q_is_free = false;
    bool dc_p_is_free = false;

    bool participates_active_balance = false;
    double participation_factor = 0.0;

    double fixed_p_mw = 0.0;
    double fixed_q_mvar = 0.0;

    double flex_up_mw = 0.0;
    double flex_down_mw = 0.0;

    std::string coordination_group_id;
};
```

---

# 13. 方程生成器设计

---

# 13.1 VariableRegistry

```cpp
class VariableRegistry {
public:
    VariableId add_ac_angle(int bus);
    VariableId add_ac_voltage(int bus);
    VariableId add_dc_voltage(int bus);

    VariableId add_ac_injection_p(int device);
    VariableId add_ac_injection_q(int device);
    VariableId add_dc_injection_p(int device);

    VariableId add_dcdc_pin(int conv);
    VariableId add_dcdc_pout(int conv);
    VariableId add_dcdc_duty(int conv);

    VariableId add_vsc_modulation(int conv);

    VariableId add_balance_lambda(int island);
    VariableId add_slack_p(int bus);
    VariableId add_slack_q(int bus);
};
```

关键要求：

```text
变量只在需要时注册。
```

---

# 13.2 EquationBuilder

```cpp
class UnifiedEquationBuilder {
public:
    void add_ac_power_balance_equations();
    void add_dc_power_balance_equations();

    void add_acdc_converter_equations();
    void add_dcdc_converter_equations();

    void add_control_equations();
    void add_droop_equations();
    void add_participation_equations();

    void add_constitutive_equations();
    void add_slack_equations();

    void add_active_limit_equations_if_mode_limited();
};
```

---

# 13.3 AC/DC 方程生成示例

```cpp
if (role.controls_dc_v_rigid) {
    add_equation(Vdc(bus_dc) - vdc_set);
}

if (role.controls_dc_v_droop) {
    add_equation(Pdc(conv) - P0 - k * (vdc_set - Vdc(bus_dc)));
}

if (role.controls_ac_q) {
    add_equation(Qac(conv) - q_set);
}

if (role.controls_ac_v) {
    add_equation(Vac(bus_ac) - vac_set);
}

if (role.controls_ac_p) {
    add_equation(Pac(conv) - pac_set);
}

if (role.controls_dc_p) {
    add_equation(Pdc(conv) - pdc_set);
}

add_equation(Pac(conv) + Pdc(conv) + Ploss(conv));
```

---

# 13.4 DC/DC 方程生成示例

```cpp
if (role.controls_out_voltage) {
    add_equation(Vdc(bus_out) - vout_set);
}

if (role.controls_in_voltage) {
    add_equation(Vdc(bus_in) - vin_set);
}

if (role.controls_power) {
    add_equation(Ptransfer(conv) - p_set);
}

add_equation(Pin(conv) + Pout(conv) + Ploss(conv));

if (physical.topology == BUCK) {
    add_equation(Vout(conv) - D(conv) * Vin(conv));
}

if (physical.topology == BOOST) {
    add_equation(Vout(conv) * (1 - D(conv)) - Vin(conv));
}
```

---

# 14. 校核流程

推荐完整校核流程如下。

```text
1. 读取系统数据。
2. 统一单位与基准。
3. 构建 AC 导电岛。
4. 构建 DC 导电岛。
5. 构建 DC/DC 功率耦合图。
6. 解析设备控制角色。
7. 解析设备物理能力。
8. 执行设备控制自由度校核。
9. 执行 AC 岛参考与平衡能力校核。
10. 执行 DC 岛参考、多 V 源协调与功率平衡校核。
11. 执行 DC/DC 控制方向与端口参考校核。
12. 注册潮流变量。
13. 生成等式方程。
14. 检查方程数与变量数。
15. 检查结构秩。
16. 执行初始点物理可行性预校核。
17. 求解统一潮流或分块潮流。
18. 检查不等式限值。
19. 若触限，执行模式切换并重新求解。
20. 输出 Fatal/Error/Warning/Info 报告。
```

---

# 15. 报告系统与规则编号

---

# 15.1 等级定义

| 等级 | 含义 | 是否允许求解 |
|---|---|---|
| Fatal | 方程不闭合、缺少参考、控制逻辑矛盾 | 禁止 |
| Error | 结构合法但设备限值或协调规则不满足 | 默认禁止，可强制 |
| Warning | 可能可解但有风险 | 允许 |
| Info | 解释性信息 | 允许 |

---

# 15.2 推荐规则编号

## AC 岛

```text
ACISLAND-REF-01:
AC island has no angle reference.

ACISLAND-REF-02:
Multiple rigid AC angle references without coordination.

ACISLAND-BALANCE-01:
AC island has no active power balancing source.

ACISLAND-QSUPPORT-01:
AC island has insufficient reactive power support.
```

---

## DC 岛

```text
DCISLAND-REF-01:
DC island has no voltage-forming source.

DCISLAND-MULTIV-01:
Multiple rigid DC voltage sources have inconsistent setpoints at electrically identical or zero-impedance nodes.

DCISLAND-MULTIV-02:
Multiple rigid DC voltage sources produce non-unique or uncoordinated power sharing.

DCISLAND-DROOP-01:
DC droop sources exist but total droop gain is zero.

DCISLAND-DROOP-02:
Remaining DC droop sources cannot balance island after limits.

DCISLAND-MS-01:
Exactly one master is required in master-slave group.

DCISLAND-PARTICIPATION-01:
Participation factors must sum to one.

DCISLAND-PARTICIPATION-02:
Rigid Vdc constraints and participation equations over-constrain the island.

DCISLAND-BALANCE-01:
Fixed DC power imbalance has no flexible balancing source.

DCISLAND-BALANCE-02:
Required balancing power exceeds available flexible range.

DCISLAND-SOLVABILITY-01:
DC constant-power load may exceed network transfer capability.
```

---

## AC/DC 控制

```text
ACDC-CTRL-01:
AC/DC cannot impose incompatible active-power-side constraints.

ACDC-CTRL-02:
AC/DC cannot simultaneously control AC reactive power, AC voltage and power factor.

ACDC-GFM-01:
DC grid-forming AC/DC converter cannot also impose AC or DC active power as hard constraint.

ACDC-GFM-02:
AC/DC dual-side grid-forming requires explicit energy buffer or balancing mechanism.
```

---

## AC/DC 物理

```text
ACDC-PHYS-01:
Modulation index out of range.

ACDC-PHYS-02:
AC current exceeds converter current limit.

ACDC-PHYS-03:
DC current exceeds converter current limit.

ACDC-PHYS-04:
Apparent power exceeds converter rating.

ACDC-PHYS-05:
DC voltage is insufficient for requested AC voltage under modulation limit.

ACDC-PHYS-06:
Loss model inconsistent with power direction or yields negative loss.
```

---

## DC/DC 控制

```text
DCDC-CTRL-01:
Ordinary DC/DC cannot rigidly control both terminal voltages.

DCDC-CTRL-02:
Ordinary DC/DC cannot specify both terminal powers.

DCDC-CTRL-03:
If DC/DC controls output voltage, input-side DC island must have a voltage-forming source.

DCDC-CTRL-04:
If DC/DC controls input voltage, output-side DC island must have a voltage-forming source.

DCDC-CTRL-05:
Power-controlled DC/DC requires voltage references on both sides.
```

---

## DC/DC 物理

```text
DCDC-PHYS-01:
Duty ratio out of range.

DCDC-PHYS-02:
Input or output current exceeds limit.

DCDC-PHYS-03:
Voltage conversion ratio infeasible for converter topology.

DCDC-PHYS-04:
Power direction violates unidirectional converter setting.

DCDC-PHYS-05:
Converter loss model yields negative loss.
```

---

## 方程结构

```text
PF-EQ-COUNT-01:
Number of equations and variables mismatch.

PF-JAC-STRUCT-01:
Structural rank deficiency detected before numerical solve.

PF-JAC-RANK-01:
Numerical Jacobian rank deficiency detected.

PF-VAR-UNUSED-01:
Registered variable is not used by any equation.

PF-EQ-DUP-01:
Duplicate or dependent control equations detected.

ACNODE-Q-01:
Q node has free active power but no external equation to determine it.

ACNODE-FREEP-01:
Free active power injection has no external equation to determine it.
```

---

## 单位与基准

```text
UNIT-BASE-01:
Inconsistent AC/DC voltage base or power base.

UNIT-BASE-02:
Converter modulation equation missing voltage base definition.

UNIT-BASE-03:
Current limit check uses inconsistent engineering units.
```

---

# 16. 对现有代码的直接修改建议

---

# 16.1 `compute_dc_components`

删除或禁用 DC/DC 合并 DC 岛逻辑：

```cpp
for (const auto& dcdc : sys.dc.dcdc_converters) {
    adj[f].push_back(t);
    adj[t].push_back(f);
}
```

改为：

```cpp
// DC/DC converters are power-electronic interfaces.
// They should not merge two DC conductive islands.
// Their coupling must be represented in the DC power coupling graph.
```

---

# 16.2 `DCBusType::DC_V`

不应无条件认为 `DC_V` bus 本身就是硬电压源。

原逻辑如果类似：

```cpp
if (bus.type == DCBusType::DC_V) {
    summary.hard_vdc_sources += 1;
}
```

应改为：

```cpp
if (bus.type == DCBusType::DC_V) {
    summary.declared_v_buses += 1;
}
```

真正提供电压参考的应是设备角色：

```text
AC/DC DC grid-forming
DC/DC voltage-controlled port
DC storage DC_V
DC external grid
explicit ideal DC voltage source
energy router DC port
```

---

# 16.3 AC/DC `p_set_mw` 语义拆分

禁止一个字段承担多重含义。

将：

```json
"p_set_mw": 1.0
```

拆成：

```json
{
  "p_set_mw": null,
  "p_schedule_mw": 1.0,
  "p_initial_mw": 0.0,
  "p_is_hard_constraint": false
}
```

如果 DC 构网时：

```cpp
if (role.controls_dc_v_rigid && ac_p_is_hard_constraint) {
    fatal("ACDC-GFM-01",
          "DC grid-forming AC/DC converter cannot impose AC active power.");
}
```

如果只是初值：

```cpp
info("ACDC-GFM-INFO",
     "p_initial_mw is used only as an initial guess.");
```

---

# 16.4 DC/DC Voltage 模式重构

由：

```cpp
DCDCControlMode::Voltage
```

改为：

```cpp
DCDCControlQuantity::Voltage
DCDCControlledPort::Input | Output
```

并增加：

```cpp
if (controls_output_voltage && !input_island_has_vref) {
    fatal("DCDC-CTRL-03",
          "Output-voltage-controlled DC/DC requires input-side voltage reference.");
}

if (controls_input_voltage && !output_island_has_vref) {
    fatal("DCDC-CTRL-04",
          "Input-voltage-controlled DC/DC requires output-side voltage reference.");
}
```

---

# 16.5 多 VDC 源处理

不要简单：

```cpp
if (hard_vdc_sources > 1) warning(...);
```

应改为：

```cpp
if (rigid_vdc_sources.size() > 1) {
    if (has_zero_impedance_conflict(rigid_vdc_sources) &&
        !same_vdc_setpoint(rigid_vdc_sources)) {
        fatal("DCISLAND-MULTIV-01",
              "Multiple rigid DC voltage sources have inconsistent setpoints.");
    }

    if (has_participation_equations_also(rigid_vdc_sources)) {
        error("DCISLAND-PARTICIPATION-02",
              "Rigid Vdc constraints and participation equations may over-constrain the island.");
    }

    check_power_limits_for_rigid_vdc_sources();
}
```

---

# 17. 分阶段实施计划

---

# 17.1 阶段 1：语义与拓扑重构

优先级最高。

完成：

1. 统一端口功率正方向；
2. 统一 AC/DC 电压单位和 p.u. 基准；
3. DC/DC 不再合并 DC 导电岛；
4. 区分 AC 岛、DC 导电岛、DC 功率耦合图；
5. AC/DC `p_set_mw` 语义拆分；
6. DC/DC 控制端口拆分；
7. 引入 `DeviceControlRole`；
8. 引入 `DevicePhysicalCapability`。

---

# 17.2 阶段 2：结构可解性校核

完成：

1. AC 岛角度参考检查；
2. DC 岛电压参考检查；
3. DC 多电压源协调检查；
4. 主从、droop、参与因子检查；
5. 设备自由度冲突检查；
6. 方程-变量数量检查；
7. 结构秩检查；
8. 自由注入闭合检查。

---

# 17.3 阶段 3：设备物理可行性

完成：

1. AC/DC 调制比校核；
2. AC/DC AC 电流校核；
3. AC/DC DC 电流校核；
4. AC/DC 容量校核；
5. DC/DC 占空比校核；
6. DC/DC 电压变比校核；
7. DC/DC 电流校核；
8. 单向/双向功率方向校核；
9. 损耗非负校核。

---

# 17.4 阶段 4：分块统一潮流

实现：

1. AC 潮流；
2. DC 电阻网络潮流；
3. AC/DC 损耗耦合；
4. DC/DC 端口功率耦合；
5. droop 分担；
6. 参与因子平衡；
7. 限值切换；
8. 松弛迭代。

---

# 17.5 阶段 5：联立 Newton

实现：

1. 动态变量注册；
2. 动态方程生成；
3. 稀疏雅可比；
4. 自动微分或解析导数；
5. active-set 限值处理；
6. 残差缩放；
7. 结构秩预检查；
8. 多模式切换。

---

# 17.6 阶段 6：高级控制

扩展：

1. 储能 SOC；
2. PV MPPT 与弃光；
3. 多 DC_V 二次控制；
4. 多源最优参与因子；
5. 多端口能量路由器；
6. AC/DC 双侧构网；
7. 时间序列潮流；
8. OPF 与经济调度。

---

# 18. 最终实践建议

---

## 18.1 短期必须做

如果资源有限，最先做以下 8 件事：

1. **DC/DC 不再合并 DC 岛**；
2. **AC/DC DC 构网时禁止硬指定 AC/DC 有功**；
3. **`p_set_mw` 拆成硬约束、调度值、初值**；
4. **DC/DC 电压控制必须声明控制端口**；
5. **多 VDC 源不再简单报错，而是检查协调机制**；
6. **加入方程-变量数量检查**；
7. **加入调制比、占空比、电流、容量基本校核**；
8. **统一功率符号和单位基准**。

---

## 18.2 中期必须做

1. 建立 `DeviceControlRole`；
2. 建立 `DevicePhysicalCapability`；
3. 建立 `DcConductiveIsland` 与 `DcPowerCouplingGraph`；
4. 建立 `UnifiedEquationBuilder`；
5. 支持 Q_NODE 和 free P 注入；
6. 支持 droop 多源分担；
7. 支持限值切换；
8. 支持结构秩校核。

---

## 18.3 长期必须做

1. 联立 AC/DC Newton；
2. active-set 限值处理；
3. 自动微分或解析稀疏雅可比；
4. 多端口能量路由器；
5. 储能 SOC 与时间序列；
6. AC/DC 双侧构网；
7. OPF 化约束处理。

---

# 19. 关键结论

## 19.1 DC 网络允许多个 V 节点

允许，但必须明确：

- 多刚性源是否电压设定冲突；
- 是否存在零阻抗冲突；
- 功率分配是否由网络自然决定；
- 是否额外加入了参与因子导致过约束；
- 是否采用 droop、主从或二次控制；
- 所有源是否满足 P/I/S 限值。

---

## 19.2 AC/DC 可以在 DC 侧构网

AC/DC 在 DC 侧构网时：

$$
V_{dc} = V_{dc}^{set}
$$

或：

$$
P =
P^0
+
K(V^{set}-V_{dc})
$$

但不能同时硬指定：

$$
P_{ac}
$$

或：

$$
P_{dc}
$$

其有功应由：

1. DC 岛平衡；
2. AC/DC 损耗；
3. AC 网络潮流；
4. 设备限值；

共同决定。

---

## 19.3 DC/DC 是功率电子接口，不是导线

普通 DC/DC 不应合并两个 DC 电压岛。

它应作为：

```text
两个 DC 岛之间的受控功率耦合设备。
```

---

## 19.4 物理约束必须加入，但不能全写成等式

正确形式是：

$$
F(x,m)=0
$$

$$
g(x,m)\leq 0
$$

限值触发后通过模式切换、active-set 或优化方法处理。

---

## 19.5 统一潮流应采用设备方程驱动

传统 PQ/PV/Slack 应降级为特殊情况。

更通用的方式是：

```text
网络节点状态变量
+
设备注入变量
+
设备控制方程
+
设备物理方程
+
系统平衡方程
```

---

# 20. 最终推荐架构

建议将现有模块从：

```text
基于模式字符串的规则检查器
```

升级为：

```text
控制角色解析
    ↓
设备物理能力解析
    ↓
AC/DC 岛与耦合图分析
    ↓
控制自由度校核
    ↓
方程-变量闭合校核
    ↓
设备物理可行性校核
    ↓
统一方程生成
    ↓
分块潮流 / 联立 Newton
    ↓
限值切换
    ↓
报告系统
```

对应抽象为：

```text
ControlMode
    ↓
DeviceControlRole
    ↓
DevicePhysicalCapability
    ↓
IslandCoordinationModel
    ↓
VariableRegistry
    ↓
UnifiedPowerFlowEquation
    ↓
Solver
    ↓
Validator
    ↓
Report
```

---

# 21. 一句话总结

**混合 AC/DC 多变换器系统不能再靠传统 PQ/PV/Slack 与简单模式字符串校核维持正确性；必须升级为“设备控制角色 + 物理约束 + 岛级协调 + 方程闭合校核 + 统一潮流求解”的体系。DC 多 V 节点、AC/DC DC 侧构网、DC/DC 端口电压控制、Q_NODE、droop、多源分担都可以支持，但前提是每一个自由度都有方程闭合，每一个控制目标不互相冲突，每一个设备运行点满足物理限值。**