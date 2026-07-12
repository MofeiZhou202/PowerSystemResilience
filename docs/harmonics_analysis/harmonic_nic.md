> Documentation Sync (2026-07-12)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: design/analysis reference; confirm behavior against current implementation before adopting conclusions.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# 双端口 NIC 模块在频域谐波模型中的初始电压、电流如何分配？

对于双端口 **NIC，Network-Interfacing Converter**，最关键的是先明确它在 HPF 中的**端口角色**。

根据 Becker 等论文的建模方式，典型 NIC 被视为一个连接 AC 子系统和 DC 子系统的双端口资源：

```text
AC port：grid-following
DC port：grid-forming
```

也就是：

```text
AC 侧控制电流 / 功率注入
DC 侧控制直流电压
```

因此，典型 NIC 的资源响应写为：

$$
\begin{bmatrix}
\hat{\mathbf{I}}^{AC}_{r} \\
\hat{\mathbf{V}}^{DC}_{s}
\end{bmatrix}
=
\hat{\mathbf{F}}_{NIC}
\left(
\hat{\mathbf{V}}^{AC}_{r},
\hat{\mathbf{I}}^{DC}_{s},
\hat{\mathbf{w}}_{NIC},
\hat{\mathbf{y}}_{o,NIC}
\right)
$$

其中：

- $$r \in \mathcal{R}^{AC}_{2}$$：NIC 的 AC 端口是 AC 子系统中的 grid-following 双端口节点；
- $$s \in \mathcal{S}^{DC}_{2}$$：NIC 的 DC 端口是 DC 子系统中的 grid-forming 双端口节点；
- 输入变量是：
  - AC 端口电压 $$\hat{\mathbf{V}}^{AC}_{r}$$；
  - DC 端口电流 $$\hat{\mathbf{I}}^{DC}_{s}$$；
- 输出变量是：
  - AC 端口电流 $$\hat{\mathbf{I}}^{AC}_{r}$$；
  - DC 端口电压 $$\hat{\mathbf{V}}^{DC}_{s}$$。

因此在初始化时，不是简单地把 AC/DC 两侧电压和电流都作为独立未知量，而是要按照端口角色分配。

---

# 1. 典型 NIC 的端口分配原则

对于常见 **AC grid-following / DC grid-forming** NIC：

| 端口 | 所属集合 | 资源行为 | HPF 状态中应放入的未知量 | 由资源模型输出的量 |
|---|---|---|---|---|
| AC 端口 | $$\mathcal{R}^{AC}_{2}$$ | grid-following | $$\hat{\mathbf{V}}^{AC}_{r}$$ | $$\hat{\mathbf{I}}^{AC}_{r}$$ |
| DC 端口 | $$\mathcal{S}^{DC}_{2}$$ | grid-forming | $$\hat{\mathbf{I}}^{DC}_{s}$$ | $$\hat{\mathbf{V}}^{DC}_{s}$$ |

所以初始状态向量中应包含：

$$
\hat{x}_{NIC,init}
=
\begin{bmatrix}
\hat{\mathbf{V}}^{AC}_{r,init} \\
\hat{\mathbf{I}}^{DC}_{s,init}
\end{bmatrix}
$$

而资源模型会根据该初值计算：

$$
\hat{\mathbf{I}}^{AC}_{r,init}
$$

$$
\hat{\mathbf{V}}^{DC}_{s,init}
$$

---

# 2. 基波 / 零频初始化

## 2.1 AC 端口基波电压初始化

从已有统一 AC/DC 潮流结果中取 NIC 所在 AC 母线电压：

$$
V^{AC}_{r,1,init}
=
V^{AC}_{r,1,PF}
$$

如果是单相正序潮流：

$$
V^{AC}_{r,1,PF}
=
v_r e^{j\theta_r}
$$

如果展开为三相：

$$
V^{AC}_{a,1}
=
V_r e^{j\theta_r}
$$

$$
V^{AC}_{b,1}
=
V_r e^{j(\theta_r-2\pi/3)}
$$

$$
V^{AC}_{c,1}
=
V_r e^{j(\theta_r+2\pi/3)}
$$

---

## 2.2 AC 端口基波电流初始化

虽然 AC 端口电流不是 HPF 状态中的未知量，但它是 NIC 资源模型 operating point 的一部分，必须保存。

基于 PF 结果中的 NIC AC 侧功率：

$$
S^{AC}_{NIC}
=
P^{AC}_{NIC}
+
jQ^{AC}_{NIC}
$$

按“注入网络为正”的约定：

$$
I^{AC}_{NIC,1}
=
\frac{
\left(S^{AC}_{NIC}\right)^*
}{
\left(V^{AC}_{r,1}\right)^*
}
$$

即：

```cpp
Iac1_nic = conj(Sac_nic) / conj(Vac1_port);
```

三相平衡时，若 $$S^{AC}_{NIC}$$ 是三相总功率，则每相功率为：

$$
S_{\phi}
=
\frac{S^{AC}_{NIC}}{3}
$$

于是：

$$
I_{\phi,1}
=
\frac{S_{\phi}^*}{V_{\phi,1}^*}
$$

---

## 2.3 DC 端口稳态电压初始化

DC 端口电压由 NIC 资源模型输出，但 operating point 中应保存 PF 解：

$$
V^{DC}_{s,0,init}
=
V^{DC}_{s,0,PF}
$$

对于典型 DC grid-forming NIC，若该 NIC 控制 DC 电压，则：

$$
V^{DC}_{s,0,PF}
\approx
V^{DC,set}_{NIC}
$$

---

## 2.4 DC 端口稳态电流初始化

DC 端口电流是 HPF 状态中的未知量，因此需要放入初始状态。

若 PF 已给出 NIC 的 DC 侧功率：

$$
P^{DC}_{NIC}
$$

则：

$$
I^{DC}_{NIC,0}
=
\frac{P^{DC}_{NIC}}{V^{DC}_{s,0}}
$$

这里必须遵循统一符号约定。

如果你采用：

```text
bus injection positive
```

即正功率表示从设备注入 DC 网络，则：

$$
I^{DC}_{inj,0}
=
\frac{P^{DC}_{inj}}{V^{DC}_{0}}
$$

若 NIC 从 DC 网吸收功率，则 $$P^{DC}_{inj}<0$$，电流自然为负。

---

# 3. NIC 功率符号的一致性

这是最容易出错的地方。

你当前 C++ 模块采用：

```text
Positive active or reactive power at a bus means injection into the network.
```

即：

> 对 AC 和 DC 两侧，功率正号都表示注入对应网络。

那么对一个无损 NIC：

$$
P^{AC}_{inj}
+
P^{DC}_{inj}
=
0
$$

考虑损耗：

$$
P^{AC}_{inj}
+
P^{DC}_{inj}
+
P_{loss}
=
0
$$

其中 $$P_{loss}>0$$。

例如：

## 情况 A：DC 向 AC 送电

```text
DC 侧吸收功率
AC 侧注入功率
```

则：

$$
P^{AC}_{inj}>0
$$

$$
P^{DC}_{inj}<0
$$

且：

$$
P^{AC}_{inj}
+
P^{DC}_{inj}
+
P_{loss}
=
0
$$

---

## 情况 B：AC 向 DC 送电

```text
AC 侧吸收功率
DC 侧注入功率
```

则：

$$
P^{AC}_{inj}<0
$$

$$
P^{DC}_{inj}>0
$$

同样满足：

$$
P^{AC}_{inj}
+
P^{DC}_{inj}
+
P_{loss}
=
0
$$

---

# 4. 多频率初值分配

对于 NIC，需要初始化四类频域量：

```text
AC port voltage      V_ac_hat
AC port current      I_ac_hat
DC port voltage      V_dc_hat
DC port current      I_dc_hat
```

但在典型 hybrid HPF 状态中，只放：

```text
V_ac_hat for AC grid-following port
I_dc_hat for DC grid-forming port
```

---

## 4.1 AC 端口电压初值

$$
\hat{\mathbf{V}}^{AC}_{r,init}
=
\begin{bmatrix}
V^{AC}_{r,1} \\
V^{AC}_{r,5} \\
V^{AC}_{r,7} \\
\vdots
\end{bmatrix}
$$

设置：

$$
V^{AC}_{r,h,init}
=
\begin{cases}
V^{AC}_{r,1,PF}, & h=1 \\
V^{AC,bg}_{r,h}, & h\ne1 \text{ 且有背景谐波} \\
V^{AC,pre}_{r,h}, & h\ne1 \text{ 且做了线性预解} \\
0, & h\ne1
\end{cases}
$$

推荐优先级：

```text
线性预解 > 背景谐波 > 0
```

---

## 4.2 AC 端口电流初值

虽然不一定放入全局状态，但要给 NIC operating point 和资源响应用：

$$
\hat{\mathbf{I}}^{AC}_{r,init}
=
\begin{bmatrix}
I^{AC}_{r,1} \\
I^{AC}_{r,5} \\
I^{AC}_{r,7} \\
\vdots
\end{bmatrix}
$$

其中：

$$
I^{AC}_{r,1}
=
\frac{
\left(S^{AC}_{NIC}\right)^*
}{
\left(V^{AC}_{r,1}\right)^*
}
$$

非基波：

$$
I^{AC}_{r,h}
=
I^{AC,src}_{NIC,h}
-
Y^{AC,out}_{NIC,h}V^{AC}_{r,h}
+
K_{ad,hr}I^{DC}_{s,r}
$$

若只做 spectrum 初始化，则：

$$
I^{AC}_{r,h}
=
I^{AC,src}_{NIC,h}
$$

其中常见谱模型：

$$
I^{AC,src}_{NIC,h}
=
\frac{M_h}{100}
|I^{AC}_{r,1}|
e^{j\theta_h}
$$

---

## 4.3 DC 端口电压初值

DC 端口电压是资源输出，但也应保存为 operating point：

$$
\hat{\mathbf{V}}^{DC}_{s,init}
=
\begin{bmatrix}
V^{DC}_{s,0} \\
V^{DC}_{s,2} \\
V^{DC}_{s,6} \\
V^{DC}_{s,12} \\
\vdots
\end{bmatrix}
$$

设置：

$$
V^{DC}_{s,r,init}
=
\begin{cases}
V^{DC}_{s,0,PF}, & r=0 \\
V^{DC,bg}_{s,r}, & r\ne0 \text{ 且有背景纹波} \\
V^{DC,pre}_{s,r}, & r\ne0 \text{ 且做了线性预解} \\
0, & r\ne0
\end{cases}
$$

---

## 4.4 DC 端口电流初值

DC 端口电流是典型 NIC 的 HPF 状态未知量：

$$
\hat{\mathbf{I}}^{DC}_{s,init}
=
\begin{bmatrix}
I^{DC}_{s,0} \\
I^{DC}_{s,2} \\
I^{DC}_{s,6} \\
I^{DC}_{s,12} \\
\vdots
\end{bmatrix}
$$

零频：

$$
I^{DC}_{s,0}
=
\frac{
P^{DC}_{NIC}
}{
V^{DC}_{s,0}
}
$$

非零纹波：

$$
I^{DC}_{s,r}
=
I^{DC,src}_{NIC,r}
-
Y^{DC,out}_{NIC,r}V^{DC}_{s,r}
+
K_{da,rh}V^{AC}_{r,h}
$$

如果只做初始估计：

$$
I^{DC}_{s,r}
=
I^{DC,src}_{NIC,r}
$$

或使用线性预解结果对应的 DC 纹波电流。

---

# 5. 对应到状态向量的打包方式

论文式 hybrid HPF 的状态向量为：

$$
\hat{x}
=
\begin{bmatrix}
\hat{\mathbf{I}}^{AC}_{S} \\
\hat{\mathbf{V}}^{AC}_{R} \\
\hat{\mathbf{I}}^{DC}_{S} \\
\hat{\mathbf{V}}^{DC}_{R}
\end{bmatrix}
$$

对于典型 NIC：

```text
AC side belongs to R_AC_2
DC side belongs to S_DC_2
```

所以：

```text
x contains:
    V_AC at NIC AC port
    I_DC at NIC DC port

x does not directly contain:
    I_AC at NIC AC port
    V_DC at NIC DC port
```

打包如下：

```cpp
// NIC AC side: grid-following node -> voltage unknown
for h in ac_orders:
    x[index.Vac_R(nic.ac_bus, h)] = Vac_init(nic.ac_bus, h);

// NIC DC side: grid-forming node -> current unknown
for r in dc_orders:
    x[index.Idc_S(nic.dc_bus, r)] = Idc_init(nic.dc_bus, r);
```

---

# 6. 如果采用直接节点电压法

如果你的第一版 HPF 不采用论文的 hybrid 参数法，而是采用：

$$
\hat{\mathbf{Y}}\hat{\mathbf{V}}
-
\hat{\mathbf{I}}_{resource}
\left(
\hat{\mathbf{V}}
\right)
=
0
$$

那么状态中只包含：

$$
\hat{\mathbf{V}}^{AC}
$$

和：

$$
\hat{\mathbf{V}}^{DC}
$$

此时 NIC 的初始化更简单：

```text
AC port voltage = PF AC voltage
DC port voltage = PF DC voltage
```

即：

$$
V^{AC}_{r,1}=V^{AC}_{r,1,PF}
$$

$$
V^{DC}_{s,0}=V^{DC}_{s,0,PF}
$$

非基波和纹波由：

```text
背景值 / 线性预解 / 0
```

初始化。

NIC 的 AC/DC 电流由资源模型在 residual 中计算，不放入状态向量。

---

# 7. NIC 初始值分配的推荐优先级

建议按照如下优先级生成 NIC 初值。

## 7.1 零频 / 基波

```text
优先使用 PF result 中的 converter transfer report
其次使用 bus injection 反算
最后使用 setpoint 估算
```

也就是：

1. `vsc_transfers` 或 converter report；
2. 节点功率平衡反算；
3. converter setpoint。

---

## 7.2 非基波 / 纹波

优先级：

```text
CoupledLinearPreSolve
    >
LinearPreSolve
    >
measured / specified background spectrum
    >
converter spectrum source
    >
zero
```

也就是说，最推荐：

```text
先用 PF 结果生成 NIC 谐波/纹波源
再做 AC/DC 线性耦合预解
最后用预解结果初始化 NIC 两侧频域量
```

---

# 8. 实用初始化流程

针对每个 NIC：

```text
Step 1:
    读取 NIC 的 AC bus 和 DC bus

Step 2:
    从 PF 结果取：
        Vac1(ac_bus)
        Vdc0(dc_bus)
        Pac_nic
        Qac_nic
        Pdc_nic
        Ploss_nic

Step 3:
    计算：
        Iac1 = conj(Pac + jQac) / conj(Vac1)
        Idc0 = Pdc / Vdc0

Step 4:
    将 operating point 保存到 NIC 模型：
        Vac1, Iac1, Vdc0, Idc0, P/Q/Pdc/Ploss, mode

Step 5:
    初始化 AC 端口电压向量：
        h=1 使用 Vac1
        h≠1 使用预解/背景/0

Step 6:
    初始化 DC 端口电流向量：
        r=0 使用 Idc0
        r≠0 使用预解/纹波源/0

Step 7:
    若采用 hybrid HPF:
        pack V_ac_hat into R_AC
        pack I_dc_hat into S_DC

Step 8:
    若采用 direct nodal HPF:
        pack V_ac_hat and V_dc_hat
```

---

# 9. 伪代码

````markdown
```cpp
void initialize_nic_harmonic_state(
    const NIC& nic,
    const PowerFlowResult& pf,
    const HarmonicOperatingPoint& op,
    const HarmonicIndex& hidx,
    const ResourcePartition& part,
    HarmonicInitialState& init,
    const HarmonicPowerFlowOptions& opt)
{
    int ac_bus = nic.ac_bus;
    int dc_bus = nic.dc_bus;

    // 1. Fundamental / DC operating point
    Complex Vac1 = op.Vac1[op.ac_node_map.at(ac_bus)];
    double  Vdc0 = op.Vdc0[op.dc_node_map.at(dc_bus)];

    double Pac = op.get_converter_ac_p(nic.id);
    double Qac = op.get_converter_ac_q(nic.id);
    double Pdc = op.get_converter_dc_p(nic.id);

    Complex Sac(Pac, Qac);

    // Bus-injection-positive convention
    Complex Iac1 = std::conj(Sac) / std::conj(Vac1);

    double Vdc_safe = std::max(std::abs(Vdc0), opt.min_vdc_pu);
    double Idc0 = Pdc / Vdc_safe;

    // 2. Save NIC operating point
    auto& cop = init.operating_point.nic_points[nic.id];
    cop.Vac1 = Vac1;
    cop.Iac1 = Iac1;
    cop.Vdc0 = Vdc0;
    cop.Idc0 = Idc0;
    cop.Pac = Pac;
    cop.Qac = Qac;
    cop.Pdc = Pdc;
    cop.control_mode = nic.control_mode;

    // 3. Build AC port voltage harmonics
    for (int h : hidx.ac_orders) {
        Complex Vh = 0.0;

        if (h == 1) {
            Vh = Vac1;
        } else if (init.has_linear_presolve_ac_voltage(ac_bus, h)) {
            Vh = init.get_linear_presolve_ac_voltage(ac_bus, h);
        } else if (nic.has_background_ac_voltage(h)) {
            Vh = nic.background_ac_voltage(h);
        }

        init.Vac_hat[hidx.ac_voltage_index(ac_bus, h)] = Vh;
    }

    // 4. Build DC port current ripples
    for (int r : hidx.dc_orders) {
        Complex Ir = 0.0;

        if (r == 0) {
            Ir = Complex(Idc0, 0.0);
        } else if (init.has_linear_presolve_dc_current(dc_bus, r)) {
            Ir = init.get_linear_presolve_dc_current(dc_bus, r);
        } else if (nic.has_dc_ripple_current_source(r)) {
            Ir = nic.dc_ripple_current_source(r, Idc0);
        }

        init.Idc_hat[hidx.dc_current_index(dc_bus, r)] = Ir;
    }

    // 5. Pack state depending on formulation
    if (opt.formulation == HarmonicFormulation::HybridParameter) {
        // AC side of typical NIC is R_AC_2: voltage unknown
        for (int h : hidx.ac_orders) {
            init.x[hidx.state_index_Vac_R(ac_bus, h)] =
                init.Vac_hat[hidx.ac_voltage_index(ac_bus, h)];
        }

        // DC side of typical NIC is S_DC_2: current unknown
        for (int r : hidx.dc_orders) {
            init.x[hidx.state_index_Idc_S(dc_bus, r)] =
                init.Idc_hat[hidx.dc_current_index(dc_bus, r)];
        }
    }
}
```
````

---

# 10. 若 NIC 控制角色相反怎么办？

虽然论文说实际中通常是：

```text
AC grid-following
DC grid-forming
```

但理论上也可能：

```text
AC grid-forming
DC grid-following
```

例如某些孤岛 AC 微网由 converter 成网，而 DC 侧作为功率跟随端。

此时端口分配反过来：

| 端口 | 所属集合 | HPF 状态未知量 | 资源输出 |
|---|---|---|---|
| AC 端口 | $$\mathcal{S}^{AC}_{2}$$ | $$\hat{\mathbf{I}}^{AC}$$ | $$\hat{\mathbf{V}}^{AC}$$ |
| DC 端口 | $$\mathcal{R}^{DC}_{2}$$ | $$\hat{\mathbf{V}}^{DC}$$ | $$\hat{\mathbf{I}}^{DC}$$ |

资源响应变成：

$$
\begin{bmatrix}
\hat{\mathbf{V}}^{AC}_{s} \\
\hat{\mathbf{I}}^{DC}_{r}
\end{bmatrix}
=
\hat{\mathbf{F}}_{NIC}
\left(
\hat{\mathbf{I}}^{AC}_{s},
\hat{\mathbf{V}}^{DC}_{r}
\right)
$$

初始化应变为：

```text
AC 侧放 I_ac_hat
DC 侧放 V_dc_hat
```

因此建议在代码中不要硬编码，而是给 NIC 增加端口角色字段：

```cpp
enum class PortBehavior {
    GridForming,
    GridFollowing
};

struct NICPortRole {
    PortBehavior ac_port;
    PortBehavior dc_port;
};
```

初始化时根据角色自动分配。

---

# 11. 推荐的通用分配规则

可以总结为一句话：

> grid-forming 端口在 HPF 状态中放“电流初值”，grid-following 端口在 HPF 状态中放“电压初值”。

即：

| 端口行为 | 控制量 | 状态未知量 | 初值来源 |
|---|---|---|---|
| Grid-forming | 电压 | 电流 | 由 PF 功率和端口电压反算 |
| Grid-following | 电流 | 电压 | 由 PF 节点电压给定 |

所以对于每个 NIC 端口：

```cpp
if port is grid_forming:
    initialize current from PF power / voltage
    pack current into state
else if port is grid_following:
    initialize voltage from PF bus voltage
    pack voltage into state
```

---

# 12. 最终推荐

对于论文中的典型 NIC：

```text
AC 端口：grid-following
DC 端口：grid-forming
```

初值分配为：

## 放入 HPF 状态向量

$$
\boxed{
\hat{x}_{NIC,init}
=
\begin{bmatrix}
\hat{\mathbf{V}}^{AC}_{r,init} \\
\hat{\mathbf{I}}^{DC}_{s,init}
\end{bmatrix}
}
$$

其中：

$$
V^{AC}_{r,1,init}
=
V^{AC}_{r,1,PF}
$$

$$
I^{DC}_{s,0,init}
=
\frac{P^{DC}_{NIC,PF}}{V^{DC}_{s,0,PF}}
$$

非基波部分：

$$
V^{AC}_{r,h,init}
=
V^{AC,pre}_{r,h}
\quad
\text{or}
\quad
V^{AC,bg}_{r,h}
\quad
\text{or}
\quad
0
$$

$$
I^{DC}_{s,r,init}
=
I^{DC,pre}_{s,r}
\quad
\text{or}
\quad
I^{DC,src}_{s,r}
\quad
\text{or}
\quad
0
$$

## 保存为 operating point，但不一定放入状态向量

$$
I^{AC}_{r,1,PF}
=
\frac{
\left(P^{AC}_{NIC}+jQ^{AC}_{NIC}\right)^*
}{
\left(V^{AC}_{r,1,PF}\right)^*
}
$$

$$
V^{DC}_{s,0,PF}
=
V^{DC}_{s,0}
$$

这样就能保证：

- 与论文的双端口 NIC 表达一致；
- 与你现有 AC/DC PF 的 bus-injection-positive 约定一致；
- AC/DC 功率平衡一致；
- 后续资源 Jacobian 中的 AC/DC 耦合项有正确的初始工作点；
- 频域 Newton warm start 更稳定。
