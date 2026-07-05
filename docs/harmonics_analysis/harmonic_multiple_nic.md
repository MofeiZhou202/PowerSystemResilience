> Documentation Sync (2026-07-05)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: design/analysis reference; confirm behavior against current implementation before adopting conclusions.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# 多换流器复杂直流网络中如何保持 AC 端口与 DC 端口稳态功率平衡？

对于含有多个 VSC / NIC / DCDC / Energy Router 的复杂 DC 网络，保持 AC 端口与 DC 端口稳态功率平衡的关键是：

> **不要只在单个换流器局部写功率平衡，而要在统一 AC/DC 潮流方程中同时满足：AC 网络功率平衡、DC 网络功率平衡、换流器端口功率平衡和损耗模型。**

也就是说，每个换流器都要满足：

$$
P^{AC}_{m,inj}
+
P^{DC}_{m,inj}
+
P^{loss}_{m}
=
0
$$

同时整个 DC 网络还要满足：

$$
\sum_{k \in \mathcal{N}_{dc}}
P^{DC}_{k,inj}
-
\sum_{\ell \in \mathcal{E}_{dc}}
P^{loss}_{\ell}
=
0
$$

在复杂多换流器 DC 网络中，应当通过**统一变量、统一符号、统一残差装配**来保证平衡，而不是靠事后修正。

---

# 1. 统一符号约定

你当前程序采用：

```text
bus injection positive
```

即：

> 正功率表示设备向其连接的网络注入功率。

因此：

- AC 侧正功率：换流器向 AC 网络注入；
- DC 侧正功率：换流器向 DC 网络注入；
- 负功率：从对应网络吸收。

对第 $$m$$ 个 AC/DC 换流器：

$$
P^{AC}_{m,inj}
+
P^{DC}_{m,inj}
+
P^{loss}_{m}
=
0
$$

其中：

$$
P^{loss}_{m} \ge 0
$$

---

## 1.1 功率方向示例

### DC 向 AC 送电

```text
DC 网络 -> 换流器 -> AC 网络
```

则：

$$
P^{AC}_{m,inj} > 0
$$

$$
P^{DC}_{m,inj} < 0
$$

$$
P^{AC}_{m,inj}
+
P^{DC}_{m,inj}
+
P^{loss}_{m}
=
0
$$

等价于：

$$
P^{AC}_{m,inj}
=
-
P^{DC}_{m,inj}
-
P^{loss}_{m}
$$

---

### AC 向 DC 送电

```text
AC 网络 -> 换流器 -> DC 网络
```

则：

$$
P^{AC}_{m,inj} < 0
$$

$$
P^{DC}_{m,inj} > 0
$$

$$
P^{DC}_{m,inj}
=
-
P^{AC}_{m,inj}
-
P^{loss}_{m}
$$

---

# 2. 不要直接强行指定两侧功率

对一个 AC/DC 换流器，不能同时独立指定：

$$
P^{AC}_{m}
$$

和：

$$
P^{DC}_{m}
$$

否则会过约束，除非二者严格满足损耗平衡。

正确做法是：

| 控制模式 | 独立给定 | 由功率平衡计算 |
|---|---|---|
| PQ mode | $$P^{AC}, Q^{AC}$$ | $$P^{DC}$$ |
| VDC/Q mode | $$V^{DC}, Q^{AC}$$ | $$P^{AC}, P^{DC}$$ 由网络和控制共同决定 |
| PDC/Q mode | $$P^{DC}, Q^{AC}$$ | $$P^{AC}$$ |
| DC slack / VDC forming | $$V^{DC}$$ | 换流器承担 DC 网络功率不平衡 |
| Droop mode | $$V^{DC}$$-$$P$$ 关系 | $$P$$ 由下垂方程决定 |

---

# 3. 多换流器 DC 网络的平衡层级

复杂直流网络中有多个平衡层级。

## 3.1 单个换流器端口平衡

每个 AC/DC 换流器：

$$
P^{AC}_{m}
+
P^{DC}_{m}
+
P^{loss}_{m}
=
0
$$

每个 DCDC 换流器：

$$
P^{in}_{m}
+
P^{out}_{m}
+
P^{loss}_{m}
=
0
$$

每个 Energy Router：

$$
\sum_{p \in \mathcal{P}_m}
P_{m,p}
+
P^{loss}_m
=
0
$$

---

## 3.2 DC 节点功率平衡

对每个 DC 节点 $$k$$：

$$
\Delta P^{DC}_k
=
P^{DC,spec}_k
+
\sum_{m \in \mathcal{C}_k}
P^{DC}_{m,k}
-
P^{DC,calc}_k
=
0
$$

其中：

$$
P^{DC,calc}_k
=
V^{DC}_k
\left(
\mathbf{G}_{dc}\mathbf{V}^{DC}
\right)_k
$$

---

## 3.3 AC 节点功率平衡

对每个 AC 节点 $$i$$：

$$
\Delta P^{AC}_i
=
P^{AC,spec}_i
+
\sum_{m \in \mathcal{C}_i}
P^{AC}_{m,i}
-
P^{AC,calc}_i
=
0
$$

$$
\Delta Q^{AC}_i
=
Q^{AC,spec}_i
+
\sum_{m \in \mathcal{C}_i}
Q^{AC}_{m,i}
-
Q^{AC,calc}_i
=
0
$$

---

## 3.4 全局 DC 网络功率平衡

整体上应满足：

$$
\sum_{m \in \mathcal{C}_{AC/DC}}
P^{DC}_{m,inj}
+
\sum_{g \in \mathcal{G}_{dc}}
P^{DC}_{g}
-
\sum_{\ell \in \mathcal{L}_{dc}}
P^{DC}_{\ell}
-
P^{loss}_{dc,line}
-
\sum_{d \in \mathcal{DCDC}} P^{loss}_{d}
=
0
$$

如果 DC 网络存在多个孤岛，每个 DC island 都必须单独满足功率平衡。

---

# 4. DC slack / VDC 控制器的作用

复杂 DC 网络必须有足够的电压控制资源，否则 DC 潮流不闭合。

每个 DC 连通岛至少需要：

```text
一个 DC voltage-forming resource
```

例如：

- VDC/Q 控制的 NIC；
- DC slack converter；
- 储能 DC/DC voltage source；
- Energy router voltage-forming port。

如果某个 DC island 没有 VDC-forming 设备，那么 DC 电压没有参考，方程奇异。

---

## 4.1 单个 VDC slack

若一个 DC island 中只有一个 VDC 控制换流器，则它吸收所有 DC 功率不平衡：

$$
P^{DC}_{slack}
=
-
\left(
\sum_{k \ne slack}
P^{DC}_{k,inj}
-
P^{loss}_{dc}
\right)
$$

再由换流器平衡得到 AC 侧功率：

$$
P^{AC}_{slack}
=
-
P^{DC}_{slack}
-
P^{loss}_{slack}
$$

---

## 4.2 多个 VDC-forming 换流器

如果一个 DC island 有多个 VDC 控制换流器，不能全部刚性设定完全相同的 DC 电压而没有功率分担规则，否则可能过约束。

应采用：

### 方法 A：一个主 slack，其余 P 控制

```text
1 个 VDC slack
其他 converter 使用 P/Q 或 Pdc/Q
```

简单可靠。

---

### 方法 B：分布式 DC slack

设参与分担的换流器集合为：

$$
\mathcal{S}_{dc}
$$

总 DC 不平衡功率：

$$
\Delta P_{dc,island}
$$

分配给换流器 $$m$$：

$$
\Delta P^{DC}_{m}
=
\alpha_m
\Delta P_{dc,island}
$$

其中：

$$
\sum_{m \in \mathcal{S}_{dc}}
\alpha_m
=
1
$$

---

### 方法 C：VDC-P 下垂控制

对换流器 $$m$$：

$$
P^{DC}_{m}
=
P^{DC,0}_{m}
+
K^{dc}_{m}
\left(
V^{DC}_{m}
-
V^{DC,set}_{m}
\right)
$$

或反号形式：

$$
P^{DC}_{m}
=
P^{DC,0}_{m}
-
K^{dc}_{m}
\left(
V^{DC}_{m}
-
V^{DC,set}_{m}
\right)
$$

符号取决于你定义的注入方向，必须在代码中统一。

下垂控制可自然实现多个 converter 分担 DC 平衡。

---

# 5. 推荐的统一 AC/DC Newton 残差设计

你已有统一 AC/DC Newton 模块，推荐在其中显式装配 converter 平衡。

---

## 5.1 状态变量

基础状态：

$$
x
=
\begin{bmatrix}
\theta_{ac} \\
V_{ac} \\
V_{dc}
\end{bmatrix}
$$

如果换流器功率作为未知量，可以扩展为：

$$
x
=
\begin{bmatrix}
\theta_{ac} \\
V_{ac} \\
V_{dc} \\
P^{AC}_{conv} \\
P^{DC}_{conv} \\
Q^{AC}_{conv}
\end{bmatrix}
$$

但如果你已有实现是把 converter 功率作为状态相关注入，也可以不扩展，只在残差里根据控制模式计算。

---

## 5.2 残差集合

推荐统一残差包括：

```text
1. AC P mismatch
2. AC Q mismatch
3. DC P mismatch
4. Converter internal power-balance mismatch, if converter powers are explicit variables
5. Converter control equations, e.g. Vdc setpoint, Q setpoint, droop equation
```

若 converter 功率不是独立变量，内部功率平衡通过函数直接消元，例如：

$$
P^{DC}_{m}
=
-
P^{AC}_{m}
-
P^{loss}_m
$$

那么不需要单独的 converter balance residual。

---

# 6. 两种实现策略

## 6.1 策略一：消元式 converter 模型

这是你当前模块更像的方式。

例如 PQ 模式：

给定：

$$
P^{AC}_{m}=P^{set}_m
$$

$$
Q^{AC}_{m}=Q^{set}_m
$$

则：

$$
P^{DC}_{m}
=
-
P^{AC}_{m}
-
P^{loss}_m
$$

直接装配到 DC bus。

优点：

- 状态少；
- 实现简单；
- 收敛快。

缺点：

- 多种控制模式下 Jacobian 较复杂；
- 多 slack / droop 时需要特殊处理。

---

## 6.2 策略二：显式 converter 变量模型

将 converter 端口功率作为未知量：

$$
P^{AC}_{m},Q^{AC}_{m},P^{DC}_{m}
$$

并添加方程：

### 功率平衡

$$
P^{AC}_{m}
+
P^{DC}_{m}
+
P^{loss}_m
=
0
$$

### 控制方程

PQ 模式：

$$
P^{AC}_{m}
-
P^{set}_m
=
0
$$

$$
Q^{AC}_{m}
-
Q^{set}_m
=
0
$$

VDC/Q 模式：

$$
V^{DC}_{m}
-
V^{DC,set}_m
=
0
$$

$$
Q^{AC}_{m}
-
Q^{set}_m
=
0
$$

Droop 模式：

$$
P^{DC}_{m}
-
P^{DC,0}_{m}
-
K^{dc}_m
\left(
V^{DC}_{m}
-
V^{DC,set}_{m}
\right)
=
0
$$

优点：

- 结构清晰；
- 多换流器、多控制模式更通用；
- 更适合之后扩展到 OPF 和 HPF。

缺点：

- 状态维度更大；
- 需要扩展 Jacobian。

---

# 7. 多换流器时的推荐做法

对于复杂直流网络，推荐采用：

```text
基波 PF：
    使用显式 converter 变量模型或半显式模型

HPF 初始化：
    使用 PF 得到的 converter operating points

谐波 HPF：
    使用每个 converter 的 operating point 构造频域响应
```

即每个 converter 都保存：

```cpp
struct ConverterOperatingPoint {
    double Pac_inj;
    double Qac_inj;
    double Pdc_inj;
    double Ploss;

    Complex Vac1;
    Complex Iac1;

    double Vdc0;
    double Idc0;

    ConverterControlMode mode;
    int dc_island_id;
    double slack_participation;
};
```

并保证：

```cpp
abs(Pac_inj + Pdc_inj + Ploss) < tolerance
```

---

# 8. 多换流器 DC island 的功率平衡算法

建议在 PF 后处理和 HPF 初始化前增加一个校正 / 校验步骤。

---

## 8.1 识别 DC island

```cpp
auto dc_islands = find_connected_components(dc_network);
```

对每个 island：

```cpp
converters_in_island
dc_loads_in_island
dc_sources_in_island
dcdc_ports_in_island
energy_router_ports_in_island
```

---

## 8.2 检查 VDC-forming 资源

```cpp
if island has no VDC-forming resource:
    throw error or assign slack
```

---

## 8.3 计算 island 功率不平衡

按 bus-injection-positive：

$$
\Delta P_{island}
=
\sum_{k \in island} P^{DC}_{inj,k}
-
P^{DC,line\_loss}_{island}
$$

在严格节点功率方程收敛后，理论上：

$$
\Delta P_{island}
\approx 0
$$

但实际由于损耗模型、后处理误差，可能有小偏差。

---

## 8.4 分配到 slack converters

如果存在分布式 slack：

$$
P^{DC}_{m,new}
=
P^{DC}_{m,old}
-
\alpha_m
\Delta P_{island}
$$

然后更新其 AC 侧功率：

$$
P^{AC}_{m,new}
=
-
P^{DC}_{m,new}
-
P^{loss}_{m,new}
$$

注意：如果损耗依赖功率，需要迭代更新。

---

# 9. 损耗模型的一致处理

换流器功率平衡必须包含损耗，否则 AC/DC 两侧会长期有偏差。

常见损耗：

## 9.1 线性效率模型

若 AC 侧为输出功率：

$$
P_{loss}
=
\left(
\frac{1}{\eta}-1
\right)
|P_{out}|
$$

也可简化为：

$$
P_{loss}
=
(1-\eta)|P|
$$

但要明确 $$P$$ 是输入还是输出。

---

## 9.2 电流型损耗模型

$$
P_{loss}
=
a
+
bI
+
cI^2
$$

其中：

$$
I^{AC}
\approx
\frac{|S^{AC}|}{V^{AC}}
$$

或：

$$
I^{DC}
=
\frac{|P^{DC}|}{V^{DC}}
$$

---

## 9.3 推荐

为了保持 AC/DC 两侧严格平衡，建议选择一个“主功率变量”。

例如以 AC 侧为主：

$$
P^{DC}
=
-
P^{AC}
-
P_{loss}(P^{AC},Q^{AC},V^{AC},V^{DC})
$$

或者以 DC 侧为主：

$$
P^{AC}
=
-
P^{DC}
-
P_{loss}(P^{DC},V^{DC})
$$

不要两边分别算。

---

# 10. HPF 初始工况中的功率平衡

你问的是复杂 DC 网络下如何保持稳态功率平衡，这对 HPF 初始 operating point 很关键。

在把 PF 映射到 HPF 前，应保证每个 NIC：

$$
\epsilon_m
=
P^{AC}_{m,PF}
+
P^{DC}_{m,PF}
+
P^{loss}_{m,PF}
$$

满足：

$$
|\epsilon_m| < \varepsilon
$$

如果不满足，不要直接进入 HPF，否则 converter 小信号线性化点不一致。

---

## 10.1 推荐修正策略

如果误差很小，可按控制模式修正。

### PQ 模式

保持 AC 侧设定不变，修正 DC 侧：

$$
P^{DC}_{new}
=
-
P^{AC}_{set}
-
P^{loss}
$$

---

### VDC/Q 模式

保持 DC 电压约束和 DC 网络平衡，修正 AC 侧：

$$
P^{AC}_{new}
=
-
P^{DC}
-
P^{loss}
$$

---

### PDC/Q 模式

保持 DC 侧设定，修正 AC 侧：

$$
P^{AC}_{new}
=
-
P^{DC}_{set}
-
P^{loss}
$$

---

### Droop 模式

按 droop 重新计算 $$P^{DC}$$，然后更新 AC 侧：

$$
P^{DC}_{new}
=
P^{DC,0}
+
K^{dc}
\left(
V^{DC}
-
V^{DC,set}
\right)
$$

$$
P^{AC}_{new}
=
-
P^{DC}_{new}
-
P^{loss}
$$

---

# 11. 映射到 NIC 初始电流

修正后的功率用于计算端口电流：

## AC 侧

$$
I^{AC}_{m,1}
=
\frac{
\left(P^{AC}_{m}+jQ^{AC}_{m}\right)^*
}{
\left(V^{AC}_{m,1}\right)^*
}
$$

## DC 侧

$$
I^{DC}_{m,0}
=
\frac{
P^{DC}_{m}
}{
V^{DC}_{m,0}
}
$$

这样 AC/DC 两侧电流对应同一个功率平衡点。

---

# 12. 多换流器平衡伪代码

````markdown
```cpp
void enforce_converter_power_balance_for_hpf_initialization(
    const HybridPowerSystem& sys,
    const PowerFlowResult& pf,
    HarmonicOperatingPoint& op,
    const PowerFlowOptions& opt)
{
    auto dc_islands = find_dc_islands(sys.dc);

    for (const auto& island : dc_islands) {
        auto converters = find_converters_in_dc_island(sys, island);

        auto vdc_forming = filter_vdc_forming_converters(converters);

        if (vdc_forming.empty()) {
            throw std::runtime_error(
                "DC island has no voltage-forming converter.");
        }

        // First enforce local converter balance
        for (auto& conv_id : converters) {
            auto& c = op.converter(conv_id);

            double Ploss = compute_converter_loss(c, opt.loss_model);

            switch (c.control_mode) {
            case ConverterControlMode::PQ_MODE:
                // Keep AC P/Q fixed, adjust DC P.
                c.Pdc_inj = -c.Pac_inj - Ploss;
                break;

            case ConverterControlMode::PDC_Q:
                // Keep DC P and Q fixed, adjust AC P.
                c.Pac_inj = -c.Pdc_inj - Ploss;
                break;

            case ConverterControlMode::VDC_Q:
            case ConverterControlMode::VDC_VAC:
                // Usually DC voltage is controlled; P emerges from network.
                // Prefer keeping DC-side solved injection and adjust AC P.
                c.Pac_inj = -c.Pdc_inj - Ploss;
                break;

            case ConverterControlMode::DROOP_VDC_P:
                c.Pdc_inj = c.Pdc0
                    + c.Kdroop * (c.Vdc0 - c.Vdc_set);
                c.Pac_inj = -c.Pdc_inj - Ploss;
                break;
            }

            // Recompute currents from balanced powers.
            c.Iac1 = std::conj(Complex(c.Pac_inj, c.Qac_inj))
                   / std::conj(c.Vac1);

            c.Idc0 = c.Pdc_inj / std::max(c.Vdc0, opt.min_vdc_pu);

            double eps = c.Pac_inj + c.Pdc_inj + Ploss;
            if (std::abs(eps) > opt.converter_balance_tol) {
                warn_converter_balance_error(c.id, eps);
            }
        }

        // Then optionally check island-level balance.
        double island_balance = compute_dc_island_power_balance(sys, op, island);

        if (std::abs(island_balance) > opt.dc_island_balance_tol) {
            distribute_dc_island_mismatch(
                island_balance,
                vdc_forming,
                op,
                opt
            );
        }
    }
}
```
````

---

# 13. 进入 HPF 前的推荐校验

建议在 `build_harmonic_initial_state` 末尾做：

```cpp
validate_converter_balance(op);
validate_dc_island_balance(op);
validate_ac_converter_current_consistency(op);
validate_dc_converter_current_consistency(op);
```

对应数学检查：

## Converter local balance

$$
\max_m
\left|
P^{AC}_{m}
+
P^{DC}_{m}
+
P^{loss}_{m}
\right|
<
\varepsilon_{conv}
$$

## DC island balance

$$
\left|
\sum_{k \in island} P^{DC}_{inj,k}
-
P^{DC,line\_loss}_{island}
\right|
<
\varepsilon_{island}
$$

## AC current consistency

$$
\left|
V^{AC}_{m,1}
\left(I^{AC}_{m,1}\right)^*
-
\left(
P^{AC}_{m}
+
jQ^{AC}_{m}
\right)
\right|
<
\varepsilon
$$

## DC current consistency

$$
\left|
V^{DC}_{m,0}I^{DC}_{m,0}
-
P^{DC}_{m}
\right|
<
\varepsilon
$$

---

# 14. 与频域谐波模型的衔接

在 HPF 中，每个 NIC 的 operating point 应使用平衡后的值：

```cpp
NICOperatingPoint {
    Vac1
    Iac1
    Vdc0
    Idc0
    Pac_inj
    Qac_inj
    Pdc_inj
    Ploss
}
```

然后用于：

1. 谐波电流源幅值缩放：

$$
I_{h}
=
r_h |I_{ac,1}|
e^{j\theta_h}
$$

2. DC 纹波源幅值缩放：

$$
I_{r}^{dc}
=
r_r |I_{dc,0}|
e^{j\theta_r}
$$

3. 小信号模型线性化：

$$
\Delta y
=
G(s;x_0)\Delta u
$$

4. AC/DC 耦合 Jacobian 计算。

---

# 15. 最终推荐

对于含多个换流器的复杂 DC 网络，建议采用以下规则：

```text
1. 统一采用 bus-injection-positive 符号。
2. 每个 AC/DC 换流器必须满足：
       Pac_inj + Pdc_inj + Ploss = 0
3. 每个 DC island 至少有一个 VDC-forming 资源。
4. 不允许同一个换流器两侧功率被独立过约束。
5. 多个 VDC-forming converter 必须使用：
       主 slack / 分布式 slack / droop
   三者之一。
6. HPF 初始化前，先校正并冻结 operating point。
7. 用平衡后的 Pac, Qac, Pdc 计算 Iac1 和 Idc0。
```

最核心的两条公式是：

$$
\boxed{
P^{AC}_{m,inj}
+
P^{DC}_{m,inj}
+
P^{loss}_{m}
=
0
}
$$

以及：

$$
\boxed{
I^{AC}_{m,1}
=
\frac{
\left(P^{AC}_{m,inj}+jQ^{AC}_{m,inj}\right)^*
}{
\left(V^{AC}_{m,1}\right)^*
}
,
\qquad
I^{DC}_{m,0}
=
\frac{
P^{DC}_{m,inj}
}{
V^{DC}_{m,0}
}
}
$$

只要这两个关系在每个换流器 operating point 中成立，后续频域谐波模型中的 NIC / VSC 初始电压、电流、谐波源缩放和 Jacobian 线性化都会保持一致。