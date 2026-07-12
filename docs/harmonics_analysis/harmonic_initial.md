> Documentation Sync (2026-07-12)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: design/analysis reference; confirm behavior against current implementation before adopting conclusions.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# 如何将交直流混合潮流基波结果高效映射到频域谐波模型初始工况？

你已有一个**统一迭代交直流混合潮流模块**，这是做混合 AC/DC 谐波潮流的最大优势。因为频域谐波模型并不是从零开始，而是应当以基波潮流的稳态解作为 **operating point**，再围绕该工作点建立：

- 频率相关网络模型；
- 资源频域响应模型；
- 换流器小信号模型；
- 谐波源频谱；
- DC 纹波模型；
- Newton 初值。

可以把这个过程理解为：

```text
基波 AC/DC 潮流结果
        ↓
Operating Point Extraction
        ↓
Frequency-Domain Initialization
        ↓
Harmonic Power Flow Newton / Linear Solve
```

下面给出一个适合编码实现的完整映射方案。

---

# 1. 总体映射目标

现有基波潮流一般求得：

$$
x_{PF}
=
\begin{bmatrix}
\theta_{ac} \\
v_{ac} \\
v_{dc}
\end{bmatrix}
$$

以及派生结果：

- AC 节点电压 $$V_{i,1}$$；
- AC 支路基波电流 $$I_{\ell,1}$$；
- AC 节点功率注入 $$P_i,Q_i$$；
- DC 节点电压 $$V^{dc}_{k,0}$$；
- DC 支路电流 $$I^{dc}_{\ell,0}$$；
- VSC / DCDC / EnergyRouter 的功率转移；
- converter 控制模式；
- 负荷状态；
- 电容器、开关、调压器状态。

频域谐波模型需要初始化：

$$
\hat{x}_{HPF}
=
\begin{bmatrix}
\hat{\mathbf{I}}^{AC}_{S} \\
\hat{\mathbf{V}}^{AC}_{R} \\
\hat{\mathbf{I}}^{DC}_{S} \\
\hat{\mathbf{V}}^{DC}_{R}
\end{bmatrix}
$$

或如果采用直接节点电压法：

$$
\hat{x}_{HPF}
=
\begin{bmatrix}
\hat{\mathbf{V}}^{AC} \\
\hat{\mathbf{V}}^{DC}
\end{bmatrix}
$$

其中每个变量是多频率堆叠向量，例如：

$$
\hat{V}^{AC}_i
=
\begin{bmatrix}
V^{AC}_{i,1} \\
V^{AC}_{i,5} \\
V^{AC}_{i,7} \\
V^{AC}_{i,11} \\
\vdots
\end{bmatrix}
$$

你的目标是高效构造这些初值，而不是让 HPF 从零向量开始迭代。

---

# 2. 建议增加一个 OperatingPoint 数据层

不要直接从 `PowerFlowResult` 到处取字段。建议先抽象一个中间层：

```cpp
struct HarmonicOperatingPoint {
    // AC fundamental phasors
    ComplexVector Vac1;              // per AC phase-node or bus
    ComplexVector Iac_inj1;          // nodal injection currents
    ComplexVector Iac_branch1;       // branch currents

    // AC powers
    Vector Pac_inj;
    Vector Qac_inj;
    Vector Sac_abs;

    // DC steady-state
    Vector Vdc0;
    Vector Idc_inj0;
    Vector Idc_branch0;
    Vector Pdc_inj;

    // Converter states
    std::vector<ConverterOperatingPoint> converters;

    // Load states
    std::vector<LoadOperatingPoint> loads;

    // Network/control states
    DeviceStateSnapshot device_states;

    // Index maps
    ACNodeMap ac_node_map;
    DCNodeMap dc_node_map;
};
```

其中 converter operating point 建议包含：

```cpp
struct ConverterOperatingPoint {
    int converter_id;

    ComplexVector Vac1_port;     // AC port phase voltages
    ComplexVector Iac1_port;     // AC port phase currents

    double Vdc0_port;
    double Idc0_port;

    double Pac;
    double Qac;
    double Pdc;
    double Ploss;

    double modulation_index;
    double power_angle;
    double firing_angle;
    double extinction_angle;

    ConverterControlMode control_mode;
};
```

这样频域模型初始化只依赖这个统一 `HarmonicOperatingPoint`，不依赖基波 PF 求解器内部细节。

---

# 3. 第一步：从基波结果恢复 AC 基波相量

如果你的 PF 输出是：

- 电压幅值 `vm[i]`；
- 相角 `va[i]`；
- 标幺值。

则恢复 AC 节点基波相量：

$$
V_{i,1}
=
v_i e^{j\theta_i}
$$

代码：

```cpp
for each ac bus i:
    Vac1[i] = vm[i] * exp(j * va_rad[i]);
```

如果你做三相相域谐波，则推荐映射到**相节点**：

```text
(bus, phase) -> phase-node index
```

对于正序基波，若原基波 PF 是单相正序结果，可展开为：

$$
V_{a,1}
=
V_1 \angle \theta
$$

$$
V_{b,1}
=
V_1 \angle \left(\theta - 120^\circ\right)
$$

$$
V_{c,1}
=
V_1 \angle \left(\theta + 120^\circ\right)
$$

即：

```cpp
Va = V * exp(j * theta);
Vb = V * exp(j * (theta - 2*pi/3));
Vc = V * exp(j * (theta + 2*pi/3));
```

如果你已有三相潮流结果，则直接使用各相结果，不要再假设平衡。

---

# 4. 第二步：从基波功率恢复 AC 注入电流

很多谐波源频谱是以基波电流为基准的，例如：

$$
I_h
=
r_h |I_1| e^{j\theta_h}
$$

所以需要从基波结果得到每个设备的基波电流。

对于节点注入功率：

$$
S_i
=
P_i + jQ_i
$$

有：

$$
S_i
=
V_i I_i^*
$$

因此：

$$
I_i
=
\left(
\frac{S_i}{V_i}
\right)^*
=
\frac{S_i^*}{V_i^*}
$$

如果采用“注入网络为正”的符号约定：

$$
I_{inj,i,1}
=
\frac{S_{inj,i,1}^*}{V_{i,1}^*}
$$

代码：

```cpp
Iinj1[i] = conj(Sinj[i]) / conj(Vac1[i]);
```

注意若负荷数据中 `p_mw > 0` 表示消费，则注入功率为：

$$
S_{inj}
=
-
(P_L + jQ_L)
$$

所以负荷基波电流注入为：

$$
I_{load,inj,1}
=
-\frac{(P_L+jQ_L)^*}{V_1^*}
$$

---

# 5. 第三步：提取 DC 稳态量

DC 基波不是 $$h=1$$，而是零频稳态分量：

$$
r=0
$$

从基波 AC/DC PF 中得到：

$$
V^{dc}_{k,0}
$$

$$
I^{dc}_{k,0}
$$

$$
P^{dc}_{k,0}
=
V^{dc}_{k,0} I^{dc}_{k,0}
$$

对于 DC 节点：

$$
I^{dc}_{inj,k,0}
=
\frac{P^{dc}_{inj,k}}{V^{dc}_{k,0}}
$$

代码：

```cpp
Idc0[k] = Pdc_inj[k] / max(Vdc0[k], Vdc_min);
```

如果是恒功率负荷，注意符号：

```cpp
Idc_load_inj[k] = -Pload_dc[k] / Vdc0[k];
```

---

# 6. 第四步：将基波结果映射为多频率初始向量

## 6.1 AC 频率集合

例如：

```cpp
ac_orders = {1, 5, 7, 11, 13, 17, 19, 23};
```

频域节点电压初始化：

$$
V^{AC}_{i,h,0}
=
\begin{cases}
V^{AC}_{i,1,PF}, & h=1 \\
V^{bg}_{i,h}, & h \ne 1 \text{ 且有背景谐波} \\
0, & h \ne 1 \text{ 且无背景谐波}
\end{cases}
$$

即：

```cpp
for each ac node i:
    for each h:
        if h == 1:
            Vhat_ac(i,h) = op.Vac1[i];
        else if has_background_voltage(i,h):
            Vhat_ac(i,h) = background_voltage(i,h);
        else:
            Vhat_ac(i,h) = 0.0;
```

---

## 6.2 DC 频率集合

例如：

```cpp
dc_orders = {0, 2, 6, 12, 18, 24};
```

DC 电压初始化：

$$
V^{DC}_{k,r,0}
=
\begin{cases}
V^{DC}_{k,0,PF}, & r=0 \\
V^{ripple,bg}_{k,r}, & r \ne 0 \text{ 且有背景纹波} \\
0, & r \ne 0
\end{cases}
$$

代码：

```cpp
for each dc node k:
    for each r:
        if r == 0:
            Vhat_dc(k,r) = op.Vdc0[k];
        else if has_dc_background_ripple(k,r):
            Vhat_dc(k,r) = dc_background_ripple(k,r);
        else:
            Vhat_dc(k,r) = 0.0;
```

---

# 7. 对 hybrid 参数 HPF 的初值映射

如果采用论文中的 hybrid 参数形式，状态不是所有节点电压，而是：

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

即：

- grid-forming 节点：初始化其电流；
- grid-following 节点：初始化其电压。

---

## 7.1 AC grid-forming 节点电流初值

对于 $$s \in \mathcal{S}^{AC}$$：

$$
\hat{I}^{AC}_{s,h,0}
=
\begin{cases}
I^{AC}_{s,1,PF}, & h=1 \\
I^{AC,src}_{s,h}, & h \ne 1 \text{ 且该资源注入谐波} \\
0, & h \ne 1
\end{cases}
$$

---

## 7.2 AC grid-following 节点电压初值

对于 $$r \in \mathcal{R}^{AC}$$：

$$
\hat{V}^{AC}_{r,h,0}
=
\begin{cases}
V^{AC}_{r,1,PF}, & h=1 \\
V^{bg}_{r,h}, & h \ne 1 \text{ 且有背景谐波} \\
0, & h \ne 1
\end{cases}
$$

---

## 7.3 DC grid-forming 节点电流初值

对于 $$s \in \mathcal{S}^{DC}$$：

$$
\hat{I}^{DC}_{s,r,0}
=
\begin{cases}
I^{DC}_{s,0,PF}, & r=0 \\
I^{DC,ripple}_{s,r}, & r \ne 0 \\
0, & r \ne 0
\end{cases}
$$

---

## 7.4 DC grid-following 节点电压初值

对于 $$r \in \mathcal{R}^{DC}$$：

$$
\hat{V}^{DC}_{r,r_f,0}
=
\begin{cases}
V^{DC}_{r,0,PF}, & r_f=0 \\
V^{DC,ripple,bg}_{r,r_f}, & r_f \ne 0 \\
0, & r_f \ne 0
\end{cases}
$$

---

# 8. 高效生成谐波源初值

很多设备谐波由基波电流或额定电流缩放。

## 8.1 按基波电流缩放

若设备频谱给出：

```text
h: magnitude_percent, angle_deg
```

则：

$$
I_{dev,h}
=
\frac{M_h}{100}
|I_{dev,1}|
e^{j\theta_h}
$$

但是要注意相角参考。

更稳妥的做法是让谐波相角基于基波电流相角：

$$
I_{dev,h}
=
\frac{M_h}{100}
|I_{dev,1}|
e^{j\left(h\angle I_{dev,1}+\phi_h\right)}
$$

或工程简化为：

$$
I_{dev,h}
=
\frac{M_h}{100}
|I_{dev,1}|
e^{j\left(\angle I_{dev,1}+\phi_h\right)}
$$

两种方式要在选项中明确：

```cpp
enum class HarmonicAngleReference {
    AbsoluteAngle,
    RelativeToFundamentalCurrent,
    RelativeToFundamentalVoltage,
    SequenceBased
};
```

---

## 8.2 三相相序初始化

如果设备是三相平衡谐波源，则相序规则为：

| 谐波次数 | 相序 |
|---|---|
| $$h=3k+1$$ | 正序 |
| $$h=3k+2$$ | 负序 |
| $$h=3k$$ | 零序 |

正序：

$$
I_{b,h}
=
I_{a,h} e^{-j\frac{2\pi}{3}}
$$

$$
I_{c,h}
=
I_{a,h} e^{j\frac{2\pi}{3}}
$$

负序：

$$
I_{b,h}
=
I_{a,h} e^{j\frac{2\pi}{3}}
$$

$$
I_{c,h}
=
I_{a,h} e^{-j\frac{2\pi}{3}}
$$

零序：

$$
I_{a,h}=I_{b,h}=I_{c,h}
$$

代码：

```cpp
Complex phase_factor(int h, Phase ph) {
    if (h % 3 == 1) { // positive
        if (ph == A) return 1.0;
        if (ph == B) return exp(-j * 2*pi/3);
        if (ph == C) return exp( j * 2*pi/3);
    } else if (h % 3 == 2) { // negative
        if (ph == A) return 1.0;
        if (ph == B) return exp( j * 2*pi/3);
        if (ph == C) return exp(-j * 2*pi/3);
    } else { // zero
        return 1.0;
    }
}
```

---

# 9. Converter operating point 的高效映射

这是混合 AC/DC 谐波初始化的核心。

---

## 9.1 VSC / NIC 的基波端口量

对于每个 VSC / NIC，从基波结果提取：

$$
V^{AC}_{port,1}
$$

$$
I^{AC}_{port,1}
$$

$$
V^{DC}_{port,0}
$$

$$
I^{DC}_{port,0}
$$

$$
P_{ac},Q_{ac},P_{dc},P_{loss}
$$

如果你的 `PowerFlowResult` 已经有 `vsc_transfers`，建议直接从里面构造。

---

## 9.2 PQ 模式 NIC 初始化

PQ 模式下 AC 侧注入功率给定：

$$
S_{ac}^{set}
=
P^{set}
+
jQ^{set}
$$

基波 AC 侧电流：

$$
I_{ac,1}
=
\frac{S_{ac}^{set*}}{V_{ac,1}^*}
$$

DC 侧电流：

$$
I_{dc,0}
=
-\frac{P_{ac}+P_{loss}}{V_{dc,0}}
$$

---

## 9.3 VDC/Q 模式 NIC 初始化

VDC/Q 控制下，基波 PF 已经求得 converter 的有功转移：

$$
P^{tr}
=
k^{vdc}
\left[
(V^{dc})^2
-
(V^{dc,set})^2
\right]
$$

或者直接使用 PF 结果中的 converter transfer。

AC 侧：

$$
S_{ac}
=
P^{conv,ac}
+
jQ^{set}
$$

$$
I_{ac,1}
=
\frac{S_{ac}^*}{V_{ac,1}^*}
$$

DC 侧：

$$
I_{dc,0}
=
\frac{P^{conv,dc}}{V_{dc,0}}
$$

---

## 9.4 调制比和功角估计

如果后续需要 VSC 小信号模型，需要估算 converter 内部电压。

假设滤波阻抗：

$$
Z_f
=
R_f+jX_f
$$

换流器内部基波电压：

$$
E_{conv,1}
=
V_{ac,1}
+
Z_f I_{ac,1}
$$

若采用两电平 VSC 近似：

$$
|E_{conv,1}|
\approx
\frac{m V_{dc,0}}{2\sqrt{2}}
$$

则调制比初值：

$$
m_0
=
\frac{2\sqrt{2}|E_{conv,1}|}{V_{dc,0}}
$$

功角：

$$
\delta_0
=
\angle E_{conv,1}
$$

代码：

```cpp
Econv = Vac_port + Zf * Iac_port;
m0 = 2.0 * sqrt(2.0) * abs(Econv) / Vdc0;
delta0 = arg(Econv);
```

并限幅：

```cpp
m0 = clamp(m0, 0.0, m_max);
```

---

## 9.5 LCC 运行点映射

如果有 LCC，需从基波潮流得到：

- 直流电流 $$I_d$$；
- 直流电压 $$V_d$$；
- 交流线电压 $$V_{LL}$$；
- 换相电抗 $$X_c$$；
- 触发角 $$\alpha$$ 或熄弧角 $$\gamma$$。

整流器：

$$
V_d
=
V_{d0}\cos\alpha
-
\frac{3}{\pi}X_c I_d
$$

因此：

$$
\cos\alpha
=
\frac{
V_d+\frac{3}{\pi}X_c I_d
}{
V_{d0}
}
$$

$$
\alpha
=
\cos^{-1}
\left(
\frac{
V_d+\frac{3}{\pi}X_c I_d
}{
V_{d0}
}
\right)
$$

其中：

$$
V_{d0}
=
\frac{3\sqrt{2}}{\pi}aV_{LL}
$$

这样可以从 PF 结果反推出 LCC 的触发角初值。

---

# 10. 资源小信号模型 operating point 的映射

如果采用论文中的 CIDER/NIC 闭环频域模型，需要将 operating point 映射到线性化点：

$$
\hat{\mathbf{y}}_{o}
$$

通常包括：

```text
AC terminal voltage fundamental phasor
AC terminal current fundamental phasor
DC-link voltage
DC current
P/Q setpoints
PLL angle
dq-axis steady-state values
controller integrator states
modulation index
```

---

## 10.1 abc 到 dq 的映射

对三相基波相量，可以构造同步旋转坐标下的稳态分量。

如果 PLL 角为：

$$
\theta_{pll}
=
\angle V_{a,1}
$$

则 dq 电压近似：

$$
V_d
=
|V_1|
$$

$$
V_q
=
0
$$

电流：

$$
I_d
=
\frac{P}{1.5V_d}
$$

$$
I_q
=
-\frac{Q}{1.5V_d}
$$

对于三相功率定义：

$$
P
=
\frac{3}{2}
\left(
V_d I_d + V_q I_q
\right)
$$

$$
Q
=
\frac{3}{2}
\left(
V_q I_d - V_d I_q
\right)
$$

当 $$V_q=0$$：

$$
I_d
=
\frac{2P}{3V_d}
$$

$$
I_q
=
-\frac{2Q}{3V_d}
$$

这对初始化 VSC 控制器状态非常有用。

---

## 10.2 控制器积分状态初值

例如电流 PI 控制：

$$
v_{cmd,d}
=
k_p(i_d^{ref}-i_d)
+
\xi_d
+
\omega L i_q
+
v_d
$$

稳态下：

$$
i_d^{ref}=i_d
$$

所以：

$$
\xi_d
=
v_{cmd,d}
-
\omega L i_q
-
v_d
$$

类似：

$$
\xi_q
=
v_{cmd,q}
+
\omega L i_d
-
v_q
$$

这可以减少频域 Newton 初期的不匹配。

---

# 11. 更高效的 warm start 策略

## 11.1 零谐波初值不是最优

最简单初值是：

```text
基波 = PF 结果
其他谐波 = 0
```

但对于存在背景谐波或强谐波源的系统，Newton 可能多迭代几步。

更优策略是先做一次**线性谐波预解**。

---

## 11.2 线性预解初始化

对每个谐波频率，先忽略资源非线性和 AC/DC 耦合，只求：

$$
\mathbf{Y}_{h}\mathbf{V}_{h}^{(0)}
=
\mathbf{I}_{h}^{src}
$$

得到：

$$
\mathbf{V}_{h}^{(0)}
$$

再将该解作为 HPF Newton 初值。

流程：

```text
1. 从基波 PF 得到 I1
2. 根据 spectrum 生成 Ih
3. 对每个 h 解 Yh Vh = Ih
4. 用 Vh 初始化 HPF
```

这比全部非基波置零好很多。

---

## 11.3 AC/DC 耦合预解

如果 NIC 耦合明显，可以用线性耦合预解：

$$
\begin{bmatrix}
\mathbf{Y}_{ac,h} & -\mathbf{K}_{ad,hr} \\
-\mathbf{K}_{da,rh} & \mathbf{Y}_{dc,r}
\end{bmatrix}
\begin{bmatrix}
\mathbf{V}_{ac,h}^{(0)} \\
\mathbf{V}_{dc,r}^{(0)}
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{I}_{ac,h}^{src} \\
\mathbf{I}_{dc,r}^{src}
\end{bmatrix}
$$

这样得到的初值更接近最终 HPF 解。

---

# 12. 避免重复计算的缓存机制

高效映射不仅是数学初始化，还包括工程实现效率。

建议缓存以下内容：

## 12.1 频率相关网络矩阵缓存

```cpp
struct HarmonicNetworkCache {
    std::unordered_map<int, SparseMatrix> Yac_by_h;
    std::unordered_map<int, SparseMatrix> Ydc_by_r;

    std::unordered_map<int, SparseFactorization> factor_ac_by_h;
    std::unordered_map<int, SparseFactorization> factor_dc_by_r;

    size_t topology_build_id;
    size_t device_state_id;
};
```

只要拓扑、电容器状态、开关状态不变：

- 稀疏模式不变；
- 很多频率下的矩阵模式不变；
- 可以复用 symbolic factorization。

---

## 12.2 Operating point 缓存

```cpp
struct OperatingPointCache {
    PowerFlowResult base;
    HarmonicOperatingPoint hop;

    size_t pf_solution_id;
    size_t system_state_id;
};
```

当负荷、发电、converter setpoint 小幅变化时，可以：

- 复用上一次 harmonic state；
- 更新基波 operating point；
- 增量更新谐波源幅值。

---

## 12.3 Spectrum 注入缓存

```cpp
struct HarmonicInjectionCache {
    std::unordered_map<DeviceId, ComplexVector> ac_harmonic_currents;
    std::unordered_map<DeviceId, ComplexVector> dc_ripple_currents;
};
```

如果 spectrum 不变，只需要按基波电流幅值缩放。

---

# 13. 建议的映射函数设计

建议实现一个专门函数：

```cpp
HarmonicInitialState build_harmonic_initial_state(
    const HybridPowerSystem& system,
    const PowerFlowResult& base,
    const HarmonicPowerFlowOptions& opt
);
```

其内部步骤：

```text
1. build_ac_phase_node_map
2. build_dc_node_map
3. extract_ac_fundamental_voltage
4. extract_ac_fundamental_current
5. extract_dc_steady_voltage_current
6. extract_converter_operating_points
7. generate_harmonic_injections
8. optionally solve linear harmonic preflow
9. pack state vector for selected HPF formulation
```

---

# 14. 伪代码：完整映射流程

````markdown
```cpp
HarmonicInitialState build_harmonic_initial_state(
    const HybridPowerSystem& sys,
    const PowerFlowResult& pf,
    const HarmonicPowerFlowOptions& opt)
{
    HarmonicInitialState init;

    // 1. Build index maps
    auto ac_map = build_ac_harmonic_node_map(sys, opt.frequency_set);
    auto dc_map = build_dc_harmonic_node_map(sys, opt.frequency_set);

    // 2. Extract operating point
    HarmonicOperatingPoint op;
    op.Vac1 = extract_ac_voltage_phasors(sys, pf, ac_map);
    op.Vdc0 = extract_dc_voltages(sys, pf, dc_map);

    op.Iac_inj1 = compute_ac_nodal_injection_currents(sys, pf, op.Vac1);
    op.Idc_inj0 = compute_dc_nodal_injection_currents(sys, pf, op.Vdc0);

    op.converters = extract_converter_operating_points(sys, pf, op);

    // 3. Initialize full harmonic voltage arrays
    init.Vac_hat = zero_complex_vector(ac_map.total_size());
    init.Vdc_hat = zero_complex_vector(dc_map.total_size());

    for (auto node : ac_map.nodes) {
        init.Vac_hat[ac_map.index(node, 1)] = op.Vac1[node.local_index];

        for (int h : opt.frequency_set.ac_orders) {
            if (h == 1) continue;

            if (has_background_ac_harmonic(sys, node, h)) {
                init.Vac_hat[ac_map.index(node, h)] =
                    get_background_ac_harmonic(sys, node, h, op);
            }
        }
    }

    for (auto node : dc_map.nodes) {
        init.Vdc_hat[dc_map.index(node, 0)] = op.Vdc0[node.local_index];

        for (int r : opt.frequency_set.dc_orders) {
            if (r == 0) continue;

            if (has_background_dc_ripple(sys, node, r)) {
                init.Vdc_hat[dc_map.index(node, r)] =
                    get_background_dc_ripple(sys, node, r, op);
            }
        }
    }

    // 4. Generate harmonic injections from spectra
    init.Iac_src_hat = generate_ac_harmonic_current_sources(sys, op, ac_map, opt);
    init.Idc_src_hat = generate_dc_ripple_current_sources(sys, op, dc_map, opt);

    // 5. Optional linear warm start
    if (opt.initialization == HarmonicInitializationMode::LinearPreSolve) {
        auto Yac_hat = build_ac_multifrequency_ybus(sys, ac_map, op, opt);
        auto Ydc_hat = build_dc_multifrequency_ybus(sys, dc_map, op, opt);

        solve_linear_harmonic_warm_start(
            Yac_hat, Ydc_hat,
            init.Iac_src_hat, init.Idc_src_hat,
            init.Vac_hat, init.Vdc_hat,
            opt
        );
    }

    // 6. Pack according to solver formulation
    if (opt.formulation == HarmonicFormulation::DirectNodalVoltage) {
        init.x = pack_direct_voltage_state(init.Vac_hat, init.Vdc_hat);
    } else if (opt.formulation == HarmonicFormulation::HybridParameter) {
        auto partition = classify_harmonic_nodes(sys, op);
        init.x = pack_hybrid_state(
            init.Vac_hat,
            init.Vdc_hat,
            op.Iac_inj1,
            op.Idc_inj0,
            init.Iac_src_hat,
            init.Idc_src_hat,
            partition,
            ac_map,
            dc_map
        );
    }

    init.operating_point = op;
    return init;
}
```
````

---

# 15. 直接节点电压法的初值打包

如果状态是：

$$
x=
\begin{bmatrix}
\hat{V}^{AC} \\
\hat{V}^{DC}
\end{bmatrix}
$$

则直接：

```cpp
x = concatenate(Vac_hat, Vdc_hat);
```

如果使用实数展开：

$$
x_{real}
=
\begin{bmatrix}
\Re(x) \\
\Im(x)
\end{bmatrix}
$$

代码：

```cpp
Vector pack_real_expanded(const ComplexVector& z) {
    Vector x(2 * z.size());
    for (int i = 0; i < z.size(); ++i) {
        x[i] = real(z[i]);
        x[i + z.size()] = imag(z[i]);
    }
    return x;
}
```

---

# 16. Hybrid 参数法的初值打包

对于状态：

$$
\hat{x}
=
\begin{bmatrix}
\hat{I}^{AC}_{S} \\
\hat{V}^{AC}_{R} \\
\hat{I}^{DC}_{S} \\
\hat{V}^{DC}_{R}
\end{bmatrix}
$$

打包规则：

```cpp
for s in S_AC:
    x.push(Iac_S_hat[s])

for r in R_AC:
    x.push(Vac_R_hat[r])

for s in S_DC:
    x.push(Idc_S_hat[s])

for r in R_DC:
    x.push(Vdc_R_hat[r])
```

其中：

- $$I^{AC}_{S,h=1}$$ 来自基波 PF；
- $$I^{AC}_{S,h\ne1}$$ 来自 spectrum 或线性预解；
- $$V^{AC}_{R,h=1}$$ 来自基波 PF；
- $$V^{AC}_{R,h\ne1}$$ 来自背景谐波或线性预解；
- $$I^{DC}_{S,r=0}$$ 来自 DC PF；
- $$V^{DC}_{R,r=0}$$ 来自 DC PF。

---

# 17. 一致性校验非常重要

映射完成后，建议做以下校验。

## 17.1 基波功率一致性

检查：

$$
S_i^{recovered}
=
V_{i,1} I_{i,1}^*
$$

是否等于 PF 中的注入功率：

$$
S_i^{PF}
=
P_i^{PF}
+
jQ_i^{PF}
$$

误差：

$$
\epsilon_S
=
\max_i
|S_i^{recovered}-S_i^{PF}|
$$

---

## 17.2 DC 功率一致性

检查：

$$
P^{dc}_{k}
=
V^{dc}_{k,0} I^{dc}_{k,0}
$$

---

## 17.3 Converter 功率一致性

检查：

$$
P_{ac}
+
P_{dc}
+
P_{loss}
\approx
0
$$

---

## 17.4 谐波初值合理性

检查：

```text
非基波电压幅值不应明显超过基波
DC 纹波不应超过 Vdc0 的合理比例
谐波电流不应超过设备额定限制
```

---

# 18. 推荐的初始化模式

建议提供四种初始化模式：

```cpp
enum class HarmonicInitializationMode {
    ZeroHarmonics,
    BackgroundOnly,
    SourceSpectrum,
    LinearPreSolve,
    CoupledLinearPreSolve
};
```

## 18.1 ZeroHarmonics

```text
基波/直流稳态来自 PF
其他频率全部置零
```

最快，但收敛性一般。

---

## 18.2 BackgroundOnly

```text
基波来自 PF
外部电网背景谐波给定
其他源置零
```

适合评估上级系统谐波传播。

---

## 18.3 SourceSpectrum

```text
基波来自 PF
根据设备 spectrum 生成谐波电流源
电压初值仍为 0 或背景值
```

简单且常用。

---

## 18.4 LinearPreSolve

```text
先解一次 Yh Vh = Ih
用该 Vh 作为 HPF 初值
```

推荐默认。

---

## 18.5 CoupledLinearPreSolve

```text
考虑 AC/DC converter 耦合块
联合预解 AC 谐波和 DC 纹波
```

适合混合 AC/DC 系统，尤其 NIC 较强时。

---

# 19. 性能优化重点

## 19.1 不要每次都重建索引

索引只与网络拓扑和频率集合有关：

```cpp
HarmonicIndex build once;
```

---

## 19.2 不要显式形成所有大块矩阵

若是多频率块对角结构，可以按频率分别存储：

```cpp
Yac_by_h[h]
Ydc_by_r[r]
```

只有在做耦合 Newton 或 Toeplitz 模型时才形成大矩阵。

---

## 19.3 使用 symbolic factorization 复用

不同谐波频率下：

- 矩阵数值不同；
- 稀疏结构通常相同。

因此可复用 symbolic pattern：

```cpp
analyzePattern(Y_pattern);
factorize(Y_numeric_h);
```

---

## 19.4 Converter operating point 不要重复计算

converter 的：

- $$I_1$$；
- $$P,Q$$；
- $$V_{dc}$$；
- $$m_0,\delta_0$$；
- dq 稳态值；

都应在 `HarmonicOperatingPoint` 中缓存。

---

# 20. 最推荐的高效映射流程

综合来看，建议你实现如下默认流程：

```text
1. 调用已有统一 AC/DC PF
2. 构造 HarmonicOperatingPoint
3. 提取 AC 基波相量和 DC 稳态电压
4. 提取 converter operating points
5. 根据 spectrum 生成 AC 谐波源和 DC 纹波源
6. 构造频率相关 Yac,h 和 Ydc,r
7. 做一次 linear pre-solve
8. 将结果打包到 HPF 初始状态
9. 进入 unified HPF Newton
```

即：

```text
PF result
  → operating point
  → spectrum injections
  → linear harmonic pre-solve
  → HPF Newton warm start
```

这是效率和收敛性之间最好的折中。

---

# 21. 最终建议

如果你已经有统一迭代 AC/DC 潮流模块，那么不要把 HPF 初始化写成简单的“全部谐波置零”。更好的工程方案是：

> 以基波潮流结果作为 operating point，先生成设备谐波注入，再做一次线性谐波预求解，最后把该结果打包成论文 HPF 所需的 hybrid state。

最核心的映射公式是：

## AC 基波电压

$$
V_{i,1}
=
v_i e^{j\theta_i}
$$

## AC 基波注入电流

$$
I_{i,1}
=
\frac{S_{i,1}^*}{V_{i,1}^*}
$$

## DC 稳态电流

$$
I^{dc}_{k,0}
=
\frac{P^{dc}_{k,0}}{V^{dc}_{k,0}}
$$

## 谐波源缩放

$$
I_{h}
=
\frac{M_h}{100}
|I_1|
e^{j\theta_h}
$$

## 线性预解

$$
\mathbf{Y}_{h}
\mathbf{V}_{h}^{(0)}
=
\mathbf{I}_{h}^{src}
$$

## Coupled warm start

$$
\begin{bmatrix}
\mathbf{Y}_{ac,h} & -\mathbf{K}_{ad,hr} \\
-\mathbf{K}_{da,rh} & \mathbf{Y}_{dc,r}
\end{bmatrix}
\begin{bmatrix}
\mathbf{V}_{ac,h}^{(0)} \\
\mathbf{V}_{dc,r}^{(0)}
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{I}_{ac,h}^{src} \\
\mathbf{I}_{dc,r}^{src}
\end{bmatrix}
$$

这样可以显著减少谐波 Newton 迭代次数，并且能保证谐波模型与现有 AC/DC 基波潮流模块在功率、符号、控制模式和设备状态上保持一致。
