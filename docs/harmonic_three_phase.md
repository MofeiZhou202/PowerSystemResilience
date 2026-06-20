# 如何将混合 AC/DC 谐波潮流模型扩展到三相？

要把前面的**混合 AC/DC 谐波潮流模型**扩展到三相，本质上是把 AC 子系统从“单相正序等值”扩展为：

```text
三相相域 phase-domain AC 网络
+
DC 网络
+
三相 AC/DC 双端口 NIC
+
多频率谐波耦合
```

核心变化是：

> AC 侧所有节点电压、电流、导纳矩阵、资源响应和换流器端口量都要按相别展开为 $$a,b,c$$ 三个相节点；DC 侧仍然是单极 / 双极节点；NIC 成为“三相 AC 端口 + DC 端口”的多端口资源。

---

# 1. 三相扩展后的总体状态

原来单相 AC/DC 频域状态可以写成：

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

三相后，AC 部分改成相节点集合：

$$
\mathcal{N}_{ac}^{\phi}
=
\left\{
(i,\phi)
\mid
i\in\mathcal{N}_{ac},
\phi\in\{a,b,c\}
\right\}
$$

对每个相节点：

$$
\hat{V}^{AC}_{i,\phi}
=
\begin{bmatrix}
V^{AC}_{i,\phi,h_1} \\
V^{AC}_{i,\phi,h_2} \\
\vdots \\
V^{AC}_{i,\phi,h_{n_h}}
\end{bmatrix}
$$

因此三相 HPF 状态为：

$$
\hat{x}_{3\phi}
=
\begin{bmatrix}
\hat{\mathbf{I}}^{AC,abc}_{S} \\
\hat{\mathbf{V}}^{AC,abc}_{R} \\
\hat{\mathbf{I}}^{DC}_{S} \\
\hat{\mathbf{V}}^{DC}_{R}
\end{bmatrix}
$$

其中：

$$
\hat{\mathbf{V}}^{AC,abc}
=
\begin{bmatrix}
\hat{V}_{1,a} \\
\hat{V}_{1,b} \\
\hat{V}_{1,c} \\
\hat{V}_{2,a} \\
\hat{V}_{2,b} \\
\hat{V}_{2,c} \\
\vdots
\end{bmatrix}
$$

---

# 2. 三相 AC 网络建模

## 2.1 三相节点导纳矩阵

单相模型中：

$$
\mathbf{I}^{AC}_{h}
=
\mathbf{Y}^{AC}_{h}
\mathbf{V}^{AC}_{h}
$$

三相后变为：

$$
\mathbf{I}^{AC,abc}_{h}
=
\mathbf{Y}^{AC,abc}_{h}
\mathbf{V}^{AC,abc}_{h}
$$

其中 $$\mathbf{Y}^{AC,abc}_{h}$$ 的维度为：

$$
3N_{ac}
\times
3N_{ac}
$$

若考虑缺相线路，则维度为实际相节点数：

$$
N_{phase}
\times
N_{phase}
$$

---

## 2.2 三相线路阻抗矩阵

对线路 $$i-j$$，每单位长度相阻抗为：

$$
\mathbf{Z}_{abc,h}
=
\begin{bmatrix}
Z_{aa,h} & Z_{ab,h} & Z_{ac,h} \\
Z_{ba,h} & Z_{bb,h} & Z_{bc,h} \\
Z_{ca,h} & Z_{cb,h} & Z_{cc,h}
\end{bmatrix}
$$

支路导纳：

$$
\mathbf{Y}_{series,h}
=
\mathbf{Z}_{abc,h}^{-1}
$$

相间电容 / 对地电容形成 shunt 导纳：

$$
\mathbf{Y}_{sh,h}
=
j2\pi h f_1
\mathbf{C}_{abc}
$$

装配到 Ybus：

$$
\mathbf{Y}_{ii,h}
\mathrel{+}=
\mathbf{Y}_{series,h}
+
\frac{1}{2}\mathbf{Y}_{sh,h}
$$

$$
\mathbf{Y}_{jj,h}
\mathrel{+}=
\mathbf{Y}_{series,h}
+
\frac{1}{2}\mathbf{Y}_{sh,h}
$$

$$
\mathbf{Y}_{ij,h}
\mathrel{-}=
\mathbf{Y}_{series,h}
$$

$$
\mathbf{Y}_{ji,h}
\mathrel{-}=
\mathbf{Y}_{series,h}
$$

---

## 2.3 频率相关阻抗

三相下仍然要考虑频率相关：

$$
\mathbf{Z}_{abc,h}
=
\mathbf{R}_{abc,h}
+
jh\mathbf{X}_{abc,1}
$$

可采用简化皮肤效应：

$$
\mathbf{R}_{abc,h}
=
\mathbf{R}_{abc,1}\sqrt{h}
$$

或：

$$
\mathbf{R}_{abc,h}
=
\mathbf{R}_{abc,1}
\left(
1+k_{skin}\sqrt{h}
\right)
$$

---

# 3. 三相谐波频率与相序

三相谐波的相序很重要。

对于平衡三相谐波：

| 谐波次数 | 相序 |
|---|---|
| $$h=3k+1$$ | 正序 |
| $$h=3k+2$$ | 负序 |
| $$h=3k$$ | 零序 |

即：

```text
1, 4, 7, 10, 13, ... 正序
2, 5, 8, 11, 14, ... 负序
3, 6, 9, 12, 15, ... 零序
```

注意常见电力系统低次谐波中：

| 谐波 | 相序 |
|---|---|
| 5次 | 负序 |
| 7次 | 正序 |
| 11次 | 负序 |
| 13次 | 正序 |

---

## 3.1 平衡三相谐波源相角生成

若 A 相谐波电流为：

$$
I_{a,h}
=
|I_h|e^{j\theta_h}
$$

则：

### 正序谐波 $$h=3k+1$$

$$
I_{b,h}
=
I_{a,h}e^{-j\frac{2\pi}{3}}
$$

$$
I_{c,h}
=
I_{a,h}e^{j\frac{2\pi}{3}}
$$

---

### 负序谐波 $$h=3k+2$$

$$
I_{b,h}
=
I_{a,h}e^{j\frac{2\pi}{3}}
$$

$$
I_{c,h}
=
I_{a,h}e^{-j\frac{2\pi}{3}}
$$

---

### 零序谐波 $$h=3k$$

$$
I_{a,h}
=
I_{b,h}
=
I_{c,h}
$$

---

# 4. 三相下 NIC 模型如何扩展？

单相等值 NIC 响应为：

$$
\begin{bmatrix}
\hat{I}^{AC}_{r} \\
\hat{V}^{DC}_{s}
\end{bmatrix}
=
\hat{F}_{NIC}
\left(
\hat{V}^{AC}_{r},
\hat{I}^{DC}_{s}
\right)
$$

三相后，AC 端口电压和电流变成三相向量：

$$
\hat{\mathbf{V}}^{AC,abc}_{r}
=
\begin{bmatrix}
\hat{V}^{AC}_{r,a} \\
\hat{V}^{AC}_{r,b} \\
\hat{V}^{AC}_{r,c}
\end{bmatrix}
$$

$$
\hat{\mathbf{I}}^{AC,abc}_{r}
=
\begin{bmatrix}
\hat{I}^{AC}_{r,a} \\
\hat{I}^{AC}_{r,b} \\
\hat{I}^{AC}_{r,c}
\end{bmatrix}
$$

典型 AC grid-following / DC grid-forming NIC：

$$
\begin{bmatrix}
\hat{\mathbf{I}}^{AC,abc}_{r} \\
\hat{V}^{DC}_{s}
\end{bmatrix}
=
\hat{\mathbf{F}}_{NIC,3\phi}
\left(
\hat{\mathbf{V}}^{AC,abc}_{r},
\hat{I}^{DC}_{s}
\right)
$$

其中：

- 输入：三相 AC 端口电压 + DC 端口电流；
- 输出：三相 AC 端口电流 + DC 端口电压。

---

# 5. 三相 NIC 初始值分配

对于典型 NIC：

```text
AC 侧：grid-following
DC 侧：grid-forming
```

三相 hybrid HPF 状态中放入：

$$
\hat{x}_{NIC,3\phi}
=
\begin{bmatrix}
\hat{\mathbf{V}}^{AC,abc}_{r} \\
\hat{I}^{DC}_{s}
\end{bmatrix}
$$

即：

```text
AC 端口放三相电压
DC 端口放直流电流
```

---

## 5.1 AC 端口基波三相电压

如果基波 PF 是三相结果，直接取：

$$
V_{a,1}^{PF},V_{b,1}^{PF},V_{c,1}^{PF}
$$

如果基波 PF 是单相正序结果，则展开为：

$$
V_{a,1}
=
V_1e^{j\theta}
$$

$$
V_{b,1}
=
V_1e^{j(\theta-2\pi/3)}
$$

$$
V_{c,1}
=
V_1e^{j(\theta+2\pi/3)}
$$

---

## 5.2 AC 端口基波三相电流

若 NIC 给出三相总功率：

$$
S^{AC}_{NIC}
=
P^{AC}_{NIC}
+
jQ^{AC}_{NIC}
$$

平衡分配：

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

即：

$$
I_{a,1}
=
\frac{
\left(S^{AC}_{NIC}/3\right)^*
}{
V_{a,1}^*
}
$$

$$
I_{b,1}
=
\frac{
\left(S^{AC}_{NIC}/3\right)^*
}{
V_{b,1}^*
}
$$

$$
I_{c,1}
=
\frac{
\left(S^{AC}_{NIC}/3\right)^*
}{
V_{c,1}^*
}
$$

如果 NIC 支持不平衡控制，则每相功率独立：

$$
I_{\phi,1}
=
\frac{
\left(P_{\phi}+jQ_{\phi}\right)^*
}{
V_{\phi,1}^*
}
$$

---

## 5.3 DC 端口电流

DC 端口仍为：

$$
I^{DC}_{0}
=
\frac{P^{DC}_{NIC}}{V^{DC}_{0}}
$$

三相 AC 侧总有功与 DC 侧平衡：

$$
P^{AC}_{a}
+
P^{AC}_{b}
+
P^{AC}_{c}
+
P^{DC}
+
P^{loss}
=
0
$$

也就是：

$$
P^{AC}_{3\phi}
+
P^{DC}
+
P^{loss}
=
0
$$

---

# 6. 三相下的 converter 功率平衡

对第 $$m$$ 个三相 NIC：

$$
P^{AC}_{m,3\phi}
+
P^{DC}_{m}
+
P^{loss}_{m}
=
0
$$

其中：

$$
P^{AC}_{m,3\phi}
=
\sum_{\phi \in \{a,b,c\}}
\Re
\left\{
V_{m,\phi,1}
I_{m,\phi,1}^{*}
\right\}
$$

无功：

$$
Q^{AC}_{m,3\phi}
=
\sum_{\phi \in \{a,b,c\}}
\Im
\left\{
V_{m,\phi,1}
I_{m,\phi,1}^{*}
\right\}
$$

DC 侧：

$$
P^{DC}_{m}
=
V^{DC}_{m,0} I^{DC}_{m,0}
$$

因此：

$$
\boxed{
\sum_{\phi}
\Re
\left\{
V_{m,\phi,1}
I_{m,\phi,1}^{*}
\right\}
+
V^{DC}_{m,0} I^{DC}_{m,0}
+
P^{loss}_{m}
=
0
}
$$

---

# 7. 三相下的 AC/DC 耦合 Jacobian

单相 NIC 耦合 Jacobian 为：

$$
\frac{\partial \hat{I}^{AC}}
{\partial \hat{V}^{AC}}
$$

$$
\frac{\partial \hat{I}^{AC}}
{\partial \hat{I}^{DC}}
$$

$$
\frac{\partial \hat{V}^{DC}}
{\partial \hat{V}^{AC}}
$$

$$
\frac{\partial \hat{V}^{DC}}
{\partial \hat{I}^{DC}}
$$

三相后变为：

$$
\frac{\partial \hat{\mathbf{I}}^{AC,abc}}
{\partial \hat{\mathbf{V}}^{AC,abc}}
$$

维度为：

$$
3n_h \times 3n_h
$$

$$
\frac{\partial \hat{\mathbf{I}}^{AC,abc}}
{\partial \hat{I}^{DC}}
$$

维度为：

$$
3n_h \times n_r
$$

$$
\frac{\partial \hat{V}^{DC}}
{\partial \hat{\mathbf{V}}^{AC,abc}}
$$

维度为：

$$
n_r \times 3n_h
$$

$$
\frac{\partial \hat{V}^{DC}}
{\partial \hat{I}^{DC}}
$$

维度为：

$$
n_r \times n_r
$$

因此 NIC 的局部 Jacobian 块为：

$$
\mathbf{J}_{NIC}
=
\begin{bmatrix}
\frac{\partial \hat{\mathbf{I}}^{AC,abc}}{\partial \hat{\mathbf{V}}^{AC,abc}}
&
\frac{\partial \hat{\mathbf{I}}^{AC,abc}}{\partial \hat{I}^{DC}}
\\
\frac{\partial \hat{V}^{DC}}{\partial \hat{\mathbf{V}}^{AC,abc}}
&
\frac{\partial \hat{V}^{DC}}{\partial \hat{I}^{DC}}
\end{bmatrix}
$$

---

# 8. 三相 Norton NIC 模型

工程上可以先实现三相 Norton 形式：

$$
\hat{\mathbf{I}}^{AC,abc}
=
\hat{\mathbf{I}}^{src,abc}_{ac}
-
\hat{\mathbf{Y}}^{out,abc}_{ac}
\hat{\mathbf{V}}^{AC,abc}
+
\hat{\mathbf{K}}_{ad}^{abc}
\hat{I}^{DC}
$$

$$
\hat{V}^{DC}
=
\hat{V}^{src}_{dc}
-
\hat{Z}^{out}_{dc}
\hat{I}^{DC}
+
\hat{\mathbf{K}}_{da}^{abc}
\hat{\mathbf{V}}^{AC,abc}
$$

其中：

$$
\hat{\mathbf{Y}}^{out,abc}_{ac}
$$

可以是：

- 三相对角；
- 三相耦合矩阵；
- LCL 滤波器矩阵；
- 正负零序变换后的阻抗再变回 abc。

---

# 9. 三相 load / DER / CIDER 模型

## 9.1 三相负荷

每相独立：

$$
I_{\phi,h}
=
Y_{\phi,h}V_{\phi,h}
+
I_{\phi,h}^{src}
$$

对不平衡负荷，每相功率不同：

$$
S_{\phi}
=
P_{\phi}
+
jQ_{\phi}
$$

基波电流：

$$
I_{\phi,1}
=
\frac{S_{\phi}^*}{V_{\phi,1}^*}
$$

---

## 9.2 Delta 连接负荷

若负荷为 delta 连接，应先计算线电压：

$$
V_{ab}=V_a-V_b
$$

$$
V_{bc}=V_b-V_c
$$

$$
V_{ca}=V_c-V_a
$$

支路电流：

$$
I_{ab}
=
Y_{ab}V_{ab}
$$

节点注入：

$$
I_a
=
I_{ab}-I_{ca}
$$

$$
I_b
=
I_{bc}-I_{ab}
$$

$$
I_c
=
I_{ca}-I_{bc}
$$

谐波下对每个频率重复。

---

## 9.3 单相 DER / 单相充电桩

单相设备只接入某一相：

```text
PV on phase a
EV charger on phase b
single-phase load on phase c
```

其谐波注入只装配到对应相节点。

---

# 10. 三相频域网络矩阵结构

多频率三相网络可以写成：

$$
\hat{\mathbf{Y}}^{AC,abc}
=
\operatorname{blkdiag}
\left(
\mathbf{Y}^{abc}_{h_1},
\mathbf{Y}^{abc}_{h_2},
\ldots,
\mathbf{Y}^{abc}_{h_n}
\right)
$$

如果考虑频率耦合，则有非对角块：

$$
\hat{\mathbf{Y}}^{AC,abc}
=
\begin{bmatrix}
\mathbf{Y}_{h_1h_1} & \mathbf{Y}_{h_1h_2} & \cdots \\
\mathbf{Y}_{h_2h_1} & \mathbf{Y}_{h_2h_2} & \cdots \\
\vdots & \vdots & \ddots
\end{bmatrix}
$$

对于网络本身通常是块对角，频率耦合主要来自：

- VSC 控制；
- PLL；
- PWM；
- DC-link；
- 非线性负荷；
- LTP 模型。

---

# 11. 三相 hybrid 参数矩阵

三相网络仍可构造论文中的 hybrid matrix：

$$
\begin{bmatrix}
\hat{\mathbf{V}}^{abc}_{S} \\
\hat{\mathbf{I}}^{abc}_{R}
\end{bmatrix}
=
\begin{bmatrix}
\hat{\mathbf{H}}^{abc}_{SS} & \hat{\mathbf{H}}^{abc}_{SR} \\
\hat{\mathbf{H}}^{abc}_{RS} & \hat{\mathbf{H}}^{abc}_{RR}
\end{bmatrix}
\begin{bmatrix}
\hat{\mathbf{I}}^{abc}_{S} \\
\hat{\mathbf{V}}^{abc}_{R}
\end{bmatrix}
$$

只不过这里的 $$S,R$$ 是**相节点集合**，而不是普通母线集合。

---

# 12. 数据结构建议

## 12.1 相节点索引

```cpp
enum class Phase { A, B, C, N };

struct ACPhaseNode {
    int bus_id;
    Phase phase;
};

struct ThreePhaseHarmonicIndex {
    std::vector<ACPhaseNode> ac_phase_nodes;
    std::vector<int> ac_orders;
    std::vector<int> dc_orders;

    int ac_voltage_index(int bus, Phase ph, int h) const;
    int ac_current_index(int bus, Phase ph, int h) const;
    int dc_voltage_index(int bus, int r) const;
    int dc_current_index(int bus, int r) const;
};
```

---

## 12.2 三相 NIC operating point

```cpp
struct ThreePhaseNICOperatingPoint {
    int ac_bus;
    int dc_bus;

    Complex Va1, Vb1, Vc1;
    Complex Ia1, Ib1, Ic1;

    double Vdc0;
    double Idc0;

    double Pa, Pb, Pc;
    double Qa, Qb, Qc;

    double Pac_total;
    double Qac_total;
    double Pdc;
    double Ploss;

    ConverterControlMode control_mode;
};
```

---

# 13. 三相初始化伪代码

````markdown
```cpp
void initialize_three_phase_nic(
    const NIC& nic,
    const PowerFlowResult& pf,
    const ThreePhaseHarmonicIndex& hidx,
    HarmonicInitialState& init)
{
    int ac_bus = nic.ac_bus;
    int dc_bus = nic.dc_bus;

    // 1. AC three-phase voltages
    Complex Va = get_phase_voltage(pf, ac_bus, Phase::A);
    Complex Vb = get_phase_voltage(pf, ac_bus, Phase::B);
    Complex Vc = get_phase_voltage(pf, ac_bus, Phase::C);

    double Vdc = get_dc_voltage(pf, dc_bus);

    // 2. AC power allocation
    Complex Sa, Sb, Sc;

    if (nic.has_phase_power_setpoints()) {
        Sa = Complex(nic.Pa, nic.Qa);
        Sb = Complex(nic.Pb, nic.Qb);
        Sc = Complex(nic.Pc, nic.Qc);
    } else {
        Complex Stotal(nic.Pac, nic.Qac);
        Sa = Stotal / 3.0;
        Sb = Stotal / 3.0;
        Sc = Stotal / 3.0;
    }

    // 3. Compute phase currents
    Complex Ia = std::conj(Sa) / std::conj(Va);
    Complex Ib = std::conj(Sb) / std::conj(Vb);
    Complex Ic = std::conj(Sc) / std::conj(Vc);

    // 4. Enforce converter power balance
    double Pac = real(Va * std::conj(Ia)
                    + Vb * std::conj(Ib)
                    + Vc * std::conj(Ic));

    double Ploss = compute_converter_loss(nic, Pac, Vdc);

    double Pdc = -Pac - Ploss;
    double Idc = Pdc / std::max(Vdc, nic.min_vdc);

    // 5. Initialize AC voltage harmonics at NIC AC port
    for (int h : hidx.ac_orders) {
        for (Phase ph : {Phase::A, Phase::B, Phase::C}) {
            Complex Vh = 0.0;

            if (h == 1) {
                Vh = (ph == Phase::A) ? Va :
                     (ph == Phase::B) ? Vb : Vc;
            } else if (init.has_linear_presolve_voltage(ac_bus, ph, h)) {
                Vh = init.get_linear_presolve_voltage(ac_bus, ph, h);
            }

            init.Vac_hat[hidx.ac_voltage_index(ac_bus, ph, h)] = Vh;
        }
    }

    // 6. Initialize DC current harmonics at NIC DC port
    for (int r : hidx.dc_orders) {
        Complex Ir = 0.0;

        if (r == 0) {
            Ir = Complex(Idc, 0.0);
        } else if (init.has_linear_presolve_dc_current(dc_bus, r)) {
            Ir = init.get_linear_presolve_dc_current(dc_bus, r);
        } else if (nic.has_dc_ripple_spectrum(r)) {
            Ir = nic.dc_ripple_current(r, Idc);
        }

        init.Idc_hat[hidx.dc_current_index(dc_bus, r)] = Ir;
    }

    // 7. Pack for hybrid HPF
    if (nic.ac_port_is_grid_following()) {
        for (int h : hidx.ac_orders) {
            for (Phase ph : {Phase::A, Phase::B, Phase::C}) {
                init.x[hidx.state_index_Vac_R(ac_bus, ph, h)] =
                    init.Vac_hat[hidx.ac_voltage_index(ac_bus, ph, h)];
            }
        }
    }

    if (nic.dc_port_is_grid_forming()) {
        for (int r : hidx.dc_orders) {
            init.x[hidx.state_index_Idc_S(dc_bus, r)] =
                init.Idc_hat[hidx.dc_current_index(dc_bus, r)];
        }
    }
}
```
````

---

# 14. 三相扩展的推荐实现路线

你可以按以下顺序实现：

## Level 1：三相相域网络 + 谐波电流源

```text
Yabc,h Vabc,h = Iabc,h
```

支持：

- 三相不平衡线路；
- 单相负荷；
- 三相负荷；
- 三相谐波源；
- THD per phase。

---

## Level 2：三相 VSC/NIC Norton 模型

```text
Iabc,h = Isrc,abc,h - Yout,abc,h Vabc,h
```

支持：

- 三相输出阻抗；
- LCL 滤波器；
- 正负序不同控制响应。

---

## Level 3：三相 AC/DC 耦合

```text
[Iabc_ac, Vdc] = F_NIC(Vabc_ac, Idc)
```

支持：

- AC 不平衡引起 DC 二倍频纹波；
- DC 纹波引起 AC 侧边带；
- 多 NIC 耦合。

---

## Level 4：dq / sequence / abc 混合控制模型

对于三相 VSC 控制，内部常用 dq 模型，但网络是 abc 模型。

需要增加变换：

$$
abc \leftrightarrow dq0
$$

以及正负序分解。

---

# 15. 最关键的总结

将模型扩展到三相，需要做四件事：

## 1. AC 网络从母线展开到相节点

$$
i
\rightarrow
(i,a),(i,b),(i,c)
$$

并建立：

$$
\mathbf{Y}^{abc}_{h}
$$

---

## 2. NIC 的 AC 端口从标量变为三相向量

$$
\hat{V}^{AC}
\rightarrow
\hat{\mathbf{V}}^{AC,abc}
$$

$$
\hat{I}^{AC}
\rightarrow
\hat{\mathbf{I}}^{AC,abc}
$$

典型 NIC：

$$
\boxed{
\begin{bmatrix}
\hat{\mathbf{I}}^{AC,abc} \\
\hat{V}^{DC}
\end{bmatrix}
=
\hat{\mathbf{F}}_{NIC,3\phi}
\left(
\hat{\mathbf{V}}^{AC,abc},
\hat{I}^{DC}
\right)
}
$$

---

## 3. 三相功率平衡使用三相总功率

$$
\boxed{
\sum_{\phi=a,b,c}
\Re
\left\{
V_{\phi,1}I_{\phi,1}^{*}
\right\}
+
V^{DC}_{0}I^{DC}_{0}
+
P^{loss}
=
0
}
$$

---

## 4. 谐波源和初值按相序分配

$$
h=3k+1:
\text{正序}
$$

$$
h=3k+2:
\text{负序}
$$

$$
h=3k:
\text{零序}
$$

对于工程实现，最推荐的三相初始状态分配是：

```text
典型 NIC:
    AC grid-following port:
        pack V_ac,a/b/c,h into HPF state

    DC grid-forming port:
        pack I_dc,r into HPF state

    operating point:
        save I_ac,a/b/c,1 and V_dc,0 for resource response
```

这样可以自然兼容：

- 三相不平衡潮流；
- 单相 DER / EV / 负荷；
- 三相 VSC/NIC；
- AC 不平衡到 DC 二倍频纹波的耦合；
- DC 纹波到 AC 谐波的传播；
- 论文中的 single-port / two-port resource HPF 框架。