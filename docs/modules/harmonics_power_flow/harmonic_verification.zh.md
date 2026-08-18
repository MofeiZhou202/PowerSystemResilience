> 本文档为 [harmonic_verification.md](harmonic_verification.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

> 文档同步：2026-07-12。
> 范围：已对照当前仓库结构、CMake 预设/选项和注册测试目标复核。
> 状态：实现支撑的验证参考；求解器变化后必须重跑所列测试。
> 权威来源：文档与实现冲突时，以 `src/`、`include/`、`tests/` 和 CMake 文件为准。

# 谐波潮流正确性验证

本文记录谐波潮流（HPF）模块的正确性验证。实现位于
`src/harmonics_power_flow/harmonics_power_flow.cpp`，公共 API 位于
`include/hacdcpf/analysis/harmonics_power_flow.hpp`。历史理论材料已归档于
`docs/archive/theory/harmonics/`；当前模块的唯一活动文档目录为
`docs/modules/harmonics_power_flow/`。

## 摘要

求解器已对照设计文档与标准谐波潮流（HPF）理论进行审查，单元测试套件得到加强，
修复了一个真实 bug，并与两个独立的外部实现进行了交叉验证。**单相线性核心、
诺顿（Norton）电流注入约定、支路潮流恢复以及 THD/IHD 公式，现与一个独立的
numpy 参考实现和 OpenDSS 引擎在约 1e-13 pu 的精度内一致。**

## 当前 API 范围

当前公共谐波 API 比最初的 Level-1 线性穿透求解器更宽：

* 单相 / 正序混合 AC/DC HPF，具备自动 VSC 到 NIC 频谱转换、可选诺顿输出导纳、
  频率相关的 AC 电阻，以及可选的直流纹波支路/并联元件频率相关性。
* 三相 abc 域 HPF，具备正/负/零序路由、变压器零序/接线组别效应、
  不平衡分相源，以及分相 THD 报告。
* 完整 NIC 桥的耦合三相 AC + DC HPF，包括线性的 AC→DC 与 DC→AC 纹波/谐波
  耦合增益。
* 面向非线性谐波资源的 Newton 族：单相、实部/虚部恒功率、跨次频率混频、
  三相变体，以及带双线性 NIC 跨域耦合的堆叠混合 AC/DC Newton 求解。
* 后处理辅助工具：IEEE 519-2014 / GB/T 14549-1993 电压畸变校核、驱动点与
  序分量频率扫描、谐振检测、支路电流 THD/TDD、K 因子，以及谐波损耗指标。

## 1. 代码审查发现

对照文档中的方程对实现逐行审查后，确认基础求解器在数学上正确且与文档一致。
确认正确的条目（及其对应的锚定测试）：

* 各次阻抗 `Z(h)=r+j·h·x`、线路充电 `j·h·b`、并联元件 `g+j·h·b`、
  电源接地 `1/(r+j·h·x'')`、集肤效应 `R(h)` 模型。
* 电流注入"流入网络为正"；`from→to` 支路电流 `ys·(Vf−Vt)`。
* THD/IHD：`sqrt(Σ_{h≠fund}|V_h|²)/|V_fund|·100`。
* 三相按次数的序分量路由 `((h%3))` → 正/负/零序，以及对称分量的注入/读出。
* NIC 运行点 `I_ac,1 = conj(S_ac)/conj(V_ac,1)`、`I_dc,0 = P_dc/V_dc,0`。
* Newton 雅可比 `Ĵ = Ŷ − ∂Î_res/∂V̂`；针对非全纯恒功率负荷的实部/虚部
  （2N）形式；双线性频率混频导数。
* 频率扫描驱动点阻抗 `Z_dp=(Ŷ⁻¹)_kk`；K 因子、TDD、`I²R(h)` 损耗。

## 2. 已修复的 bug

**直流支路潮流电流忽略了纹波电抗。** 直流支路潮流循环按 `|(Vf−Vt)/r|`
（仅电阻）计算，而当设置了 `dc_ripple_model.branch_x_pu` 时网络求解使用
`Z(r)=r_pu+j·r·x_pu`，因此在较高纹波次数下报告的直流支路电流被高估。
已修复为使用与 `build_dc_ybus` 相同的随次数变化的阻抗
（`harmonics_power_flow.cpp`，直流支路潮流循环）。由一个新测试锚定：
在辐射状直流馈线中，基尔霍夫电流定律（KCL）强制支路电流精确等于注入的
纹波电流（1.0 pu）；修复前的公式返回 2.6 pu。

已审查但**不是** bug 的条目：三相平衡负序旋转是正确的；`ThreePhaseLoad`
没有 `scaling` 字段（因此不存在字段被丢弃的问题）。

## 3. 新增的测试覆盖

`tests/test_harmonics_power_flow.cpp` 从 45 个用例增加到 50 个（断言从 315
增至 344），补上了此前未测试的路径：

1. 带纹波电抗的直流支路潮流（锚定 §2 的修复）。
2. 三相负荷作为并联阻抗的路径（此前没有任何三相测试启用它）——
   解析正序值 + 严格衰减。
3. 由**收敛的基础潮流**推导的 NIC 运行点（`vsc_transfers` 路径；其余所有
   NIC 测试都是手动给定设定值），基于 `build_ieee14_acdc()`。
4. 单相 NIC 诺顿输出导纳 `y_out_ac` 装配（解析 `|V|=1.2` 对比 `1.5` 理想值）。
5. 基波电压被压低情况下的 THD（分母鲁棒性）。

## 4. 已记录的简化

已在头文件中标注以便用户知悉：负荷电导 `P/V²` 不做频率/集肤缩放；NIC 的
`P_dc = −P_ac` 无损假设（现也在采用该假设时写入 `HPFResult::message`）；
`solve_harmonic_power_flow_3ph` 仅覆盖 AC 网络并忽略 `auto_nic_from_vscs`。

## 5. 外部交叉验证

`tools/harmonics_validation/` 在一个规范的两母线算例（`case.json`，唯一事实
来源）上，针对两个独立代码库验证求解器：

| 参考实现 | 结果 |
|-----------|--------|
| 独立 numpy 节点求解器 | max \|Δ\| = **2.8e-17** pu |
| OpenDSS 引擎（OpenDSSDirect.py 0.9.4） | max \|Δ\| = **6.0e-13** pu |

此项工作中的一个值得注意的发现：OpenDSS 的 `Line` 元件默认带有频率相关 /
大地回路阻抗模型，因此在对等比较中将串联支路建模为 OpenDSS `Reactor`
（`Z(h)=r+j·h·x`），与求解器和文档保持一致。

复现方法：

```bash
cmake --build build_rel --target validate_harmonics_xref
cd tools/harmonics_validation
python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt
python compare.py        # -> OVERALL: PASS
```

## 6. 范围

已实现的求解器是文档所述的 **"Level 1" 直接节点穿透**方法，而非论文中的完整
混合参数 Newton 法；这是有意设计（见头文件序言）。Newton 族（非线性、非全纯
恒功率、跨次混频、混合 AC/DC 双线性）由单元测试套件的"退化为线性"测试与
解析不动点测试覆盖。
