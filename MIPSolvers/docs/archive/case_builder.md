# SCUC Case Builder — 接口文档

> 源文件：`include/mipsolvers/scuc/case_builder.hpp` / `src/scuc/case_builder.cpp`

---

## 目录

1. [概述](#1-概述)
2. [C++ 接口](#2-c-接口)
3. [3-bus 标准算例](#3-3-bus-标准算例)
4. [6-bus 标准算例](#4-6-bus-标准算例)
5. [IEEE 39-bus 标准算例](#5-ieee-39-bus-标准算例)
6. [JSON 序列化](#6-json-序列化)
7. [性能基准](#7-性能基准)

---

## 1. 概述

`case_builder` 提供三个预构建的 `SCUCInput` 对象，用于单元测试、集成测试和性能基准。
所有算例均包含完整的母线拓扑、支路参数、发电机报价、负荷曲线和时序曲线，可直接传入 `scuc_solve()`。

```cpp
#include "mipsolvers/scuc/case_builder.hpp"

namespace mipsolvers::scuc {
  SCUCInput build_3bus_case(int T = 3,  double dt = 1.0);
  SCUCInput build_6bus_case(int T = 24, double dt = 1.0,
                            bool with_wind    = false,
                            bool with_storage = false);
  SCUCInput build_ieee39_case(int T = 24, double dt = 1.0,
                              bool with_wind  = false,
                              bool with_solar = false);
  std::string scuc_input_to_json(const SCUCInput& inp, int indent = 2);
}
```

---

## 2. C++ 接口

### `build_3bus_case`

```cpp
SCUCInput build_3bus_case(int T = 3, double dt = 1.0);
```

| 参数 | 说明 |
|------|------|
| `T` | 调度时段数（默认 3）|
| `dt` | 时段长度（小时，默认 1.0）|

**返回**：3 母线、2 机组的最小验证算例，用于快速冒烟测试。

### `build_6bus_case`

```cpp
SCUCInput build_6bus_case(int T = 24, double dt = 1.0,
                          bool with_wind    = false,
                          bool with_storage = false);
```

| 参数 | 说明 |
|------|------|
| `T` | 调度时段数（默认 24）|
| `dt` | 时段长度（小时）|
| `with_wind` | 是否在 bus 3 添加 100 MW 风电 |
| `with_storage` | 是否在 bus 5 添加 240 MWh 储能（60 MW 充放电功率）|

### `build_ieee39_case`

```cpp
SCUCInput build_ieee39_case(int T = 24, double dt = 1.0,
                            bool with_wind  = false,
                            bool with_solar = false);
```

| 参数 | 说明 |
|------|------|
| `T` | 调度时段数（默认 24）|
| `dt` | 时段长度（小时）|
| `with_wind` | 是否添加 3 个风电站（总容量 1050 MW）|
| `with_solar` | 是否添加 2 个光伏站（总容量 450 MW）|

---

## 3. 3-bus 标准算例

最小验证系统，用于快速单元测试和冒烟测试。

### 拓扑

```
  G1 (bus0, coal)      G2 (bus1, gas)
       │                    │
  ─────┴────────────────────┴──── bus2 (load)
```

### 系统参数

| 项目 | 值 |
|------|-----|
| 母线数 | 3 |
| 机组数 | 2 |
| 支路数 | 2（bus0–bus2 200MW，bus1–bus2 200MW）|
| 总负荷（基准）| 230 MW |
| 负荷曲线 | 双峰正弦形（峰值系数 1.0，谷值系数 0.65）|

### 发电机参数

| 机组 | 母线 | Pmin (MW) | Pmax (MW) | 爬坡 (MW/min) | 最小开/停机 (h) | 启动费 ($) | 空载 ($/h) |
|------|------|-----------|-----------|---------------|-----------------|------------|------------|
| Coal_G1 | 0 | 80 | 400 | 6 | 4 / 4 | 3,000 | 150 |
| Gas_G2 | 1 | 50 | 200 | 5 | 2 / 2 | 1,200 | 60 |

**报价段（超出 Pmin 的 MW 容量）：**

| 机组 | 段1 (MW @ $/MWh) | 段2 (MW @ $/MWh) | 段3 (MW @ $/MWh) |
|------|-------------------|-------------------|-------------------|
| Coal_G1 | 80 @ 22 | 200 @ 27 | 120 @ 33 |
| Gas_G2 | 50 @ 28 | 100 @ 35 | 50 @ 42 |

### 默认配置

| 字段 | 值 |
|------|-----|
| `voll` | 8,000 $/MWh |
| `vocc` | 200 $/MWh |
| `spinning_reserve_req` | 10% |
| `regulation_up_req` | 5% |
| `regulation_down_req` | 5% |
| `enable_market_cuts` | true |

---

## 4. 6-bus 标准算例

中等规模系统，可选风电和储能，用于集成测试。

### 拓扑

```
  G1 (bus0, coal)
    │
  ──┴──── bus1 ─── bus2 ─── bus3 ─── bus4 ─── bus5
         G2 (gas)          [Load2][Load3][Load4][Load5]
                            Wind? (bus3, 100MW)
                                            Storage? (bus5, 240MWh)
```

### 系统参数

| 项目 | 值 |
|------|-----|
| 母线数 | 6 |
| 机组数 | 3（Coal 400MW、Gas 200MW、Peaker 80MW）|
| 支路数 | 7 |
| 总负荷（基准）| 500 MW |
| 风电容量（可选）| 100 MW（bus 3）|
| 储能容量（可选）| 240 MWh / 60 MW 充放（bus 5）|

### 发电机参数

| 机组 | 母线 | Pmin | Pmax | 爬坡 (MW/min) | 最小开/停机 (h) | 启动费 ($) | 空载 ($/h) |
|------|------|------|------|---------------|-----------------|------------|------------|
| Coal_G1 | 0 | 80 | 400 | 6 | 4 / 4 | 3,000 | 150 |
| Gas_G2 | 1 | 50 | 200 | 5 | 2 / 2 | 1,200 | 60 |
| Peaker | 3 | 10 | 80 | 8 | 1 / 1 | 500 | 20 |

### 储能参数（`with_storage = true`）

| 参数 | 值 |
|------|-----|
| 接入母线 | bus 5 |
| 最大充放电 | 60 MW |
| 容量 | 240 MWh |
| 综合效率 | 90% |
| 初始 SOC | 50% |
| 最小 SOC | 10% |

---

## 5. IEEE 39-bus 标准算例

基于 IEEE 39 节点新英格兰系统（10 台发电机组，46 条支路）。

### 系统规模

| 项目 | 值 |
|------|-----|
| 母线数 | 39 |
| 机组数 | 10（G1–G10）|
| 支路数 | 46（34 条输电线 + 7 条升压变压器 + G10 两条联络线）|
| 总 Pmax | 7,329 MW |
| 总 Pmin | 1,300 MW |
| 负荷母线数 | 17 |
| 总负荷（基准）| 4,599 MW |
| 负荷曲线峰值系数 | 1.000（t≈10h）|
| 负荷曲线谷值系数 | 0.650（夜间低谷）|

### 发电机参数

| 机组 | 母线(0-idx) | Pmin (MW) | Pmax (MW) | 爬坡 (MW/min) | 最小开机 (h) | 最小停机 (h) | 启动费 ($) | 空载 ($/h) |
|------|-------------|-----------|-----------|---------------|-------------|-------------|------------|------------|
| G1 | 38 | 250 | 1,100 | 15 | 5 | 5 | 15,000 | 400 |
| G2 | 30 | 100 | 650 | 10 | 4 | 4 | 8,000 | 220 |
| G3 | 31 | 150 | 725 | 12 | 4 | 4 | 9,000 | 250 |
| G4 | 32 | 150 | 650 | 12 | 4 | 4 | 9,000 | 240 |
| G5 | 33 | 100 | 508 | 10 | 3 | 3 | 7,500 | 180 |
| G6 | 34 | 50 | 687 | 8 | 3 | 3 | 8,500 | 200 |
| G7 | 35 | 100 | 580 | 10 | 3 | 3 | 8,000 | 190 |
| G8 | 36 | 50 | 564 | 8 | 2 | 2 | 6,000 | 150 |
| G9 | 37 | 50 | 865 | 10 | 2 | 2 | 9,000 | 220 |
| G10 | 29 | 300 | 1,100 | 15 | 6 | 6 | 16,000 | 450 |

### 折线报价段（超出 Pmin 的容量，$/MWh）

| 机组 | 段1 (MW @ $/MWh) | 段2 (MW @ $/MWh) | 段3 (MW @ $/MWh) |
|------|-------------------|-------------------|-------------------|
| G1 | 250 @ 20 | 550 @ 25 | 300 @ 31 |
| G2 | 100 @ 22 | 350 @ 28 | 200 @ 34 |
| G3 | 150 @ 21 | 375 @ 27 | 200 @ 33 |
| G4 | 150 @ 23 | 300 @ 29 | 200 @ 35 |
| G5 | 100 @ 24 | 250 @ 30 | 158 @ 38 |
| G6 | 50 @ 26 | 350 @ 33 | 287 @ 41 |
| G7 | 100 @ 25 | 280 @ 31 | 200 @ 39 |
| G8 | 50 @ 27 | 300 @ 34 | 214 @ 43 |
| G9 | 50 @ 22 | 450 @ 28 | 365 @ 36 |
| G10 | 300 @ 19 | 550 @ 24 | 250 @ 29 |

**注**：`bid_segments` 中 `quantity` 表示**超出** $P_{\min}$ 的容量（MW），
最大总出力 = $P_{\min} + \sum_k \text{quantity}_k$（受 $P_{\max}$ 上限约束）。

### 负荷分布

| 母线 | 基准负荷 (MW) | 母线 | 基准负荷 (MW) | 母线 | 基准负荷 (MW) |
|------|--------------|------|--------------|------|--------------|
| 0 | 97.6 | 8 | 522.0 | 19 | 628.0 |
| 2 | 322.0 | 14 | 320.0 | 21 | 274.0 |
| 3 | 500.0 | 15 | 329.0 | 22 | 247.5 |
| 6 | 233.8 | 16 | 158.0 | 24 | 308.6 |
| 7 | 6.0 | 25 | 224.0 | 27 | 139.0 |
| 9 | 8.5 | 28 | 281.0 | — | — |

总基准负荷：**4,599 MW**（不含零值母线）

### 时序曲线

**负荷曲线**：各负荷点共用同一双峰正弦形曲线（`make_load_profile`）

$$\text{pu}(t) = 0.65 + 0.35 \cdot \max\bigl(\mathrm{G}_1(t),\, \mathrm{G}_2(t)\bigr)$$

其中 $\mathrm{G}_1$ 以 10:00 为中心，$\mathrm{G}_2$ 以 19:00 为中心，$\sigma = 2\,\text{h}$。

| 时段（h） | 0 | 4 | 8 | 10 | 12 | 16 | 19 | 20 | 24 |
|-----------|-----|-----|-----|-----|-----|-----|-----|-----|-----|
| 负荷系数 pu | 0.65 | 0.66 | 0.87 | 1.00 | 0.77 | 0.68 | 1.00 | 0.96 | 0.65 |
| 系统负荷 MW | 2,989 | 3,035 | 4,001 | 4,599 | 3,541 | 3,127 | 4,599 | 4,415 | 2,989 |

**风电曲线**（`with_wind = true`）：夜间高、午后低（`make_wind_profile`），峰值系数约 1.0，最低系数约 0.1（午后 13:00 附近）。

| 风电站 | 母线 | 额定容量 | 24h 均值（估算）|
|--------|------|----------|-----------------|
| Wind 1 | 5 | 400 MW | ~250 MW |
| Wind 2 | 14 | 300 MW | ~190 MW |
| Wind 3 | 26 | 350 MW | ~220 MW |
| 合计 | — | 1,050 MW | ~660 MW |

**光伏曲线**（`with_solar = true`）：正午高、夜间零（`make_solar_profile`）。

| 光伏站 | 母线 | 额定容量 |
|--------|------|----------|
| PV 1 | 21 | 250 MW |
| PV 2 | 36 | 200 MW |

### 初始机组状态

```cpp
inp.initial_status.commitment = {1, 1, 0, 0, 1, 0, 0, 0, 0, 1};
// G1 G2 G3 G4 G5 G6 G7 G8 G9 G10
inp.initial_status.dispatch   = {500, 300, 0, 0, 200, 0, 0, 0, 0, 600};
```

初始在线机组：G1（500 MW）、G2（300 MW）、G5（200 MW）、G10（600 MW），合计 **1,600 MW**。

### 默认 SCUC 配置

| 字段 | 值 |
|------|-----|
| `n_segments` | 3 |
| `spinning_reserve_req` | 5% 系统负荷 |
| `regulation_up_req` | 3% 系统负荷 |
| `regulation_down_req` | 2% 系统负荷 |
| `pfr_reserve_req_mw` | 300 MW |
| `voll` | 10,000 $/MWh |
| `vocc` | 300 $/MWh |
| `M1_line_slack_penalty` | 1e6 $/MW |
| `M2_renewable_curtail_penalty` | 80 $/MW（`with_wind=true` 时）|
| `enable_market_cuts` | true |

---

## 6. JSON 序列化

```cpp
/// 将 SCUCInput 序列化为 JSON 字符串。
/// indent ≥ 0 → 缩进格式；indent = -1 → 紧凑格式
std::string scuc_input_to_json(const SCUCInput& inp, int indent = 2);
```

```cpp
// 示例：生成 IEEE 39-bus 24h 输入并写入文件
SCUCInput inp = build_ieee39_case(24, 1.0, true, false);
std::string json = scuc_input_to_json(inp, 2);
std::ofstream("/tmp/ieee39_24h.json") << json;
```

---

## 7. 性能基准

测试环境：macOS ARM64（Apple M4 Pro），Release 模式，MIP 间隙 1%，HiGHS 4.x。

### 各求解器耗时对比

| 算例 | T | 新能源/储能 | StrictHiGHS | HiGHS | Gurobi | NativeBranchAndCut | 目标函数 ($) | 割平面 |
|------|---|----------|-------------|-------|--------|--------------------|------------|------|
| 3-bus | 6 | 无 | 9 ms | 7 ms | 3 ms | 7 ms | 746,317 | 32 |
| 6-bus | 8 | 风+储 | 14 ms | 14 ms | 6 ms | 14 ms | 76,478 | 86 |
| IEEE 39-bus | 4 | 风 | 5 ms | 5 ms | 4 ms | 5 ms | 203,130 | 122 |
| IEEE 39-bus | 24 | 风+光 | 44 ms | 45 ms | 74 ms | 43 ms | 891,466 | 898 |
| **IEEE 39-bus** | **24** | **风+光+储** | **27 ms** | **27 ms** | **81 ms** | **26 ms** | **891,466** | **900** |

> **说明**：
> - SCIP 在本框架中仅支持 MINLP，不参与 MILP 调度。
> - Auto 模式（`solver="Auto"`）默认优先选用已安装且许可证有效的 Gurobi，如不可用则依次回退到 StrictHiGHS → HiGHS → NativeBranchAndCut。
> - MIP 间隙均为 0.000%，所有算例均收敛，切负荷为零。
> - 「风+光」目标函数（891,466 $）低于纯风案例（976,359 $）：光伏日间出力替代了高价调峰机组。
> - 「风+光+储」在风+光基础上添加 2 台电池（bus 3: 200 MW/800 MWh，bus 19: 150 MW/600 MWh），目标函数不变，割平面增 2 条。

### IEEE 39-bus 24h 费用分解（风电+光伏+储能算例）

| 费用项目 | 金额 ($) | 占比 |
|---------|---------|------|
| 能量费用 | 745,543 | 83.6% |
| 启动费用 | 49,500 | 5.5% |
| 空载费用 | 60,000 | 6.7% |
| 备用费用 | 36,423 | 4.1% |
| 惩罚费用 | 0 | 0.0% |
| **合计** | **891,466** | 100% |

切负荷：**0 MWh**（无切负荷），功率平衡误差 < 5 MW/时段。

### IEEE 39-bus 24h IPM 根节点 + 交叉迭代测试（风电+光伏）

测试路径：`tests/test_market_simulation.cpp` → `[market][strict_highs][ipm_root][ieee39]`

| 求解配置 | 目标函数 ($) | 节点数 | 根节点迭代 | 交叉迭代 | 耗时 |
|---------|------------|--------|-----------|---------|------|
| StrictHiGHS 单纯形根（基线） | 891,466 | 1 | 330（单纯形） | — | 55 ms |
| **StrictHiGHS IPM根 + 交叉** | **891,466** | **1** | **27（IPM）** | **6** | **57 ms** |

**问题规模**：6,360 列，6,946 行（含等式约束）。

> - IPM 用 27 次内点迭代求解根节点 LP 松弛，交叉阶段（6 次枢轴）将内点解恢复为顶点基，
>   为后续所有节点 LP 重用该基（warm-start），节点数维持为 1（根节点即最优）。
> - 两种配置目标函数一致（差 < 0.5%），验证 IPM→交叉→单纯形 路径的正确性。
> - 运行方式：`MIPSOLVERS_BC_TIMELINE=1 ./build_mipsolvers/test_market_simulation "[ipm_root]"`

### 快速复现

```bash
# Release 模式构建（CMAKE_BUILD_TYPE=Release）
cmake -S . -B build_rel -DCMAKE_BUILD_TYPE=Release
cmake --build build_rel --target test_market_simulation -j8

# 运行全求解器 benchmark
./build_rel/test_market_simulation "[market][benchmark]" 2>/dev/null

# 运行 IEEE 39-bus 24h 完整流程
./build_rel/test_market_simulation "[market][viz][ieee39]" 2>/dev/null | \
  grep -E "切负荷|SCUC|收敛|求解器"

# 运行 IPM 根节点 + 交叉迭代正确性测试（含交叉诊断日志）
MIPSOLVERS_BC_TIMELINE=1 ./build_rel/test_market_simulation "[ipm_root]" 2>&1
```

---

*文档日期：2026-05-15 | 对应源文件：`include/mipsolvers/scuc/case_builder.hpp`，`src/scuc/case_builder.cpp`*
