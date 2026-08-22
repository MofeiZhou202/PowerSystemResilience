# 谐波潮流验证契约

本页是机器可读证据的中文入口，完整理论、接口和审计见
[谐波潮流专著](harmonics_power_flow_manual.tex)。实现以
`include/hacdcpf/analysis/harmonics_power_flow.hpp`、
`src/harmonics_power_flow/harmonics_power_flow.cpp` 和注册测试为准。

## 固定门限

| 比较 | 门限 | 当前结果 |
|---|---:|---:|
| Native / 独立 NumPy | `2e-8 pu` | `5.349715355e-10 pu` |
| Native / OpenDSS | `2e-6 pu` | `5.333277457e-10 pu` |
| Native / GridLAB-D | `2e-5 pu`，至少 20 个切片 | `5.349716508e-10 pu`，24 个切片 |
| IEEE13 / OpenDSS | `2e-3 pu` | `1.652e-3 pu`，164 点 |

14 个 AC、三相、DC 和混合 AC/DC 案例全部通过。设备级 VSC 等值网络与 OpenDSS 的最大复电压差为
`5.075e-10 pu`。C++ 回归为 63 cases / 412 assertions；新增 HSS 零耦合退化、DC 电容 SI 闭式解、
两频非对角块闭式解、四类换流器、JSON 身份和 1000 节点/20 频率/4 VSC 稀疏存储门。

GridLAB-D 5.3.0 没有原生谐波阶次 API。验证器对每个可表达 AC 次数启动独立进程，显式写入
`R(h)+j h X`、源阻抗和恒流注入，再由实测复电压/复电流辨识传递阻抗。这是实际数值验证，
但其范围是解耦频率切片，不扩大解释为 PWM/EMT/FFT 波形验证。CTest 强制要求 GridLAB-D 存在并
产生至少 20 个切片，不能 skip。

机器可读结果：

- `external_data/harmonics_validation/cross_engine_matrix.json`
- `external_data/harmonics_validation/ieee13_opendss.json`
- `external_data/harmonics_validation/device_opendss_report.json`

复现：

```bash
cmake --build --preset macos-release --target test_harmonics_power_flow \
  validate_harmonics_xref validate_harmonics_ieee13
ctest --test-dir build/macos-release -R 'harmonics_(cross_engine_matrix|ieee13_opendss)' \
  --output-on-failure
```
