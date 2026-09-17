# 性能证据采集工具契约

本页描述仓库内轻量性能驱动的输入、证据口径与限制。脚本只生成本地 JSON、日志和原始响应，
仓库不提交运行产物，也不因单次墙钟时间给出性能结论或 CI 门槛。

## 工具范围

| 工具 | 作用 | 关键证据 |
|---|---|---|
| `tools/module_api_performance.py` | 在独立 `run_gui_server` 进程上执行既有 GUI API 回归 | 服务二进制 SHA-256、请求/响应原文、HTTP 墙钟时间、求解状态摘要 |
| `tools/module_performance_audit.py` | 串行执行模块代表性 C++ 测试 | 命令、退出/超时/跳过状态、JUnit 用例状态、进程墙钟时间、可选进程树峰值工作集 |
| `tools/power_flow_performance.py` | 对同一请求、同一算例比较 PF 线性求解后端 | 算例与服务哈希、有效选项、收敛/残差、求解器 profiling、状态向量最大绝对差 |
| `tools/run_module_performance_matrix.py` | 编排 Windows Release 的 C++、扩展、注册和原生测试阶段 | 源码/依赖/可执行文件清单及每项独立状态 |

`tools/module_performance_audit_test.py` 是注册的纯 Python 回归测试，不生成基准结论。

## 使用边界

模块审计接受 CMake 单配置的 `build/tests/` 和多配置的 `build/tests/Release/` 布局。
安装 `psutil` 时采样被测进程及其后代的工作集；未安装时
`observed_peak_working_set_bytes` 为 `null` 且 `memory_sampling` 为
`unavailable`，其他证据仍有效。超时清理使用独立进程组，在 Windows 上调用系统
`taskkill /T /F`，在 POSIX 上终止整个进程组。

Windows 总控脚本要求已经完成 Release 构建，并要求 `--dependency` 指向用于构建的
干净 MIPSolvers 工作树；脏依赖会被显式拒绝。若可选的
`tools/opf_api_performance.py` 不存在，对应步骤记录为
`unavailable` 而不是伪装成已运行。扩展阶段还依赖脚本中列出的外部引擎、算例和历史
benchmark 工作目录；缺失条件必须保留为失败或不可用证据。

PF 后端比较会为每个后端启动独立服务，强制开启 solver profiling，并要求各次结果收敛、
包含交流电压且 `vm`/`va`/`vdc` 的跨后端最大绝对差小于 `1e-6`。这个门槛只验证同请求
结果一致性，不代表对外部引擎的精度认证。HTTP 墙钟时间包含序列化、传输和解析，不得标为
纯求解器耗时。

## 最小验证

```bash
python3 tools/module_performance_audit_test.py
python3 tools/module_performance_audit.py --help
python3 tools/module_api_performance.py --help
python3 tools/power_flow_performance.py --help
python3 tools/run_module_performance_matrix.py --help
python3 -m py_compile tools/module_*performance*.py tools/power_flow_performance.py \
  tools/run_module_performance_matrix.py
```

`--inventory-only` 可在不运行 C++ 测试的情况下生成文档引用清单。任何跨机器比较必须保持
Git revision、依赖 revision、服务二进制、算例内容、请求、后端与线程环境一致，并保留原始报告。
