# MIPSolvers 文档总览

本目录只保存需要随代码长期维护的文档。一次性分析、阶段路线图、带日期的
性能快照以及由 TeX 生成的 PDF 不属于长期文档；算法现状以源码、测试和本页
链接的实现文档为准。

## 阅读路径

第一次使用或审查代码建议依次阅读：

1. [用户手册](user_manual.md)：安装、建模、求解器选择、C++/Python/AML、
   SCUC、结果判读和故障处理。
2. [求解器实现与算法审查手册](solvers.md)：总架构、全部求解器、数学原理、
   数据结构、执行流程、数值保护和源码定位。
3. [测试与基准结果](testing.md)：当前工作树的可复现测试结果、覆盖范围和
   尚未验证的风险。
   NETLIB 的 90 案例、14 算法配置详细结果见
   [NETLIB 求解器全面基准](netlib_benchmark.md)。
4. [数值方法](numerical_methods.md)：缩放、KKT、稀疏分解、迭代改进等公共
   数值约定。
5. [Native LP 内点法设计](native_ipm_design.md)：现代 IPM 文献、当前实现
   差距、HSD/IP-PMM 路线和可证伪验收门槛。
6. [Engine API](engine.md)：C++ 模型、选项、结果和适配器接口参考。
7. [CPLEX Callable Library](cplex_callable_library.md)：可选 MILP adapter 的
   模型映射、跨平台构建合同和验证记录。

## 用户文档

| 文档 | 内容 |
|---|---|
| [构建与部署](build_and_deploy.md) | CMake、依赖、安装及离线 Windows 构建 |
| [Python API](python_api.md) | Python 求解接口 |
| [AML Python API](aml_python_api.md) | 代数建模层接口 |
| [SCUC 模块](scuc_module.md) | 安全约束机组组合模型、流程与 C++ 接口 |
| [SCUC 数据格式](data_format_spec.md) | 输入、内部索引和输出 JSON 约定 |
| [SCUC 算例构造器](case_builder.md) | 内置算例和复现方法 |
| [锥规划专题](conic_sdp.md) | LP/SOCP/SDP 的锥内点法详细推导 |
| [Windows CI 验证](windows_ci_validation.md) | Windows 发布验证清单 |
| [NETLIB 求解器基准](netlib_benchmark.md) | 90 个标准 LP 的正确性、性能、失败分析与复现 |
| [教程](tutorial/) | 演示程序和可生成幻灯片的教程源文件 |

## 维护规则

- 设计结论进入 `solvers.md` 或相应稳定专题，不再创建
  `*_plan.md`、`*_audit_YYYY-MM-DD.md` 一类文件。
- 测试结果写明提交、构建目录、命令、平台和日期；没有本机复现的数据不得
  写成当前性能结论。
- 文档引用源码时使用“文件 + 类型/函数名”，不依赖易漂移的固定行号。
- PDF、测试日志、JSON 结果和图表均为构建产物，不提交到 `docs/`；需要保留
  的公式源文件应先合并到 Markdown。
- 修改算法入口、模型结构、默认适配器或终止语义时，必须同步更新
  `solvers.md` 和相关测试。
