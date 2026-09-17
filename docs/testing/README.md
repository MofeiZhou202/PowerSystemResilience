# 测试证据与验证方法

本分类区分“当前已验证证据”和“建议验证方法”。前者以开发状态和注册测试为准，后者见各模块
手册的验证章节。

| 内容 | 文档 |
|---|---|
| 当前构建与回归基线 | [开发状态](../overview/development_status.md) |
| 模块审计深度与缺陷 | [模块代码审计](module_code_audit.md) |
| 可复现性能证据采集 | [性能工具契约](performance_tooling.md) |
| 谐波跨引擎校核 | [谐波正确性验证](../modules/harmonics_power_flow/harmonic_verification.md) |
| OPF、PF 与模块验证矩阵 | [模块手册索引](../modules/README.md) |
| 研究复现与数值证据 | [论文工作区](../latex/paper/README.md) |

文档验收至少执行链接检查、占位符扫描、`git diff --check` 和 LaTeX 编译抽检。
