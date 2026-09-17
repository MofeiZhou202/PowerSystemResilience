# io 模块技术手册

本目录是 `src/io/` 的唯一模块文档目录。

本目录对应源码 `src/io/`，说明范围为外部格式映射、导入报告与往返验证。公共头文件通常位于
`include/hacdcpf/io/`；若头文件采用仓库的跨模块布局，准确位置以主手册“数据来源”节为准。

Windows I/O 可移植性契约：

- `etap_fidelity_check` 为每次调用创建唯一临时工作簿，并用 RAII 在正常返回或异常路径清理；固定文件名会使并行 CTest 相互覆盖。
- GridLAB-D 在 Windows 上通过 `cmd.exe /C`的 `std::system` 语义启动。完整命令和所有路径均加引号，以支持 `Program Files` 等含空格路径；`GLPATH` 等本次运行环境在同一命令中设置。Windows 路径仍未实现 POSIX 分支的超时终止。
- SVG 文件流直接使用原生 `std::filesystem::path`；报错位置和默认模型名从 `path::u8string()` 转为 UTF-8，避免 Windows 本地代码页损坏中文文件名。

回归覆盖 ETAP 四任务并发 fidelity、无外部素材的中文/Delta SVG 往返，以及不依赖 Windows 主机的 GridLAB-D 启动源码契约。真实 GridLAB-D 运行仍受可执行文件和模块路径可用性门控。

- 主文档：[io_manual.tex](io_manual.tex)
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../README.md)

在本目录执行两遍 `xelatex io_manual.tex` 生成目录和交叉引用。公式、默认值、结果口径和限制必须随
公开头文件、实现与注册测试同步更新；PDF 与 LaTeX 辅助文件不作为规范源提交。
