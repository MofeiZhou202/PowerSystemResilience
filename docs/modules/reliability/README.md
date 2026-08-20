# reliability 模块技术手册

本目录是 `src/reliability/` 的唯一模块文档目录。

本目录对应源码 `src/reliability/`，说明范围为蒙特卡洛、FMEA、三阶段、解析频率—持续时间、运行年/日历年
故障率精确换算、物理成功路径最小割集、信息服务结构可靠性、L1 保护/FRT 轨迹类聚合，以及给定轨迹和
在线 Mass-Matrix DAE 下的 L2 保护—信息系统精确联合事件树。公共头文件通常位于
`include/hacdcpf/reliability/`；若头文件采用仓库的跨模块布局，准确位置以主手册“数据来源”节为准。

- 主文档：[reliability_manual.tex](reliability_manual.tex)
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../README.md)

手册必须以中文给出理论推导、实现映射、算例与验证证据；不得用英文附录补足篇幅。2026-08-20 曾生成的英文
理论附录已从主手册撤除，其 114 页结果不再作为有效验收基线。后续页数只统计中文正文。

理论对象表只列本模块实际执行的功能。非线性交流无功/电压认证、LCC 换相、多端口能量路由器与二次损耗
属于本入口明确拒绝的模型边界，统一使用“模型边界说明”标注。

在本目录执行两遍 `xelatex reliability_manual.tex` 生成目录和交叉引用。公式、默认值、结果口径和限制必须随
公开头文件、实现与注册测试同步更新；PDF 与 LaTeX 辅助文件不作为规范源提交。
