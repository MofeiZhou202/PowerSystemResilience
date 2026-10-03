# resilience 模块技术手册

[Web 指标与参考书映射、公式及数值证据](metrics.md)

[暴雨内涝与雷击场景](weather_scenarios.md)：小时尺度场景生成、AC/DC 支路故障和遵守时间窗的恢复模式。

[多灾害场景调研与接入设计](multi_hazard_sources.md)：书中灾种、开源模型、接口扩展和后续覆盖计划。

本目录是 `src/resilience/` 的唯一模块文档目录。

本目录对应源码 `src/resilience/`，主手册按“理论—实现—接口—数值—审计”闭环组织，覆盖：

- 韧性曲线、EENS、经验风险与静态/动态分层理论；
- 灾害故障、修复阶段、启发式恢复、严格 hybrid AC/DC 多时段 MIP 与 RA 分阶段 MILP；
- 固定储能和 MESS 时空路由/SOC、canonical 投影与稳定 ID 归因；
- MIP 到 DAE 的事件桥、母线级调度重放、多保真认证、自动反馈割、可执行性门和 fail-closed 失败语义；
- 严格 MIP 的完整变量/目标/逐约束推导、动态证书的残差与误差传播推导；
- C++/HTTP/JSON 全字段契约、固定台风案例、2048 对收敛风险样本、三快照 OpenDSS/GridLAB-D
  数值对照、测试目录与准入矩阵。

公共头文件位于 `include/hacdcpf/resilience/`。案例的机器可读证据位于
`external_data/resilience_validation/`；风险证据是事件条件研究，外部对照只认证共同覆盖的平衡正序
冻结 AC 稳态，二者都不构成年度风险或完整 hybrid/DAE 同构验证。

- 主文档：[resilience_manual.tex](resilience_manual.tex)
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../README.md)

推荐从仓库根目录编译到统一输出目录：

```bash
python3 /Users/tianyangzhao/.codex/plugins/cache/openai-bundled/latex/0.2.5/scripts/compile_latex.py \
  "$PWD/docs/modules/resilience/resilience_manual.tex" \
  --engine xelatex --output-directory "$PWD/output/pdf" --json
```

公式、默认值、结果口径和限制必须随公开头文件、实现与注册测试同步更新；PDF 与 LaTeX 辅助文件不作为
规范源提交。
