# 多代表场景整体规划契约

## 使用流程

Web 弹性仿真平台按“指标选择 → 场景生成 → 主动防御 → 快速恢复 → 指标输出”运行。场景生成保留全部聚类代表场景；主动防御对这些代表场景做一次共同的灾前资源配置。快速恢复再选择其中一个代表场景，使用该配置查看逐时恢复与指标。这里不另造预测场景，也不把快速恢复的单场景结果当作规划目标。

## 当前数学口径

`src/resilience/resilience_portfolio.cpp` 从 `ResilienceScenarioResult.intensities[].clusters[]` 读取全部代表场景。设非空灾害等级组数为 $G$，组 $g$ 中簇 $c$ 的生成权重为 $p_{gc}$，则设计权重为

$$w_{gc}=\frac{1}{G}\frac{p_{gc}}{\sum_k p_{gk}},\qquad \sum_{g,c}w_{gc}=1.$$

各灾害等级等权仅是设计假设；簇内样本权重是条件权重，均不能解释为灾害年发生概率。所有场景共享备用电源与移动储能的节点、容量。首版对在役 AC 节点计算站点分数：原始节点负荷与负荷设备负荷，加上全部代表场景中相邻 AC 故障支路的“设计权重 × 修复时长”。备用电源取最高分节点，移动储能取次高分节点（单节点时同址）。每种资源的演示功率为 $\operatorname{clip}(0.15P_{\rm AC},0.2,1.0)$ MW，移动储能额定能量为额定功率乘 4 h。这是启发式选址与演示容量，**不具备全局最优性，也不是投资成本优化**。

对每个代表场景，用原始网络及同一规划网络分别运行 `RAStyleStageMILP`，原样使用生成的 AC/DC 故障及修复时间、48 h 标准负荷/风/光曲线和分设备负荷绑定。输出各场景切负荷电量 $E_{gc}^{0},E_{gc}^{1}$（MWh），设计加权结果 $\sum w_{gc}E_{gc}$ 和最差代表场景 $\max E_{gc}$。若场景资料不完整、求解未完成或不可行，规划请求报错，不用零值替代。规划评估启用移动储能调度；快速恢复的唯一用户开关允许关闭该调度，因此单场景恢复值可能与规划表中的该场景值不同。规划评估的 MIP 时间限制为每次 10 s；当前不报告投资回收、年可靠性或动态安全认证。

生成的零故障代表场景明确设置 `default_fault_count=0`，不会继承恢复模块默认的两个合成故障。各代表场景互相独立，规划最多并行评估 4 个场景，每个场景的求解器设为单线程；结果按原场景顺序归并并在任何场景失败时整体报错。这只改变执行安排，不删减本批次中的场景，也不改变权重公式。用户在界面选择更多簇或更多台风等级时，仍会增加评估次数。

当前求解器尚未在这一规划口径中纳入固定储能扩容、V2G、燃料供应与可靠投资价格。未规划设备保持原算例配置。规划资源只添加到评估和恢复请求的系统副本，不修改用户原始网络。结果 `limitations` 明确报告这些范围。

## HTTP 与身份

`POST /api/session/generate_scenarios` 响应增加 `scenario_generation_id` 和 `model_revision`，服务端缓存强类型聚类集。`POST /api/session/resilience/portfolio_plan` 请求：

```json
{"scenario_generation_id":"rgen-1","add_generator":true,"add_mobile_storage":true}
```

响应含 `plan_id`、方案资源稳定 `.index` 与 AC 节点 `.index`、全部 `scenarios[]`、设计加权及最差失供 MWh、`model_scope` 和 `limitations`。规划从服务端缓存场景读取；客户端不能替换场景或权重。`POST /api/session/run_distribution_resilience` 带 `portfolio_plan_id` 时先在请求局部系统中应用该方案，响应回显该 ID。模型替换或重新生成场景使旧方案失效；过期 ID 与 `apply_demo_data=true` 均拒绝。方案只保存在服务进程中。

## 实现与验证

- 模型与公式：`include/hacdcpf/resilience/resilience_portfolio.hpp`、`src/resilience/resilience_portfolio.cpp`。
- 会话与 JSON：`tests/run_gui_server.cpp`，Edition 能力声明在 `src/server/edition_profile.cpp`。
- Web：`web/js/core/resilience_portal.js`、`web/js/app.js`。
- 自动化：`tests/test_resilience_portfolio.cpp` 检查两灾害组中全部三簇的权重、稳定资源 ID、加权算式、缺失事件拒绝与零故障不注入合成故障；`tests/e2e/resilience_edition_gui_e2e.mjs` 检查完整方案 ID 流；`tests/e2e/weather_scenario_gui_e2e.mjs` 用暴雨及雷击真实算例检查生成→规划→恢复。

数值验证状态和任何未通过项以 `docs/overview/development_status.md` 的最新记录为准。
