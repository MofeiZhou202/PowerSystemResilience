# 暂态运行时契约

> 本文档为 [transient_runtime.md](transient_runtime.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

更新日期：2026-07-12

暂态模块是一个相量域的机电暂态仿真。其实现位于 `include/hacdcpf/dynamics`
与 `src/dynamics`。历史性的数学背景保留在
`archive/theory/dynamics_electromechanical_transient_design.md`；该归档设计
不定义当前行为。本文档负责定义运行时契约。

## 求解路径

`POST /api/session/run_transient` 构建一个 `DynamicSystem`，可选地执行潮流
初始化和动态修整（dynamic trimming），应用预定事件，对模型进行积分，并
记录选定的快照。响应包含初始化诊断、求解器统计信息、母线电压/频率矩阵、
设备序列、已应用事件记录、警告，以及可选的模态/CSV 数据。

服务器会缓存序列化后的富空间（rich-space）结果。GUI 通过以下方式获取单个
采样帧：

```http
GET /api/session/transient/frame?index=N
```

## 一帧可以显示什么

- 原创（authored）AC 和 DC 母线电压；
- 原创 AC 母线频率；
- 实际记录到的动态设备 P/Q、电流、控制器/状态量以及在运指标；
- 仅当存在 SOC（荷电状态，state-of-charge）序列时才显示 SOC；
- 已应用事件的位置及其 active/applied 状态。

## 禁止的推断

暂态帧不得使用静态潮流公式计算支路 P/Q。只有在动态求解器为该采样点记录
了端子电流或端子 P/Q 之后，才允许显示动态支路箭头。在此之前，该帧返回：

```json
{
  "capabilities": { "branch_power": false },
  "geo_ac_branches": [],
  "geo_dc_branches": [],
  "ac_circuit_breaker_flows": []
}
```

## 验证边界

GridLAB-D/OpenDSS 对比仅验证其明确共享的模型范围。平衡快照对比并不能为
动态逆变器、电机、直流链路（DC-link）、储能或事件方程提供认证。电压健康
检查失败与初始化不收敛仍然是可见的错误；回放不得把失败的运行重新标记为
已收敛。

当动态健康守卫启用时，`power_system.json` 可能暴露出非常大的初始 DC 电
压。这是动态模型/初始化缺陷，而非画布映射问题，必须在求解器中解决，而不
得在渲染器中掩盖。
