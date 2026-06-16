# 07 - CyberRT 通信框架

> 来源：`apollo_docs_md/框架设计/软件核心/CyberRT/`

---

## CyberRT 概述

CyberRT 是 Apollo 自研的实时通信框架，替代了 ROS，是 Apollo 各模块之间通信的基础设施。Planning 模块的所有数据输入输出都通过 CyberRT 完成。

### 核心概念

| 概念 | 说明 | 对应 Planning 中的使用 |
|------|------|----------------------|
| **Node** | 最小运行单元，代表一个模块进程 | `planning_component` 是一个 Node |
| **Component** | 特殊的 Node，由消息触发运行 | PlanningComponent 由 PredictionObstacles 触发 |
| **Channel** | 基于 Topic 的发布/订阅通信 | Planning 订阅 `/prediction`、`/localization` 等，发布 `/planning` |
| **Reader** | 数据读取者 | Planning 读取 Perception、Prediction、Localization 数据 |
| **Writer** | 数据写入者 | Planning 向 Control 发布轨迹 |
| **Service** | 请求/响应通信 | 不太常用 |

### 通信机制

```
上游模块                           Planning                        下游模块
┌──────────┐   Channel/Reader   ┌──────────┐   Channel/Writer   ┌──────────┐
│ Perception│ ────────────────→ │          │ ────────────────→  │ Control  │
│ Prediction│ ────────────────→ │ Planning │                   │          │
│ Routing   │ ────────────────→ │          │                   │          │
│ Localiz.  │ ────────────────→ │          │                   │          │
└──────────┘                    └──────────┘                   └──────────┘
```

### 调度机制

CyberRT 支持多种调度策略：
- **CLASSIC_POOL**：经典线程池
- **CHOREOGRAPHY**：编排调度（减少线程切换）
- 默认使用 `CHOREOGRAPHY` 模式

对于比赛来说，不需要深入修改调度机制，只需了解 Planning 的数据流向即可。

### 插件机制

CyberRT 支持动态加载插件（`.so` 文件），Planning 的 Scenario/Task/TrafficRule 插件都基于此机制。配置文件通过 `cyberfile.xml` 声明依赖。

### 组件机制

Planning 作为 Component 运行：
1. CyberRT 启动 → 加载 PlanningComponent
2. 收到 Prediction 消息 → 触发 `Proc()` 函数
3. 内部调用 Planner → Scenario → Task 链
4. 发布轨迹到 `/planning` Channel

---

## 比赛相关

- **不需要修改 CyberRT**，只需了解 Planning 的通信接口
- Planning 输入 Channel：`/prediction`, `/localization`, `/routing_response`, `/perception`
- Planning 输出 Channel：`/planning` (ADCTrajectory)
- 调试工具：`cyber_monitor` 可查看各 Channel 的消息频率和内容

---

## 参考

- CyberRT 通信机制：`apollo_docs_md/框架设计/软件核心/CyberRT/Cyber_RT_通信机制.md`
- CyberRT 调度机制：`apollo_docs_md/框架设计/软件核心/CyberRT/Cyber_RT_调度机制.md`
- CyberRT 组件机制：`apollo_docs_md/框架设计/软件核心/CyberRT/Cyber_RT_组件机制.md`
- CyberRT 插件机制：`apollo_docs_md/框架设计/软件核心/CyberRT/Cyber_RT_插件机制.md`
