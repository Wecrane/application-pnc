# planning_interface_base 模块分析报告

> 53 文件 / ~4,500 行 | 接口抽象层

## 功能概述

定义了整个 Planning 系统的 **四层接口抽象**：
- **Planner 基类**：规划器接口（1 文件）
- **Scenario 基类**：场景接口 + 状态机（16 文件）
- **Task 基类**：任务接口 + 公共算法库（31 文件）
- **TrafficRule 基类**：交通规则接口（5 文件）

## 子模块分析

### 1. planner_base（1 文件，121 行）

| 类 | 关键方法 | 说明 |
|-----|----------|------|
| `Planner` | `Plan()` = 0, `Stop()` = 0 | 抽象规划器基类 |
| `PlannerWithReferenceLine` | `PlanOnReferenceLine()` | 增加单参考线规划接口 |

> ⚠️ **注意**：`Init()` 和 `Reset()` 有默认实现（非纯虚），`LoadConfig<T>()` 通过 `__cxa_demangle` 自动发现配置。

### 2. scenario_base（16 文件，~1,800 行）

**两级状态机设计**：

| 层级 | 类 | 状态枚举 | 说明 |
|------|-----|----------|------|
| **Scenario** | `Scenario` | `UNKNOWN → PROCESSING → DONE` | 场景级状态机 |
| **Stage** | `Stage` | `READY → RUNNING → FINISHED / ERROR` | 阶段级状态机 |

**关键流程**：
```
Scenario::Process()
├── 当前 Stage 为空? → CreateStage(首个)
├── current_stage_->Process() → StageResult
├── RUNNING  → 保持 (STATUS_PROCESSING)
├── FINISHED → next_stage 为空? DONE : 切换 Stage
└── ERROR    → STATUS_UNKNOWN

Stage::ExecuteTaskOnReferenceLine()
├── for each ReferenceLineInfo:
│   ├── for each Task in task_list_:
│   │   └── Task::Execute(frame, ref_line_info)
│   └── CombinePathAndSpeedProfile()
└── 失败? → fallback_task_->Execute()
```

**Stage 基类变体**：
- `BaseStageCreep`：蠕行 Stage（模板方法模式，子类实现 `GetOverlapStopInfo()`）
- `BaseStageCruise`：巡航 Stage
- `BaseStageTrafficLightCreep` / `BaseStageTrafficLightCruise`：红绿灯专用

### 3. task_base（31 文件，~3,700 行）

**Task 基类**（`Task`）：定义 `Execute()` 虚函数（非纯虚），派生类可选覆写。

**三大公共算法库**：

| 子模块 | 核心类 | 用途 |
|--------|--------|------|
| `path_util/` | `PathBoundsDeciderUtil`, `PathOptimizerUtil`, `PathAssessmentDeciderUtil` | 路径边界/优化/评估 |
| `road_graph/` | `DpRoadGraph`, `WaypointSampler`, `TrajectoryCost` | DP 路径规划图 |
| `lane_change_util/` | `LaneChangeUtil` | 变道工具函数 |

**TrajectoryFallbackTask**：默认的 fallback Task，生成安全停车轨迹。

### 4. traffic_rules_base（5 文件，~325 行）

| 类 | 关键方法 | 说明 |
|-----|----------|------|
| `TrafficRule` | `ApplyRule()` = 0 | 交通规则抽象基类 |
| `TrafficDecider` | `Init()`, `Execute()` | 规则链表执行器 |

```
TrafficDecider::Init()
└── 按 pipeline 配置顺序创建 TrafficRule 实例 → rule_list_

TrafficDecider::Execute(frame, ref_line_info)
├── for each rule in rule_list_:
│   ├── rule->Reset()
│   └── rule->ApplyRule(frame, ref_line_info)
└── BuildPlanningTarget(ref_line_info)
```

## 设计模式
- **状态模式**：Scenario-Stage 两级状态机
- **模板方法**：`BaseStageCreep::ProcessCreep()`
- **工厂模式**：`PluginManager::CreateInstance<Stage>()` / `<Task>()` / `<TrafficRule>()`
- **责任链**：`TrafficDecider::rule_list_` 串联执行
- **命令模式**：`Task::Execute()` 封装操作
