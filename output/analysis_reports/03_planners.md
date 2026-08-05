# planners 模块分析报告

> 56 文件 / ~8,362 行 | 4 种规划器实现

## 功能概述

| 规划器 | 文件数 | 行数 | 特点 | 使用场景 |
|--------|--------|------|------|----------|
| **PublicRoadPlanner** ⭐ | 6 | 350 | 轻量调度器，场景管理模式 | **主力规划器**，所有常规场景 |
| **LatticePlanner** | 28 | 3,613 | Frenet 采样 + 代价评估 | 需要多轨迹候选评估 |
| **NaviPlanner** | 19 | 3,975 | 不依赖高精地图 | 导航模式 |
| **RTKReplayPlanner** | 3 | 424 | 轨迹文件回放 | 调试/测试/录制回放 |

---

## 1. PublicRoadPlanner（主力规划器）

### 架构
```
PublicRoadPlanner (继承 PlannerWithReferenceLine)
└── ScenarioManager (场景管理器)
    ├── scenario_list_ (所有注册的 Scenario)
    └── current_scenario_ (当前激活的 Scenario)
```

### 核心流程
```
Plan(start_point, frame)
├── scenario_manager_.Update(start_point, frame)
│   ├── 当前场景 STATUS_PROCESSING? → 禁止切换（优先级保护）
│   ├── for each scenario: IsTransferable()?
│   └── 找到可切换场景 → Exit() → Reset() → Enter()
├── scenario_->Process(start_point, frame)
│   └── → Scenario 两级状态机 → Stage → Task 链
└── STATUS_DONE? → 再次 Update()（触发场景切换）
```

### ScenarioManager 切换策略
- **优先级保护**：当前场景未完成时，不可被抢占
- **默认回退**：Reset() 恢复为 `LANE_FOLLOW`（默认场景）
- **转移判定**：每个 Scenario 自定义 `IsTransferable()` 条件

---

## 2. LatticePlanner（格点采样规划器）

### 核心流程（6 步标准流程）
```
PlanOnReferenceLine()
├── 1. 参考线离散化 → vector<PathPoint>
├── 2. PathMatcher 匹配起点
├── 3. Cartesian → Frenet (init_s, init_d)
├── 4. PredictionQuerier → PathTimeGraph → 时空障碍物图
├── 5. Trajectory1dGenerator 生成横向/纵向 1D 轨迹束
└── 6. TrajectoryEvaluator 代价评估 → 选最优
```

### 子模块
| 子模块 | 核心类 | 职责 |
|--------|--------|------|
| `behavior/` | `PathTimeGraph`, `FeasibleRegion`, `PredictionQuerier`, `CollisionChecker` | S-T 图构建、可行区域、碰撞检测 |
| `trajectory_generation/` | `EndConditionSampler`, `Trajectory1dGenerator`, `TrajectoryEvaluator`, `TrajectoryCombiner`, `LateralOSQPOptimizer` | 终点采样、轨迹生成、代价评估、横纵向组合 |

---

## 3. NaviPlanner（导航模式规划器）

### 架构特点
- **不依赖高精地图**：只使用导航路线（routing）
- **简化决策链**：无 Scenario 状态机，只有 Task 链
- **2 个 Task**：`NaviPathDecider` + `NaviSpeedDecider`
- **S-T 图**：`NaviSpeedTsGraph` 实现速度规划

```
NaviPlanner
├── NaviPathDecider  → 路径生成（基于参考线 + 障碍物）
└── NaviSpeedDecider → 速度生成（基于 NaviSpeedTsGraph）
    └── NaviSpeedTsGraph → S-T 图 DP 搜索
```

---

## 4. RTKReplayPlanner（回放规划器）

### 工作方式
```
Init()
└── ReadTrajectoryFile(FLAGS_rtk_trajectory_filename)
    └── 解析 TSV 文件 (x,y,z,v,a,kappa,dkappa,time,theta,s)

PlanOnReferenceLine()
├── QueryPositionMatchedPoint()  → 最近邻匹配
├── 提取 [matched_index, matched_index+forward_buffer]
├── 尾部补点（零速保持）
└── SetTrajectory(DiscretizedTrajectory)
```

**无实时规划逻辑**：纯粹的位置查找 + 轨迹切片回放。
