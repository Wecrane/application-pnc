# tasks 模块分析报告

> 138 文件 / ~27,000 行 | 35 个可复用规划任务

## 功能概述

Task 是 Planning 系统的最小可复用执行单元。每个 Task 继承 `Task` 基类，实现 `Execute()` 方法。Stage 通过 pipeline.pb.txt 配置 Task 链顺序。

---

## 任务分类

### 📏 路径生成类 (PathGeneration)

| Task | 行数 | 功能 |
|------|------|------|
| `lane_borrow_path_generic` | 1,085 | 借道绕行路径（通用版） |
| `lane_borrow_path` | 942 | 借道绕行路径 |
| `lane_change_path_generic` | 728 | 变道路径（通用版） |
| `lane_change_path` | 673 | 变道路径 |
| `pull_over_path` | 721 | 靠边停车路径 |
| `lane_follow_path` | 354 | 车道跟随路径 |
| `fallback_path` | 253 | 回退/安全路径 |
| `reverse_path` | 352 | 倒车路径 |
| `square_path` | 340 | 广场路径 |
| `reuse_path` | 490 | 复用上一帧路径 |

路径生成通用流程：`DecidePathBounds → OptimizePath → AssessPath`

### 🏎️ 速度规划类 (SpeedOptimizer)

| Task | 行数 | 功能 |
|------|------|------|
| `piecewise_jerk_speed_nonlinear` | 2,057 | 非线性分段 jerk 速度优化 (IPOPT) |
| `st_bounds_decider` | 1,889 | S-T 图可行区域边界决策 |
| `speed_bounds_decider` | 1,274 | 速度边界决策 |
| `speed_decider` | 791 | 速度决策 |
| `piecewise_jerk_speed` | 331 | 线性分段 jerk 速度优化 (QP) |
| `path_time_heuristic` | 1,663 | S-T 图 DP 启发式搜索 |
| `reverse_speed` | 214 | 倒车速度规划 |

### 🧠 决策类 (Decider)

| Task | 行数 | 功能 |
|------|------|------|
| `obstacle_nudge_decider` | 874 | 障碍物绕行决策 |
| `rss_decider` | 661 | RSS 责任敏感安全决策 |
| `path_decider` | 367 | 路径决策 |
| `path_reference_decider` | 677 | 路径参考线决策 |
| `rule_based_stop_decider` | 415 | 基于规则的停车决策 |

### 🅿️ 开放空间类 (OpenSpace)

| Task | 行数 | 功能 |
|------|------|------|
| `open_space_roi_decider` | 2,560 | 开放空间 ROI 决策 |
| `open_space_roi_decider_park` | 2,782 | 泊车 ROI 决策 |
| `open_space_trajectory_provider` | 1,700 | 开放空间轨迹提供 |
| `open_space_trajectory_optimizer_park` | 1,254 | 泊车轨迹优化器 |
| `open_space_trajectory_partition` | 1,038 | 轨迹分割 |
| `open_space_trajectory_post_process` | 1,034 | 轨迹后处理 |
| `open_space_path_planning` | 323 | 开放空间路径规划 |
| `open_space_fallback_decider` | 568 | 回退决策 |
| `open_space_fallback_decider_park` | 744 | 泊车回退决策 |
| `open_space_pre_stop_decider` | 305 | 预停车决策 |
| `open_space_replan_decider` | 116 | 重规划决策 |

### 🛡️ 安全/回退类 (Fallback)

| Task | 行数 | 功能 |
|------|------|------|
| `smooth_stop_trajectory_fallback` | 451 | 平滑停车回退 |
| `fast_stop_trajectory_fallback` | 171 | 快速停车回退（默认 fallback_task） |

---

## Task 执行模型

```
Stage::ExecuteTaskOnReferenceLine()
├── for each ReferenceLineInfo:
│   ├── for each Task in task_list_:
│   │   ├── Task::Execute(frame, ref_line_info)
│   │   └── 失败? → 中断, 执行 fallback_task_
│   └── CombinePathAndSpeedProfile()
└── RecordDebugInfo(latency_stats)
```

---

## Task 继承体系

```
Task (基类, 定义 Execute() 接口)
├── Decider         → 决策类 Task
├── PathGeneration  → 路径生成类 Task
├── SpeedOptimizer  → 速度规划类 Task
└── TrajectoryOptimizer → 轨迹优化类 Task
```

## 关键算法

| 算法 | 实现位置 | 说明 |
|------|----------|------|
| **分段 Jerk 速度优化 (QP)** | `piecewise_jerk_speed_problem` | OSQP 求解，线性约束 |
| **分段 Jerk 速度优化 (NLP)** | `piecewise_jerk_speed_nonlinear_optimizer` | IPOPT 求解，非线性 |
| **S-T 图 DP 搜索** | `gridded_path_time_graph` | 网格化 DP (row:s, col:t) |
| **RSS 安全距离** | `rss_decider` (ad_rss 库) | 纵向/横向安全距离 |
| **障碍物绕行计算** | `nudge_calculation` | 横向 offset 计算 |
