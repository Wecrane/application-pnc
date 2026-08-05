# planning_open_space 模块分析报告

> 53 文件 / ~23,500 行 | 开放空间规划（泊车/非结构化道路）

## 功能概述

`planning_open_space` 处理 **非结构化道路场景**（停车场、广场等）的轨迹规划。核心流程：

```
粗轨迹生成 (Hybrid A*) → 轨迹平滑 (IPOPT) → 优化后轨迹
```

---

## 子模块分析

### 1. trajectory_smoother/（34 文件，17,398 行）⭐ 代码量最大

轨迹平滑器集群，使用 **IPOPT 非线性求解器** 对粗轨迹进行平滑优化。

#### DistanceApproach 系列（距离逼近法）

| 类 | 行数 | 说明 |
|-----|------|------|
| `DistanceApproachInterface` | — | IPOPT 求解器抽象接口（TNLP） |
| `DistanceApproachIpoptInterface` | 1,985 | 标准 IPOPT 求解器 |
| `DistanceApproachIpoptFixedTsInterface` | 1,109 | 固定时间步长 IPOPT |
| `DistanceApproachIpoptFixedDualInterface` | 1,020 | 固定对偶变量 IPOPT |
| `DistanceApproachIpoptRelaxEndInterface` | 1,169 | 松弛终点约束 IPOPT |
| `DistanceApproachIpoptRelaxEndSlackInterface` | 1,236 | 松弛终点+松弛变量 IPOPT |
| `DistanceApproachIpoptCudaInterface` | 2,859 | **CUDA 加速版**（代码量最大） |
| `DistanceApproachProblem` | — | 问题建模 |

> ⚠️ 头文件使用 `#define` 宏（tag_f, tag_g, tag_L, HPOFF），存在全局命名空间污染风险。

#### DualVariableWarmStart 系列（对偶变量热启动）

| 类 | 行数 | 说明 |
|-----|------|------|
| `DualVariableWarmStartProblem` | 407 | 对偶问题建模 |
| `DualVariableWarmStartIpoptInterface` | 815 | IPOPT 热启动 |
| `DualVariableWarmStartIpoptQpInterface` | 835 | IPOPT+QP 热启动 |
| `DualVariableWarmStartOsqpInterface` | 523 | OSQP 热启动 |
| `DualVariableWarmStartSlackOsqpInterface` | 568 | 松弛变量 OSQP 热启动 |

> ⚠️ `DualVariableWarmStartProblem` 存在双构造函数设计缺陷和 `// park generic` 场景特化代码混入。

#### IterativeAnchoringSmoother

| 类 | 行数 | 说明 |
|-----|------|------|
| `IterativeAnchoringSmoother` | 786 | 迭代锚点平滑器（外层循环调用 IPOPT） |

---

### 2. coarse_trajectory_generator/（11 文件，3,683 行）

| 类 | 行数 | 说明 |
|-----|------|------|
| `HybridAStar` | 902 | **混合 A\* 搜索**：4D 状态 (x, y, phi, direction) |
| `ReedSheppPath` | 1,334 | **Reed-Shepp 曲线**：最短路径连接（含倒车） |
| `GridSearch` | — | 网格搜索（A\* 的离散化替代） |
| `Node3d` | — | 3D 搜索节点 |

**Hybrid A\* 核心参数**：
- 4D 状态空间：(x, y, φ, direction)
- 启发式：RS 曲线长度 + 障碍物距离惩罚
- 扩展：AnalyticExpansion + RSPCheck
- 含 15+ 可配置 penalty 权重

---

### 3. 其他

| 子模块 | 文件 | 行数 | 说明 |
|--------|------|------|------|
| `tools/` | 3 | 1,503 | `DistanceApproachProblemWrapper`, `OpenSpaceRoiWrapper` |
| `utils/` | 4 | 642 | 辅助函数（坐标转换、碰撞检测） |
| `proto/` | 1 | 334 | Protobuf 定义 |

---

## 开放空间规划完整流程

```mermaid
graph LR
    A[停车场/广场场景] --> B[ROI 决策<br/>open_space_roi_decider]
    B --> C[粗轨迹<br/>Hybrid A* + Reed-Shepp]
    C --> D[轨迹分割<br/>trajectory_partition]
    D --> E[轨迹平滑<br/>IPOPT 迭代锚点]
    E --> F[轨迹后处理<br/>trajectory_post_process]
    F --> G[输出轨迹]
```

## 已知设计问题

| 问题 | 文件 | 严重度 |
|------|------|--------|
| 双构造函数（通用/泊车混用） | `dual_variable_warm_start_problem.h`, `hybrid_a_star.h` | 🔴 高 |
| 宏污染（tag_f/tag_g/tag_L/HPOFF） | `distance_approach_interface.h` | 🟡 中 |
| 访问修饰符顺序混乱 | `dual_variable_warm_start_problem.h` | 🟢 低 |
| 15+ penalty 权重散落 | `hybrid_a_star.h` | 🟢 低 |
