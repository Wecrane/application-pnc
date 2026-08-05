# planning_base 模块分析报告

> 250 文件 / ~40,000 行 | Planning 基础设施库

## 功能概述

`planning_base` 是 Planning 系统的 **共享基础库**，提供核心数据结构、数学算法、参考线处理等通用能力，被所有上层模块依赖。

---

## 子模块分析

### 1. common/（101 文件，15,708 行）

#### 核心容器类

| 类 | 行数 | 职责 |
|-----|------|------|
| `Frame` | 579 | **单帧规划容器**：持有参考线、障碍物、车辆状态、红绿灯等所有数据 |
| `ReferenceLineInfo` | 1,640 | **参考线视角**：每条参考线的规划数据（路径/速度/决策） |
| `Obstacle` | 811 | 障碍物抽象：静态/动态，预测轨迹，SL 投影 |
| `PlanningContext` | — | 规划状态机上下文：场景转移判定 |
| `DependencyInjector` | — | 依赖注入容器 |
| `LocalView` | — | 聚合 CyberRT 输入数据的视图 |

#### 轨迹/路径/速度数据结构

| 类 | 说明 |
|-----|------|
| `DiscretizedTrajectory` | 离散化轨迹（继承 `std::vector<TrajectoryPoint>`） |
| `DiscretizedPath` | 离散化路径 |
| `FrenetFramePath` | Frenet 坐标系路径 |
| `PathData` | 路径数据封装 |
| `SpeedData` | 速度数据封装 |
| `StBoundary` | S-T 边界 |
| `StPoint` | S-T 点 |
| `StGraphData` | S-T 图数据 |

#### 1D 轨迹类族

| 类 | 说明 |
|-----|------|
| `ConstantDecelerationTrajectory1d` | 恒定减速度 |
| `ConstantJerkTrajectory1d` | 恒定 jerk |
| `PiecewiseAccelerationTrajectory1d` | 分段加速度 |
| `PiecewiseJerkTrajectory1d` | 分段 jerk |
| `StandingStillTrajectory1d` | 静止轨迹 |

#### 其他重要类

| 类 | 说明 |
|-----|------|
| `TrajectoryStitcher` | 轨迹拼接（跨帧平滑） |
| `TrajectoryEvaluator` | 轨迹评估器 |
| `SpeedProfileGenerator` | 速度曲线生成器 |
| `ObstacleBlockingAnalyzer` | 障碍物阻塞分析 |
| `SpeedLimit` | 速度限制 |
| `SLPolygon` | SL 坐标系多边形 |
| `IndexedList` / `IndexedQueue` | 模板索引容器（⚠️ `IndexedQueue` 存在封装问题） |

---

### 2. math/（79 文件，12,942 行）

#### 曲线库 (curve1d/)

| 类 | 说明 |
|-----|------|
| `Curve1d` (抽象基类) | `Evaluate(order, param)` + `ParamLength()` |
| `CubicPolynomialCurve1d` | 三次多项式 |
| `QuarticPolynomialCurve1d` | 四次多项式 |
| `QuinticPolynomialCurve1d` | 五次多项式 |
| `QuinticSpiralPath` | 五次螺旋线 |
| `PiecewiseQuinticSpiralPath` | 分段五次螺旋线 |

#### 样条平滑 (smoothing_spline/)

| 类 | 说明 |
|-----|------|
| `Spline1d` / `Spline2d` | 1D/2D 样条表示 |
| `Spline1dConstraint` / `Spline2dConstraint` | 样条约束 |
| `Spline1dKernel` / `Spline2dKernel` | 样条核函数 |
| `OsqpSpline1dSolver` / `OsqpSpline2dSolver` | OSQP 样条求解器 |
| `AffineConstraint` | 仿射约束 |

#### 离散点平滑 (discretized_points_smoothing/)

| 类 | 求解器 | 说明 |
|-----|--------|------|
| `FemPosDeviationSmoother` | OSQP / IPOPT / SQP | FEM 位置偏差平滑 |
| `CosThetaSmoother` | IPOPT | Cos-Theta 角度平滑 |

#### 分段 Jerk 优化 (piecewise_jerk/)

| 类 | 说明 |
|-----|------|
| `PiecewiseJerkProblem` | 分段 jerk 优化问题基类 |
| `PiecewiseJerkPathProblem` | 路径优化特化 |
| `PiecewiseJerkSpeedProblem` | 速度优化特化 |

#### 约束检查 (constraint_checker/)

| 类 | 说明 |
|-----|------|
| `ConstraintChecker` | 2D 约束检查 |
| `ConstraintChecker1d` | 1D 约束检查 |

---

### 3. reference_line/（20 文件，5,481 行）

| 类 | 行数 | 职责 |
|-----|------|------|
| `ReferenceLine` | 1,003 | 参考线核心类：XY↔SL 转换、平滑、拼接 |
| `ReferenceLineProvider` | 1,032 | 参考线提供器：从路由生成 + 动态更新 |
| `SpiralReferenceLineSmoother` | 390 | 螺旋线参考线平滑器 |
| `SpiralProblemInterface` | 755 | 螺旋线问题求解接口 |

---

### 4. learning_based/（25 文件，3,079 行）

| 子模块 | 说明 |
|--------|------|
| `img_feature_renderer/` | 鸟瞰图特征渲染（Birdview） |
| `model_inference/` | 模型推理接口 |

---

### 5. 其他

| 子模块 | 说明 |
|--------|------|
| `proto/` (15 文件) | Planning 内部 protobuf 定义（PlanningConfig, TaskConfig 等） |
| `gflags/` (2 文件) | 全局命令行标志（FLAGS_use_navigation_mode 等） |
| `tools/` (8 文件) | 辅助工具（log_util.py 等） |
