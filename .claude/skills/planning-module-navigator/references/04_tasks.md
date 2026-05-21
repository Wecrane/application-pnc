# 任务插件（Tasks）详解

Task 插件在 Stage 中按顺序依次执行，负责具体的路径规划、速度规划、决策判断等。

---

## 路径规划类 Task

### LaneFollowPath — 车道跟随路径

**路径**：`modules/planning/tasks/lane_follow_path/`

**功能**：生成沿当前车道行驶的路径。

---

### LaneBorrowPath / LaneBorrowPathGeneric — 借道路径

**路径**：`modules/planning/tasks/lane_borrow_path/`、`lane_borrow_path_generic/`

**功能**：决策生成借道路路径。当道路前方有障碍物长时间停留阻塞道路，车辆无法在当前车道内绕过时，往相邻车道借道绕过障碍物，经过后立即回原车道。

**核心函数**：
1. **IsNecessaryToBorrowLane()**：判断是否需要借道。检查条件包括：
   - 单一参考线、侧向通行速度范围
   - 阻塞障碍物远离路口
   - 长周期阻塞障碍物
   - 障碍物在目标位置前
   - 障碍物是否可移动

2. **DecidePathBounds()**：分析周围车道、车辆位置和静态障碍物，决定路径边界

3. **OptimizePath()**：计算路径曲率约束、曲率变化率和参考路径，调用 `PathOptimizerUtil::OptimizePath`

4. **AssessPath()**：评估候选路径数据，检查有效性（是否为空、远离参考线、远离道路、碰撞、终点在逆向车道），选择最优路径

**配置注册**：
```
task { name: "LANE_BORROW_PATH" type: "LaneBorrowPath" }
```

> 来源：`modules/planning/tasks/lane_borrow_path_generic/README_cn.md`

---

### LaneChangePath — 变道路径

**路径**：`modules/planning/tasks/lane_change_path/`、`lane_change_path_generic/`

**功能**：生成变道所需的路径。

---

### FallbackPath — 回退路径

**路径**：`modules/planning/tasks/fallback_path/`

**功能**：当正常规划失败时生成回退路径。

---

### PullOverPath — 靠边停车路径

**路径**：`modules/planning/tasks/pull_over_path/`

**功能**：生成靠边停车所需的路径。

---

### ReusePath — 路径复用

**路径**：`modules/planning/tasks/reuse_path/`

**功能**：复用上一帧的规划路径。

---

### ReversePath — 倒车路径

**路径**：`modules/planning/tasks/reverse_path/`

**功能**：生成倒车路径。

---

### SquarePath — 广场路径

**路径**：`modules/planning/tasks/square_path/`

**功能**：广场场景的路径规划。

---

## 速度规划类 Task

### PiecewiseJerkSpeed — 分段 Jerk 速度优化

**路径**：`modules/planning/tasks/piecewise_jerk_speed/`

**功能**：使用分段 jerk（加加速度）优化算法生成平滑的速度剖面。

---

### PiecewiseJerkSpeedNonlinear — 非线性分段 Jerk 速度优化

**路径**：`modules/planning/tasks/piecewise_jerk_speed_nonlinear/`

**功能**：非线性版本的速度优化。

---

### SpeedBoundsDecider — 速度边界决策

**路径**：`modules/planning/tasks/speed_bounds_decider/`

**功能**：计算车辆在 ST 图中的速度边界约束。

---

### StBoundsDecider — ST 边界决策

**路径**：`modules/planning/tasks/st_bounds_decider/`

**功能**：在 ST 图中计算障碍物和安全边界。

---

## 决策类 Task

### RuleBasedStopDecider — 规则停车决策

**路径**：`modules/planning/tasks/rule_based_stop_decider/`

**功能**：基于规则判断是否需要停车。

---

### RSSDecider — RSS 安全检查

**路径**：`modules/planning/tasks/rss_decider/`

**功能**：基于 RSS（Responsibility Sensitive Safety）模型进行安全检查和决策。

---

### PathDecider — 路径决策

**路径**：`modules/planning/tasks/path_decider/`

**功能**：最终决定选择哪条路径。

---

### PathLaneBorrowDecider — 借道决策

**路径**：`modules/planning/tasks/path_lane_borrow_decider/`

**功能**：决策是否允许借道及借道方向。

---

### PathAssessmentDecider — 路径评估决策

**路径**：`modules/planning/tasks/path_assessment_decider/`

**功能**：对候选路径进行效用评估和排序。

---

### ObstacleNudgeDecider — 障碍物微调避让

**路径**：`modules/planning/tasks/obstacle_nudge_decider/`

**功能**：对障碍物进行细微的横向避让调整。

---

### ObstacleAggressiveNudgeDecider — 激进避让

**路径**：`modules/planning/tasks/obstacle_aggressive_nudge_decider/`

**功能**：更激进的障碍物避让策略。

---

### ObstacleStopDecider — 障碍物停车

**路径**：`modules/planning/tasks/obstacle_stop_decider/`

**功能**：判断是否因障碍物需要停车。

---

### CreepDecider — 爬行决策

**路径**：`modules/planning/tasks/creep_decider/`

**功能**：在需要缓慢前行的场景中决策爬行行为。

---

### OpenSpaceFallbackDecider — 自由空间回退

**路径**：`modules/planning/tasks/open_space_fallback_decider/`

**功能**：Open space 规划失败时的回退决策。

---

### OpenSpacePreStopDecider — 自由空间预停车

**路径**：`modules/planning/tasks/open_space_pre_stop_decider/`

**功能**：Open space 场景中的预停车决策。

---

### OpenSpaceROIDecider — 自由空间 ROI

**路径**：`modules/planning/tasks/open_space_roi_decider/`

**功能**：确定 Open space 规划中的感兴趣区域（ROI）。

---

## 其他 Task

### LearningModel — 学习模型

**路径**：`modules/planning/tasks/learning_model/`

**功能**：基于学习模型的轨迹预测。

---

### DPStSpeedOptimizer — DP ST 速度优化

**路径**：`modules/planning/tasks/dp_st_speed/`

**功能**：基于动态规划的 ST 速度优化。

---

### DPPolyPathOptimizer — DP 多项式路径优化

**路径**：`modules/planning/tasks/dp_poly_path/`

**功能**：基于动态规划和多项式拟合的路径优化。

---

### QPSplinePathOptimizer — QP 样条路径优化

**路径**：`modules/planning/tasks/qp_spline_path/`

**功能**：基于二次规划样条曲线的路径优化。

---

> 来源：各子模块 README_cn.md、目录结构分析
