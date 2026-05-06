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

**功能**：计算速度的上下边界约束。

---

### STBoundsDecider — ST 边界决策

**路径**：`modules/planning/tasks/st_bounds_decider/`

**功能**：在 ST（距离-时间）图中计算可行区域边界。

---

### SpeedDecider — 速度决策

**路径**：`modules/planning/tasks/speed_decider/`

**功能**：做出速度相关决策（如停车、减速、加速）。

---

### PathTimeHeuristic — 路径-时间启发式

**路径**：`modules/planning/tasks/path_time_heuristic/`

**功能**：基于启发式算法生成初步的速度规划。

---

### ReverseSpeed — 倒车速度

**路径**：`modules/planning/tasks/reverse_speed/`

**功能**：倒车时的速度规划。

---

## 决策/判断类 Task

### RuleBasedStopDecider — 规则停车决策

**路径**：`modules/planning/tasks/rule_based_stop_decider/`

**功能**：基于规则判断是否需要停车（如遇停止线、行人、终点等）。

---

### ObstacleNudgeDecider — 障碍物微调决策

**路径**：`modules/planning/tasks/obstacle_nudge_decider/`

**功能**：决策是否需要微调路径以避让障碍物（在车道内轻微偏移）。

---

### PathDecider — 路径决策

**路径**：`modules/planning/tasks/path_decider/`

**功能**：对路径进行最终决策选择。

---

### PathReferenceDecider — 路径参考决策

**路径**：`modules/planning/tasks/path_reference_decider/`

**功能**：选择路径规划的参考线。

---

### RSSDecider — RSS 安全决策

**路径**：`modules/planning/tasks/rss_decider/`

**功能**：基于 Responsibility-Sensitive Safety（RSS）模型进行安全检查。

---

## OpenSpace 类 Task（泊车/自由空间）

### OpenSpaceFallbackDecider — 自由空间回退决策

**路径**：`modules/planning/tasks/open_space_fallback_decider/`

---

### OpenSpacePathPlanning — 自由空间路径规划

**路径**：`modules/planning/tasks/open_space_path_planning/`

---

### OpenSpacePreStopDecider — 自由空间预停车决策

**路径**：`modules/planning/tasks/open_space_pre_stop_decider/`

---

### OpenSpaceReplanDecider — 自由空间重规划决策

**路径**：`modules/planning/tasks/open_space_replan_decider/`

---

### OpenSpaceROIDecider — 自由空间 ROI 决策

**路径**：`modules/planning/tasks/open_space_roi_decider/`

---

### OpenSpaceTrajectoryOptimizerPark — 自由空间轨迹优化（泊车）

**路径**：`modules/planning/tasks/open_space_trajectory_optimizer_park/`

---

### OpenSpaceTrajectoryPartition — 自由空间轨迹分区

**路径**：`modules/planning/tasks/open_space_trajectory_partition/`

---

### OpenSpaceTrajectoryPostProcess — 自由空间轨迹后处理

**路径**：`modules/planning/tasks/open_space_trajectory_post_process/`

---

### OpenSpaceTrajectoryProvider — 自由空间轨迹提供

**路径**：`modules/planning/tasks/open_space_trajectory_provider/`

---

## 回退/安全类 Task

### FastStopTrajectoryFallback — 快速停止轨迹回退

**路径**：`modules/planning/tasks/fast_stop_trajectory_fallback/`

**功能**：在异常情况下快速生成停止轨迹。

---

### SmoothStopTrajectoryFallback — 平滑停止轨迹回退

**路径**：`modules/planning/tasks/smooth_stop_trajectory_fallback/`

**功能**：在需要停止时生成平滑的减速停止轨迹。

---

## Task 开发流程

1. 在 `modules/planning/tasks/` 下新建目录
2. 创建 Task 类（实现 `Execute` 接口）
3. 定义配置 proto 和 default_conf.pb.txt
4. 配置 plugins.xml
5. 在目标 Scenario 的 `conf/pipeline.pb.txt` 中对应 Stage 下注册：
   ```
   task { name: "XXX" type: "XxxClassName" }
   ```

> 来源：各 Task 子模块 README_cn.md
