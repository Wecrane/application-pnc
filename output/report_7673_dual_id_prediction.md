# 行人 7673「静态 obs7673 + 动态 obs7673_0」双标记机制研究报告

- 日期：2026-08-01
- 范围：prediction 模块（modules/prediction/）、planning 障碍物转换层（modules/planning/planning_base/common/obstacle.cc）
- 日志：data/log/planning.log.INFO.20260801-220748.921814、data/log/prediction.log.INFO.20260801-220748.921790
- 结论性质：纯代码 + 日志分析，未改动任何代码

---

## 0. 一句话结论

**`_0` 后缀不是感知/预测加的，而是 planning 在把 `PredictionObstacle` 转成内部 `Obstacle` 时加的**。同一个预测障碍物 id=7673：
- prediction 判定**静止**（`is_static=true`、无 trajectory）→ planning 生成 id=`"7673"`
- prediction 判定**运动**（`is_static=false`、有 trajectory）→ planning 生成 id=`"7673_0"`（第 0 条轨迹）

所以 `7673` 与 `7673_0` 是**同一个行人在静止/运动两种状态下的两种投影**，不是感知产出了两个障碍物。

---

## 1. `_0` 后缀的来源 —— planning 侧 `Obstacle::CreateObstacles`

文件：`modules/planning/planning_base/common/obstacle.cc:197-242`

```cpp
std::list<std::unique_ptr<Obstacle>> Obstacle::CreateObstacles(
    const prediction::PredictionObstacles& predictions) {
  for (const auto& prediction_obstacle : predictions.prediction_obstacle()) {
    ...
    const auto perception_id =
        std::to_string(prediction_obstacle.perception_obstacle().id());  // "7673"
    if (prediction_obstacle.trajectory().empty()) {
      // ① 无预测轨迹 → 单一 Obstacle，id = "7673"（静止标记）
      obstacles.emplace_back(
          new Obstacle(perception_id, prediction_obstacle.perception_obstacle(),
                       prediction_obstacle.priority().priority(),
                       prediction_obstacle.is_static()));
      continue;
    }

    // ② 有 1..N 条预测轨迹 → 每条轨迹生成一个 Obstacle
    int trajectory_index = 0;
    for (const auto& trajectory : prediction_obstacle.trajectory()) {
      ...
      const std::string obstacle_id =
          absl::StrCat(perception_id, "_", trajectory_index);   // "7673_0", "7673_1", ...
      obstacles.emplace_back(
          new Obstacle(obstacle_id, prediction_obstacle.perception_obstacle(),
                       trajectory, prediction_obstacle.priority().priority(),
                       prediction_obstacle.is_static()));
      ++trajectory_index;
    }
  }
  return obstacles;
}
```

要点：
- id 是 `std::string`，由 `perception_obstacle().id()`（int32）转字符串后按需拼 `_<idx>`。
- `"7673"` 和 `"7673_0"` 携带的是**同一份 `PerceptionObstacle`**（位置/速度/类型/多边形完全相同），只有 `id`、`is_static`、`trajectory` 不同。
- `Obstacle::PerceptionId()`（obstacle.h:80）返回 int32 基 id，两个 Obstacle 都是 7673 —— 这是 planning 侧识别"同一行人"的关键。

---

## 2. prediction 侧：`is_static` 与 trajectory 的生成条件

### 2.1 输出结构（`PredictionObstacle`）

`external/common-msgs/.../prediction_obstacle.pb.h` 确认字段：
- `perception_obstacle = 1`
- `predicted_period = 3`（= `FLAGS_prediction_trajectory_time_length` = 6.0s）
- `is_static = 7`（默认 false）
- `trajectory`（repeated）、`priority`、`interactive_tag = 9`

### 2.2 主流程 `PredictorManager::PredictObstacle`

文件：`modules/prediction/predictor/predictor_manager.cc:291`

```cpp
if (obstacle->ToIgnore()) { ... RunEmptyPredictor ... priority=IGNORE }
else if (obstacle->IsStill()) {
    RunEmptyPredictor(...);          // ← 静止：空预测器，不生成轨迹
} else {
    switch (obstacle->type()) {
      case PerceptionObstacle::PEDESTRIAN:
        if (FLAGS_enable_multi_agent_pedestrian_evaluator) { RunEmptyPredictor(...); }
        else { RunPedestrianPredictor(...); }   // ← 运动：FreeMovePredictor
        ...
    }
}
...
prediction_obstacle->set_is_static(obstacle->IsStill());
```

- `RunEmptyPredictor` → `EmptyPredictor::Predict`（empty_predictor.cc）只 `SetPredictorType` 就返回，**不产生任何 trajectory**。
- `RunPedestrianPredictor` → `pedestrian_predictor_` = **FREE_MOVE_PREDICTOR**（prediction_conf.pb.txt 中 `obstacle_conf { obstacle_type: PEDESTRIAN, obstacle_status: MOVING, evaluator_type: SEMANTIC_LSTM_EVALUATOR, predictor_type: FREE_MOVE_PREDICTOR }`）。
- 日志确认：`Defined pedestrian obstacle predictor [1]`（1 = FREE_MOVE），且 `--noenable_multi_agent_pedestrian_evaluator`，所以行人走 FreeMove 分支。

### 2.3 静止判定 `Obstacle::IsStill()` / `SetMotionStatus()`

文件：`modules/prediction/container/obstacles/obstacle.cc:95, 1305-1376`

```cpp
bool Obstacle::IsStill() { return feature_history_.front().is_still(); }

void Obstacle::SetMotionStatus() {
  ...
  if (type_ == PEDESTRIAN || BICYCLE) {
    speed_threshold = FLAGS_still_pedestrian_speed_threshold;  // 0.2 m/s
    pos_std = FLAGS_still_pedestrian_position_std;             // 0.5 m
  }
  ...
  if (history_size == 1) {
    if (speed < speed_threshold) is_still = true;              // 首帧按速度
    ...
  }
  // 多帧：滑动窗口 len ∈ [min_still_obstacle_history_length=4, max_...=10]
  //   speed < speed_threshold          → still
  //   speed_sensibility < threshold    → moving
  //   否则按窗口内位置漂移 distance > 2*distance_std → moving，否则 still
}
```

gflags 默认值（`modules/prediction/common/prediction_gflags.cc`，prediction.conf 未覆盖）：
- `prediction_trajectory_time_length = 6.0`（行人轨迹时长）
- `still_pedestrian_speed_threshold = 0.2`（m/s）
- `still_pedestrian_position_std = 0.5`
- `min/max_still_obstacle_history_length = 4 / 10`
- `max_history_time = 7.0`

### 2.4 评估器对静止障碍直接跳过

文件：`modules/prediction/evaluator/evaluator_manager.cc:71,256`

```cpp
if (obstacle_ptr->IsStill()) { continue; }   // 并行分组
if (obstacle->IsStill()) { continue; }       // 串行
```

→ 静止行人不会被 SEMANTIC_LSTM_EVALUATOR 评估，`feature.predicted_trajectory()` 为空。

### 2.5 轨迹生成（运动行人）`FreeMovePredictor::Predict`

文件：`modules/prediction/predictor/free_move/free_move_predictor.cc`

```cpp
double prediction_total_time = FLAGS_prediction_trajectory_time_length;  // 6.0s
if (feature.predicted_trajectory().empty()) {
  // 从当前位置+速度+加速度画 free-move 轨迹
  DrawFreeMoveTrajectoryPoints(position, velocity, acc, theta, 0.0,
                               prediction_total_time, ...);
} else {
  // 语义评估器已有轨迹 → 外推到 6s
}
```

---

## 3. 关键问题 1 的答案（时序上）

| 阶段 | prediction 判定 | 是否输出 trajectory | is_static | planning Obstacle id | ST boundary |
|------|----------------|--------------------|-----------|---------------------|-------------|
| 行人静止 | `IsStill()=true` | 否（EmptyPredictor） | true | `7673` | 静态竖直带 |
| 行人运动 | `IsStill()=false` | 是（FreeMove，6s） | false | `7673_0` | 动态斜带（随轨迹） |

**完全符合你的猜想**：行人静止时 prediction 不输出 trajectory → is_static=true → planning 生成 `7673`；行人一动就输出 trajectory 并新建 `_0` 子障碍。

---

## 4. 关键问题 2：静态 `7673` 会不会一直存在？

用 planning 日志（220748）按 ST boundary 构建逐条统计（`reference_line_info.cc:422`）：

```
22:07:50 ~ 22:08:40.36    → 全部是 id:7673        （静止标记，约 50s）
22:08:40.50 ~ 22:08:51.96 → 全部是 id:7673_0      （动态，约 11.5s，共 116 条）
22:08:52 ~ 22:09:15+      → 恢复 id:7673           （行人在 22:08:52 停稳）
```

- **单帧内永远只有一种 id**（prediction 输出的是单一 `PredictionObstacle`，不可能同帧"既有轨迹又没轨迹"），所以"同时存在"是**时间上的先后/交替**而非同帧并存。本次 220748 日志是一次干净切换；而 add12 提交（7abb9a6）记录的 2109 回放里出现过 `7673/7673_0` **逐帧交替** → STOP 决策闪烁（t=18.85/19.35 决策消失）。
- prediction 输出里，静态 `7673` 只要行人被判静止就**每帧都会输出**，不会"滞留过期副本"；但 planning 内部**按 id 索引的状态表会留下旧记录**（见 §6），这才是"旧标记不消失"的真正影响点。

---

## 5. 关键问题 3：planning 的 `IsStatic` 用 prediction 标记还是自己算速度？

**用 prediction 的标记，不算速度。**

文件：`modules/planning/planning_base/common/obstacle.cc:90-94` + `obstacle.h:82`

```cpp
is_static_ = (is_static || obstacle_priority == ObstaclePriority::IGNORE);
is_virtual_ = (perception_obstacle.id() < 0);
speed_ = std::hypot(perception_obstacle.velocity().x(),
                    perception_obstacle.velocity().y());   // 仅存值，IsStatic() 不用
```

- `IsStatic() { return is_static_; }` 直接取自 `prediction_obstacle.is_static()`，另有 `priority==IGNORE` 也强制静态。
- `IsVirtual()` = `perception_obstacle.id() < 0`（负 id = 虚拟障碍，如停止墙）。
- `Type()` = `perception_obstacle_.type()`（PEDESTRIAN 等，来自感知）。
- 含义：prediction 把 `7673_0` 标成动态（`is_static=false`），planning 就把它当动态走 `CheckIsFollow` → FOLLOW / `HandlePedestrianStop` 分支，即使其速度很小。

---

## 6. 关键问题 4：planning 侧如何识别"同一行人的静态+动态双标记"

两种等价手段：
1. **`Obstacle::PerceptionId()`**（obstacle.h:80，int32）：`7673` 与 `7673_0` 都返回 7673。
2. **对 `Obstacle::Id()` 去 `_` 后缀**：`id.substr(0, id.find('_'))`。

由于两个 Obstacle 共享同一份 `PerceptionObstacle`（位置/速度/类型相同），按基 id 合并后天然一致。

**已有实践（git 历史）**：
- `7abb9a6` add12「fence 用基 id 消除 STOP 闪烁」：`HandlePedestrianStop` 的记录/查找/erase 全部改为 `base_id = id.substr(0, id.find('_'))`，让 `7673` 与 `7673_0` 共享 `ped_fixed_fence_s_` 栅栏。
- `de34a95` add13「id 完全变化时按位置复用栅栏」：基 id 也对不上时的兜底。
- **注意**：当前 HEAD（`1695aef` revert 回退到 `c904b46`）**不含** add12/add13，`speed_decider.h:130` 的 `ped_fixed_fence_s_` 仍按原始 `Id()`（`"7673"`/`"7673_0"`）索引 —— 这就是当前代码重新面对该问题的原因。

另外，`CheckStopForPedestrian` 里 `pedestrian_stop_time` 计时表同样按 `obstacle->Id()` 索引，id 切换也会打断"静止计时"（`kPedestrianStopTimeout` 逻辑）。

---

## 7. 附：prediction 日志里为什么搜不到 7673

`modules/prediction/conf/prediction.conf` 末尾有：

```
# 彻底关闭 prediction 日志（仅FATAL），减少IO/内存
--minloglevel=3
```

所以 `prediction.log.INFO.20260801-220748.921790` 只有启动注册（predictor/evaluator/plugin）和 evaluator 耗时统计，**没有任何障碍物明细**。7673 的行为证据应来自：
- planning 日志 `reference_line_info.cc:422`（`build reference line st boundary. id:...`）
- 回放 dump：`scripts/dump_frame_obstacles.py`、`dump_ped_dec.py` 等（如 `output/dump_7673_1759a.txt`、`output/_analysis_7673/`）

---

## 8. 给后续调参/改动的可操作清单

1. 若想减少"移动窗口期"（`7673_0` 出现的 11.5s 太长）：调大 `still_pedestrian_speed_threshold`（0.2 → 0.5~1.0）或 `still_pedestrian_position_std`，让 prediction 更晚/更早判运动（需在 prediction.conf 加 flag，prediction 属于只读上游，谨慎）。
2. planning 侧合并双标记：栅栏/计时表 key 一律用 `PerceptionId()` 或去后缀的 base_id（复刻 add12/add13）。
3. 关注 `ObstaclePriority::IGNORE`：`is_static_ = is_static || priority==IGNORE`，对 7673_0 若想按静态处理，也可在 prediction 输出层给静止行人标 IGNORE（或 planning 侧识别 base_id 后统一按静态处理）。
