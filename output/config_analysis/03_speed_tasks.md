# Apollo 11.0 EDU — 速度相关 Task 配置参数全量分析报告

> 分析日期：2026-08-01
> 工程：`/home/skye/application-pnc`（Apollo 11.0 EDU 官方原始版本）
> 说明：本次为**纯分析**，未修改任何代码与配置文件。每个参数均标注：当前配置值、proto 默认值、代码位置、作用说明、对车辆行为的影响，以及与 8 大赛题的关联度。

## 关键背景知识（影响速度规划的全局量）

在分析前先列出与本报告强相关的全局参数（gflag / 车辆参数），它们与 Task 配置共同决定最终速度：

| 全局量 | 值 | 来源 | 影响 |
|--------|-----|------|------|
| `FLAGS_planning_upper_speed_limit` | 31.3 m/s | `planning_gflags.cc` | QP/NLP 速度上限兜底 |
| `FLAGS_longitudinal_jerk_lower/upper_bound` | -4.0 / 2.0 | `planning_gflags.cc` | QP/NLP 的 jerk 硬边界 |
| `FLAGS_min_stop_distance_obstacle` | 6.0 m | `planning_gflags.cc` | 静态障碍物 STOP 停车墙放在障碍物前 6m |
| `FLAGS_max_stop_distance_obstacle` | 10.0 m | `planning_gflags.cc` | 静态障碍物最大停车距离 |
| `FLAGS_follow_min_distance` | 3.0 m | `planning_gflags.cc` | 跟车最小距离（决策层兜底） |
| `FLAGS_yield_distance` | 5.0 m | `planning_gflags.cc` | 让行最小距离（决策层兜底） |
| `FLAGS_follow_time_buffer` | 2.5 s | `planning_gflags.cc` | 跟车时距缓冲 |
| `FLAGS_speed_lon_decision_horizon` | 200.0 m | `planning_gflags.cc` | DP 障碍物代价计算的距离范围 |
| `FLAGS_nonstatic_obstacle_nudge_l_buffer` | 0.4 m | `planning_gflags.cc` | 非换道时动态障碍物 nudge 横向缓冲 |
| `vehicle_param.max_acceleration` | 2.3 m/s² | `vehicle_param.pb.txt` | QP/NLP 加速度上界 |
| `vehicle_param.max_deceleration` | -7.0 m/s² | `vehicle_param.pb.txt` | QP/NLP 减速度下界 |
| `vehicle_param.max_abs_speed_when_stopped` | 0.03 m/s | `vehicle_param.pb.txt` | "已停车"速度阈值 |

---

# 1. SpeedDecider（跟车/停车/让行/超车距离决策）⭐重点

配置文件：`modules/planning/tasks/speed_decider/conf/default_conf.pb.txt`
Proto：`modules/planning/tasks/speed_decider/proto/speed_decider.proto`
核心代码：`modules/planning/tasks/speed_decider/speed_decider.cc`

**模块作用**：DP 速度规划完成后，`MakeObjectDecision()` 根据 ST 边界与速度剖面为每个障碍物打上纵向决策标签（STOP / FOLLOW / YIELD / OVERTAKE / IGNORE），并计算每个决策的"围栏距离"（fence distance）。后续 ST 边界映射和 QP 优化都以此为依据。

## 1.1 `follow_min_obs_lateral_distance: 2.5`（proto 默认 2.5）

- **代码位置**：`speed_decider.cc` → `CheckIsFollow()`：
  ```cpp
  const double obstacle_l_distance =
      std::min(std::fabs(obstacle.PerceptionSLBoundary().start_l()),
               std::fabs(obstacle.PerceptionSLBoundary().end_l()));
  if (obstacle_l_distance > config_.follow_min_obs_lateral_distance()) {
    return false;   // 横向距离过大 → 不判定为跟车
  }
  ```
- **作用**：判定"跟车"的最小横向距离门槛。取障碍物 SL 边界横向距离（start_l / end_l 绝对值）的最小值，若大于 2.5m，则该障碍物不构成跟车对象（通常落入 YIELD/IGNORE 分支）。
- **对车辆行为影响**：横向距离 < 2.5m 的前方障碍物才会被当作"跟车目标"；否则主车按让行或忽略处理。
- **赛题相关性**：★★★★★ 动态跟随、障碍物处理。若前方障碍物横向偏移较大，调大此值可让主车更积极跟车而非让行/停车。

## 1.2 `follow_min_time_sec: 2.0`（proto 默认 2.0）

- **代码位置**：`speed_decider.cc` → `CheckIsFollow()`：
  ```cpp
  // cross lane but be moving to different direction
  if (boundary.max_t() - boundary.min_t() < config_.follow_min_time_sec()) {
    return false;   // ST 边界时间跨度太短 → 不算跟车
  }
  ```
- **作用**：障碍物 ST 边界的时间跨度须 ≥2s 才判定为跟车。用于区分"横穿主车车道的障碍物"（ST 边界时间短）与"同向持续障碍物"（ST 边界时间长）。
- **对车辆行为影响**：横向穿行的行人/车辆不会被误判为跟车目标，从而避免主车长时间跟随横穿物。
- **赛题相关性**：★★★★★ 动态跟随、人行道行人避让。此参数直接决定"横穿障碍物是否触发跟车逻辑"。

## 1.3 `stop_follow_distance: 2.0`（proto 默认 2.0）

- **代码位置**：`speed_decider.cc` → `EstimateProperFollowGap()`：
  ```cpp
  double follow_distance = config_.stop_follow_distance();  // 跟车距离基准（截距）
  ```
- **作用**：跟车距离分段线性函数的**起点/截距**（速度 0 时的跟车距离）。整个跟车距离 = `stop_follow_distance` + 各速度段的斜率累计。
- **对车辆行为影响**：速度 0 时跟车距离 2.0m；随速度升高跟车距离增大（见 1.4）。该值是"停车/跟车距离"的下限基准，直接影响低速场景（减速带、拥堵）下主车与障碍物的间距。
- **赛题相关性**：★★★★★ 障碍物停车 ≥2m、动态跟随。这是控制"跟车/停车距离"最直接的两个参数之一（另一个是 1.4 的调度函数）。

## 1.4 `follow_distance_scheduler`（分段 (speed, slope) 表）⭐核心

配置文件内容：
```
follow_distance {
  speed: 0.0   slope: 2.0
}
follow_distance {
  speed: 2.7   slope: 1.0
}
follow_distance {
  speed: 5.4   slope: 0.75
}
follow_distance {
  speed: 8.1   slope: 0.5
}
```

- **代码位置**：
  - `Init()`（`speed_decider.cc` ~57 行）把每对 `(speed, slope)` 装入 `follow_distance_function_` 并排序；
  - `EstimateProperFollowGap()`（~612 行）分段线性插值：
    ```cpp
    double follow_distance = config_.stop_follow_distance();
    for (int i = 1; i < follow_distance_function_.size(); i++) {
      if (adc_speed <= follow_distance_function_[i].first) {
        follow_distance += follow_distance_function_[i-1].second *
                           (adc_speed - follow_distance_function_[i-1].first);
        break;
      } else {
        follow_distance += follow_distance_function_[i-1].second *
                           (follow_distance_function_[i].first -
                            follow_distance_function_[i-1].first);
      }
    }
    if (adc_speed > follow_distance_function_.back().first) {
      follow_distance += follow_distance_function_.back().second *
                         (adc_speed - follow_distance_function_.back().first);
    }
    ```
- **作用**：按自车速度计算**跟车距离**（跟随围栏距障碍物 ST 边界的最小 s 间隙）。计算结果 `follow_distance_s` 为负值，写入 FOLLOW 决策，`CreateFollowDecision()` 中：
  ```cpp
  const double reference_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + follow_distance_s;
  ```
  即跟随围栏放在"障碍物 ST 边界 − 跟车距离"处。
- **对车辆行为影响**（按当前配置计算）：
  | 自车速度 v (m/s) | 跟车距离 (m) |
  |---|---|
  | 0 | 2.0 |
  | 2.7 | 7.4 |
  | 5.4 | 10.1 |
  | 8.1 | 12.1 |
  | 10 | 13.1 |
  | 15 | 15.6 |
  速度越高跟车越远（坡度递减 → 高速时增量变缓）。
- **赛题相关性**：★★★★★ 动态跟随、障碍物停车、减速带。**这是控制跟车距离的核心参数**。若要"紧贴前车/慢速跟车"，调小各段 slope 与 `stop_follow_distance`；若要更安全跟车，调大。

## 1.5 配置中未显式设置、使用 proto 默认值的参数

以下参数在 `default_conf.pb.txt` 中**未出现**，因此运行时采用 proto 默认值，一并列出：

- **`is_stop_for_pedestrain = false`**（默认 false）
  - 代码：`MakeObjectDecision()` 中 `config_.is_stop_for_pedestrain() && CheckStopForPedestrian(...)` → 为 true 时行人**强制 STOP**（带 4s 停止计时器，行人静止超时后才放行）。
  - 影响：关闭时行人走 YIELD/FOLLOW 逻辑；开启时所有行人直接停车。
  - 赛题：★★★★★ 人行道/行人避让。若赛题要求"见行人必停"，应开启。
- **`keep_clear_last_point_speed = 0.8`**（默认 0.8）
  - 代码：`CheckKeepClearCrossable()`：速度剖面末点 s ≤ KEEP_CLEAR 边界 max_s **且**末点速度 < 0.8 m/s → 判定不可穿越（KEEP_CLEAR 被阻塞）。
  - 影响：KEEP_CLEAR（如公交站/禁停区）内低速则视为阻塞。
  - 赛题：★★☆☆☆ 公交站台（KEEP_CLEAR）相关。
- **`overtake_time_buffer = 3.0`、`overtake_min_distance = 10.0`**
  - 代码：`EstimateProperOvertakingGap()`：`超车间隙 = max(max(自车速度, 目标速度) × 3.0, 10.0)`。
  - 影响：决定超车围栏与障碍物的纵向间隙。
  - 赛题：★★★☆☆ 借道绕行/超车类场景。
- **`yield_distance_buffer = 5.0`**
  - 代码：`CreateYieldDecision()`：`yield_distance_s = max(-边界.min_s, -5.0)`，让行围栏放在障碍物前 5m。
  - 影响：让行停车点距障碍物的最小距离。
  - 赛题：★★★☆☆ 交叉口让行、行人让行。

> 补充：静态障碍物 STOP 的停车墙距离由 gflag `FLAGS_min_stop_distance_obstacle=6.0` 决定（`CreateStopDecision(..., -FLAGS_min_stop_distance_obstacle)`，围栏放在障碍物 ST 边界前 6m）。此值与 Task 配置无关，但直接影响"障碍物停车距离"，调参时需一并考虑。

---

# 2. SpeedBoundsDecider（速度边界 / 限速 / ST 边界映射）⭐重点

配置文件：`modules/planning/tasks/speed_bounds_decider/conf/default_conf.pb.txt`
Proto：`modules/planning/tasks/speed_bounds_decider/proto/speed_bounds_decider.proto`
核心代码：`speed_bounds_decider.cc`、`speed_limit_decider.cc`、`st_boundary_mapper.cc`

**模块作用**：① 把障碍物映射到 ST 图（`STBoundaryMapper`）；② 沿路径生成速度上限（`SpeedLimitDecider`，取地图限速/弯道向心限速/nudge 限速的最小值）；③ 装载 ST 图数据供 DP/QP 使用。

## 2.1 `total_time: 7.0`（proto 默认 7.0）

- **代码位置**：
  - `speed_bounds_decider.cc` Process()：`STBoundaryMapper(..., config_.total_time(), ...)`（ST 边界 t 轴范围）；
  - `st_graph_data->LoadData(..., total_time_by_conf, ...)`（t 轴搜索范围）。
- **作用**：ST 图的时间长度（秒）。DP/QP 只在 [0, total_time] 内搜索速度曲线。
- **对车辆行为影响**：7s 是默认规划时域。低速停车场景若 7s 内减不到 0，可能无法规划出完整停车轨迹（会触发 fallback）。
- **赛题相关性**：★★★★☆ 所有停车/减速赛题。停车距离较远或速度较高时需增大 total_time。

## 2.2 `boundary_buffer: 0.25`（proto 默认 0.1）

- **代码位置**：`st_boundary_mapper.cc` → `MapStopDecision()`：
  ```cpp
  point_pairs.emplace_back(
      STPoint(s_min, planning_max_time_),
      STPoint(s_max + speed_bounds_config_.boundary_buffer(), planning_max_time_));
  boundary.SetCharacteristicLength(speed_bounds_config_.boundary_buffer());
  ```
- **作用**：STOP 边界在时间末端（t=total_time）的 s 上界额外外扩 0.25m，同时作为边界的"特征长度"。保证停车墙在规划时域末端仍覆盖障碍物后方区域，避免 DP/QP 在最后一刻"穿透"停车边界。
- **对车辆行为影响**：停车边界在时域末端向后延伸，停车点更稳定。配置 0.25 比默认 0.1 略大，停车边界覆盖更充分。
- **赛题相关性**：★★★★☆ 人行道/红绿灯/停止标志停车。数值过小可能导致时域末端边界不足。

## 2.3 `max_centric_acceleration_limit: 0.8`（proto 默认 2.0）⭐

- **代码位置**：`speed_limit_decider.cc`：
  ```cpp
  const double speed_limit_from_centripetal_acc =
      std::sqrt(speed_bounds_config_.max_centric_acceleration_limit() /
                std::fmax(std::fabs(discretized_path.at(i).kappa()),
                          speed_bounds_config_.minimal_kappa()));
  ```
- **作用**：弯道限速的向心加速度上限。弯道限速 $v = \sqrt{a_{cent}/|\kappa|}$。配置值 0.8 m/s² 远小于默认 2.0，即**弯道/路口限速更保守**。
- **对车辆行为影响**：例如曲率 0.05 /m 处：$v=\sqrt{0.8/0.05}=4.0$ m/s（默认 2.0 时为 6.3 m/s）。**直接影响路口转弯、匝道限速**。
- **赛题相关性**：★★★★★ 绿灯路口 ≤5m/s、减速带。此参数是"弯道/路口限速"的最直接控制项；赛题要求路口低速时保持或调小该值。

## 2.4 `minimal_kappa`（配置未设置，proto 默认 0.00001）

- **代码位置**：`speed_limit_decider.cc`：`std::fmax(|kappa|, minimal_kappa)`，防止直道（kappa≈0）除以 0 导致限速无穷大。
- **作用**：向心限速公式中的最小曲率钳制值。
- **对车辆行为影响**：几乎无感知影响，仅数值稳定性。
- **赛题相关性**：无直接关系。

## 2.5 `point_extension: 0.0`（proto 默认 1.0）

- **代码位置**：`st_boundary_mapper.cc` → `GetOverlapBoundaryPoints()`：
  ```cpp
  lower_points->emplace_back(low_s - speed_bounds_config_.point_extension(), ...);
  upper_points->emplace_back(high_s + speed_bounds_config_.point_extension(), ...);
  ```
- **作用**：障碍物 ST 边界沿 s 方向的外扩量。配置为 0.0 表示**不做外扩**（默认 1.0m）。
- **对车辆行为影响**：ST 边界不额外放大，障碍物占用的 s 范围更精确；DP/QP 可在更贴近障碍物的位置行驶。若调大，障碍物"虚拟膨胀"，停车/让行距离更远、更保守。
- **赛题相关性**：★★★★☆ 障碍物停车、借道绕行。调大可增加绕障安全余量；调小可更贴近障碍物通过。

## 2.6 `lowest_speed: 0.1`（proto 默认 2.5）⭐

- **代码位置**：`speed_limit_decider.cc`：
  ```cpp
  curr_speed_limit =
      std::fmax(speed_bounds_config_.lowest_speed(),
                std::min({speed_limit_from_reference_line,
                          speed_limit_from_centripetal_acc,
                          speed_limit_from_nearby_obstacles}));
  ```
- **作用**：最终限速的**下限**。配置从默认 2.5 m/s 降到 **0.1 m/s**。
- **对车辆行为影响**：默认 2.5 意味着即使前方要停，限速也不会低于 2.5 m/s（车辆始终保持低速滑行）；配置为 0.1 后，限速可降至接近 0，**允许车辆真正完全停下**。这是本工程为"停车类赛题"做的关键调参。
- **赛题相关性**：★★★★★ 人行道/红绿灯/停止标志停车、障碍物停车。**必须保持 0.1**（或更小）才能让车辆在停车线前完全停稳；若改回默认 2.5，车辆可能停不住。

## 2.7 `collision_safety_range`（配置未设置，proto 默认 1.0）

- **代码位置**：`speed_limit_decider.cc`：判断自车与 nudge 障碍物的横向距离是否进入安全范围（`is_close_on_left/right` 计算）。
- **作用**：nudge 时判定"障碍物是否贴得太近需要降速"的横向安全距离。
- **对车辆行为影响**：横向距离 < 安全范围 1.0m 时触发 nudge 限速。
- **赛题相关性**：★★★☆☆ 借道绕行（贴障通行）。

## 2.8 `static_obs_nudge_speed_ratio: 0.6`

- **代码位置**：`speed_limit_decider.cc`：
  ```cpp
  if (ptr_obstacle->IsStatic()) {
    nudge_speed_ratio = speed_bounds_config_.static_obs_nudge_speed_ratio();  // 0.6
  } else {
    nudge_speed_ratio = speed_bounds_config_.dynamic_obs_nudge_speed_ratio(); // 0.8
  }
  speed_limit_from_nearby_obstacles = nudge_speed_ratio * speed_limit_from_reference_line;
  ```
- **作用**：静态障碍物 nudge（绕行）时，限速降为参考线限速的 0.6 倍。
- **对车辆行为影响**：绕行静态障碍物（锥桶、违停车辆）时明显减速（限速×0.6）。
- **赛题相关性**：★★★★☆ 借道绕行、锥桶避障。调小比值 → 绕障更慢更稳。

## 2.9 `dynamic_obs_nudge_speed_ratio: 0.8`

- **代码位置**：同上（动态障碍物分支）。
- **作用**：动态障碍物 nudge 时限速为参考线限速的 0.8 倍。
- **对车辆行为影响**：绕行动态障碍物时轻度减速。
- **赛题相关性**：★★★☆☆ 动态跟随/绕行。

## 2.10 `enable_nudge_slowdown: true`（proto 默认 true）

- **代码位置**：`speed_limit_decider.cc`：
  ```cpp
  if (speed_bounds_config_.enable_nudge_slowdown()) {
    curr_speed_limit = fmax(lowest_speed, min({ref_limit, centripetal_limit, nearby_limit}));
  } else {
    curr_speed_limit = fmax(lowest_speed, min({ref_limit, centripetal_limit}));
  }
  ```
- **作用**：总开关。为 true 时把 nudge 限速纳入最终限速；false 时忽略 nudge 限速（只受地图与弯道限速）。
- **对车辆行为影响**：关闭后绕障不再减速。
- **赛题相关性**：★★★★☆ 借道绕行、锥桶。通常保持 true。

## 2.11 `lane_change_obstacle_nudge_l_buffer: 0.3`（proto 默认 0.3）

- **代码位置**：`st_boundary_mapper.cc` → `GetOverlapBoundaryPoints()`：
  ```cpp
  double l_buffer =
      planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE
          ? speed_bounds_config_.lane_change_obstacle_nudge_l_buffer()  // 0.3
          : FLAGS_nonstatic_obstacle_nudge_l_buffer;                    // 0.4
  ```
- **作用**：**换道过程中**障碍物 ST 边界映射用的横向缓冲（0.3m）；非换道时用 gflag 0.4m。
- **对车辆行为影响**：换道时障碍物占用范围判定更宽松（缓冲更小），避免换道中误报碰撞。
- **赛题相关性**：★★★☆☆ 换道/变道赛题。

## 2.12 `max_trajectory_len: 1000.0`（proto 默认 1000.0）

- **代码位置**：`st_boundary_mapper.cc`：`auto path_len = std::min(config_.max_trajectory_len(), ...)`。
- **作用**：ST 边界映射的最大路径长度上限（1000m），防止异常超长路径。
- **对车辆行为影响**：正常场景不触发。
- **赛题相关性**：无直接关系。

---

# 3. STBoundsDecider（ST 可行驶边界）

配置文件：`modules/planning/tasks/st_bounds_decider/conf/default_conf.pb.txt`
Proto：`modules/planning/tasks/st_bounds_decider/proto/st_bounds_decider.proto`
核心代码：`st_bounds_decider.cc`

**模块作用**（`use_st_drivable_boundary` 开启时生效）：以"可行驶区域"方式生成 ST 上下边界（STOP/YIELD 为 s 上界、OVERTAKE 为下界、FOLLOW 为动态上界），替代逐障碍物 ST 边界，供 DP/QP 使用。

## 3.1 `total_time: 7.0`（proto 默认 7.0）

- **代码位置**：
  - `st_bounds_decider.cc` → `InitSTBoundsDecider()`：`st_obstacles_processor_.Init(path_len, config_.total_time(), ...)`（障碍物 ST 边界映射 t 轴）；
  - `GenerateFallbackSTBound()`：`for (curr_t = 0; curr_t <= config_.total_time(); ...)`（fallback 边界生成）。
- **作用**：ST 可行驶边界的时间长度（秒），与 SpeedBoundsDecider 的 `total_time` 对齐（均 7.0）。
- **对车辆行为影响**：决定可行驶 ST 边界覆盖的时域；小于 SpeedBoundsDecider 的 total_time 时边界不完整。
- **赛题相关性**：★★★★☆ 与 2.1 相同，停车/减速场景需保持一致性。

> 注：该 Task 仅在 `FLAGS_use_st_drivable_boundary=true` 时被启用（与 SpeedBoundsDecider 二选一/先后关系取决于 pipeline）。本工程两个配置的 total_time 均为 7.0，保持一致。

---

# 4. PathTimeHeuristic（DP 速度规划 / DP-ST 搜索）⭐

配置文件：`modules/planning/tasks/path_time_heuristic/conf/default_conf.pb.txt`
Proto：`modules/planning/tasks/path_time_heuristic/proto/path_time_heuristic.proto`
核心代码：`path_time_heuristic_optimizer.cc`、`gridded_path_time_graph.cc`、`dp_st_cost.cc`

**模块作用**：在 ST 图上做动态规划，搜索一条粗速度剖面（DP 结果），作为 QP/NLP 的初值与参考线。包含两套子配置：
- `default_speed_config`：普通跟车场景；
- `lane_change_speed_config`：换道场景（`path_time_heuristic_optimizer.cc` 中按 `IsChangeLanePath()` 二选一）。

## 4.1 网格采样参数（`default_speed_config` / `lane_change_speed_config`）

### `unit_t: 1.0`（默认 1.0）
- **代码**：`gridded_path_time_graph.cc`：`unit_t_ = config.unit_t()`，时间轴采样间隔；`dimension_t = ceil(total_time/unit_t)+1`。`dp_st_cost.cc` 中所有速度/加速度/jerk 计算均以 `unit_t` 为差分步长。
- **作用**：DP 时间采样分辨率（1s）。越小网格越密、解越精细但计算量越大。
- **赛题**：★★★☆☆ 停车精度。可调小以更精细逼近停车曲线。

### `dense_dimension_s: 101`（默认 41）/ `dense_unit_s: 0.1`（默认 0.5）/ `sparse_unit_s: 1.0`（默认 1.0）
- **代码**：`InitCostTable()`：前 `dense_dimension_s` 个 s 网格用 `dense_unit_s` 步长（0.1m），之后用 `sparse_unit_s`（1.0m）。
- **作用**：s 轴双重分辨率——近端 0.1m 加密（前 101 点 ≈ 前 10m），远端 1m 稀疏。配置比默认更密（101 点 / 0.1m），**低速近端 s 分辨率提高 5 倍**，停车/贴障决策更精细。
- **赛题**：★★★★☆ 停车距离控制。加密近端网格有助于精确控制停车位置。

### `max_acceleration: 2.0`（默认 4.5）/ `max_deceleration: -4.0`（默认 -4.5）
- **代码**：`gridded_path_time_graph.cc`：`max_acceleration_ = min(|vehicle.max_acc|, |config.max_acc|)`；`max_deceleration_ = -min(...)`。DP 搜索中加速度/减速度硬边界。
- **作用**：DP 阶段允许的最大加/减速。配置 max_acc=2.0 比默认 4.5 小，max_dec=-4.0 比默认 -4.5 稍小。
- **对车辆行为影响**：DP 粗解不会产生超过 ±2.0/-4.0 的加速度，间接限制最终加减速上限（QP 还会再受车辆参数 2.3/-7.0 约束）。
- **赛题**：★★★★★ 加减速度控制。减速带、停车场景希望平稳减速时可调小 |max_deceleration|；注意 `lane_change_speed_config` 中为 -2.5（换道更平缓）。

## 4.2 代价权重（`default_speed_config`）

### `speed_weight: 0.0`（默认 0.0）
- **代码**：`gridded_path_time_graph.cc` 中 `GetSpeedCost` 由各 penalty 权重组成；`speed_weight` 本身在本版本 DP 代价中未直接使用（保留字段）。
- **作用**：保留字段（无实际影响）。
- **赛题**：无。

### `accel_weight: 10.0`（默认 10.0）/ `jerk_weight: 10.0`（默认 10.0）
- **代码**：`dp_st_cost.cc` `GetAccelCost()`/`JerkCost()` 中 `accel_penalty * accel²`、`jerk_coeff * jerk²` 的缩放基础（与 `accel_penalty`/`positive_jerk_coeff` 相乘）。注：`accel_weight`/`jerk_weight` 在本版本代码中未直接读取，实际加速度代价由 `accel_penalty`/`decel_penalty` 控制（见 4.4）。
- **作用**：平滑性相关保留字段。
- **赛题**：★★☆☆☆。

### `obstacle_weight: 1.0`（默认 1.0）
- **代码**：`dp_st_cost.cc` `GetObstacleCost()`：`cost += obstacle_weight * default_obstacle_cost * s_diff²`。
- **作用**：障碍物接近代价的全局缩放。越大 DP 越"远离"障碍物。
- **赛题**：★★★★☆ 障碍物停车/绕行。调大 → 更早、更远避让障碍物。

### `reference_weight: 0.0`（默认 0.0）
- **代码**：`GetReferenceCost()`：`reference_weight * (s - s_ref)² * unit_t`。
- **作用**：对参考 ST 点的偏离代价（本工程为 0，不生效）。
- **赛题**：无。

### `go_down_buffer: 5.0` / `go_up_buffer: 5.0`（默认 5.0）⚠️ 死参数
- **代码**：全 planning 目录搜索 `go_down_buffer`/`go_up_buffer`，**仅在 proto 与 conf 中出现，无任何 .cc/.h 使用**。
- **作用**：本版本为**未生效参数**（保留字段）。README 中无对应说明。
- **赛题**：无（调了也不生效，需改代码）。

### `default_obstacle_cost: 1e4`（默认 1e10）
- **代码**：`GetObstacleCost()`：`cost += obstacle_weight * default_obstacle_cost * s_diff²`。
- **作用**：接近障碍物时的二次代价系数。配置 1e4 远小于默认 1e10，表示"靠近障碍物代价相对温和"（默认 1e10 几乎等于硬约束）。
- **对车辆行为影响**：DP 允许一定程度贴近障碍物（配合后续 QP/STOP 硬边界兜底）；若调大则 DP 提前绕开。
- **赛题**：★★★★☆ 贴障通行/绕行。小值利于近距离通过窄道。

### `default_speed_cost: 1e3`（默认 1.0）
- **代码**：`GetSpeedCost()` 中作为所有速度 penalty 的**公共缩放因子**（乘到 exceed/low/reference penalty 上）。
- **作用**：速度代价总缩放。1e3 使速度偏离代价被放大，DP 更倾向于贴近限速巡航。
- **赛题**：★★★★☆ 限速类赛题。调大 → 更贴限速；调小 → 速度偏离容忍度更高。

### `exceed_speed_penalty: 1e3`（默认 10.0）
- **代码**：`GetSpeedCost()`：`det_speed=(v-v_limit)/v_limit > 0` 时 `cost += exceed_speed_penalty * default_speed_cost * det_speed² * unit_t`。
- **作用**：**超速惩罚**。配置 1e3 × default_speed_cost(1e3) = 1e6 级，超速代价极大 → DP 几乎不会超出限速。
- **赛题**：★★★★★ 减速带 ≤3m/s、无人人行道 ≤5m/s、绿灯路口 ≤5m/s。此参数保证 DP 严格服从速度上限。

### `low_speed_penalty: 10.0`（默认 2.5）
- **代码**：`GetSpeedCost()`：`det_speed < 0` 时 `cost += low_speed_penalty * default_speed_cost * (-det_speed) * unit_t`。
- **作用**：**低速惩罚**（线性）。鼓励尽量以接近限速行驶、避免长时间龟速。
- **赛题**：★★★☆☆。若希望"慢速通过"（如减速带），可调小 low_speed_penalty 减少慢速代价；若希望快速通行，调大。

### `reference_speed_penalty: 10.0`（默认 1.0）
- **代码**：`GetSpeedCost()`：`if (enable_dp_reference_speed) cost += reference_speed_penalty * default_speed_cost * |v - cruise_speed| * unit_t`。
- **作用**：对偏离**巡航速度**（参考线巡航速度）的惩罚（线性）。配置 10.0 比默认 1.0 大 → DP 更积极贴近巡航速度。
- **赛题**：★★★★☆ 决定"正常行驶段"速度；在限速场景需与 exceed_speed_penalty 平衡。

### `keep_clear_low_speed_penalty: 10.0`（默认 10.0）
- **代码**：`GetSpeedCost()`：速度 < 停车阈值且位于 KEEP_CLEAR 范围时 `cost += keep_clear_low_speed_penalty * unit_t * default_speed_cost`。
- **作用**：在 KEEP_CLEAR（禁停区）内低速停留的惩罚，防止停在禁停区。
- **赛题**：★★★☆☆ 公交站台（KEEP_CLEAR）场景。

## 4.3 `default_speed_config` 加/减速度与 jerk 惩罚

### `accel_penalty: 1.0`（默认 2.0）/ `decel_penalty: 1.0`（默认 2.0）
- **代码**：`GetAccelCost()`：加速时 `cost = accel_penalty * accel²`；减速时 `cost = decel_penalty * accel²`，并叠加两个 sigmoid 软约束项（对 `max_acceleration`/`max_deceleration` 的软惩罚）。
- **作用**：DP 中加速度代价。配置均为 1.0（比默认 2.0 小）→ 对加速度惩罚更轻，DP 加减速更"大胆"。
- **赛题**：★★★★☆ 加减速度控制。调大 → 加减速更平滑。

### `positive_jerk_coeff: 1.0`（默认 1.0）/ `negative_jerk_coeff: 1.0`（默认 300.0）
- **代码**：`JerkCost()`：`jerk>0 ? positive_jerk_coeff*jerk² : negative_jerk_coeff*jerk²`。
- **作用**：jerk（加加速度）惩罚系数。配置将 negative_jerk_coeff 从默认 300 降到 **1.0** → **强烈允许负 jerk（快速减速）**。
- **对车辆行为影响**：减速过程 jerk 代价大幅降低，DP 可生成更急促的减速（配合 QP 仍有 jerk 硬边界 -4~2 限制）。对"尽快停下来"有利，但舒适性下降。
- **赛题**：★★★★★ 停车类赛题（要求稳定停在 1.5-2.0m 处）。此调参明显倾向"快停"。

## 4.4 `spatial_potential_penalty: 1e2`（默认 1.0）

- **代码**：`GetSpatialPotentialCost()`：`(total_s - s) * spatial_potential_penalty`。
- **作用**：**距离终点（s 前进）的奖励**——走得越远代价越小（等效推力）。配置 1e2 远大于默认 1.0 → DP 强烈倾向于"尽快往前走"（最小时间遍历）。
- **对车辆行为影响**：车辆更积极地前进、避免停在原地；在必须停车的点由 STOP 硬边界（kInf 代价）覆盖。
- **赛题**：★★★★★ 通行效率类赛题。注意：此值过大可能使 DP 在减速带/停车线前"冲到最后一刻才停"，需与 max_deceleration、STOP 边界配合。
- `lane_change_speed_config.spatial_potential_penalty = 1e5`（更大）→ 换道时更积极前进。

## 4.5 开关参数

### `enable_multi_thread_in_dp_st_graph: false`（默认 false）
- **代码**：`gridded_path_time_graph.cc` `CalculateTotalCost()` 中按此开关并行计算曲线代价。
- **作用**：单线程（本工程关闭）。
- **赛题**：无（性能项）。

### `enable_dp_reference_speed: true`（默认 true）
- **代码**：`GetSpeedCost()` 中启用后叠加 `reference_speed_penalty` 项。
- **作用**：DP 结果向巡航速度靠拢。
- **赛题**：★★★★☆ 与 reference_speed_penalty 联动。

### `is_lane_changing: true`（仅 lane_change_speed_config，默认 false）
- **代码**：标记字段（本版本代码中未直接读取该字段做分支，配置选择由 `IsChangeLanePath()` 完成）。
- **作用**：标记换道配置。
- **赛题**：★★☆☆☆。

### `lane_change_speed_config` 与 default 的关键差异
- `dense_dimension_s: 21`、`dense_unit_s: 0.25`：换道时 s 网格更粗（计算量小）；
- `max_deceleration: -2.5`：换道减速更平缓；
- `spatial_potential_penalty: 1e5`：换道更积极前进；
- 其余权重与 default 一致。

---

# 5. PiecewiseJerkSpeed（QP 速度平滑）⭐重点

配置文件：`modules/planning/tasks/piecewise_jerk_speed/conf/default_conf.pb.txt`
Proto：`modules/planning/tasks/piecewise_jerk_speed/proto/piecewise_jerk_speed.proto`
核心代码：`piecewise_jerk_speed_optimizer.cc`

**模块作用**：以 DP 结果为参考（warm start 参考线），用分段 jerk QP 平滑速度曲线，得到最终速度剖面。ST 边界中 STOP/YIELD/FOLLOW 作为 s 上界（硬约束），OVERTAKE 作为 s 下界。

## 5.1 `acc_weight: 1.0`（proto 默认 1.0）

- **代码**：`piecewise_jerk_speed_optimizer.cc`：`piecewise_jerk_problem.set_weight_ddx(config_.acc_weight())`。
- **作用**：QP 目标函数中**加速度项**权重。加速度上限由车辆参数（2.3/-7.0）与 jerk 边界（-4/2）约束。
- **对车辆行为影响**：权重越大 → 加速度越小、越平缓。配置 1.0 与默认一致。
- **赛题**：★★★★☆ 加减速度控制。

## 5.2 `jerk_weight: 3.0`（proto 默认 10.0）

- **代码**：`set_weight_dddx(config_.jerk_weight())`。
- **作用**：QP 目标中 **jerk（加加速度）项**权重。配置从默认 10.0 降到 **3.0**。
- **对车辆行为影响**：jerk 惩罚减轻 → 速度曲线更"激进"（允许更快的加减速变化），响应更快但舒适性略降。**配合 DP 的 negative_jerk_coeff=1.0，整体减速更利落**。
- **赛题**：★★★★★ 停车/限速转换场景。停车需要较快减速时可保持较小值；若追求平顺可调大。

## 5.3 `kappa_penalty_weight: 200.0`（proto 默认 1000.0）

- **代码**：
  ```cpp
  PathPoint path_point = path_data.GetPathPointWithPathS(path_s);
  penalty_dx.push_back(std::fabs(path_point.kappa()) * config_.kappa_penalty_weight());
  ```
  `set_penalty_dx(penalty_dx)` 作为对速度的曲率惩罚（弯道处惩罚高速）。
- **作用**：**弯道速度惩罚**——路径曲率越大，对速度的二次惩罚越强。配置 200 比默认 1000 小 → **弯道降速更温和**。
- **对车辆行为影响**：弯道限速主要由 SpeedBoundsDecider 的向心限速（硬上界）保证；此权重是软性惩罚，让 QP 主动在弯道减速。配置偏小 → 弯道速度更接近上限。
- **赛题**：★★★★★ 绿灯路口 ≤5m/s、减速带。若需路口/弯道更慢，调大此权重或调小 `max_centric_acceleration_limit`。

## 5.4 `ref_s_weight: 1.0`（proto 默认 10.0）

- **代码**：`piecewise_jerk_problem.set_x_ref(config_.ref_s_weight(), std::move(x_ref))`，`x_ref` 来自 DP 参考速度曲线的 s(t)。
- **作用**：对**参考位置（DP 曲线 s）**的跟随权重。配置从默认 10.0 降到 **1.0**。
- **对车辆行为影响**：QP 不那么严格跟随 DP 的 s(t) 形状，有更大自由度生成自己的位置曲线（可偏离 DP）。
- **赛题**：★★★★☆ 停车位置控制。调大 → QP 更贴近 DP（DP 越准越有利）；调小 → QP 自主性更强。

## 5.5 `ref_v_weight: 10.0`（proto 默认 10.0）

- **代码**：`std::vector<double> dx_ref_weight(num_of_knots, config_.ref_v_weight());` → `set_dx_ref(dx_ref_weight, dx_ref)`，`dx_ref` 为巡航速度（受限速钳制）。
- **作用**：对**参考速度（巡航速度）**的跟随权重。
- **对车辆行为影响**：权重越大越贴近巡航速度；在限速/停车段由速度上界（硬约束）接管。
- **赛题**：★★★★☆ 限速类赛题。正常段巡航速度保持靠它。

## 5.6 `follow_distance_buffer: 0.1`（proto 默认 8.0）⚠️ 配置已改但代码未生效

- **代码**：在 `piecewise_jerk_speed_optimizer.cc` 中 **未读取**该字段（FOLLOW 分支直接 `s_upper_bound = fmin(s_upper_bound, s_upper)`，注释 `// TODO(Hongyi): unify follow buffer on decision side`）。全目录搜索仅 proto/conf/README 出现。
- **作用**：设计意图为"跟车时在 FOLLOW 边界基础上再留 0.1m 缓冲"，但**本版本代码未实现**。
- **对车辆行为影响**：**当前不生效**。跟车距离实际由 speed_decider 的 `follow_distance_scheduler` + `stop_follow_distance` 决定（见第 1 节）。
- **赛题**：★★★★★ 动态跟随。注意：**改此值无效**，要调整跟车距离须改 speed_decider 配置。

---

# 6. PiecewiseJerkSpeedNonlinear（NLP 非线性速度优化）

配置文件：`modules/planning/tasks/piecewise_jerk_speed_nonlinear/conf/default_conf.pb.txt`
Proto：`modules/planning/tasks/piecewise_jerk_speed_nonlinear/proto/piecewise_jerk_speed_nonlinear.proto`
核心代码：`piecewise_jerk_speed_nonlinear_optimizer.cc`

**模块作用**：QP 平滑后再用 IPOPT 非线性优化（NLP），目标函数含向心加速度、参考速度、软边界等非线性项，输出最终速度曲线。流程：QP（warm start）→ 曲率/限速平滑 → NLP。

## 6.1 `acc_weight: 2.0`（proto 默认 500.0）

- **代码**：QP 阶段 `set_weight_ddx(config_.acc_weight())`；NLP 阶段 `ptr_interface->set_w_overall_a(config_.acc_weight())`。
- **作用**：整体**加速度权重**。配置 2.0 远小于默认 500 → 对加速度的惩罚非常轻。
- **对车辆行为影响**：允许较大加速度变化，速度曲线更"敢加敢减"。
- **赛题**：★★★★★ 加减速度控制。若需平稳，调大。

## 6.2 `jerk_weight: 3.0`（proto 默认 100.0）

- **代码**：`set_weight_dddx(...)` / `set_w_overall_j(...)`。
- **作用**：**jerk 权重**。配置 3.0 远小于默认 100 → jerk 惩罚极轻。
- **对车辆行为影响**：加减速切换更突兀（jerk 大），响应快。与 QP 的 jerk_weight=3.0 一致，整体调参方向是"允许快加减速"。
- **赛题**：★★★★★ 停车类（快停）与舒适性权衡。

## 6.3 `lat_acc_weight: 1000.0`（proto 默认 500.0）

- **代码**：`ptr_interface->set_w_overall_centripetal_acc(config_.lat_acc_weight())`。
- **作用**：**向心（横向）加速度权重**。配置 1000 比默认 500 大 → 更强烈惩罚横向加速度。
- **对车辆行为影响**：弯道/路口速度被 NLP 更积极地压低（横向加速度 $v^2\cdot\kappa$ 受惩罚），配合 speed_bounds 的向心限速。
- **赛题**：★★★★★ 绿灯路口 ≤5m/s、弯道。**调大此值可显著压低转弯速度**。

## 6.4 `s_potential_weight: 0.05`（proto 默认 10.0）

- **代码**：当 `use_smoothed_dp_guide_line=false` 时 `set_w_reference_spatial_distance(config_.s_potential_weight())`；本工程为 true 走 DP 指引线分支，权重固定 10.0（代码内写死）。
- **作用**：最小时间遍历（推进 s）权重。
- **对车辆行为影响**：本工程因 `use_smoothed_dp_guide_line=true`，此参数**不生效**（使用固定 10.0 的 DP 指引线权重）。
- **赛题**：★★☆☆☆（当前不生效）。

## 6.5 `ref_v_weight: 5.0`（proto 默认 10.0）

- **代码**：`ptr_interface->set_w_reference_speed(config_.ref_v_weight())`，参考速度为巡航速度。
- **作用**：对**巡航速度**的软性吸引权重。配置 5.0 比默认 10.0 小 → 对巡航速度跟随较弱。
- **对车辆行为影响**：速度不必严格贴近巡航速度，有更多自由（尤其在限速/停车过渡段）。
- **赛题**：★★★★☆ 限速类。若需保持巡航，调大。

## 6.6 `ref_s_weight: 100.0`（proto 默认 10.0）

- **代码**：QP 阶段 `set_x_ref(config_.ref_s_weight(), x_ref)`（x_ref 为 DP 曲线的 s(t)）。
- **作用**：对**DP 指引线位置**的跟随权重。配置 100.0 远大于默认 10.0 → **QP 阶段强跟随 DP**。
- **对车辆行为影响**：QP 结果几乎贴着 DP 曲线走，DP 的停车点/跟车点被保留；若 DP 不准，问题会被放大。
- **赛题**：★★★★★ 停车位置/跟车距离。DP（第 4 节）与 QP 联动，改 DP 权重即可联动。

## 6.7 `soft_s_bound_weight: 1e6`（proto 默认 10.0）

- **代码**：`if (use_soft_bound_in_nonlinear_speed_opt) ptr_interface->set_w_soft_s_bound(config_.soft_s_bound_weight())`。
- **作用**：软边界违反惩罚权重。配置 1e6 极大（默认 10）→ **软边界几乎等同硬约束**。
- **对车辆行为影响**：本工程 `use_soft_bound_in_nonlinear_speed_opt=false`（见 6.10），**此参数当前不生效**。
- **赛题**：★★★☆☆（若开启软边界则控制停车边界严格度）。

## 6.8 `use_warm_start: true`（proto 默认 true）

- **代码**：`OptimizeByNLP()` 中把 QP 结果 `(s,v,a)` 作为 NLP 初值。
- **作用**：NLP 热启动，收敛更快更稳。
- **赛题**：无（求解器设置）。

## 6.9 `use_smoothed_dp_guide_line: true`（proto 默认 false）

- **代码**：`OptimizeByNLP()`：true → `set_reference_spatial_distance(*distance)`（QP/DP 平滑结果作为空间参考，权重固定 10.0）；false → 用常数参考 + `s_potential_weight`。
- **作用**：把 DP 平滑后的 s(t) 作为 NLP 空间参考。配置 true（默认 false）→ **NLP 更贴近 DP 结果**。
- **赛题**：★★★★☆ 让最终曲线继承 DP 的停车/跟车意图。

## 6.10 `use_soft_bound_in_nonlinear_speed_opt: false`（proto 默认 true）⭐

- **代码**：
  - `SetUpStatesAndBounds()`：true 时生成 `s_bounds_` + `s_soft_bounds_`（FOLLOW 软上界 = `s_upper - min(7.0, 2.5*v)`，OVERTAKE 软下界 = `s_lower + 10.0`）；false 时仅硬边界，且 FOLLOW 硬上界 = `s_upper - 8.0`。
  - `OptimizeByNLP()`：true 时传入软边界与 `soft_s_bound_weight`。
- **作用**：是否使用**软安全边界**。配置 **false**（关闭软边界，用纯硬边界）。
- **对车辆行为影响**：
  - 硬边界模式下 FOLLOW 上界 = `s_upper - 8.0`（跟车时强制留 8m），STOP/YIELD 上界 = `s_upper`（严格不能越过停车线）；
  - 软边界模式下 FOLLOW 上界 = `s_upper - min(7, 2.5*v)`（速度相关间隙），允许轻微越界（代价 1e6）。
  - 关闭软边界 → 停车/跟车约束更严格，**更利于满足"停车 1.5-2.0m / 跟车 ≥2m"的硬性要求**。
- **赛题**：★★★★★ 所有停车/跟车距离赛题。保持 false 可保证严格停车；若发现频繁 infeasible，可临时开 true 放宽。

---

# 7. RuleBasedStopDecider（规则停车决策）⭐重点

配置文件：`modules/planning/tasks/rule_based_stop_decider/conf/default_conf.pb.txt`
Proto：`modules/planning/tasks/rule_based_stop_decider/proto/rule_based_stop_decider.proto`
核心代码：`rule_based_stop_decider.cc`

**模块作用**：规则化的停车决策——① 侧向借道（side pass）停车；② 紧急换道停车（默认关）；③ 路径末端停车（过短路径）。核心判断"主车是否已在停车点停稳"用 `CheckADCStop()`。

## 7.1 `max_adc_stop_speed: 0.5`（proto 默认 0.3）⭐

- **代码**：`CheckADCStop()`：
  ```cpp
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  if (adc_speed > config_.max_adc_stop_speed()) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;   // 车速 > 0.5 视为未停车
  }
  ```
- **作用**：判定"主车已停稳"的速度阈值。配置 0.5 m/s 比默认 0.3 宽松。
- **对车辆行为影响**：车速 < 0.5 m/s 即认为已停车。用于侧向借道/换道等场景判断"是否可以放行/继续"。
- **赛题**：★★★★☆ 停车判定。数值越大越容易判定"已停车"（可能提前进入下一阶段）。

## 7.2 `max_valid_stop_distance: 1.0`（proto 默认 0.5）⭐

- **代码**：`CheckADCStop()`：
  ```cpp
  const double distance_stop_line_to_adc_front_edge = stop_point_s - adc_front_edge_s;
  if (distance_stop_line_to_adc_front_edge > config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;   // 车头距停止线 > 1.0m 视为"无效停车"
  }
  ```
- **作用**：判定"停车是否到位"的允许误差（车头前沿到停车线的距离）。配置 1.0m 比默认 0.5m 宽松。
- **对车辆行为影响**：停车点与停止线偏差 ≤1.0m 都算有效停车。**注意：此值只是"判定停车是否合格"的容差，不是停车距离本身**；实际停车距离由 STOP 决策围栏（场景配置 + `FLAGS_min_stop_distance_obstacle`）决定。
- **赛题**：★★★★★ 人行道/红绿灯/停止标志停车（1.5-2.0m）。若要求"停得离停止线更近"，需同时调小此容差并检查场景停车配置。

## 7.3 `search_beam_length: 20.0`（proto 默认 5.0）

- **代码**：`StopOnSidePass()` → `IsPerceptionBlocked(reference_line_info, config_.search_beam_length(), ...)`。
- **作用**：侧向借道盲区**搜索光束长度**（向前探测距离）。配置 20m 远大于默认 5m。
- **对车辆行为影响**：探测范围更远，更保守地判断"侧向是否被遮挡"。
- **赛题**：★★★☆☆ 借道绕行/侧向借道。

## 7.4 `search_beam_radius_intensity: 0.08`（proto 默认 0.08）

- **代码**：`IsPerceptionBlocked()` 光束半径/密度参数（与 search_beam_length 配合）。
- **作用**：盲区搜索光束的半径强度。
- **赛题**：★★☆☆☆。

## 7.5 `search_range: 3.14`（proto 默认 3.14）

- **代码**：`IsPerceptionBlocked()` 光束搜索角度范围（弧度，3.14≈180°）。
- **作用**：侧向盲区搜索角度。
- **赛题**：★★☆☆☆。

## 7.6 `is_block_angle_threshold: 0.5`（proto 默认 1.57）⭐

- **代码**：`IsPerceptionBlocked()` 中判断障碍物是否构成遮挡的角度阈值。
- **作用**：判定"被遮挡"的角度门槛。配置 0.5 rad 远小于默认 1.57 rad → **更容易判定为被遮挡**（更保守）。
- **对车辆行为影响**：借道时对侧向障碍物更敏感，更易触发停车等待。
- **赛题**：★★★★☆ 借道绕行安全性。调大则更激进（更少停车）。

## 7.7 `enable_lane_change_urgency_checking: false`（proto 默认 false）

- **代码**：`Process()` 中 `if (config_.enable_lane_change_urgency_checking()) CheckLaneChangeUrgency(frame)`。
- **作用**：是否启用"紧急换道停车"逻辑（目标车道被堵且距路由终点近时建停车墙 `lane_change_stop`）。默认关闭。
- **赛题**：★★☆☆☆ 换道类（一般保持关闭，由场景接管）。

## 7.8 `short_path_length_threshold: 20.0`（proto 默认 20.0）

- **代码**：`AddPathEndStop()`：
  ```cpp
  if (path_data.frenet_frame_path().back().s() -
          path_data.frenet_frame_path().front().s() <
      config_.short_path_length_threshold()) {
    // 在路径末端 -0.1m 建停车墙 STOP_REASON_REFERENCE_END
  }
  ```
- **作用**：路径总长小于 20m 时，在路径末端建停车墙（防止无路可走时冲出去）。
- **对车辆行为影响**：短路径（如到达终点/路口短距离）末端强制停车。
- **赛题**：★★★★☆ 停车类。若赛题有"短距离内停车"，此值决定是否触发末端停车。

> 补充：proto 中还有 `approach_distance_for_lane_change=80`、`urgent_distance_for_lane_change=50`、`enable_stop_on_side_pass=false`（均未在配置中显式设置，用默认值），分别用于紧急换道的接近距离/紧急距离/侧向借道停车开关。

---

# 8. RSSDecider（RSS 安全距离判定）

配置文件：`modules/planning/tasks/rss_decider/conf/default_conf.pb.txt`
Proto：`modules/planning/tasks/rss_decider/proto/rss_decider.proto`
核心代码：`rss_decider.cc`

**模块作用**：用 ad-rss 库计算主车与前方障碍物的 RSS 安全距离，输出 `RSSInfo`（是否安全 + 加速度限制范围）。**只输出信息，不直接改轨迹**（除 fallback 外）。

## 8.1 `enable_rss_fallback: false`（proto 默认 false）

- **代码**：`rss_decider.cc` ~316 行：
  ```cpp
  reference_line_info->mutable_rss_info()->set_is_rss_safe(false);
  if (config_.enable_rss_fallback()) {
    reference_line_info->mutable_speed_data()->clear();  // 清空速度，触发 fallback
  }
  ```
- **作用**：RSS 判定不安全时是否**清空速度数据触发安全 fallback**。默认关闭。
- **对车辆行为影响**：关闭 → RSS 仅记录信息，不影响轨迹；开启 → RSS 不安全时强制降级。
- **赛题**：★★☆☆☆ 一般保持关闭（避免误触发）。

## 8.2 `rss_max_front_obstacle_distance: 3000.0`（proto 默认 3000.0）

- **代码**：
  - 初始化 `front_obstacle_distance = config_.rss_max_front_obstacle_distance()`；
  - 遍历障碍物取最近前向距离；
  - `if (front_obstacle_distance >= config_.rss_max_front_obstacle_distance())` → 判定 RSS 安全（前方无障碍）。
- **作用**：**前向障碍物距离阈值**（3000m 相当于"前方无障碍"判定）。
- **对车辆行为影响**：>3000m 认为无前向障碍、RSS 安全。
- **赛题**：★☆☆☆☆（仅判定安全性的阈值，赛题几乎不影响）。

---

# 9. 汇总：各赛题相关参数速查表

| 赛题需求 | 最相关参数 | 建议方向 |
|---|---|---|
| 减速带 ≤3m/s | `speed_bounds` `lowest_speed`(0.1)、限速链路；DP `exceed_speed_penalty`(1e3)；QP `kappa_penalty_weight`(200) | 保持 exceed 惩罚大；如需更慢调低 QP kappa 权重或调大 |
| 人行道/红绿灯/停止标志停车 1.5-2.0m | STOP 硬边界（QP/NLP）、`rule_based_stop` `max_valid_stop_distance`(1.0)、`max_adc_stop_speed`(0.5)、`speed_bounds` `lowest_speed`(0.1)、`boundary_buffer`(0.25) | lowest_speed 必须保持 0.1 才停得稳；停车位置由场景 stop 线决定 |
| 障碍物停车 ≥2m | `speed_decider` `stop_follow_distance`(2.0) + `follow_distance_scheduler`；gflag `min_stop_distance_obstacle`(6.0)；DP `obstacle_weight`/`default_obstacle_cost`(1e4) | 停车距离由 STOP 围栏决定；跟车距离由 scheduler 决定 |
| 动态跟随 | `speed_decider` `follow_min_time_sec`(2.0)、`follow_min_obs_lateral_distance`(2.5)、`follow_distance_scheduler`；QP `follow_distance_buffer`（⚠️不生效） | 调 scheduler 的 slope/stop_follow_distance 控制跟车间距；改 follow_distance_buffer 无效 |
| 无人人行道 ≤5m/s | `speed_bounds` 限速链路 + `lowest_speed`(0.1)；QP `kappa_penalty_weight`(200)、NLP `lat_acc_weight`(1000)；DP `exceed_speed_penalty`(1e3) | 限速由 TrafficRule 生成上限，DP exceed 惩罚保证不超速 |
| 绿灯路口 ≤5m/s | `speed_bounds` `max_centric_acceleration_limit`(0.8)（弯道限速）、NLP `lat_acc_weight`(1000) | 调小向心加速度/调大横向权重 → 路口更慢 |
| 加减速度控制 | DP `max_acceleration`(2.0)/`max_deceleration`(-4.0)、`accel_penalty`/`decel_penalty`(1.0)、`negative_jerk_coeff`(1.0)；QP/NLP `acc_weight`(1.0/2.0)、`jerk_weight`(3.0) | 本工程整体已调成"允许快加减速"；要平顺则调大权重/调小减速度 |
| 通行效率 | DP `spatial_potential_penalty`(1e2)、`reference_speed_penalty`(10.0) | 已偏"积极前进" |

## 最关键参数 TOP 6

1. **`speed_decider.follow_distance_scheduler` + `stop_follow_distance`** —— 跟车/停车距离核心，动态跟随赛题必调。
2. **`speed_bounds_decider.lowest_speed = 0.1`** —— 决定能否真正停车（默认 2.5 会停不住），所有停车赛题的生命线。
3. **`speed_bounds_decider.max_centric_acceleration_limit = 0.8`** —— 弯道/路口限速最直接控制，绿灯路口 ≤5m/s 赛题相关。
4. **`rule_based_stop_decider.max_adc_stop_speed` / `max_valid_stop_distance`** —— "是否已停车/停车是否合格"的判定阈值（0.5 / 1.0）。
5. **`path_time_heuristic` 的 `exceed_speed_penalty(1e3)` + `negative_jerk_coeff(1.0)` + `spatial_potential_penalty(1e2)`** —— 保证不超速、快停、积极前进的组合。
6. **`piecewise_jerk_speed_nonlinear.use_soft_bound_in_nonlinear_speed_opt = false`** —— 硬边界严格停车/跟车（跟车留 8m、停车不可越线）。

## 重要警示（易踩坑）

- **`piecewise_jerk_speed.follow_distance_buffer`（0.1）已配置但代码未实现**，调整无效，跟车距离必须改 speed_decider。
- **`path_time_heuristic` 的 `go_down_buffer`/`go_up_buffer` 为死参数**，调了不生效。
- **`piecewise_jerk_speed_nonlinear.s_potential_weight`（0.05）因 `use_smoothed_dp_guide_line=true` 不生效**（代码固定用 10.0）。
- **`piecewise_jerk_speed_nonlinear.soft_s_bound_weight` 因 `use_soft_bound_in_nonlinear_speed_opt=false` 不生效**。
- 停车距离本身由 **STOP 决策围栏**（场景配置 + `FLAGS_min_stop_distance_obstacle=6.0`）决定，Task 配置只影响"跟车/让行距离"与"停车判定容差"。
