# 借道机制深挖：起步瞬间 borrow↔self 闪烁 → 是否触发"跟车限制"？

> 研究日期：2026-08-01 | 场景：障碍物停车避让（行人 7673，id=`6a699aefabf06e2936590839`）
> 代码版本：HEAD `745d31e`（= revert 回退到 `c904b46`：跟车限制阈值 3.5m + 人行道 expand 4.0）
> 回放：2209（`data/log/planning.log.INFO.20260801-220748.921814`，389,710 行）
> 性质：纯代码 + 日志研究，未改代码

---

## 0. 结论速览

| 问题 | 结论 |
|------|------|
| 借道闪烁是否是"跟车限制"触发源 | **极不可能是**。闪烁期间最终路径恒为 `regular/self`（借道路径生成失败），**实际下发轨迹连续无跳变**；且闪烁发生在车起步前 ~7.6s（22:08:41 vs 车起步 22:08:48.9），是"规划线跳"不是"车身跳"。"跟车限制"最可能检测车辆跟车/蠕动行为，与规划线横向闪烁无关 |
| 复发根因 | **不是行人**，是 `path_decider` 状态残留：`front_static_obstacle_cycle_counter` 每帧只 -1（10→4 需 6 帧），释放瞬间仍 ≥3；`front_static_obstacle_id` 只在 counter < -2 才清空，释放后仍残留 = **"5530"（护栏）** |
| 横向跳变多大 | 借道（LEFT_BORROW）走廊右界 ≈ -2.05，左界在 ego 处被护栏 5530 压到 ≈0.60m（远端可到 4.87m）；实际路径相对自道偏移约 **0.3~1.0m**（偏左） |
| 最优抑制 | ① 释放分支清零 counter+清 id（根治）；② 释放条件加 `counter < threshold`；③ 借道释放冷却帧；④ `IsLongTermBlockingObstacle` 排除 `UNKNOWN_UNMOVABLE`/行人（次要，5530 场景必须配合①） |
| `skip_overlap_stop_check: true` 对借道的影响 | **无直接影响**。借道由 path-bounds 阻塞决定，与 path_decider 的 STOP 决策无关 |

---

## 1. `lane_borrow_path.cc` 核心逻辑

### 1.1 `IsNecessaryToBorrowLane()` — 借道判定（5+1 项检查）

代码位置 `modules/planning/tasks/lane_borrow_path/lane_borrow_path.cc` ~L350-425。分两个分支：

```cpp
bool LaneBorrowPath::IsNecessaryToBorrowLane() {
  auto* status = injector_->planning_context()
                     ->mutable_planning_status()->mutable_path_decider();
  if (status->is_in_path_lane_borrow_scenario()) {          // ★ 已在借道
    UpdateSelfPathInfo();                                   // 自道+无阻塞 才 +1
    if (use_self_lane_ >= 6) {                              // 连续 6 帧自道畅通
      status->set_is_in_path_lane_borrow_scenario(false);
      decided_side_pass_direction_.clear();
      AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";  // line 370
    }
  } else {                                                  // ★ 未借道 → 5 项检查
    AINFO << "Blocking obstacle ID[" << status->front_static_obstacle_id() << "]";
    // ① 单参考线
    if (!HasSingleReferenceLine(*frame_)) return false;
    // ② 车速 < lane_borrow_max_speed (5.0)
    if (!IsWithinSidePassingSpeedADC(*frame_)) return false;
    // ③ 阻塞障碍离信号/停止标志 > 20m（kIntersectionClearanceDist）
    if (!IsBlockingObstacleFarFromIntersection(*reference_line_info_)) return false;
    // ④ 长期阻塞：front_static_obstacle_cycle_counter >= 3
    if (!IsLongTermBlockingObstacle()) return false;
    // ⑤ 阻塞障碍在目的地内
    if (!IsBlockingObstacleWithinDestination(*reference_line_info_)) return false;
    // ⑥ 可绕行：IsSidePassableObstacle = IsNonmovableObstacle
    if (!IsSidePassableObstacle(*reference_line_info_)) return false;

    if (decided_side_pass_direction_.empty()) {
      CheckLaneBorrow(...);   // 左右邻车道+线型(实线不可借)
      ... status->set_is_in_path_lane_borrow_scenario(true);
    }
    use_self_lane_ = 0;
    AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";  // line 422
  }
  return status->is_in_path_lane_borrow_scenario();
}
```

> 注：`lane_borrow_path`（旧版）的 `IsLongTermBlockingObstacle` **未**被注释禁用（generic 版才禁用、改由 NudgeDecider 概率累积）。本工程日志确认运行的是 `lane_borrow_path.cc`（line 370/374/422），即旧版。

### 1.2 释放条件（`UpdateSelfPathInfo` + `use_self_lane_ >= 6`）

```cpp
// lane_borrow_path.cc L347-358
void LaneBorrowPath::UpdateSelfPathInfo() {
  auto cur_path = reference_line_info_->path_data();   // ★ 最终选择的路径
  if (!cur_path.Empty() &&
      cur_path.path_label().find("self") != std::string::npos &&  // 最终路径是自道
      cur_path.blocking_obstacle_id().empty()) {                  // 且无阻塞障碍
    use_self_lane_ = std::min(use_self_lane_ + 1, 10);
  } else {
    use_self_lane_ = 0;
  }
  blocking_obstacle_id_ = cur_path.blocking_obstacle_id();
}
```

**关键**：释放只要求"最终路径连续 6 帧是自道且无阻塞 id"，**不要求 `front_static_obstacle_cycle_counter` 归零**。这正是闪烁复发的漏洞之一。

### 1.3 `IsLongTermBlockingObstacle` / 计数

```cpp
// lane_borrow_path.cc L400-410
bool LaneBorrowPath::IsLongTermBlockingObstacle() {
  if (injector_->planning_context()->planning_status().path_decider()
          .front_static_obstacle_cycle_counter() >=
      config_.long_term_blocking_obstacle_cycle_threshold()) {   // 阈值 = 3
    return true;
  }
  return false;
}
```

配置（`lane_borrow_path/conf/default_conf.pb.txt`）：
```
is_allow_lane_borrowing: true
lane_borrow_max_speed: 5.0
long_term_blocking_obstacle_cycle_threshold: 3
```

### 1.4 `IsBlockingObstacle` / `IsSidePassableObstacle` 判定标准

阻塞障碍本体来自 **path_decider 状态里的 `front_static_obstacle_id`**（见 §2），`IsSidePassableObstacle` 再调用通用分析器：

```cpp
// obstacle_blocking_analyzer.cc IsNonmovableObstacle
bool IsNonmovableObstacle(const ReferenceLineInfo& rli, const Obstacle& o) {
  // ① 太远（start_s > adc.end_s + 35m）→ 不可绕行
  if (o.PerceptionSLBoundary().start_s() > adc_sl.end_s() + kAdcDistanceThreshold) return false;
  // ② 停靠车辆（IsParkedVehicle：路边 / 停车位）→ 可绕行
  if (IsParkedVehicle(rli.reference_line(), &o)) return true;
  // ③ 被其他 VEHICLE 挡住 → 不可绕行
  for (auto* other : ...) { ... if 挡住 → return false; }
  return true;   // ★ 默认非移动 → 可绕行
}
```

→ **静态障碍只要在 35m 内、非停靠、无车辆挡道，一律"可绕行"**。护栏 5530（UNKNOWN_UNMOVABLE，静态）天然满足。

---

## 2. `path_decider.cc` — `GetBlockingObstacle()` 与 cycle_counter 维护

```cpp
// path_decider.cc L78-99（Process 内）
std::string blocking_obstacle_id;
auto* status = ...mutable_path_decider();
if (reference_line_info->GetBlockingObstacle() != nullptr) {
    blocking_obstacle_id = reference_line_info->GetBlockingObstacle()->Id();
    int c = status->front_static_obstacle_cycle_counter();
    status->set_front_static_obstacle_cycle_counter(std::max(c, 0));       // 去负
    status->set_front_static_obstacle_cycle_counter(std::min(c + 1, 10));  // +1，上限 10
    status->set_front_static_obstacle_id(blocking_obstacle_id);
} else {
    int c = status->front_static_obstacle_cycle_counter();
    status->set_front_static_obstacle_cycle_counter(std::min(c, 0));       // 去正
    status->set_front_static_obstacle_cycle_counter(std::max(c - 1, -10)); // -1，下限 -10
    if (status->front_static_obstacle_cycle_counter() < -2) {              // ★ 只在 < -2 才清 id
        std::string id = " ";
        status->set_front_static_obstacle_id(id);
    }
}
```

`GetBlockingObstacle()` 由 **`LaneFollowPath::AssessPath`**（自道任务）在每帧末尾设置：

```cpp
// lane_follow_path.cc L265-271
*final_path = curr_path_data;                       // 自道
reference_line_info_->SetBlockingObstacle(curr_path_data.blocking_obstacle_id());
// reference_line_info.cc L1137: blocking_obstacle_ = path_decision_.Find(id);
```

自道的 `blocking_obstacle_id` 来自 path-bounds（`GetBoundaryFromStaticObstacles` → `UpdatePathBoundaryBySLPolygon` 检测到走廊被静态障碍截断时 `*blocked_id = obs id`）。

**两个决定闪烁的数值特性：**
1. **counter 每帧只 ±1**（钳制 [-10, 10]）。行人持续阻塞 ≥3 帧 → counter 到 10；行人离开后从 10 递减到 <3 需要 **8 帧**。
2. **id 只在 counter < -2 才清空** → 释放后 counter≈4 时 id 仍是旧值（"5530"）。

---

## 3. 配置

| 配置 | 值 | 文件 |
|------|-----|------|
| `is_allow_lane_borrowing` | true | `lane_borrow_path/conf/default_conf.pb.txt` |
| `lane_borrow_max_speed` | 5.0 m/s | 同上 |
| `long_term_blocking_obstacle_cycle_threshold` | 3 | 同上 |
| `skip_overlap_stop_check` | **true** | `path_decider/conf/default_conf.pb.txt` |
| `static_obstacle_buffer` | 0.3 | 同上 |
| `enable_scenario_side_pass_multiple_parked_obstacles` | true（gflag） | `planning_gflags.cc` |
| `obstacle_lat_buffer` | 0.4 → path-bounds 缓冲 = 1.055+0.4 = 1.455m | 同上 |
| `static_obstacle_speed_threshold` | 0.5（>0.5 视为移动） | 同上 |

---

## 4. 2209 回放逐帧证据（核心）

### 4.1 时间线

| planning 时间 | 事件 | 证据行 |
|---------------|------|--------|
| 22:07:57–40.36 | 行人 7673（静态）阻塞自道 | `blocked at 7673, ..., l_lower≈-2.07, l_upper≈1.29`（数百帧） |
| 22:08:08.36 | 进入借道 | `lane_borrow_path.cc:422] Switch SELF→BORROW` |
| 22:08:13–40.36 | **借道为最终路径** | `PATH_END_regular/leftforward`（rule_based_stop_decider 虚拟障碍） |
| 22:08:40.36 | 行人 SL 上界收窄出车道（开始移动） | 最后 `blocked at 7673` 于 40.359 |
| 22:08:40.50 | 行人 → 7673_0（moving，`is_static=false`）→ **退出 path-bounds 作用域** | `reference_line_info.cc:422 ... id:7673_0` |
| 22:08:40.505 | 自道阻塞者换成**护栏 5530**（唯一一次） | `lane_follow_path.cc:268] regular/self5530`；`blocked at 5530, 98.5, l_lower:-0.92` |
| 22:08:40.59→41.09 | 自道无阻塞 ×6 帧（40.59/40.72/40.79/40.90/41.00/41.09） | `lane_follow_path.cc:268] regular/self`（空 blocking id） |
| **22:08:41.093** | **释放**（use_self_lane_=6） | `lane_borrow_path.cc:370] Switch BORROW→SELF` |
| **22:08:41.185** | **复发**（counter≈4 ≥ 3，id="5530"） | `lane_borrow_path.cc:374] Blocking obstacle ID[5530]` + `:422] Switch SELF→BORROW` |
| 22:08:41.185→41.784 | 自道无阻塞 ×6 帧（借道路径再次生成失败） | `lane_follow_path.cc:268] regular/self` |
| **22:08:41.784** | **最终释放**（counter 已 <3，不再复发） | `lane_borrow_path.cc:370] Switch BORROW→SELF` |

### 4.2 关键事实链

1. **`PATH_END_regular/leftforward` 最后出现于 22:08:40.359**，此后最终路径恒为 `regular/self` → **借道在 40.36 后每一帧都生成失败**，闪烁期间实际轨迹不变。
2. **复发触发者 = 护栏 5530**（`Blocking obstacle ID[5530]`），不是行人 7673。行人离开后：
   - `front_static_obstacle_id` 残留 "5530"（counter=4 未到 -2 清除线）；
   - `IsLongTermBlockingObstacle` 因 counter=4 ≥ 3 通过；
   - `IsSidePassableObstacle(5530)` = `IsNonmovableObstacle` = true（护栏静态、35m 内、无车挡道）；
   - `IsBlockingObstacleFarFromIntersection/WithinDestination` 均通过；
   - `IsWithinSidePassingSpeedADC`：车仍停（v≈0 < 5.0）→ 通过。
   → 5 项全过 → 复发。
3. **复发后借道路径又失败**（最终仍 self）→ 6 帧后再次释放 → 此时 counter 已 <3 → 稳定。

### 4.3 为什么借道在 40.36 后生成失败？

- 40.36 前：行人 7673（静态、在车道内 l∈[-0.70,1.29]）是阻塞者，借道走廊（左扩到邻车道）从行人左侧绕行成功。
- 40.50 起：行人变移动（7673_0）退出作用域，走廊内仅剩**护栏 5530**（左肩，SL l≈1.96~2.26，远端投影横穿车道）。护栏在 ego 处把借道左边界从 ~4.87m 压到 **0.60m**（`update left_bound: l 4.87→0.60`），走廊仅 [-2.05, +0.60]（宽 2.65m ≈ 车宽 2.11m + 缓冲），QP 无法给出平滑路径 → `OptimizePath`/`AssessPath` 失败 → 借道空转。

---

## 5. 关键问题回答

### 5.1 起步瞬间为何借道释放后又复发？

**不是行人"SL 边界短暂仍阻塞"，而是 path_decider 状态残留的"时间惯性"**：
- counter 从 10 每帧 -1，释放时（6 帧后）仍在 4（≥3）；
- id 只在 counter < -2 才清空，释放后仍是旧阻塞者 **5530**；
- 5530（护栏）恰好满足全部 6 项借道检查 → 下一帧立即复发。

### 5.2 借道切换导致轨迹线横向跳变多大？

- 自道走廊：[-2.05, +1.98]，中心 ≈ 0。
- LEFT_BORROW 走廊：右界 ≈ -2.05；左界在 ego 处被护栏 5530 压到 ≈ **0.60m**（远离护栏处可到 ~4.87m）。
- 实际借道路径相对自道中心**偏左约 0.3~1.0m**（受护栏限制，不是大幅偏左）。
- **关键**：闪烁期间（41.09→41.78）最终路径恒为 self，**无实际轨迹跳变**；真正的"线跳"只有 40.36→40.5 那一次（借道→自道）。

### 5.3 借道闪烁会不会触发"跟车限制"？

**结论：不会，且关系极弱。** 理由：

1. **闪烁是"线跳"不是"车跳"**：借道路径在闪烁期间全部生成失败，下发轨迹连续；车体 y 全程恒定（`2209_restart_precise.txt`：veh_y 4437610.774→4437610.751，13m 行程仅 2.3cm）。
2. **时间错位**：闪烁发生在 22:08:41（车起步前 ~7.6s，行人刚开始移动、车完全停止），而车真正起步在 22:08:48.9（t=55.4）。"跟车限制"若检测车辆行为，完全对不上。
3. **"跟车限制"语义**（`follow_limit_detection_research.md` §4）：最可能是"主车不应跟随正在移动/横向离开车道的行人，应停车等待" + "长时间低速蠕动跟车"检测。本回放中行人移动后决策恒为 STOP（`HandlePedestrianStop` 固定栅栏，无 FOLLOW），起步时行人已横向 ≥3.5m，车辆直线通过——无任何跟车行为。
4. **真正的"跟车限制"嫌疑仍是起步时机**：`HandlePedestrianStop::kClearLateral = 3.5`（speed_decider.cc L625），车在行人横向刚 3.5m、仍以 ~2.1m/s 移动时就起步（t=55.4）。这与 git 历史把阈值 3.5→6.0→8.0 + 要求静止（c8018da/5af4e4f/ee910fd）的修复方向完全一致。

### 5.4 如何抑制（让起步后稳定走自道、不借道）

| 方案 | 位置 | 原理 | 推荐度 |
|------|------|------|:---:|
| **A. 释放时清零 counter + 清 id** | `lane_borrow_path.cc` 释放分支（line ~370） | 释放瞬间 `front_static_obstacle_cycle_counter(0)` + `front_static_obstacle_id("")` → 下一帧 `IsLongTermBlockingObstacle` 立即失败（0<3），从根上杜绝复发 | ⭐⭐⭐ 最小改动、根治 |
| **B. 释放条件加 `counter < threshold`** | `IsNecessaryToBorrowLane` 的 `use_self_lane_>=6` 条件 | 释放不仅要求 6 帧自道畅通，还要求 counter 已衰减到 <3 | ⭐⭐⭐ 等效且更稳健（多等 1~2 帧） |
| **C. 借道释放冷却帧** | 加成员 `last_release_frame_` | 释放后 N 帧（如 30 帧）内禁止再次进入借道 | ⭐⭐ 简单粗暴，通用 |
| **D. `IsLongTermBlockingObstacle` 排除 `UNKNOWN_UNMOVABLE`** | lane_borrow_path.cc | 护栏/路沿不该成为"可绕行的阻塞者"（path_decider 已对 UNKNOWN_UNMOVABLE 加 ignore，lane_borrow 却仍认它可绕行） | ⭐⭐ 消除 5530 复发源；但本场景必须配合 A/B 才彻底（counter 仍在高位时会换别的静态障碍复发） |
| **E. 排除行人** | `IsLongTermBlockingObstacle` 检查阻塞者类型 | 行人走后（moving）不触发借道 | ⭐ 本场景复发源是护栏不是行人，单独无效 |
| **F. 起步后禁止借道** | `IsWithinSidePassingSpeedADC` | 要求借道必须在停车/极低速持续 N 帧才允许 | ⭐ 起步瞬间 v≈0 仍会过 5.0 阈值，需要加"持续帧数" |

**最优组合**：**A（或 B）+ D**。A 根治时间惯性复发；D 从语义上让护栏不再被当作可绕行阻塞者。若想彻底避免本场景任何借道闪烁，可直接 `is_allow_lane_borrowing: false`（停车避让场景本就不需要借道），但会影响其他借道绕行场景，需按场景/配置作用域权衡。

---

## 6. `skip_overlap_stop_check: true` 对借道的影响

**无直接影响。** 分析：

- `skip_overlap_stop_check=true` 只影响 `PathDecider::MakeStaticObstacleDecision` 中"横向重叠的静态障碍"分支（path_decider.cc L205 附近）：原应加 STOP，现仅打日志 `skip_overlap_stop_check`，**不加任何决策**（既非 STOP 也非 IGNORE）。
- 借道触发链路是 **path-bounds 阻塞 → `SetBlockingObstacle` → counter/id → `IsNecessaryToBorrowLane`**，与 path_decider 的 STOP 决策完全无关。
- `IsWithinPathDeciderScopeObstacle` 只排除 IGNORE 决策的障碍；STOP 决策不排除，所以即使关闭该 flag（重叠障碍得 STOP），障碍仍进 path-bounds、仍会阻塞、仍会触发借道。
- 实测佐证：22:08:08–40.36 `skip_overlap_stop_check` 打印 321 次（恰好覆盖整个借道期），借道照常发生。

> 该 flag 的真实作用：避免 path_decider 给静止障碍（如行人）附加与 speed_decider 冲突的 STOP 决策，减少 STOP 闪烁。对"跟车限制"无直接影响。

---

## 7. 附：关键代码摘录

```cpp
// lane_borrow_path.cc —— 释放分支（复发漏洞 #1：#2）
if (use_self_lane_ >= 6) {
    mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
    decided_side_pass_direction_.clear();
    AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";   // L370
}

// path_decider.cc —— counter 慢衰减 + id 迟清除（复发漏洞 #3：#4）
else {
    ... std::max(c - 1, -10);
    if (counter < -2) { id = " "; set_front_static_obstacle_id(id); }
}

// 复发帧 22:08:41.185 的完整证据
// I0801 22:08:41.185837 lane_follow_path.cc:268] regular/self          ← 自道无阻塞
// I0801 22:08:41.185850 lane_borrow_path.cc:374] Blocking obstacle ID[5530]  ← 残留护栏!
// I0801 22:08:41.185878 lane_borrow_path.cc:422] Switch ... to LANE-BORROW.   ← 复发

// 借道走廊被护栏压缩（复发生成失败原因）
// path_bounds_decider_util.cc:311] update left_bound: s 63.2, l 4.87 -> 0.619  ← 左界被护栏压到 0.62
// path_bounds_decider_util.cc:294] obs id: 5530, obs l: 1.97484, 1.79769e+308 ← 护栏 SL

// speed_decider.cc HandlePedestrianStop —— 真正的"跟车限制"防线（kClearLateral=3.5）
static constexpr double kClearLateral = 3.5;
if (obs_l < kClearLateral) { /* 固定 STOP 栅栏 */ }
else { /* 释放 + YIELD */ }
```

---

## 8. 引用数据/文件
- `output/_analysis_7673/borrow_switches.txt`、`report_7673_220748_follow_limit.md`、`pbd269_7673.txt`
- `data/log/planning.log.INFO.20260801-220748.921814`（2209 回放，逐帧证据）
- `output/path_right_veer_source_analysis.md`（上轮"右偏"溯源）、`output/follow_limit_detection_research.md`（跟车限制判定研究）
- 源码：`tasks/lane_borrow_path/lane_borrow_path.cc`、`tasks/path_decider/path_decider.cc`、`tasks/lane_follow_path/lane_follow_path.cc`、`tasks/speed_decider/speed_decider.cc`、`planning_base/common/obstacle_blocking_analyzer.cc`、`planning_interface_base/.../path_bounds_decider_util.cc`、`planning_base/common/reference_line_info.cc`
