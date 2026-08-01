# 停车避让"轨迹朝右偏"源码溯源分析报告

> 分析日期：2026-08-01 | 场景：障碍物停车避让（行人 7673，赛题六）
> 任务：纯代码 + 数据分析，找出"行人离开后自车起步，轨迹线有时朝右偏一下"的代码来源
> 结论先行：**"朝右偏" = 停车阶段左借道（LANE-BORROW）路径与起步阶段自车道（SELF-LANE）路径的切换闪烁**，每次 borrow→self 切换，规划轨迹线从"偏左的借道线"跳回"居中的自道线"，视觉上就是"朝右偏一下"。物理上车身保持笔直（回放 veh_y 恒定）。

---

## 0. 结论速览

| 问题 | 结论 |
|------|------|
| 1. FOLLOW 障碍会进 path_decider nudge 吗？ | **不会**。path_decider 只处理 `IsStatic()` 障碍，FOLLOW 障碍是运动的 → 直接跳过；且 path_decider 在路径生成**之后**运行，其 ObjectNudge 不改变路径形状（纯报告用） |
| 2. 参考线会随障碍/车道边界偏移吗？ | **不会**。参考线来自 pnc_map（地图/路由）+ discrete_points smoother（FEM_POS_DEVIATION），纯地图驱动，障碍不进参考线 |
| 3. "朝右偏一下"最可能来源 | **lane_borrow_path 的 BORROW↔SELF 路径切换闪烁**（`IsNecessaryToBorrowLane()`，0.7s 内 3 次切换，回放实锤）。次要来源：起步起点 l≠0 时的回中（ExtendBoundaryByADC + l_weight） |
| 4. 横向数据右偏证据 | `lateral_*.txt` / `vehlat_*.txt` 全是**右转**场景（切内弯 ≤0.96m），非行人避让场景；行人避让场景的 `2209_restart_precise.txt` 显示车身**笔直**（veh_y 恒定、traj_yoff≈0） |

**⚠️ 关键偏差核对**：当前 HEAD=`1695aef`（revert 回退到 0801_190923 提交包）**回退了 add11 的 `path_decider 对 PEDESTRIAN continue`**（`git log` 显示 ca11ae6→1695aef）。当前 `path_decider.cc` 里 grep `PEDESTRIAN` 为空——**静态行人仍会被 path_decider 处理**。但 `skip_overlap_stop_check: true` 关闭了重叠 STOP 分支，实际影响有限（见 §3.3）。

---

## 1. 当前代码/配置状态核对

### 1.1 git 状态
- 分支：`fix/barrier-stop-and-old-path-config`，HEAD=`1695aef revert: 回退到0801_190923提交包对应版本(c904b46)`
- `path_decider.cc` 历史：`1695aef`(revert) ← `ca11ae6`(add11: path_decider 不处理行人) ← `c159da7`(revert add9c+add10)
- **结论**：add11 的"path_decider 对 PEDESTRIAN continue"已被 revert 掉（回归）；speed_decider 的 add4/add7/add9-12（HandlePedestrianStop、kClearLateral=3.5、road_furniture ignore）**仍在**。

### 1.2 生效配置（profile 覆盖 + 源码默认）

`profiles/default/modules/planning/tasks/lane_follow_path/conf/default_conf.pb.txt`（当前）：
```protobuf
is_extend_lane_bounds_to_include_adc: true
extend_buffer: 0.2
path_optimizer_config {
  l_weight: 2.0
  dl_weight: 20.0
  ddl_weight: 1000.0
  dddl_weight: 1000.0
  lateral_derivative_bound_default: 2.0
  path_reference_l_weight: 100.0   # ← 弱贴线（旧版为 10000）
}
```
> 与记忆"恢复旧版 100000/500000/10000"**不一致**——当前是弱贴线版本（dddl=1000/l_weight=2.0/path_reference_l_weight=100）。

`modules/planning/tasks/lane_borrow_path/conf/default_conf.pb.txt`：
```protobuf
is_allow_lane_borrowing: true
lane_borrow_max_speed: 5.0
long_term_blocking_obstacle_cycle_threshold: 3
path_optimizer_config { l_weight: 1.0  dl_weight: 20.0  ddl_weight: 1000.0
                        dddl_weight: 50000.0  path_reference_l_weight: 100.0 }
```

`modules/planning/tasks/path_decider/conf/default_conf.pb.txt`：
```protobuf
static_obstacle_buffer: 0.3
skip_overlap_stop_check: true
```

关键 gflags（`planning_gflags.cc` + profile planning.conf）：
- `lateral_ignore_buffer = 3.0`（默认，未覆盖）
- `obstacle_lat_buffer = 0.3`（profile 覆盖 0.4→0.3）
- `static_obstacle_speed_threshold = 0.5`
- `nonstatic_obstacle_nudge_l_buffer = 0.4`
- `pedestrian_stop_distance = 1.75`、`min_stop_distance_obstacle = 6.0`（默认）

pipeline 顺序（`scenarios/lane_follow/conf/pipeline.pb.txt`）：
```
LANE_CHANGE_PATH → LANE_FOLLOW_PATH → LANE_BORROW_PATH → FALLBACK_PATH
→ PATH_DECIDER → RULE_BASED_STOP_DECIDER → SPEED_BOUNDS_PRIORI → SPEED_HEURISTIC
→ SPEED_DECIDER → SPEED_BOUNDS_FINAL → PIECEWISE_JERK_SPEED
```
**关键：PATH_DECIDER 在路径生成之后** → 它的决策不可能改变路径形状。

---

## 2. 路径生成链路全景（lane_follow_path 内部）

```
LaneFollowPath::Process
 ├─ GetStartPointSLState()                      # 起始点 SL（含当前 l）
 ├─ DecidePathBounds()                          # 1)InitPathBoundary 2)GetBoundaryFromSelfLane(车道边界)
 │                                              3)ExtendBoundaryByADC(is_include_adc=true, 0.2)
 │                                              4)GetBoundaryFromStaticObstacles(静态障碍 nudge/block)
 ├─ OptimizePath()                              # piecewise_jerk 优化
 │   ├─ UpdatePathRefWithBound(bound, path_reference_l_weight, &ref_l, &weight_ref_l)
 │   └─ OptimizePath(init, end, ref_l, weight_ref_l, bound, ddl_bounds, jerk_bound, config,...)
 └─ AssessPath() → SetBlockingObstacle(blocking_obstacle_id)
```

### 2.1 ref_l 权重逻辑（`path_optimizer_util.cc:330` `UpdatePathRefWithBound`）

```cpp
for (size_t i = 0; i < ref_l->size(); i++) {
  bool is_need_update_path_ref =
      (path_boundary[i].l_lower.type == BoundType::OBSTACLE ||
       path_boundary[i].l_upper.type == BoundType::OBSTACLE) &&
      (...towing_l 越界判断...);
  if (is_need_update_path_ref) {
    ref_l->at(i) = (path_boundary[i].l_lower.l + path_boundary[i].l_upper.l) / 2.0;
    weight_ref_l->at(i) = weight;     // = path_reference_l_weight = 100
  } else {
    weight_ref_l->at(i) = 0;          // 其余点完全不贴 ref_l
  }
}
```
- 默认 ref_l=0（参考线中心），但**权重只在被 OBSTACLE 边界约束的点上生效**（weight=100）。
- 其余点靠 `l_weight=2.0` 拉回 l=0。
- **含义**：障碍物把 path bound 挤偏时，ref_l 被拉到"被挤窄的走廊中点"且权重 100 → 路径贴障碍物侧边界走；障碍消失、bound 恢复后，路径回到 l=0。任何 OBSTACLE bound 的出现/消失都会造成路径横向迁移。

### 2.2 静态障碍对 bound 的处理（`path_bounds_decider_util.cc`）

`GetSLPolygons` 过滤（`IsWithinPathDeciderScopeObstacle`, line 577）：
```cpp
bool PathBoundsDeciderUtil::IsWithinPathDeciderScopeObstacle(const Obstacle& obstacle) {
  if (obstacle.IsVirtual()) return false;
  if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() &&
      obstacle.IsIgnore()) return false;             // 仅排除 IGNORE
  if (!obstacle.IsStatic() ||
      obstacle.speed() > FLAGS_static_obstacle_speed_threshold) return false;  // 0.5
  return true;
}
```
`UpdatePathBoundaryBySLPolygon` 的 nudge buffer：
```cpp
double PathBoundsDeciderUtil::GetBufferBetweenADCCenterAndEdge() {
  return (adc_half_width + FLAGS_obstacle_lat_buffer);   // 1.055 + 0.3 = 1.355m
}
```
- 障碍中心 `obs_l=(l_lower+l_upper)/2`；`last_max_nudge_l < obs_l → RIGHT_NUDGE`，否则 `LEFT_NUDGE`。
- RIGHT_NUDGE：`l_lower -= 1.355`，若压到右边界 → `blocked_id=obs_id`，路径被截断（BLOCKED）。
- LEFT_NUDGE：`l_upper += 1.355`，压到左边界 → blocked。

---

## 3. 问题 1：FOLLOW/STOP 障碍是否进入 path_decider 的 nudge？

### 3.1 path_decider（任务本身）— 不
`path_decider.cc MakeStaticObstacleDecision` 开头：
```cpp
if (!obstacle->IsStatic() || obstacle->IsVirtual()) {
    continue;                                   // ← 运动的 FOLLOW 障碍直接跳过
}
```
- **FOLLOW 障碍是运动的（speed>0）→ `IsStatic()` 为 false → 永不进入 nudge 计算。**
- **已有 STOP 决策的障碍**：`if (obstacle->HasLongitudinalDecision() && obstacle->LongitudinalDecision().has_stop()) continue;` → 跳过。
- **运行时机**：path_decider 在 pipeline 中位于路径生成之后，其 `ObjectNudge` 只通过 `ReferenceLineInfo::SetObjectDecisions` 导出到 ADCTrajectory 显示，**不改变路径**（grep 确认 `ObjectNudge` 在 planning_interface_base 内无消费方）。
- nudge 阈值（对**静态**障碍）：
  ```cpp
  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;   // 1.055+3.0 = 4.055m
  const double min_nudge_l = half_width + config_.static_obstacle_buffer()/2.0;  // 1.055+0.15 = 1.205m
  ```
  静态障碍 SL 边界与 [curr_l±1.205] 重叠 → STOP（但 `skip_overlap_stop_check=true` 跳过）；整体在左/右 → LEFT/RIGHT_NUDGE（distance_l=±0.3）；横向 >4.055m → IGNORE。

### 3.2 path_bounds（真正塑造路径的地方）— 仅静态，FOLLOW 不参与
- `GetBoundaryFromStaticObstacles` 走 `IsWithinPathDeciderScopeObstacle`：**只收静态（speed≤0.5）+ 非 ignore + 非 virtual**。FOLLOW 障碍不参与。
- **注意**：静态障碍即使**已有 STOP 决策**（如 speed_decider 给静止行人建的 STOP）**仍会进入 path bounds**（该过滤只排除 IGNORE）→ 可 block/nudge 路径。这正是行人 7673 静止时把自车道路径截断、触发借道的原因。

### 3.3 回归点（重点）
当前 `path_decider.cc` 中 grep `PEDESTRIAN` **为空** —— add11 的"`PEDESTRIAN → continue`"已被 revert。后果：
- 静态行人（speed=0，如场景行人 50s 前）会被 path_decider 处理；由于 `skip_overlap_stop_check=true`，重叠 STOP 分支被跳过，但 **blocking obstacle STOP 分支**仍可能触发（`obstacle->Id()==blocking_obstacle_id && !is_in_path_lane_borrow_scenario()`）。在借道场景下该分支被 `is_in_path_lane_borrow_scenario` 门控，影响有限。
- 记忆 add10 记录的"感知 SL 噪声 → STOP 点错位 60m → 撞飞"风险在当前代码下理论仍存在，但本次 2209 回放未复现（路径决策 '-'，正常通过）。

---

## 4. 问题 2：参考线是否随障碍/车道边界偏移？

**不会。**
- 参考线来源：`reference_line_provider.cc`（`ReferenceLineProvider`）→ pnc_map（`LaneFollowMap`，地图/路由）→ `DiscretePointsReferenceLineSmoother`（`discrete_points_smoother_config.pb.txt`：FEM_POS_DEVIATION，`weight_fem_pos_deviation=1e10`，curb_shift=0.2）。
- smoother 输入是**地图中心线/路由路径**，不接收障碍物。障碍物只出现在 `ReferenceLineInfo::AddObstacleHelper`（把感知障碍投影到参考线得 SL boundary），**反向**（障碍→SL），不改变参考线本身。
- 车道边界同样只影响 path **bounds**（`GetBoundaryFromSelfLane` 用 lane width ± 半车宽），不改参考线。
- 唯一"参考线级"偏移是**路径优化器的 ref_l**：OBSTACLE bound 处 ref_l=走廊中点（weight=100），这是路径贴边，不是参考线移动。

---

## 5. 问题 3："朝右偏一下"最可能的代码来源

### 5.1 ★ 主因：LANE-BORROW ↔ SELF-LANE 路径切换闪烁（实锤）

`output/_analysis_7673/borrow_switches.txt`（2209 回放，22:08:41 ≈ 行人放行/自车起步时刻）：
```
22:08:08.362037 lane_borrow_path.cc:422] Switch from SELF-LANE path to LANE-BORROW path.  ← 行人阻塞→左借道
22:08:41.093281 lane_borrow_path.cc:370] Switch from LANE-BORROW path to SELF-LANE path.  ← 行人离开→回自道
22:08:41.185878 lane_borrow_path.cc:422] Switch from SELF-LANE path to LANE-BORROW path.  ← 0.09s 后又回借道!!
22:08:41.784526 lane_borrow_path.cc:370] Switch from LANE-BORROW path to SELF-LANE path.  ← 又回自道
```
**0.7 秒内 3 次路径切换**，正发生在"行人离开、自车起步"的瞬间。

机制（`lane_borrow_path.cc IsNecessaryToBorrowLane`, line ~350-425）：
```cpp
if (is_in_path_lane_borrow_scenario) {              // 正在借道
  UpdateSelfPathInfo();                              // 自道+无阻塞连续帧才 +1，否则清零
  if (use_self_lane_ >= 6) {                         // 连续 6 帧自道畅通
    set_is_in_path_lane_borrow_scenario(false);
    AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";   // line 370
  }
} else {                                             // 未借道
  ... 5 项检查：单参考线 / 车速<5.0 / 阻塞离路口远 / 长期阻塞(cycle_counter>=3)
      / 目的地内 / 可绕行(IsSidePassableObstacle) ...
  set_is_in_path_lane_borrow_scenario(true);
  use_self_lane_ = 0;
  AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";     // line 422
}
```
- 切回 SELF 后 0.09s 又切回 BORROW：`front_static_obstacle_cycle_counter`（path_decider 维护，依赖 `GetBlockingObstacle()`）仍 ≥3，且行人 SL 边界仍短暂"阻塞"自道 → 5 项检查全过 → 借道复发。
- **几何方向**：借道 label=`regular/leftforward`（LEFT_BORROW，走廊左扩到邻车道），轨迹线**偏左**；SELF label=`regular/self`（居中）。每次 BORROW→SELF，规划线从偏左跳回居中 = **"朝右偏一下"**。
- 佐证：`output/dump_2012_2124.txt` 中停车阶段 `PATH_END_regular/leftforward:stop`（最终路径确实是左借道）；`rule_based_stop_decider.cc:128` `PATH_END_VO_ID_PREFIX + path_data().path_label()` 证实。

### 5.2 次要来源：起点 l≠0 的回中（ExtendBoundaryByADC + l_weight）
- `is_extend_lane_bounds_to_include_adc: true` → `ExtendBoundaryByADC` 把 path bound 起点锚定在自车当前 l ±（半宽+0.2）。
- 若停车时自车横向 l≠0（如跟随借道线略有偏移），起步时路径起点=l0，`l_weight=2.0` 把它拉回 0 → 起步瞬间产生"回中"横向漂移（方向取决于 l0 符号）。2209 回放中 veh_y 恒定 → 本次影响≈0。

### 5.3 排除项
- **path_decider 的 NUDGE**：运行在路径生成后，不改路径 → 排除。
- **path_reference_l_weight=100 弱贴线**：只在 OBSTACLE bound 处生效；起步时行人已横向>3.5m，超出 nudge buffer(1.355m) → 无 OBSTACLE bound → 排除为主因。
- **参考线平滑**：地图驱动，无障碍输入 → 排除。

---

## 6. 问题 4：横向数据中的右偏证据

### 6.1 `lateral_*.txt` / `vehlat_*.txt` —— 是"右转"场景，不是行人避让！
- 全部基于 `xh_2025_contest` 地图、`Lane_1353→Lane_697`（**停止标志右转**场景），脚本 `planning_lateral.py` 注释明确：定位转弯段 x∈[424405,424428]。
- `lateral_1847b.txt`：规划偏移在转弯段最高 **0.77m**（plan#360 前 40 点逐点 0.49→0.77）。
- `vehlat_1847.txt`：车辆实际偏移最高 **0.96m**（车道半宽 1.15m）——右转**切内弯**（向右切弯心），与 `path_reference_l_weight=100` 弱贴线直接相关（记忆：旧版 10000 无此问题）。
- `vehlat_1859.txt`：同右转场景，0.92m→0 递减（驶出弯后回中）。
- **`output/lateral_1859.txt` 不存在**——用户所指应为 `output/vehlat_1859.txt`。

### 6.2 行人 7673 避让场景 —— 车身笔直，无物理右偏
`output/2209_restart_precise.txt`（最新 2209 回放，当前代码）：
```
t=52.0  veh(x=423435.25,y=4437610.774,v=0.00) ped lat=-1.33 lon=15.3 dec=stop   ← 行人右侧，向右离开
t=55.10 veh(x=423435.26,y=4437610.774,v=0.01) ped lat=-3.24 dec=-               ← 放行（≈kClearLateral=3.5）
t=55.4  veh(x=423435.27,y=4437610.774,v=0.04) traj_yoff_veh=+0.000              ← 起步
t=59.97 veh(x=423448.80,y=4437610.751,v=5.12) traj_yoff_veh=+0.006              ← 全程笔直!
```
- veh_y 从 4437610.774 → 4437610.751（**仅 2.3cm**，纵向 13m 行程），`traj_yoff_veh`（前 4 个轨迹点相对车 y）0~+0.006m（毫米级）。
- **结论**：当前代码下，行人离开后自车物理轨迹**完全笔直**，无右偏。"朝右偏"是**规划轨迹线**（DreamView 中前方线）在 borrow↔self 切换时的**视觉跳变**，非车身位移。
- 决策列：行人移动后为 `-`（无 stop/follow/yield）——add7/add11 修复生效，无跟车。

### 6.3 path bounds 数据佐证
`output/_analysis_7673/pbd269_7673.txt`（停车阶段）：
```
last_max_nudge_l: -0.156, obs id: 7673, obs l: -0.698, 1.287     ← 行人 SL 边界横跨车道 [-0.70, +1.29]
```
- 行人中心 l≈+0.30 → `last_max_nudge_l < obs_l` → RIGHT_NUDGE → `l_lower-1.355=-2.05 ≤ 右边界` → **BLOCKED** → 自道路径截断 → 触发左借道。这解释了停车阶段为何用 `regular/leftforward`。

---

## 7. 关键代码摘录（供复核）

```cpp
// path_decider.cc —— 只处理静态障碍
if (!obstacle->IsStatic() || obstacle->IsVirtual()) { continue; }
...
const double min_nudge_l = half_width + config_.static_obstacle_buffer() / 2.0;
// static_obstacle_buffer=0.3 → min_nudge_l=1.205；lateral_radius=4.055(IGNORE 阈值)

// path_bounds_decider_util.cc —— 路径形状的 nudge 只认静态
bool PathBoundsDeciderUtil::IsWithinPathDeciderScopeObstacle(const Obstacle& o) {
  if (o.IsVirtual()) return false;
  if (o.HasLongitudinalDecision() && o.HasLateralDecision() && o.IsIgnore()) return false;
  if (!o.IsStatic() || o.speed() > FLAGS_static_obstacle_speed_threshold) return false;  // 0.5
  return true;
}
// GetBufferBetweenADCCenterAndEdge = half_width + obstacle_lat_buffer = 1.055+0.3 = 1.355

// lane_borrow_path.cc —— 右偏主因：路径切换闪烁
// line 370: Switch from LANE-BORROW path to SELF-LANE path. (use_self_lane_>=6)
// line 422: Switch from SELF-LANE path to LANE-BORROW path. (5 项检查通过)
// label = "regular/left" + borrow_lane_type → "regular/leftforward"（左借道，线偏左）

// path_optimizer_util.cc —— ref_l 只在 OBSTACLE bound 处生效
// ref_l[i]=(l_lower+l_upper)/2, weight_ref_l[i]=path_reference_l_weight(100)

// reference_line_provider.cc —— 参考线 = pnc_map + smoother（discrete_points FEM_POS_DEVIATION）
```

---

## 8. 建议排查方向（供后续决策）

1. **确认评测"跟车限制"是否看轨迹线方向/横向位移**：若看规划线（ADCTrajectory）的横向偏移，则 borrow↔self 闪烁会在行人附近产生 0.3~1.3m 的横向跳变，可能被判定为"朝障碍物方向让行/跟随"。可尝试在 `IsNecessaryToBorrowLane` 切回 SELF 前增加迟滞（如行人横向 > 阈值 + 连续 N 帧），抑制闪烁。
2. **对静止行人彻底退出 path_decider**：恢复 add11 的 `PEDESTRIAN → continue`（当前已被 revert），消除 blocking STOP/感知 SL 噪声风险。
3. **起步阶段抑制 borrow 复发**：起步瞬间车速 < lane_borrow_max_speed(5.0) 仍满足借道条件 → 借道会复发。可考虑借道释放后加冷却（cooldown）或要求 `front_static_obstacle_cycle_counter` 必须归零。
4. **若评测是几何检测（车-障碍距离+速度）**：2209 回放已满足（无跟随、行人横向>3.5m 才起步、车身笔直），重点转向云端评测报告核对触发时刻。

---

## 附：本报告引用的数据/文件
- `output/_analysis_7673/borrow_switches.txt`（路径切换实锤）
- `output/_analysis_7673/pbd269_7673.txt`（path bounds 阻塞证据）
- `output/_analysis_7673/path_decider_head.txt` / `speed_decider_head.txt` / `fence.txt` / `pedstop.txt`（空）
- `output/2209_restart_precise.txt`（起步笔直证据）
- `output/follow_limit_detection_research.md`（上轮"跟车限制"判定研究）
- `output/dump_2012_2124.txt`（停车阶段 PATH_END_regular/leftforward）
- `output/lateral_1847b.txt` / `vehlat_1847.txt` / `vehlat_1859.txt`（右转切内弯，非避让场景）
- 源码：`tasks/path_decider/path_decider.cc`、`tasks/lane_follow_path/lane_follow_path.cc`、`tasks/lane_borrow_path/lane_borrow_path.cc`、`planning_interface_base/.../path_bounds_decider_util.cc`、`.../path_optimizer_util.cc`、`planning_base/common/reference_line_info.cc`、`planning_base/reference_line/reference_line_provider.cc`、`tasks/rule_based_stop_decider/rule_based_stop_decider.cc`、`tasks/speed_decider/speed_decider.cc`
