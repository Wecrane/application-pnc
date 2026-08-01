# 2209 回放 t=55.10 放行机制精确判定 — 研究任务

> 日期：2026-08-01 · 纯研究（不改代码）
> 场景：障碍物停车避让（6a699aef…），行人 7673（5×2×1.8m）
> 代码：c904b46（HEAD 1695aef 回退后等价）
> 问题：t=55.10 的放行到底是 `kClearLateral=3.5` 的 YIELD 放行，还是 boundary 空后的 IGNORE 旁路？

---

## 0. 一句话结论

**t=55.10 的放行是「ST boundary 变空 → `MakeObjectDecision` 顶部 `boundary.IsEmpty()` → `AppendIgnoreDecision`」的 IGNORE 旁路，`kClearLateral=3.5` 从未生效。** 证据为决策标签（放行后是空 `-` 而非 `ped_yield_clear` YIELD）+ 放行时横向（-3.24m < 3.5）+ 放行无过渡（STOP 直接消失）。

---

## 1. 放行机制精确判定（带证据行）

### 1.1 三份数据交叉验证（同一时刻）

| t | 决策 (restart_precise) | 行人横向 lat | 决策明细 (ped_dec) |
|---|------|------|------|
| 54.72 | stop | -2.95 | `id=7673_0 STOP(423438.3,rc=1)` ← ped_dec 最后一条 |
| 55.02 | **stop（最后 STOP 帧）** | -3.17 | — |
| 55.10 | **`-`（第一帧空）** | **-3.24** | — |
| 55.20~59.87 | `-` 持续 | -3.32→-7.74 | 全程无 YIELD / ped_yield_clear |

证据行：
- `2209_restart_precise.txt: t=55.02 ... lat=-3.17 ... dec=stop` → `t=55.10 ... lat=-3.24 ... dec=-`
- `2209_ped_dec.txt`：t=47.22 起 `id=7673_0 STOP(423438.0~438.3,rc=1)`，栅栏固定，最后一条 t=54.72
- `2209_y_precise.txt` / `2209_ped_vs_ego.txt`：t=55.0 dec=stop → t=55.1 dec=`-`

### 1.2 判定依据（四条铁证）

1. **放行后决策标签 = `-`（空），不是 `ped_yield_clear` YIELD**（决定性）
   - 若 `kClearLateral=3.5` 触发，`HandlePedestrianStop` else 分支会
     `AddLongitudinalDecision("dp_st_graph/ped_yield_clear", yield)`（speed_decider.cc:654）。
   - YIELD 是 non-ignore 决策 → `ReferenceLineInfo::SetObjectDecisions`（reference_line_info.cc:981）**不会过滤** → 必然出现在发布的 object_decision → `dec` 应显示 `yield`。
   - 实际 55.10~59.87 全程 `dec=-`（7673 从 object_decision 消失）。

2. **放行瞬间行人横向 -3.24m < kClearLateral=3.5**
   - 即使按 SL 横向（与全局差 ~0.3m）≈ 3.5 边缘，标签证据（铁证 1）已排除。

3. **放行是「STOP 直接消失」，无 STOP→YIELD 过渡**
   - t=55.02 stop → t=55.10 `-`，中间帧不存在 yield。
   - 若是 kClearLateral，会出现 ped_fixed → ped_yield_clear 的标签跳变（至少一帧 yield）。

4. **放行前最后几帧全是 `dp_st_graph/ped_fixed` STOP，栅栏固定 423438.x**
   - 说明 HandlePedestrianStop 一直在 `obs_l < 3.5` 的 ped_fixed 分支，从未进入 else（放行）分支。

### 1.3 为什么 `dec` 显示 `-` 而不是 `ignore`

`ReferenceLineInfo::SetObjectDecisions`（reference_line_info.cc:981-989）：
```cpp
for (const auto obstacle : path_decision_.obstacles().Items()) {
    if (!obstacle->HasNonIgnoreDecision()) {
      continue;   // ← IGNORE-only 障碍物从发布的 object_decision 中完全消失
    }
    ...
}
```
`boundary.IsEmpty()` → `AppendIgnoreDecision` 只加 ignore → 7673_0 变成 ignore-only → 从消息中消失 → `dec=-`。
**这正是 boundary 空旁路的签名**（若障碍物有 YIELD 就不会消失）。

### 1.4 机制链（代码全链路）

```
SPEED_BOUNDS_PRIORI_DECIDER（SpeedBoundsDecider → st_boundary_mapper.cc）
  └─ 行人 7673_0（动态，有预测轨迹）无前序纵向决策
     ├─ PATH_DECIDER 跳过动态行人（path_decider.cc:141 `if (!obstacle->IsStatic()) continue;`）
     ├─ RULE_BASED_STOP_DECIDER 只做 side-pass/lane-change/path-end，不碰行人
     └─ ComputeSTBoundary → GetOverlapBoundaryPoints 重叠检测
          行人横向超阈值 → 预测轨迹所有点与 ADC box 不再重叠
          → lower/upper_points 空 → path_st_boundary 保持空

SPEED_DECIDER（MakeObjectDecision，speed_decider.cc:215-223）
  if (boundary.IsEmpty() || ...) {
      AppendIgnoreDecision(mutable_obstacle);   // ← t=55.10 走这里
      continue;                                  //   HandlePedestrianStop 不被调用
  }
  ...（boundary 非空时才走 CheckIsFollow/else → HandlePedestrianStop）
```

---

## 2. ST boundary 变空的横向阈值计算

### 2.1 实际生效代码：`st_boundary_mapper.cc`（不是 st_obstacles_processor.cc）

LANE_FOLLOW pipeline（`modules/planning/scenarios/lane_follow/conf/pipeline.pb.txt`）：
```
LANE_FOLLOW_PATH → PATH_DECIDER → RULE_BASED_STOP_DECIDER
→ SPEED_BOUNDS_PRIORI_DECIDER(SpeedBoundsDecider)   ← ST boundary 生成器
→ SPEED_HEURISTIC_OPTIMIZER → SPEED_DECIDER
→ SPEED_BOUNDS_FINAL_DECIDER → PIECEWISE_JERK_SPEED
```
**没有 ST_BOUNDS_DECIDER**。`st_obstacles_processor.cc` 的 `kADCSafetyLBuffer=0.1` 只对 ST_BOUNDS_DECIDER 生效，当前 LANE_FOLLOW 未启用。
（既有报告「停车避让放行方案评估.md」把 kADCSafetyLBuffer=0.1 误归到 st_boundary_mapper，需修正。）

### 2.2 重叠判定公式（st_boundary_mapper.cc:461-482 `CheckOverlap`）

```cpp
Box2d adc_box(ego_center_map_frame, path_point.theta(),
              vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);
return obs_box.HasOverlap(adc_box);
```
- `l_buffer`：`FLAGS_nonstatic_obstacle_nudge_l_buffer` = **0.4**（非换道；planning.conf:37 确认）
- 动态行人走 `CheckOverlapWithTrajectoryPoint`（st_boundary_mapper.cc:343），shape = `GetObstacleTrajectoryPolygon(trajectory_point)`（5×2m 感知多边形按预测轨迹 heading 旋转，obstacle.cc:773）

### 2.3 数值代入（车宽 2.11m 为回放实测）

**车宽 = 2.11m**（半宽 1.055）——`print_ego_box` 实测：
`output/_analysis_7673/switch_window_220841.txt`（2208 时段，即 2209 回放）：
```
print_ego_box:(423436.677979, 4437610.771886, -0.001487, 4.933000, 2.110000)
```
（`data/log/planning.log.INFO.20260801-201240.729558` 同样 width=2.110000；map.log "half vehicle width[1.05]" 佐证）

行人 7673 感知盒 5×2×1.8m（`2209_scenario_7673.json.txt` 确认 width=2），heading = 速度方向 ≈ -21°（速度 (1.93,-0.74)）。

| 项 | 值 | 说明 |
|----|----|----|
| 车半宽 | 1.055 m | vehicle width 2.11 / 2 |
| + l_buffer | +0.4 m | FLAGS_nonstatic_obstacle_nudge_l_buffer |
| = ADC box 半宽 | **1.455 m** | 2.91/2 |
| 行人 polygon 横向半投影 | **1.83 m** | (5/2)sin21° + (2/2)cos21° = 0.90+0.93 |
| **理论重叠失效线** | **≈ 3.29 m** | 1.455 + 1.83（车中心↔行人中心） |
| **实测变空点** | **≈ 3.2 m** | boundary 在 t∈(55.02,55.10)，感知 lat∈(-3.17,-3.24) 变空 |

**理论 3.29m 与实测 3.2m 高度吻合**（差异 ~0.1m 来自：ST boundary 用预测轨迹而非感知位置，预测首点滞后感知 ~0.2m（`2209_pred_7673.txt`）；Box2d 精确几何 vs 简化横向距离）。

> 注：用 st_obstacles_processor 的 kADCSafetyLBuffer=0.1 算为 1.055+0.1+1.83=2.99m，与实测 3.2 差更大，再次佐证生效的是 st_boundary_mapper 的 0.4。

### 2.4 补充：为何 STOP 能撑到横向 3.17m 才消失

- 动态行人的 ST boundary 由**预测轨迹**生成（不只当前感知位置）。
- 重叠判定是「预测轨迹任一时刻点与 ADC box 是否重叠」（st_boundary_mapper.cc:300-342）。
- 行人以横向速率 ~1.07m/s 持续远离 → 预测首点（=当前+滞后）横向从 0 增至 ~3.3m 后，所有预测点都不再与 ADC box 重叠 → boundary 空。
- 放行瞬间预测轨迹**仍非空**（n_traj=1, n_pts=59，`2209_pred_7673.txt:552`）——「预测轨迹存在」≠「ST boundary 非空」，关键在重叠。

---

## 3. 对修复方案的影响结论

1. **只改 `kClearLateral`（把 3.5 → 8/11m）必然无效**
   - `kClearLateral` 只在 `HandlePedestrianStop` 内生效，而 `HandlePedestrianStop` 只在 **boundary 非空**时才被调用（CheckIsFollow/else 分支）。
   - 2209 中 boundary 在行人横向 ≈3.2m 就空，放行由 boundary 空旁路在 -3.24m 触发——**kClearLateral=3.5 根本到不了**。改成 8/11m 更到不了。
   - 这与 1925 回放（add9 实证 8.0m 阈值永不生效）及「停车避让放行方案评估.md」结论一致。

2. **必须堵 boundary 空旁路**
   - `MakeObjectDecision` 顶部 `boundary.IsEmpty()` 分支（speed_decider.cc:218-223），对 `PEDESTRIAN` 且 `ped_fixed_fence_s_` 有记录的障碍物，改为调 `HandlePedestrianStop` + continue（而不是 `AppendIgnoreDecision`），放行点才能真正由代码控制。
   - 这是任何「推迟放行」方案（kClearLateral 增大 / 加静止判据 / 纵向距离判据）生效的前提。

3. **推荐组合（沿用既有 add9 全家桶思路）**
   - ① boundary 空时救 fence 行人（堵旁路）；
   - ② 放行条件 = 横向≥3.0m 且 速度≤0.3（静止判据，2012b 已验证可行）；
   - ③ fence key 用 base_id（7673/7673_0 共享，防 id 切换闪烁）；
   - ④ （可选）超时兜底防永不静止。

4. **阈值修正提醒**：既有文档「停车避让放行方案评估.md」将 kADCSafetyLBuffer=0.1 归给 st_boundary_mapper 有误。当前 LANE_FOLLOW 实际生效链路 = **st_boundary_mapper + FLAGS_nonstatic_obstacle_nudge_l_buffer=0.4 + 车宽 2.11m**，重叠失效线 ≈ **3.29m（理论）/ 3.2m（实测）**。

---

## 4. 关键文件/行号速查

| 位置 | 内容 |
|------|------|
| `speed_decider.cc:218-223` | `boundary.IsEmpty()` → AppendIgnoreDecision 旁路 |
| `speed_decider.cc:616-655` | HandlePedestrianStop（kClearLateral=3.5，ped_fixed / ped_yield_clear） |
| `speed_decider.cc:652-654` | ped_yield_clear YIELD 放行分支（2209 从未出现） |
| `reference_line_info.cc:981-989` | SetObjectDecisions 过滤 ignore-only → dec=`-` 的原因 |
| `st_boundary_mapper.cc:461-482` | CheckOverlap（ADC box = 宽 + 2×l_buffer） |
| `st_boundary_mapper.cc:211` | l_buffer = FLAGS_nonstatic_obstacle_nudge_l_buffer |
| `st_boundary_mapper.cc:343-390` | CheckOverlapWithTrajectoryPoint（动态行人路径） |
| `obstacle.cc:773` | GetObstacleTrajectoryPolygon（5×2 多边形按 heading 旋转） |
| `path_decider.cc:141` | `if (!obstacle->IsStatic()) continue;`（跳过动态行人） |
| `scenarios/lane_follow/conf/pipeline.pb.txt` | 任务链 = SPEED_BOUNDS_PRIORI/FINAL_DECIDER（无 ST_BOUNDS_DECIDER） |
| `output/_analysis_7673/switch_window_220841.txt` | print_ego_box width=2.110（车宽实测） |
| `profiles/default/.../planning.conf:37` | `--nonstatic_obstacle_nudge_l_buffer=0.4` |
