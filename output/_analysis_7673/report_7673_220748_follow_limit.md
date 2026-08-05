# 行人 7673「停车避让 / 跟车限制」日志分析报告

- 日志：`data/log/planning.log.INFO.20260801-220748.921814`（389,710 行，22:07:48.6 – 22:09:17.9）
- 代码版本：HEAD `1695aef` = revert 回退到 `c904b46`（跟车限制阈值 3.5m + 人行道 expand 4.0）
- 场景：Xh2026 障碍物停车避让（行人 7673），直线道路，总时长 100s
- 分析性质：纯日志 + 只读源码分析，未改代码

---

## 0. 场景设定（来自 `2209_scenario_7673.json`）

| 项目 | 值 |
|------|-----|
| 行人 7673 | PEDESTRIAN，包围盒 5.0×2.0×1.8 m |
| 行人初始位置 | (423446.666, 4437610.949)，heading -0.045 |
| 行人动作 | **仿真 t≥50s 触发**，线性加速至目标速度 **3.0 m/s** |
| 自车起点 | (423278.192, 4437610.951) |
| 自车终点 | (423491.106, ...) |
| 车道宽度 | lane_1724，左右各 1.9767 m（车道半宽 ≈1.98 m） |

---

## 1. 行人 7673 出现 / 消失时间线（planning 日志）

| 事件 | planning 时间 | 行号 | 日志 |
|------|--------------|------|------|
| 7673 首次进入参考线 ST 图 | 22:07:52.558 | 18571 | `reference_line_info.cc:422] build reference line st boundary. id:7673` |
| 7673 触发路径边界右推 | 22:07:57.265 | — | `path_bounds_decider_util.cc:269] last_max_nudge_l:-0.156, obs id:7673, obs l:-0.698,-0.186` |
| 7673 阻塞 self 路径 | 22:07:57.365 | — | `path_bounds_decider_util.cc:305] blocked at 7673, 183.278, l_lower:-2.07, l_upper:1.29` |
| **id 切换 7673 → 7673_0** | 22:08:40.50 | — | `reference_line_info.cc:422] ... id:7673_0`（116 条，至 22:08:51.96） |
| id 恢复 7673 | 22:08:52 起 | — | `... id:7673`（至 22:09:15.26 日志结束前） |
| 7673 横向位置稳定值 | 22:07:57–22:08:39 | — | obs l ∈ [-0.703, 1.299]（**在车道内**，中心≈0.3） |
| 7673 横向位置开始变化 | 22:08:39.77 | — | obs l → [0.045, 0.873] |
| 7673 已横向离开 ~11m | 22:08:56.86 | — | obs l ∈ [-11.14, -9.95] |

**结论**：7673 从 22:07:52 到日志结束（22:09:16）始终被跟踪，从未消失。行人横向位置从车道内（l≈-0.7~1.3）逐渐离开至 l≈-11m。

---

## 2. 7673 决策时间线（`/apollo/planning` 输出，record t，t=0≈22:07:53.5）

来源：`output/2209_ped_dec.txt`、`output/2209_follow_all.txt`

| record t | 决策 | 说明 |
|----------|------|------|
| 3.68 – 13.67 | `7673 STOP(423438.1, rc=4)` | rc=4 = STOP_REASON_OBSTACLE，来自 **path_decider** |
| 14.67 – 46.67 | `7673 STOP(423438.5~439.5, rc=1)` | **rc=1 = 默认值(HEAD_VEHICLE)**，来自 **speed_decider::HandlePedestrianStop 固定 fence**（不设 reason_code） |
| 19.37 – 46.67 | 附加 `PATH_END_regular/leftforward:STOP` | 借道路径存在期间 |
| 47.04 – 54.81 | `7673_0 STOP(rc=1)` | id 切换后动态行人，仍为 HandlePedestrianStop 固定 fence |
| 55.10 起 | 7673_0 决策消失 | 行人横向离开 ≥3.5m，fence 释放 |
| 全程 | `DEST:STOP` | 终点 STOP 常驻；59.59 起追加 `REF_END_1_0:STOP` |

**关键结论**：7673/7673_0 全程 **STOP（无 FOLLOW）**，且 t=14.67 起 STOP 来自 HandlePedestrianStop 的固定 fence —— **该版本防跟车逻辑确实生效**。

---

## 3. 自车运动学时间线（`output/2209_ped_vs_ego.txt`、`2209_restart_precise.txt`）

| record t | 自车 | 行人 | 说明 |
|----------|------|------|------|
| 0.9 | x=423290.6, v=5.76 | 静止 (423446.67, 4437610.95, v=0) | 起步 |
| 3.6 | — | — | obs 决策变 stop |
| 7.8 | v=11.67（峰值） | 静止 | 开始减速 |
| 20.5 | x=423431.1, v=1.26 | 静止 | **进入极慢逼近（蠕行）** |
| 37.5 | x=423435.25, v=0.00 | 静止 | **完全停稳**（距行人 11.4m，fence 前 3.5m） |
| 37.5–55.4 | 静止 x=423435.25（18s） | 静止→46.0 起移动 | 等待 |
| 46.0 | 静止 | v=0.02 → 加速 | 行人开始移动 |
| 52.0 | 静止 | lat=-1.33, v=1.37 | 行人横向仅 1.33m |
| 55.1 | 静止 | lat=-3.24, v=2.07 | **7673_0 STOP 释放（lat≈3.5m 阈值）** |
| 55.4 | v=0.04 起步 | lat=-3.47 | 自车起步 |
| 57.87 | v=3.46 | lat=-5.63 | 加速通过 |
| 59.97 | x=423446.78 | 已离开 | 到达行人原始位置，**直线通过** |

起步后轨迹（`2209_traj_lateral.txt`）：y 恒为 4437610.77，横向偏移仅 +0.001 m —— **自车沿原车道直线通过，无绕行/右偏**。

---

## 4. 「跟车限制检测 20 分失败」核心嫌疑点

1. **蠕行逼近（t=20.5–37.5，约 17s）**：自车在已有 STOP 决策下仍以 0.02–1.26 m/s 缓慢推进 4.2m（x=423431→423435.25）。虽然最终停在正确位置（fence 前 3.5m），但长时间低速贴近行人的过程与「跟车/蠕动」语义吻合。
2. **起步过早（t=55.4）**：自车在行人横向离开仅 **lat≈3.5m**（恰为 `HandlePedestrianStop::kClearLateral=3.5` 释放阈值）时立即起步。此时行人仍在移动（v≈2.1 m/s），且车道半宽≈1.98m + 行人半宽 1m ≈ 3m —— **行人边缘几乎还在车道边界上**。检测可能判定「行人未完全离开就启动通过」。
3. **对比 git 历史**：后续 commit 正是把该阈值不断调大 `3.5m → 6.0m (c8018da) → 8.0m (5af4e4f) → 且要求行人静止 (ee910fd)`，验证了「3.5m 释放过早」是失败根因方向。

---

## 5. 7673/7673_0 双 id 机制（详见 `output/report_7673_dual_id_prediction.md`）

- `_0` 后缀是 **planning `Obstacle::CreateObstacles` 加的**：prediction 判静止（无轨迹）→ id=`7673`（is_static=true）；判运动（有 FreeMove 轨迹）→ id=`7673_0`（is_static=false）。
- 本次回放：22:07:50–22:08:40.36 全为 `7673`；22:08:40.50–22:08:51.96 全为 `7673_0`（116 条）；22:08:52 起恢复 `7673`。
- **当前 HEAD 不含 add12/add13**，`ped_fixed_fence_s_` 与 `CheckStopForPedestrian` 计时表均按原始 `Id()`（"7673"/"7673_0"）索引 → id 切换会打断/重置固定 fence 与 4s 静止计时（add12 `7abb9a6` 用基 id 消除 STOP 闪烁、add13 `de34a95` 位置兜底均未合入）。

---

## 6. 错误 / 警告清单

**ERROR（37 个）**：
| 类型 | 次数 | 说明 |
|------|------|------|
| `file.h:271 conf file [lane_follow_stage/*.pb.txt] not found` | 28（14 类 ×2） | **14 个 task 的 stage 级自定义配置缺失**（speed_decider/path_decider/lane_follow_path/lane_borrow_path/speed_bounds_*/rule_based_stop/piecewise_jerk_speed 等），全部回退到默认配置 |
| `trajectory_stitcher.cc:213 lon_diff=208.428 Replan` | 2（22:09:15.42 / 22:09:15.52） | 日志末尾轨迹拼接误差 208m 触发重规划 |
| `on_lane_planning.cc:598/416 planner failed` | 2+2（22:07:49.33/49.83） | 启动期前两周期规划失败（初始化，N→D 换挡 22:07:50.12 正常） |

**WARN（117 个）**：
| 类型 | 次数 | 说明 |
|------|------|------|
| `destination.cc:82` 终点在参考线末端 | 80（22:07:49.9–57.7） | 终点被 PATH_END fence 影响的潜在风险 |
| `rtps_dispatcher.h:94` 消息先于发送 | 11 | 时序噪声 |
| `trajectory_stitcher.cc:313` 时间倒退 | 4 | 拼接噪声 |
| 配置缺失对应 WARN | 22 | 同 ERROR |

---

## 7. 借道 / 路径线索

| planning 时间 | 事件 |
|--------------|------|
| 22:07:57.27 | `path_bounds_decider_util.cc:287] 7673 right nudge`（右推） |
| 22:08:08.36 | `lane_borrow_path.cc:422] Switch from SELF-LANE to LANE-BORROW`；同时 `path_decider.cc:205] skip_overlap_stop_check` 开始（321 次，至 22:08:40.36） |
| 22:08:41.09 | `lane_borrow_path.cc:370] Switch from LANE-BORROW to SELF-LANE` |
| 22:08:41.19 | 又切回 LANE-BORROW |
| 22:08:41.78 | 又切回 SELF-LANE（最终回自车道） |
| 借道阻塞障碍物 | `lane_borrow_path.cc:374] Blocking obstacle ID[7673]`（111 次）、`ID[5530]`（8 次） |

- `skip_overlap_stop_check=true`（当前配置）：path_decider 对「横向重叠的静态障碍物」**跳过 STOP**（用户自定义逻辑），与借道期间（22:08:08–40）吻合。
- 最终路径：起步后（t=55.99 起）轨迹 y 恒定，**直线通过，无右偏**。

---

## 8. 结论摘要

1. 行人 7673 全程被跟踪：22:07:52 出现 → 22:09:16 仍在；横向从车道内（l≈0.3）离开至 l≈-11m。
2. 自车决策全程 STOP（无 FOLLOW），t=14.67 起为 `HandlePedestrianStop` 固定 fence（rc=1 默认值），**防跟车逻辑在本版本已生效**。
3. 自车 22:08:31 停稳于行人前 11.4m（fence 前 3.5m），静止等待 18s。
4. 行人 t≈46–50s 开始移动（场景设定 t=50s 触发、目标 3m/s）。
5. **自车在行人横向离开 ≈3.5m 时（t=55.4）起步并直线加速通过** —— 这就是「跟车限制检测」20 分失败的最可能原因：3.5m 释放阈值过小（行人仍在车道边缘、仍在移动），起步后高速贴边通过。
6. 次要问题：14 个 stage 级 task 配置缺失（回退版无）、22:09:15 两次 lon_diff=208m Replan、7673/7673_0 双 id 打断 fence/计时（add12/add13 未合入）。

**修复方向参考（git 历史已验证）**：将 `HandlePedestrianStop::kClearLateral` 从 3.5 提高至 6.0/8.0（c8018da/5af4e4f），并考虑行人静止后再起步（ee910fd）；用基 id（PerceptionId）统一 fence/计时 key（7abb9a6）。
