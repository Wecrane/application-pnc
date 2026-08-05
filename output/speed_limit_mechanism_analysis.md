# 速度限速（AddSpeedLimit）机制深度研究：为什么不生效 + 全局规划不闪烁

> 日期：2026-08-02　只读代码分析 + 实测日志验证（planning.log.INFO.20260802-224424）
> 场景：Signal_5 路口（scn8 绿灯通过 / scn3 红绿灯），参考线 229.75m，人行道 Crosswalk_52/54 位于 s≈211-227

---

## 一、限速机制完整链路（AddSpeedLimit → v_upper_bound）

```
TrafficRule (crosswalk.cc / traffic_light.cc)
  └─ AddSpeedLimit(start_s, end_s, speed)          // 写 reference_line.speed_limit_ 区间表
      └─ ReferenceLine::speed_limit_  (区间合并/求 min，按 start_s 排序)
          │
          ▼ 每帧 SpeedBoundsDecider（SPEED_BOUNDS_PRIORI/FINAL_DECIDER）
          SpeedLimitDecider::GetSpeedLimits()
            └─ 对离散路径每点 i：
                 reference_line_s = frenet_path[i].s()        // 参考线坐标（世界锚定）
                 v = reference_line_.GetSpeedLimitFromS(reference_line_s)  // 区间命中→4.5，未命中→地图限速
                 speed_limit.AppendSpeedLimit(path_s[i], v)   // ★key 是 path_s（车体局部坐标）
            └─ 存入 st_graph_data.speed_limit（StGraphData::LoadData）
          │
          ▼ SPEED_HEURISTIC_OPTIMIZER（DP）
          GriddedPathTimeGraph::InitSpeedLimitLookUp()
            └─ speed_limit_by_index_[i] = GetSpeedLimitByS(s[i])
          DpStCost::GetSpeedCost() → ★限速 = 软惩罚 exceed_speed_penalty(1000)*default_speed_cost(1000)*det²
          └─ 输出粗速度曲线 → reference_line_info.speed_data()（覆盖 10s，1s 步长 11 点）
          │
          ▼ PIECEWISE_JERK_SPEED（QP）
          for knot i (0.1s 步长, 101 点):
            path_s = reference_speed_data.EvaluateByTime(t_i).s()   // 参考=上一任务DP输出
            v_upper_bound = min( GetSpeedLimitByS(path_s), FLAGS_planning_upper_speed_limit )
            s_dot_bounds[i] = [0, v_upper_bound]                    // ★硬约束
          piecewise_jerk_problem.set_dx_bounds(s_dot_bounds)
          Optimize() 失败 → set_dx_bounds([0, max(planning_upper_speed_limit, init_v)])  // ★丢限速
                              → 重新 Optimize()（speed_optimize_fail_relax_velocity_constraint=true）
          再失败 → speed_data.clear() → TrajectoryFallbackTask 生成急停 fallback
```

### 关键函数语义

| 函数 | 位置 | 语义 |
|------|------|------|
| `AddSpeedLimit` | `reference_line.cc:951` | 区间合并：与新限速重叠处取 min，切割重排 |
| `GetSpeedLimitFromS` | `reference_line.cc:919` | 命中 `[start_s,end_s]` 返回限速；否则查地图车道限速/默认 |
| `GetSpeedLimitByS` | `speed_limit.cc:42` | `lower_bound` → 返回**第一个 s≥查询点**的值（阶梯函数，限速提前约 1 个采样点生效） |
| `SpeedLimitDecider` | `speed_limit_decider.cc:65` | 把参考线限速逐点转成 path_s 为 key 的 SpeedLimit 点表 |
| `EvaluateByTime` | `speed_data.cc:60` | t 超出参考数据范围**返回 false**（path_s 保持默认 0！） |

---

## 二、任务1：限速为什么不生效（根因，实测日志验证）

### 实测证据（22:44 日志，347 帧）

```
22:44:25  车起步 (x=423254.57, y=4438082.20, v=0)
22:44:31-38  加速至 16.3m/s（a=+2.7）
22:44:39.9  v=16.29 → ★QP 开始 primal infeasible（64 次，持续到 22:44:46.2）
22:44:40-45  车 15~16.5m/s 缓速（a=-0.4），未见 4.5 限速约束
22:44:47-48  重新加速 16.3→16.6m/s（限速"闪烁"）
22:44:48-49  ★以 16.5m/s 通过人行道区（Crosswalk_52/54 在 s≈211-227 = y 4438293-4438309）
22:44:49-54  急刹（a=-4）停在 y=4438360（终点 DEST，非人行道）
22:44:50.9-53.8  ★11 次"Speed fallback due to algorithm failure"（QP 双失败→整体急停回退）
ST 边界：仅 DEST(229.75m) + PATH_END_regular，无行人/障碍物 STOP
```

**结论：车以 16.5m/s 全速冲过人行道，只在终点停下。限速约束从未真正生效。**

### 根因 1（主因，实锤）：QP 硬约束不可行 → fallback 把限速丢掉

- `piecewise_jerk_speed_optimizer.cc`：`v_upper_bound` 是**硬约束**（`set_dx_bounds`），且用**参考轨迹的 path_s** 查表（`GetSpeedLimitByS(path_s)`）。
- 本场景限速区 `[160.8, 227.7]@4.5` 与终点 `DEST@229.75` **只差 2m**：
  - 若参考曲线不减速（DP 软惩罚可能放过），path_s 在 t≈9.4s 才到 228 → 要求 v≤4.5 直到 9.4s，然后必须在 0.6s 内从 4.5 停到 DEST(s≤229.75) → 需减速度 ~7.5 > max_decel(-6) + jerk 限制 → **不可行**。
  - `Optimize()` 返回 `primal infeasible`（日志实锤）。
- fallback：`speed_optimize_fail_relax_velocity_constraint=true`（默认）→
  `set_dx_bounds(0, max(16.667, init_v))` → **限速完全被抹掉**，重新求解成功 → 车巡航 16.667 冲过。
- 日志直接对应：QP 失败 64 次期间车保持 15~16.6m/s。

### 根因 2（需验证）：限速可能根本没被加上（GlobalSeenRedLight 误触发）

`crosswalk.cc` / `traffic_light.cc` 的加限速条件依赖全局 `GlobalSeenRedLight[signal_id]`：
- `traffic_light.cc`：信号颜色**非 GREEN/BLACK**（RED/YELLOW/UNKNOWN）任一帧 → 记 `seen_red=true` → 之后永远 `AddSpeedLimit`（绿灯限速 4.5）被跳过。
- `crosswalk.cc`：附近 30m 信号灯 `seen_red` → 斑马线限速也被跳过。
- **风险**：仿真启动首帧信号常为 `UNKNOWN` → 一旦被记为"见过红"→ 本场景整段**不再限速**。scn8（一直绿）与 scn3（真见红）只靠这一个全局静态 bool 区分，脆弱。
- 若此分支命中 → 限速从源头就不存在，DP/QP 全程 16.667，与观测完全一致。**两条路径都会产生"全速通过"，需开 record 的 `FLAGS_enable_record_debug` 或在代码打日志确认是哪条。**

### 根因 3（结构性）：DP 软惩罚 + QP 参考 path_s 查表耦合

- DP 里限速只是软惩罚（`exceed_speed_penalty`），DP 可能选择"超速冲到限速区跟前再急刹"，尤其当终点 DEST 紧随其后时 DP 的最优路径就是冲过去。
- QP 的 v 约束位置由**参考轨迹**（DP 输出）的 path_s 决定，而非 QP 自身的 s。参考不减速 → 约束被推到窗口末端 → 与 DEST 冲突 → 不可行。参考减速 → 约束提前，但 DP 输出只有 11 个点（unit_t=1s），插值后约束位置很粗，容易与 jerk 限制打架。

### 关于"提前减速"机制

- **无通用提前机制**：`speed_limit_decider.cc` 只在 `[start_s,end_s]` 内限速，无"limit_start 向前延伸"逻辑；`GetSpeedLimitByS` 的 lower_bound 语义只提前约 1 个采样点（~0.1-1m）。
- 唯一的"提前"是**调用方**：`crosswalk.cc:199` / `traffic_light.cc:166` 用 `start_s - 50.0`（50m 提前）——但只在限速真正进入 QP 且 QP 可行时才生效。
- 减速带：`reference_line_info.cc:127` 对 speed_bump ±1m 加 gflag 限速，同样无提前。

### 关于 total_time=10 窗口

- QP 只约束窗口内的 knot（10s×0.1s=101 点）。限速区进入窗口（车距限速起点 < 窗口内可达距离）后才会约束。
- DP 输出覆盖 10s（`dimension_t_=ceil(10/1)+1=11` 点），故参考 path_s 全窗口有效（不会因 `EvaluateByTime` 返回 false 而变成 0）——**但仅当 DP 成功且不触发"起点在碰撞内→全 0 速度"分支**。

---

## 三、任务2：全局规划不闪烁

### 3.1 滚动时域机制

- Apollo 每 **100ms** 一帧完整重规划：traffic rules → path → ST → DP → QP。每帧独立求解，帧间**无速度曲线记忆**（仅用上一帧 path 做 fallback）。
- 帧间输出只保证"本帧约束可行"，不保证与上一帧平滑衔接 → 任何约束"时有时无"都会变成速度曲线跳变。

### 3.2 参考线 s 是否每帧滑动？（关键澄清）

**本场景实测：参考线 s 是"世界锚定"的，不随车滑动。**

- 参考线 id 全程稳定：`_Lane_1684_..._Lane_736_10.60_24.85`（仅 10.60→10.61 微移 0.01m，是车越过首 waypoint 时参考线起点前移一次）。
- 车体在固定参考线上 s 递增（起点 y=4438082，车 y 单调增加），`AddSpeedLimit` 的 `start_s/end_s`（参考线坐标）逐帧不变 → **限速区在世界上位置稳定**。
- 但**参考线会在以下时刻重建并整体平移 s 原点**：车越过首个 waypoint、车道切换、新 routing、`UpdateReferenceLine` 检测到首尾点变化。这些时刻限速区 s 会跳变 → 速度曲线跳一下。
- 补充：`SpeedLimitDecider` 存限速用 **path_s（车体局部，车=0）**，QP 也用 path_s 查表——两处一致，随车前进限速区"迎面靠近"是正确行为，不构成闪烁。

### 3.3 闪烁的真实来源（按影响排序）

1. **QP fallback 二值化**（本场景实锤）：限速生效 ⇄ QP 不可行→放松→无限速，逐帧切换。日志中车 15→16.6 再急停的锯齿就是它。**这是"时有时无"的最大来源。**
2. **traffic rule 逐帧开关**：crosswalk 有无行人切换 STOP/限速/不限速；traffic_light 颜色切换（且 GlobalSeenRedLight 一旦置位永久关限速 → 不是闪烁而是"永久失效"）；人行道 id 变化等。
3. **全局扫描的远距限速进入/退出窗口**：`crosswalk.cc` 自定义扫描把几百米外人行道也加入 → 限速区在窗口边缘进进出出 → 车每帧看到/看不到 → 速度微颤。total_time=10 已缓解但未消除。
4. **参考线重建/换道**：s 原点跳变 → 限速区 s 整体平移。

### 3.4 让限速稳定的方案

| 方案 | 做法 | 效果/代价 |
|------|------|-----------|
| **A. 防 QP fallback 丢限速（推荐，治本）** | 修改 `piecewise_jerk_speed_optimizer.cc` fallback：放松时保留限速——`set_dx_bounds(0, max(4.5, init_v))` 而非 `max(16.667, init_v)`；或先去掉 DEST 冲突再重试；或把限速从硬约束改为**软惩罚**（像 DP，但权重高） | 限速永不消失；需调参防超速 |
| **B. 修 GlobalSeenRedLight 误触发** | 首帧 UNKNOWN 不计入"见过红"；或只在"当前帧确实为绿且历史见过红"才豁免限速；或加 3 帧去抖 | 保证 scn8 限速必然被加上 |
| **C. 限速区用绝对坐标/地图坐标** | 把 AddSpeedLimit 的 s 换算成世界坐标，或让限速随参考线平移而平移（参考线重建时按 overlap 重新投影） | 消除参考线重建造成的 s 跳变 |
| **D. 提前量足够** | 已做 50m（16.5→4.5 需 21m，含 jerk 裕量）；建议验证 `start_s-50` 在 QP 窗口内最早进入，并避免限速区紧贴 DEST | 让"看到即能减到" |
| **E. 一次规划长窗口** | total_time=10（已做）使远距限速从起步进入规划 | 已生效；代价是 QP 更大、更易不可行（需配合 A） |
| **F. 限速不闪烁兜底** | 帧间速度曲线平滑（如限制帧间 Δv），或沿用上一帧可行曲线做 warm start | 工程量大，慎用 |

**推荐组合**：A（fallback 保留限速）+ B（UNKNOWN 去抖）+ C（参考线重建时重投影限速区），配合现有 50m 提前。

---

## 四、验证建议

1. 打开 `FLAGS_enable_record_debug`（planning.conf），看 `st_graph.speed_limit` 是否含 4.5 段 → 区分"没加上"（根因2）还是"加上了但被 QP 丢"（根因1）。
2. 在 `crosswalk.cc` 加限速处、`traffic_light.cc` 加限速处打 AINFO（含 `GlobalSeenRedLight[id]` 值）→ 直接确认是否命中。
3. 在 `piecewise_jerk_speed_optimizer.cc` fallback 处打 AINFO（记录失败时的 init_v、stop_s、限速区）→ 确认 fallback 触发时刻与限速区关系。

---

## 五、关键代码位置速查

| 文件 | 行 | 说明 |
|------|----|------|
| `crosswalk.cc` | 199 | `AddSpeedLimit(start_s-50, end_s, 4.5)`（无行人/非见红灯口） |
| `traffic_light.cc` | 143,166 | `GlobalSeenRedLight` 判定 + 绿灯 `AddSpeedLimit(start_s-50, end_s, 4.5)` |
| `reference_line.cc` | 919,951 | `GetSpeedLimitFromS` / `AddSpeedLimit` |
| `speed_limit_decider.cc` | 55-120 | 逐点采样转 SpeedLimit（软 min 合并） |
| `speed_limit.cc` | 42 | `GetSpeedLimitByS`（lower_bound 阶梯） |
| `gridded_path_time_graph.cc` | 224, 573+ | DP 限速查询 / 软惩罚 cost |
| `dp_st_cost.cc` | 204 | `GetSpeedCost`（exceed_speed_penalty 软惩罚） |
| `piecewise_jerk_speed_optimizer.cc` | 156-176, 211-215 | v_upper_bound 硬约束 / QP 失败 fallback（丢限速） |
| `piecewise_jerk_problem.cc` | 165-173 | dx_bounds 硬约束矩阵 |
| `planning_gflags.cc` | 409 | `speed_optimize_fail_relax_velocity_constraint=true` |
| `speed_data.cc` | 60 | `EvaluateByTime` 超范围返回 false（path_s=0 隐患） |
