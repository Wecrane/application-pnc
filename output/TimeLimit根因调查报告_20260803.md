# 【根因调查】车到达终点后仿真不结束 → TimeLimit=0（2026-08-03）

> 调查对象：本轮云端评测 7/8 场景 TimeLimit=0（scn5 通过）
> 数据源：`output/cloud_20260803/`（planning/sim_engine/replay-engine 日志）+ `/tmp/scn001/`（08-02 旧版 sim_engine）

---

## 一、scn1(失败) vs scn5(通过) 终点行为对比

### scn1 减速带（6a7013d1f0642a2aee347b7e，TimeLimit=0）

| 项目 | 值 | 证据 |
|------|-----|------|
| 终点坐标 | (423477.669, 4438475.743) | replay-engine auto_car_info.end |
| 停稳位置 | (423482.671, 4438476.013) | 轨迹最后帧 |
| 距 DEST 停止线 | **0.56m**（车头前） | `distance_destination:0.560635` + `stops=DEST@0.560694` |
| 最后 v>0.5 帧 | planning rel≈42.47s | 轨迹 |
| 停稳后行为 | **v=0.000 持续 18+ 秒**（rel 43→60.86），位置完全不变，无蠕动/无再起步 | 轨迹最后 30 帧全部 v=0 |
| 终点决策 | P0 停稳生效 `v_up[0]=0`，主 STOP=DEST（reason DESTINATION） | pjs 日志 |
| 评测 ReachEnd | 通过，计分帧 t=46.6s 起 | sim_engine |
| 评测 TimeLimit | **0 分**，帧跑到 **t=100.1s** | sim_engine（limit:90, current:100.1） |
| command_status | **1000 次 RUNNING，0 次 FINISHED** | planning_component.cc:274 日志 |
| passed destination | **始终 0**（`Current passed destination:0`） | planning_base.cc:93 |
| REF_END / PATH_END | REF_END **0 次**（被禁用）；`distance_ref_end` 恒 DBL_MAX | 日志 grep |

### scn5 跟车（6a7014c0f0642a6f65347b9a，满分）

| 项目 | 值 | 证据 |
|------|-----|------|
| 终点坐标 | (423491.106, 4437610.338) | replay-engine auto_car_info.end |
| 停稳位置 | (423486.070, 4437610.368)，d_dest=0.56m | 轨迹 |
| 评测 ReachEnd | 通过，t=42.3s | sim_engine |
| 评测 TimeLimit | **100 分**，帧到 **t=60.1s** | sim_engine |
| command_status | **600 次 RUNNING，0 次 FINISHED** | planning_component.cc:274 |
| passed destination | 始终 0 | planning_base.cc:93 |

### 关键对比结论
- **两场景车都停在了终点（front 距 DEST 停止线 0.56m），都完全静止** —— 行为上没有本质区别
- **区别只在评测器配置的仿真时长**：scn5 `stop_trigger=60s`（<90 限），scn1 等 7 场景 `stop_trigger=100s`（>90 限）
- scn5 "通过" 是**偶然**（stop_trigger=60 < 90），**不是**"车到终点→评测正常结束"
- 仿真结束 = 服务器端 `stop_trigger`（simulation_time 条件），**与车是否到达终点无关**（本轮回放中车停终点 50+ 秒，仿真照跑）

---

## 二、评测结束机制判断

### 服务器端场景配置（replay-engine.log，8 场景全部确认）
```
storyboard { stop_trigger { conditions { by_value_condition {
  simulation_time_condition { rule: greaterOrEqual  value: 100 } } } } }
```
| 场景 | stop_trigger | TimeLimit(90s) |
|------|-------------|----------------|
| scn1/2/3/4/6/7/8 | **100s** | **必然超时 0 分** |
| scn5 | 60s | 60<90 通过 |

### Apollo 标准"提前结束"信号链
```
车停稳在终点
  → ReachedDestination()=true        (ref_line_info.cc:785)
  → has_passed_destination=true      (MakeMainMissionCompleteDecision, :885)
  → IsPlanningFinished()=true        (planning_base.cc:89)
  → command_status=FINISHED          (planning_component.cc:270, external_command 通道)
  → sim_engine 提前结束仿真
```

### 旧版（08-02，800 分）佐证
- `/tmp/scn001/scn1~8/sim_engine.log`：8 场景 TimeLimit **全 100 分**，仿真在 **19~67s** 各不同时刻提前结束（均紧随各自 ReachEnd）→ **旧版存在"车停稳→提前结束"机制**

---

## 三、根因结论（具体到代码/机制）

**当前 planning 从不发送 `command_status=FINISHED`，评测器收不到"任务完成"信号，只能跑到服务器端 stop_trigger（100s），必然超过 TimeLimit 的 90s 上限。**

阻断点（两个叠加）：
1. **REF_END 被禁用**：`profiles/default/.../planning.conf:23` → `--disable_reference_line_end_stop=true`（2026-08-02 44dd401 为赛题五实验引入，至今未恢复）→ `reference_line_end.cc:43` 早退 → 日志中 REF_END 0 次、`distance_ref_end` 恒 `1.79769e+308`(DBL_MAX)
2. **DEST 阈值太严**：车停终点前 0.56m，但 `FLAGS_passed_destination_threshold=0.05`（planning_gflags.cc:173）→ `ReachedDestination()` 恒 false → `has_passed_destination` 恒 false → `IsPlanningFinished()` 恒 false

结果：planning_component.cc 每帧都发 RUNNING（日志实证 1000/600 次），评测器只能等 stop_trigger。

---

## 四、修复建议

### 方案A（推荐，代码最小侵入）—— 车停稳即发 FINISHED
`modules/planning/planning_base/common/reference_line_info.cc` `MakeMainMissionCompleteDecision`（约 :885）：
```cpp
if (ReachedDestination()) {
    planning_context->mutable_planning_status()
        ->mutable_destination()->set_has_passed_destination(true);
} else {
    mission_complete->mutable_stop_point()->CopyFrom(main_stop.stop_point());
    mission_complete->set_stop_heading(main_stop.stop_heading());
}
```
改为（增加"已停稳在终点停止线附近"判定）：
```cpp
const double adc_speed = vehicle_state_.linear_velocity();  // 当前帧实际车速
if (ReachedDestination() ||
    (distance_destination <= 2.0 && adc_speed < 0.3)) {
    planning_context->mutable_planning_status()
        ->mutable_destination()->set_has_passed_destination(true);
} else {
    mission_complete->mutable_stop_point()->CopyFrom(main_stop.stop_point());
    mission_complete->set_stop_heading(main_stop.stop_heading());
}
```
- 车停稳（速度<0.3 且距 DEST 停止线<2m）→ has_passed_destination=true → FINISHED → 评测提前结束
- **不触碰 `ReachedDestination()`**，避免 PJS 早退（`piecewise_jerk_speed_optimizer.cc:59` 空 speed_data → `CombinePathAndSpeedProfile` 失败 → stage ERROR → command_status 反而变 ERROR）

### 方案B（配置，最简单但有风险）
`planning.conf` 加 `--passed_destination_threshold=1.5`
- ⚠️ 会让 `ReachedDestination()` 在车仍在制动时（距线 1.5m）为 true → PJS 返回空 speed_data → stage ERROR → `command_status=ERROR` 而非 FINISHED → **可能失效**，需先本地验证

### 方案C（配置，恢复官方机制）
`planning.conf` 改 `--disable_reference_line_end_stop=false`
- 恢复 REF_END（≤1.5m 触发）→ 但会**重新引入当初禁用它的"提前 28m 减速+终点蠕动"问题**，且若参考线末端超出终点较多，车停 DEST 时距 REF_END 可能仍 >1.5m 不触发

### 验证步骤
1. 改方案A → 编译 → 本地回放 scn1/scn5
2. 确认日志出现 `Set the external_command: FINISHED`（车停稳后 ~1s 内）
3. 云端重测：期望各场景 sim 在"车停稳 + 短暂余量"后提前结束（<90s），TimeLimit=100

### 前提与风险
- 本结论依赖假设：**评测器以 `command_status=FINISHED`（external_command 通道）判定任务完成**。这是 Apollo WORLDSIM 标准机制，且旧版"车停稳→仿真提前结束"（800 分）与"当前从无 FINISHED + 必跑满 stop_trigger"的现象完全自洽
- 若方案A 实测无效（FINISHED 后仿真仍不结束），则 TimeLimit 问题为服务器端 stop_trigger=100s 的配置所致，代码层无解（7/8 场景结构性无法过 90s 限），需向赛事方反馈
