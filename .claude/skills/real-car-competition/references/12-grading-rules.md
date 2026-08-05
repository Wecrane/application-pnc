# 12 官方评分细则（auto_drive_console 一手资料，2026-08-05 从实车提取）

> **来源**：实车 `~/apollo-edu_9.1.1/auto_drive_console/`（官方评分控制台，已打包到 `shiche/car_890/grading_console/`）。
> 包含：`presets.json`（全局配置）、`metric/*.json`（各赛题评分指标）、`6/7号*.json`（场景定义）、`server.py`/`runtime.py`（评分引擎）、`records/score_history.json`（历史成绩）。
> 🚨 **这是官方评分细则的第一手资料——所有调参策略应以本文档为准！**

---

## 1. 全局评分开关（presets.json）— 最关键！

```json
"cruise_speed_check": {
    "enabled": true,
    "conf_file": "/apollo/modules/planning/planning_component/conf/planning.conf",
    "min_default_cruise_speed": 1.5,
    "max_default_cruise_speed": 5.1,
    "min_planning_upper_speed_limit": 1.5,
    "max_planning_upper_speed_limit": 5.1,
    "scope": "all"
}
```

### 🚨 核心铁律（实车 planning.conf 的硬性范围）

| 参数 | 合法范围 | 越界后果 |
|------|---------|---------|
| `--default_cruise_speed` | **[1.5, 5.1]** | 低于 1.5 或高于 5.1 → **整次判 0 分**（scope=all） |
| `--planning_upper_speed_limit` | **[1.5, 5.1]** | 同上，**整次判 0** |

**解读**：
- ⚠️ **千万不要把 `planning_upper_speed_limit` 调超过 5.1**！本地文档里的 20/31.3 等值在实车会直接判 0！
- ⚠️ 也**不要低于 1.5**
- 实车当前 `default_cruise_speed=5.0`、`planning_upper_speed_limit=5.0` ✅ **都在合法范围**——**别乱动这两个全局参数！**
- `scope=all` = 全程有效，任何时刻越界整场作废

### presets 结构（4 个场次）
```json
"presets": [
  { "id": "day6_am", "label": "6号上午", "scenario_file": "6号上午_1.json", "scene_manifest": "metric/6号上午_1号赛道_scenes.json" },
  { "id": "day6_pm", "label": "6号下午", ... },
  { "id": "day7_am", "label": "7号上午", ... },
  { "id": "day7_pm", "label": "7号下午", ... }
]
```
- 当前 active = `day7_pm`
- 比赛是 4 个场次（6上/6下/7上/7下），每个场次对应不同地图 + 不同赛题组合

---

## 2. 各赛题评分指标（metric/*.json）

> 每个 metric 有 `is_critical: true` + `require_all_time_pass: true` → **全程都必须达标，不达标即失败**。

### 2.1 人行道让行（crosswalk）⭐ 赛题2
`6号上午_1号赛道_crosswalk.json`：
```json
{
  "_comment": "人行道让行：停在 Crosswalk_1 前 1.0-1.5m。",
  "metric": [
    { "name": "Collision", "is_critical": true, "require_all_time_pass": true,
      "condition": { "object_overlap_condition": { "target_object_ids": "*", "distance": 0.1, "direction": "INCLUDE_BACK", "use_score": true } } },
    { "name": "CrosswalkStop", "is_critical": true, "require_all_time_pass": true,
      "condition": { "crosswalk_stop_condition": { "min_distance": 1.0, "max_distance": 1.5, "use_score": true, "single_deduction": 20, "target_ids": ["Crosswalk_1"] } } }
  ]
}
```

**🚨 精确要求：停在 Crosswalk_1 前 1.0~1.5m！** 超出区间扣 20 分（single_deduction）。
- **调参目标**：crosswalk `stop_distance` 配成实际停车 1.25m（考虑停车偏移）
- 实车当前 `stop_distance=0.1` → **会停太近（<1.0m）→ 扣分！必须调大到 ~1.25**

### 2.2 定速巡航（cruise）⭐ 赛题0
`6号上午_1号赛道_cruise.json`：
```json
{ "name": "AccelerationLimit", "is_critical": true, "require_all_time_pass": true,
  "condition": { "acceleration_condition": { "max_acceleration": 2.0, "min_acceleration": -3.0, "use_score": true, "single_deduction": 1, "deduction_unit": 1 } } },
{ "name": "CruiseSpeedLimit", "is_critical": true, "require_all_time_pass": true,
  "condition": { "speed_condition": { "max_speed": 3.0556, "min_speed": -0.5, "use_score": true, "single_deduction": 2, "deduction_unit": 1 } } }
```

**🚨 关键**：
- 巡航速度上限 **3.0556 m/s（11 km/h）**，超速每 m/s 每帧扣 2 分
- 加速度约束：**max 2.0 / min -3.0 m/s²**（DKIT 车辆物理限制）
- ⚠️ 全局 `default_cruise_speed` 必须在 [1.5, 5.1]（presets 铁律），但**赛题限速是 3.0556**——所以巡航配置要 ≤3.0556 才安全（实车 5.0 会超速扣分！）

### 2.3 红灯停车（red_light）⭐ 赛题3
`6号下午_1号赛道_red_light.json`：
```json
{ "name": "RedLightStop", "is_critical": true, "require_all_time_pass": true,
  "condition": { "red_light_stop_condition": { "min_distance": 1.0, "max_distance": 1.5, "use_score": true, "single_deduction": 20, "target_ids": ["Signal_1"] } } }
```
**🚨 停在 Signal_1 停止线前 1.0~1.5m**，超出扣 20 分。
- 实车 traffic_light `stop_distance=0.1` → 太近 → 需调大到 ~1.25

### 2.4 区域限速（traffic_road_speed）⭐ 赛题1/7/8
`6号上午_1号赛道_traffic_road_speed.json` + 7号 `crosswalk_road_speed.json`：
```json
{ "name": "RoadSpeedLimit", "is_critical": true, "require_all_time_pass": true,
  "condition": { "speed_condition": { "max_speed": 1000.0, "min_speed": -0.5, "use_score": true,
    "speed_limit_regions": [ { "limit_min_speed": -0.5, "limit_max_speed": 1.0,
      "limit_region": { "point": [ ...多边形坐标... ] } } ] } } }
```
**🚨 关键**：
- 减速带区域（多边形内）限速 **≤1.0 m/s**（不是文档里说的 1.5！）
- 通过该区域速度必须 ≤1.0，否则每帧扣 2 分
- **调参目标**：插件 `speed_bump_limit_speed` 配 **0.8~1.0**（留裕量）
- ⚠️ 限速区域是**精确多边形坐标**（从地图提取），不是 buffer 概念——插件 buffer 需覆盖该区域

### 2.5 其他（scenes 清单）
每个场次的 `scenes.json` 定义赛题组合（map_file / vehicle_param_file / scope）：
```json
{ "boundary_mode": "full", "map_file": "/apollo_workspace/data/map_data/hehai_contest_6_am_3/base_map.bin",
  "vehicle_param_file": "dkit_vehicle_param.pb.txt",
  "scenes": [
    { "id": "crosswalk", "config": "6号上午_1号赛道_crosswalk.json", "scope": "crosswalk", "scope_ids": ["Crosswalk_1"], "scope_forward_distance": 80.0, "scope_past_distance": 8.0 },
    { "id": "cruise", "config": "..._cruise.json", "scope": "full" },
    { "id": "traffic_road_speed", "config": "..._traffic_road_speed.json", "scope": "full" }
  ] }
```

---

## 3. 各场次赛题组合（4 场次差异）

| 场次 | 地图 | 赛题 metric |
|------|------|-----------|
| 6号上午 | hehai_contest_6_am_3 | crosswalk + cruise + traffic_road_speed |
| 6号下午 | hehai_contest_6_pm_3 | red_light + cruise + traffic_road_speed |
| 7号上午 | hehai_contest_7_am_3 | crosswalk_road_speed + red_light + cruise |
| 7号下午 | hehai_contest_7_pm_3 | crosswalk + crosswalk_road_speed + cruise |

> ⚠️ **现场要确认是哪个场次**（active 现在是 day7_pm），从而知道要重点调哪些指标。
> 注意 7号场次有 `crosswalk_road_speed`（人行道 + 限速合并指标）。

---

## 4. DKIT 车辆参数（metric/dkit_vehicle_param.pb.txt）

```text
brand: DKIT
front_edge_to_center: 3.62
back_edge_to_center: 0.87
left_edge_to_center: 1.007
right_edge_to_center: 1.007
length: 4.49   width: 2.014   height: 1.98
min_turn_radius: 5.5
max_acceleration: 2.0
max_deceleration: -2.0
max_steer_angle: 8.37333
max_steer_angle_rate: 400.0
steer_ratio: 16.18
wheel_base: 2.75
wheel_rolling_radius: 0.3341
max_abs_speed_when_stopped: 0.1
brake_deadzone: 0.05
throttle_deadzone: 0.05
```

**解读**：
- 车长 4.49m，车头到中心 3.62m → **停车距离标定要考虑车头位置**
- max_acceleration 2.0 / max_deceleration -2.0 与 cruise 指标一致
- 与本地 planning 的 `max_acceleration=2.0` 匹配

---

## 5. 场景定义（6/7号_1.json）

```json
{ "type": "worldsim", "mapId": "hehai_contest_6_am_3",
  "start": { "x": ..., "y": ... }, "end": { "x": ..., "y": ... },
  "simulator_time": 120,
  "agent": [ { "motiontype": 2, "type": 2, "start_distance": 35, "speed": 1.0, ... }, ... ] }
```

- 场景是 worldsim 仿真（行人/车辆 agents 以固定距离/速度触发）
- 6号上午有多个行人 agent（start_distance 35/25/20/15，speed 1.0/0.6/0.5）→ 人行道让行
- **调参时可对照这些坐标做针对性测试**

---

## 6. 对调参策略的最终修正（⚠️ 覆盖之前的文档）

| 赛题 | 官方要求 | 之前文档建议 | **最终建议（以本文件为准）** |
|------|---------|-------------|---------------------------|
| 全局 | cruise_speed ∈ [1.5,5.1]；upper_limit ∈ [1.5,5.1] | 可调高 | **保持实车 5.0 不动！调超 5.1 整次判 0** |
| 0 定速巡航 | ≤3.0556 m/s | 3.0 | **default_cruise_speed 配 ≤3.0**（5.0 会超速） |
| 1 减速带 | 区域 ≤1.0 m/s | 1.2 | **插件 speed_bump_limit_speed=0.8~1.0** |
| 2 人行道 | 停 Crosswalk_1 前 1.0~1.5m | stop_distance 1.25 | **crosswalk stop_distance ≈1.25**（实车 0.1 太近！） |
| 3 红绿灯 | 停 Signal_1 前 1.0~1.5m | 1.25 | **traffic_light stop_distance ≈1.25**（实车 0.1 太近！） |
| 4 停止牌 | — | 1.25 | 按现场（实车 4.0 可能停太远） |
| 7/8 限速 | 区域 ≤1.0 m/s | 1.2 | **插件 limit_speed=0.8~1.0** |

> 🚨 **cruise_speed_check 是"全场一票否决"**——`planning.conf` 里两个全局参数必须留在 [1.5, 5.1]。这是实车最重要的红线！

---

## 7. 配套文件（已传回本地）

- `shiche/car_890/grading_console/auto_drive_console/` 全量（server.py / runtime.py / cyber_action.py 评分引擎源码，可研究实现）
- 4 个场次场景 json（6/7号上午下午_1.json）
- 17 个 metric json（全部赛题指标）
- `records/score_history.json`（历史成绩，可能为空）
- `presets.json`（全局配置，含 cruise_speed_check 铁律）
