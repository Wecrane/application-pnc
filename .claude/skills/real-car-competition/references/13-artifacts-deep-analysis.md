# 13 拷贝产物深度分析（评分引擎源码 + 苏州实战数据 + 赛道几何，2026-08-05）

> **来源**：实车 890 全套拷贝产物 `shiche/car_890/`（9 个 tar.gz，~500MB）全量解压分析。
> **本文档是 11（配置审计）+ 12（评分细则）的源码级补充**——回答"评分到底怎么算""历史上哪里丢分""赛道障碍物/信号灯在哪儿"。
> 🚨 **本文件重点**：①评分引擎 handler 判定源码逻辑 ②苏州赛区 10 次真实评分记录 ③4 场次赛道精确几何 ④减速带实现机制。

---

## 1. 产物全景（shiche/car_890/）

| 目录 | 大小 | 内容 | 分析结论 |
|------|------|------|---------|
| `grading_console/` | 296K | 官方评分控制台 auto_drive_console（presets + 17 metric + 4 场次场景 + runtime.py 评分引擎） | ⭐ 评分规则一手资料 |
| `xh_bak/` | 115M | 官方备份 /media/.XH_2026_bak（**grading_system_py 完整版 31 py** + 场景注入 + HQEV_BDN + 4 地图 + 2 次备份 run.log） | ⭐ 评分引擎源码 |
| `maps_tools/` | 41M | 4 张比赛地图 + grading_system_py（与 xh_bak 完全相同）+ scenario_injection + **suzhou.tar.gz（苏州赛区包）** + bdn_800.tar.gz | ⭐ 地图 + 苏州数据 |
| `extracted/` | 50M | profiles(HQEV_BDN+sample) + 容器 planning/control 编译产物 + scene_speed_limit 插件源码 + vehicle_conf | ⭐ 配置基线 |
| `scripts_biglogs/` | 673M | 官方脚本（apollo_backup_restore.sh / kill_all / launch_all）+ 8/4 大日志 | 运维流程 |
| `selected_logs/` | 1.5G | 8/3+8/4 精选 planning/control 日志 | 最后分析 |
| `replay_logs/` | 170M | 8/3 真实驾驶回放 record | 最后分析 |
| `car_more_logs/` | (打包) | 8/3 剩余大日志 | 最后分析 |

> ⚠️ **重要**：`maps_tools/grading_system_py` 与 `xh_bak/.../grading_system_py` **完全一致**（diff 无差异），只需分析一份。

---

## 2. 评分引擎源码深度解析（grading_system_py，31 py）

> 路径：`xh_bak/.XH_2026_bak/grading_system_py/`，核心 `main.py`（2062 行）+ `condition/`（12 个 handler）。

### 2.1 总体评分流程
```
bag 文件 → run_multi_scene_bag_mode → 每个 SceneSession 独立扫录包
  → 每 0.1s 一帧 feed_frame → 各 metric 的 handler evaluate
  → 场景分 = 100 - Σ(各 metric 扣分)，clamp [0,100]
  → 多场景总分 = 有数据场景的场景分平均（map_unavailable 计 0 分拉低平均）
```

### 2.2 扣分机制（_scene_deduction_score）
- 每个 metric 从 100 起扣，`delta_score` 累加，场景分 = `max(0, 100 - Σ扣分)`
- **Metric PASS 判定**：`require_all_time_pass=true` → 任何一帧 FAIL 则该 metric FAIL
- **硬失败**（delta_score=100，直接清空该场景分）：压线/越线/碰撞/移动时进入禁停区
- **软扣分**（delta_score=20 或按单位扣）：停太近 / 超速

### 2.3 CrosswalkStop 判定源码逻辑（crosswalk_stop_handler.py）
只在**有行人/非机动车占用人行横道**时检查（`active_crosswalks` 非空），无人占用时全部放行：
1. **车压入人行横道**（car_polygon 与 crosswalk 重叠）→ `crosswalk_entered_occupied` 硬失败 100 分
2. **移动中从 stopline 前跨到后** → `crosswalk_crossed_occupied` 硬失败
3. **移动中且 0 ≤ 车头距 < min_distance(1.0)** → `crosswalk_moving_too_close` 硬失败
4. **停稳后**（speed ≤ 0.1）且车头距在 [min, max] = **[1.0, 1.5]** → 合格停车 `_qualified_stop=true`
5. **停稳后车头距 < 1.0** → `crosswalk_stopped_too_close` **软扣 20 分**
6. 停稳后车头距 > 1.5 → 不算停车点，继续接近（**不扣分**）
7. 已合格停车后**在行人清空前移动** → `crosswalk_moved_before_clear` 硬失败

> 💡 **战术要点**：停车距离越接近 [1.0, 1.5] 中点越安全；**停远不扣分（可继续微调），停近直接扣 20**——宁可停远一点！
> 💡 车头位置 = 后轴中心 + `front_edge_to_center(3.62m)` × heading 方向。

### 2.4 RedLightStop 判定源码逻辑（red_light_stop_handler.py）
只在感知到目标信号灯 **RED** 时检查，绿灯/黄灯放行：
1. 主车 `current_signal` 为 RIGHT/EMERGENCY → 跳过
2. **车压停止线** → `red_overlap_stopline` 硬失败 100
3. **红灯期间越线** → `red_crossed_stopline` 硬失败 100
4. **移动中进入 0 ≤ 距 < 1.0** → `red_moving_too_close` 硬失败
5. **停稳后距在 [1.0, 1.5]** → 合格
6. **停稳后距 < 1.0** → `red_stopped_too_close` **软扣 20 分**
7. 停稳后距 > 1.5 → 继续接近（不扣分）

> 💡 与 crosswalk 同理：**停远可补救，停近扣 20**。

### 2.5 SpeedCondition（限速/巡航）源码逻辑（speed_handler.py）
- 全局模式：`speed ∈ [min_speed, max_speed]`，超速扣 `score_deducted_by_unit(over, unit, single)` 
  - cruise：max 3.0556，超 1m/s 扣 2 分/帧
- **限速区域模式**：车辆包围盒（car_polygon）与多边形重叠时，用区域限速 [-0.5, 1.0] 替代
  - ⚠️ **是"包围盒重叠"判定**——车头进入区域即生效，不是车中心！
- `name == "speed_limit"` 时取 min(地图限速, 配置限速)

### 2.6 ObjectOverlap（碰撞）源码逻辑
- 主车多边形与目标障碍物距离 ≤ 0.1m → 碰撞 `delta_score=100` 硬失败
- `direction: INCLUDE_BACK` = 所有方向都算碰撞（含后方）

### 2.7 评测输出格式（runtime.py 解析）
```
Scene: <name> [t0, t1]
Scenario: PASS/FAIL/NO_DATA
Scene Score: 63.0
Metric ... Result ... Score ... Fail/Total
  Detail: first_fail t=12.3s code=crosswalk_stopped_too_close ...
  Reasons: code=count
  Events: code[start-end]s
```
> 日志分析时可复用 `runtime.py::_parse_evaluation` 的正则解析这些字段。

---

## 3. 🚨 苏州赛区真实评分数据（score_history.json，10 次实测）

> 来源：`maps_tools/suzhou.tar.gz`（苏州赛区完整包，2026-07-28 运行）→ `/tmp/suzhou_x/auto_drive_console/records/score_history.json`
> ⚠️ 这是**官方评分系统在苏州赛区的真实运行记录**——证明评测体系真实有效，且揭示了丢分规律！

| 时间 | 总分 | road_speed | crosswalk | red_light | stop_sign |
|------|------|-----------|-----------|-----------|-----------|
| 07-28 15:03 | **90.8** | 63.0 | 100 | 100 | 100 |
| 07-28 15:14 | 54.2 | 17.0 | 100 | 100 | 0 |
| 07-28 15:18 | 86.2 | 45.0 | 100 | 100 | 100 |
| 07-28 16:19 | 25.0 | 0 | 0 | 100 | 0 |
| 07-28 16:29 | 61.0 | 44.0 | 0 | 100 | 100 |
| 07-28 17:48 | 33.3 | 0 | 0 | 100 | 0 |
| 07-28 17:57 | 27.8 | 11.0 | 0 | 100 | 0 |
| 07-28 18:11 | 58.5 | 34.0 | 100 | 100 | 0 |
| 07-28 18:17 | 57.2 | 29.0 | 100 | 100 | 0 |
| 07-28 18:21 | 59.5 | 38.0 | 100 | 100 | 0 |

### 📊 统计规律（决定性洞察）
1. **red_light 10/10 全满分** —— 红灯停车题最容易，只要停对位置就 100
2. **road_speed（限速题）从不满分**（0~63）—— ⚠️ **超速扣分是最大丢分源**！11km/h 限速极易超
3. **crosswalk 一半 100 一半 0** —— 不稳定，行人占用时停车时机/距离把握是关键
4. **stop_sign 一半 100 一半 0** —— 停止牌题（北京无此题）同样不稳
5. **最高分 90.8**（road_speed 63 拖后腿），**说明想拿 100 必须 road_speed 不丢分**

### 💡 苏州 → 北京映射
- 苏州 `road_speed`（3.056 限速 + 加速度 [-2,2]）≈ 北京 `cruise`（3.0556 + 加速度 [-3,2]）
- 苏州 `crosswalk_pedestrian`（Crosswalk_5，1.0-1.5m）≈ 北京 crosswalk
- 苏州 `red_light`（Signal_1，1.0-1.5m）≈ 北京 red_light
- 苏州 `stop_sign`（0.0-2.0m）→ 北京无
- **北京多一个"减速带区域限速"题（traffic_road_speed / crosswalk_road_speed）**，全 4 场次都有

---

## 4. 4 场次赛道精确几何（地图 base_map + 场景 json 交叉验证）

> 坐标体系：所有坐标同一 ENU 系（x≈272590, y≈3476100-3476290），车从 start 向北（y+）行驶约 103m 到 end。
> 起点 (272589.88, 3476108.50)，终点 (272594.16, 3476211.38)。

### 4.1 关键要素位置（距 start 的纵向距离）

| 元素 | 6上午 | 6下午 | 7上午 | 7下午 |
|------|-------|-------|-------|-------|
| **减速带区域**（评分多边形） | y≈3476136 **~28m** | y≈3476138 **~30m** | y≈3476134 **~26m** | y≈3476135 **~26m** |
| Crosswalk_1 | y=3476169.8 **~61m** | — | y=3476134.5 **~26m** | y=3476134.5 **~26m** |
| Crosswalk_5 | — | — | — | y=3476169.3 **~61m** |
| Signal_1（红灯题） | (常绿) y=3476135 | y=3476171.6 **~63m** | y=3476171.6 **~63m** | — |
| Signal_6（常绿） | — | y=3476136 **~28m** | — | — |
| 行人 agent | y≈3476169（Crosswalk_1 处） | 无 | 无 | y≈3476169（Crosswalk_5 处） |
| 车道数 / 路口 | 33 车道 / 4 路口 | 同 | 同 | 同（8 个 Crosswalk） |

> 计算方法：沿 y 轴距离（车头向北，Δy 即纵向 s）。

### 4.2 6号上午场景（人行道题）agent 明细
5 个行人 agent（type=2 行人，motiontype=2），全在 Crosswalk_1 (y≈3476169) 处横穿：

| id | start_distance(m) | 速度(m/s) | heading |
|----|------|------|---------|
| 5844 | 35 | 1.0 | 3.14（向西） |
| 6840 | 25 | 0.6 | 0.07（向东） |
| 6841 | 20 | 0.5 | 3.14 |
| 6844 | 15 | 0.5 | 3.14 |
| 6845 | 15 | 0.5 | 0.01（向东） |

> 💡 行人有双向横穿（东/西），速度 0.5~1.0 m/s。7下午 agent 与 6上午完全相同。

### 4.3 红灯时序（6下午 / 7上午 Signal_1）
```
state_group: 绿(3) 保持 3s → 红(1) 保持 25s → 绿(3) 保持 100s
```
> 车从 start 出发约 63m 到 Signal_1 停止线。若巡航 3.0m/s，约 21s 到达——**正好赶上红灯区间**（出发 3s 后变红，红 25s 覆盖到达时间）。⚠️ 需确认出发时机与灯序同步。

### 4.4 减速带位置与元素重叠关系（⚠️ 关键机制发现）
| 场次 | 减速带评分区域 y | 重叠地图元素 | 含义 |
|------|----------------|-------------|------|
| 6上午 | 3476133-3476139 | Signal_1（常绿） | 减速带区域内有信号灯 |
| 6下午 | 3476135-3476141 | Signal_6（常绿） | 同上 |
| 7上午 | ~3476134 | **Crosswalk_1** | 减速带 = Crosswalk_1 位置 |
| 7下午 | ~3476135 | **Crosswalk_1** | 同上 |

> 🚨 **地图里没有 speed_bump 元素**（4 张图 speed_bump: 0）！→ **限速插件的 `speed_bump_enabled` 对比赛地图无效**（没有 speed_bump overlap 可加限速）。
> 减速带限速只能靠：①crosswalk 规则（7上/7下 Crosswalk_1 在减速带位置，但 crosswalk 触发的是停车不是限速 1.0 通过）②**人为控制车速**③其他机制。
> ⚠️ **需现场确认减速带题官方期望行为**：是"限速通过"还是"停车再走"。

---

## 5. 容器默认 vs profile 覆盖（"预编译+配置覆盖"完整证据）

| 文件 | 容器默认（/apollo/modules/planning/conf/） | profile HQEV_BDN（生效） |
|------|------------------------------------------|--------------------------|
| `planning_upper_speed_limit` | **20.00** | **5.0** |
| `default_cruise_speed` | **11.18** | **5.0** |
| `use_st_drivable_boundary` | true | false |
| 其他 | 2021 年官方原始 | 车厂低速园区调校 |

> **结论**：实车 = 官方预编译二进制（2021）+ profile 配置覆盖（HQEV_BDN）。**只改配置即可生效，无需编译**（与"改哪个重启哪个"一致）。
> sample profile（实车自带样例）：`planning_upper_speed_limit=2.0`、`default_cruise_speed=1.5`、`speed_bump_speed_limit=0.5`、`min_stop_distance_obstacle=1.5` —— 更接近默认文档值。

---

## 6. 实车评测机制（msg_adapter 场景注入，感知侧）

- 实车在 **vehicle_test（纯激光）** 模式下运行，`multi_sensor_fusion` 不启动，`/apollo/perception/obstacles` 由改造后的 `msg_adapter` 发布
- **改造后的 msg_adapter**（源码在 suzhou.tar.gz）订阅注入话题 `/apollo/perception/scenario_obstacles`，把场景障碍物合并进真实感知输出
- `scenario_injector.py`（416 行）从 worldsim json 读取障碍物/信号灯，以 `sim_obstacle` 注入
- 前置条件：`buildtool build -p modules/perception/msg_adapter` + HQEV profile flag：
  ```
  --enable_scenario_injection=true
  --scenario_obstacle_topic=/apollo/perception/scenario_obstacles
  --scenario_injection_timeout_sec=0.5
  --scenario_injection_id_offset=100000
  ```

> 💡 **对 PnC 的意义**：比赛可能是实车+场景注入回灌评测，Planning 收到的障碍物/信号灯与仿真完全一致 → **仿真调参结果可直接迁移到实车**（前提：感知能识别注入障碍物）。

---

## 7. 官方运维脚本（apollo_backup_restore.sh）

- 备份/恢复 5 个关键目录：`auto_drive_console / scenario_injection / grading_system_py / modules / HQEV_BDN`
- 备份到 `/media/.XH_2026_bak/<名称>_<时间戳>/`，流程：kill_all → 打包 → aem profile use sample → 删除 → 恢复 → `buildtool build` → `aem profile use HQEV_BDN` → `aem bootstrap start --plus`
- `launch_all.launch`：9 模块启动顺序（static_transform → gnss → localization → lidar → radar → camera → lidar_perception → storytelling → prediction → ...）

---

## 8. 综合调参结论（⚠️ 覆盖/强化 11/12 的建议）

| 赛题 | 官方要求 | 实车现状 | 调参动作 |
|------|---------|---------|---------|
| 全局铁律 | cruise_speed/upper_limit ∈ [1.5,5.1] | 5.0/5.0 ✅ | **别动** |
| 定速巡航 | ≤3.0556 m/s | default_cruise_speed=5.0 | ⚠️ **5.0 会超速扣分**（苏州 road_speed 从不满分的根因）→ 需降到 ≤3.0 |
| 减速带 | 区域 ≤1.0 m/s | 插件全 false | 地图无 speed_bump，需现场确认机制 |
| 人行道 | 停 [1.0, 1.5]m | stop_distance=0.10 | **调 ~1.25**（0.1 停太近扣 20） |
| 红绿灯 | 停 [1.0, 1.5]m | stop_distance=0.10 | **调 ~1.25** |
| 加速度 | cruise: [-3,2] | longitudinal_acc=[-2,2] | 停车减速注意 -2 下限（acceleration 扣 1 分/m/s²） |

### 🎯 最大机会点（苏州数据支撑）
1. **保住 red_light/crosswalk 满分**：把 stop_distance 调到 1.25，停车题稳拿 100
2. **攻克限速题**：default_cruise_speed ≤3.0 + 减速带区域限速 —— 这是北京赛唯一"从不满分"的题，也是拉开差距的关键
3. **加速度注意**：急刹（<-3）会扣分，用 planning 的纵向加速度边界 [-2,2] 已经安全

---

## 8.5 🚨 实车日志全量分析（8/3 + 8/4，10 个 planning 日志）

> ⚠️ **重要**：全部 10 个日志都在 **hehai（海驾）赛道**（start x=272589 y=3476107）——**不是河海/苏州赛道**！
> 这辆车 8/3-8/4 用海驾比赛图测试（北京赛道训练）。**行为机制通用，但赛道不同**，河海赛道日志需补拷 hohai_contest_1 后现场测。

### 8.5.1 全部 planning 日志批量分析

| 日志 | 时段 | 地图 | 轨迹Δy | 速度(min-max/avg) | 停车原因 | 关键事件 |
|------|------|------|--------|-------------------|---------|---------|
| 8/3 15:50 | 15:50-16:02 | hehai | 107m | 0.91-1.91 / 1.63 | DEST 2840 | 🚨 Crosswalk_1 急刹 15 次 dec[4.9,6.9]；Signal_1 绿灯通过（TRAFFIC_LIGHT_PROTECTED） |
| 8/3 16:29 | 16:29-17:51 | hehai | 181m | 0.91-3.14 / 1.60 | DEST 9882 | PARK_AND_GO 场景（等起步命令）；Signal_3+Signal_1 |
| 8/4 10:05 | 10:05-10:40 | hehai | 103m | 0.90-1.88 / 1.58 | 无 | 纯巡航（35 分钟） |
| 8/4 10:56 | 10:56-11:20 | hehai | 100m | 0.92-1.77 / 1.58 | DEST 3598 | 巡航+Signal_1 |
| 8/4 14:09 | 14:09-14:20 | hehai | 103m | 0.91-1.93 / 1.57 | DEST 3704 | Signal_1 提及 2646 |
| 8/4 16:48 | 16:48-17:03 | hehai | 100m | 0.92-1.76 / 1.56 | DEST 7176 | 反复启停 |
| 8/4 18:31 | 18:31-18:49 | hehai | 105m | 1.03-3.15 / 1.63 | DEST 2290 | Signal_1 提及 3432 |
| 8/4 19:05 | 19:05-19:25 | hehai | 107m | 1.04-3.14 / 1.62 | DEST 4148 | Signal_1 提及 **11402** + Signal_6 |
| 8/4 19:28 | 19:28-19:36 | hehai | 100m | 1.03-1.81 / 1.55 | DEST 2270 | 巡航 |
| 8/4 19:36 | 19:36-19:44 | hehai | 106m | 1.03-3.16 / 1.62 | DEST 2068 | 巡航 |

### 8.5.2 🚨 关键机制发现（实车行为根因）

1. **信号灯/人行道被感知为 `UNKNOWN_UNMOVABLE` 静态障碍**（nudge_calculation.cc）
   - `build reference line st boundary. id:TL_Signal_1` + `NudgeCalculation: check obstacle id [TL_Signal_1], type [UNKNOWN_UNMOVABLE]`
   - ⚠️ **实车在 vehicle_test 模式下，信号灯/人行道被当成静态障碍**，不是走信号灯规则！比赛红灯题需确认场景注入后感知状态是否变为真实信号灯
2. **🚨 Crosswalk 规则触发时急刹**（8/3 15:50，crosswalk.cc:392）：`stop_deceleration 4.9→6.9 m/s²`
   - 根因：`stop_distance=0.1` 停太近 + 接近速度高 → 需要的减速度远超苏州评分下限 **-2.0** → 双重扣分
3. **SceneSpeedLimit 插件已加载但未启用**：加载时 `plugin have not been loaded` 警告（配置全 false）
4. **停车原因 100% STOP_REASON_DESTINATION**：全部日志只有终点停车，无其他停车场景（测试未注入障碍/红灯）

### 8.5.3 📊 苏州（河海）备赛启示

| 发现 | 对苏州影响 |
|------|-----------|
| 巡航 ~1.5-1.6 m/s | ✅ 苏州 road_speed ≤3.056 完全安全，超速风险低 |
| crosswalk 急刹 4.9-6.9 | 🚨 **苏州加速度下限 -2.0（比北京 -3.0 更严）→ 急刹扣分更狠**！stop_distance 必须调大 |
| 信号灯感知为静态障碍 | ⚠️ 苏州红灯题（Signal_1）需确认注入后感知为真实信号灯，否则不触发红灯规则 |
| SceneSpeedLimit 未启用 | ⚠️ 苏州减速带限速 ≤1.5 需靠插件或手动限速（地图无 speed_bump 时插件无效） |
| control 用 LatPlusController(LQR) | control.conf 已确认，无需动 |

> 💡 8/3 record（13M）是完整真实驾驶回放，可在容器内 `cyber_recorder play` 复现；但它是海驾赛道，非河海。

---

## 9. 配套产物速查

- 评分引擎源码：`shiche/car_890/xh_bak/.XH_2026_bak/grading_system_py/`（main.py + condition/12 handlers + tests）
- 评分控制台：`shiche/car_890/grading_console/auto_drive_console/`
- 苏州包：`shiche/car_890/maps_tools/suzhou.tar.gz`（含 score_history.json + 改造 msg_adapter 源码）
- 地图：`shiche/car_890/maps_tools/data/map_data/hehai_contest_{6_am,6_pm,7_am,7_pm}_3/`
- 场景注入：`shiche/car_890/xh_bak/.XH_2026_bak/scenario_injection/`
- 官方备份脚本：`shiche/car_890/scripts_biglogs/apollo_backup_restore.sh`
