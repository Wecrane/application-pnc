# 15 实车赛最终调参指南（评分规则反推 · 2026-08-05 定稿）

> **🚨 本指南基于实车 890 全部拷贝材料反推**：官方评分控制台（`grading_console/`）+ 评分系统源码（`grading_system_py/`）+ 4 张苏州正式地图（`maps_tools/data/map_data/hehai_contest_*_3`）+ 实车配置基线（HQEV_BDN）+ 苏州 score_history 10 次实战 + 8/3-8/4 苏州赛道实测日志。
> **目标**：只改 `profiles/HQEV_BDN/` 下的 5 个配置文件 + 重启模块，一套配置打满 4 个场次。
> **生效方式（官方确认）**：只改参数 → 改哪个模块就重启哪个模块（Dreamview+ 模块栏开关），**不编译、不用 `aem profile use default`**。
> **铁律**：所有情况以实车为准，现场先 `check_car.sh` 摸底 + 跑一圈实测，再按本指南微调。
> **📁 配置文件模板**：工作区根目录 **`realcar_tuning/`**（`ORIGINAL/`=实车原始配置、`RECOMMENDED/`=推荐参数已应用版，比赛直接上传 RECOMMENDED）。⚠️ 注意与工作区 `modules/planning/traffic_rules/`（官方源码默认值）区分！

---

## 一、评分规则精确解析（从官方评分控制台反推）

### 1.1 🚨 全局铁律（presets.json，一票否决）

```json
"cruise_speed_check": { "enabled": true,
  "conf_file": "/apollo/modules/planning/planning_component/conf/planning.conf",
  "min_default_cruise_speed": 1.5, "max_default_cruise_speed": 5.1,
  "min_planning_upper_speed_limit": 1.5, "max_planning_upper_speed_limit": 5.1,
  "scope": "all" }
```

| 参数 | 合法范围 | 越界后果 |
|------|---------|---------|
| `--default_cruise_speed` | **[1.5, 5.1]** | 整次评测判 **0 分** |
| `--planning_upper_speed_limit` | **[1.5, 5.1]** | 整次评测判 **0 分** |

> ⚠️ 实车当前 5.0/5.0 都在合法范围，**`planning_upper_speed_limit` 永远别动**；`default_cruise_speed` 可调但**必须留在 [1.5,5.1]**。

### 1.2 每个场次的赛题组合（4 场次 = 3 题/场）

| 场次 | 地图 | 停车题（scope 限定） | 巡航题（full） | 减速带限速题（full） |
|------|------|---------------------|---------------|---------------------|
| 6号上午 | `hehai_contest_6_am_3` | Crosswalk_1 停车 [1.0,1.5]m | ≤3.0556 + 加速度[-3,2] | 减速带区域 ≤1.0 |
| 6号下午 | `hehai_contest_6_pm_3` | Signal_1 红灯停车 [1.0,1.5]m | 同上 | 减速带区域 ≤1.0 |
| 7号上午 | `hehai_contest_7_am_3` | Signal_1 红灯停车 [1.0,1.5]m | 同上 | 减速带区域 ≤1.0 |
| 7号下午 | `hehai_contest_7_pm_3` | Crosswalk_5 停车 [1.0,1.5]m | 同上 | 减速带区域 ≤1.0 |

### 1.3 各赛题精确评分指标（metric/*.json 原文）

**① cruise 巡航（4 场次完全相同）**
```json
AccelerationLimit: max_acceleration=2.0, min_acceleration=-3.0, 扣 1 分/单位/帧
CruiseSpeedLimit:   max_speed=3.0556(11km/h), min_speed=-0.5,  扣 2 分/单位/帧
两者均 is_critical + require_all_time_pass（全程达标才 PASS）
```

**② crosswalk 人行道停车（6上午 Crosswalk_1 / 7下午 Crosswalk_5）**
```json
Collision:      distance=0.1, INCLUDE_BACK（碰撞硬失败 100）
CrosswalkStop:  min_distance=1.0, max_distance=1.5, single_deduction=20
```
> 判定源码逻辑（crosswalk_stop_handler.py）：**停在 [1.0,1.5]m 合格**；停近(<1.0)扣 20；压线/越线/移动中太近=硬失败 100；**停远(>1.5)不扣分可继续接近** → 宁可停远不可停近！

**③ red_light 红灯停车（6下午/7上午 Signal_1）**
```json
RedLightStop: min_distance=1.0, max_distance=1.5, single_deduction=20, target=Signal_1
```
> 与 crosswalk 同理：停近扣 20，压线硬失败，停远可补救。

**④ 减速带区域限速（traffic_road_speed / crosswalk_road_speed）**
```json
RoadSpeedLimit: speed_limit_regions: limit_max_speed=1.0, 扣 2 分/单位/帧
全局 max_speed=1000（只限区域内）
```
> ⚠️ **限速区域是精确多边形，且用"车辆包围盒重叠"判定**（车头进入区域即生效）。新版正式图限速 **≤1.0 m/s**（旧版训练场是 ≤1.5，别用错）。

---

## 二、赛道几何（从 4 张 base_map 精确解析）

### 2.1 赛道总览

- **赛道方向**：全程沿 **+y（北向）直线行驶**，x≈272586~272594，无弯道（全部 lane FORWARD/NO_TURN）
- **起点** `(272589.88, 3476108.50)` → **终点** `(272594.16, 3476211.38)`，全程 **~103m**
- **地图车道限速**：全部 **5.5556 m/s（20 km/h）**
- **🚨 speed_bump=0、stop_sign=0**（4 张图确认）→ 减速带/停止牌**不依赖地图元素**，限速插件 `speed_bump`/`stop_sign` 区段**无效**！

### 2.2 主赛道元素精确位置（距起点 Δy，起点 y=3476108.50）

| 场次 | 第 1 个元素（=减速带评分区位置） | 第 2 个元素（=停车题位置） |
|------|--------------------------------|--------------------------|
| 6上午 | **Signal_1**（常绿）停止线 @Δy **24.7m** | **Crosswalk_1** @Δy **61.3m** |
| 6下午 | **Signal_6**（常绿）停止线 @Δy **26.8m** | **Signal_1**（红灯）停止线 @Δy **62.2m** |
| 7上午 | **Crosswalk_1** @Δy **26.0m** | **Signal_1**（红灯）停止线 @Δy **62.2m** |
| 7下午 | **Crosswalk_1** @Δy **26.0m** | **Crosswalk_5** @Δy **60.8m** |

### 2.3 🚨 减速带评分区 = 第 1 个交通元素位置（关键机制发现）

| 场次 | 减速带评分多边形 Δy 范围 | 重合元素 | 含义 |
|------|------------------------|---------|------|
| 6上午 | Δy 24.7 ~ 30.5 | Signal_1 停止线 @24.7 | 减速带区=Signal_1 处 |
| 6下午 | Δy 26.9 ~ 32.6 | Signal_6 停止线 @26.8 | 减速带区=Signal_6 处 |
| 7上午 | Δy 23.0 ~ 28.9 | Crosswalk_1 @26.0 | 减速带区=Crosswalk_1 处 |
| 7下午 | Δy 23.5 ~ 29.1 | Crosswalk_1 @26.0 | 减速带区=Crosswalk_1 处 |

> **💡 决定性结论**：地图无 speed_bump 元素 → 但减速带评分区域**恰好与第一个信号灯/人行道位置重合** → **用限速插件的 `traffic_light` / `crosswalk` 区段即可覆盖减速带限速**！无需 speed_bump 区段。

---

## 三、实车配置基线（HQEV_BDN 当前值 vs 推荐值）

> 配置路径基座：`profiles/HQEV_BDN/modules/planning/planning_component/`（planning.conf 在 `conf/`，规则在 `../traffic_rules/<rule>/conf/`，任务在 `../tasks/<task>/conf/`）

### 3.1 `planning.conf`（全局 gflags）

| 参数 | 实车当前值 | 推荐值 | 依据 |
|------|-----------|--------|------|
| `--planning_upper_speed_limit` | **5.0** | **5.0（别动！）** | 铁律 [1.5,5.1]，动了判 0 |
| `--default_cruise_speed` | **5.0** | **2.8** | 评分 cruise ≤3.0556；5.0 会持续超速；日志实车速度峰值 3.14-3.16（3.0 配置仍可能超调超速），取 2.8 留 0.25 裕量 |
| `--destination_check_distance` | 20.0 | 不动 | — |
| `--min/max_stop_distance_obstacle` | 0.0/0.0 | 现场验证 | 静态障碍 STOP 距离兜底 |

> ⚠️ **default_cruise_speed=2.8 是全场最关键的修改**：地图限速 5.5556 + 配置 5.0 → 车按 ~5.0 巡航，远超 3.0556 上限 → 每帧扣 2 分。降到 2.8 后 cruise 题满分，且停车接近速度更低（减速度更小，防 SKIP/加速度扣分）。

### 3.2 停车类 TrafficRule（评分 [1.0,1.5]m，标定 1:1）

> **停车距离标定（仿真 800 分实测）**：`stop_distance=1.75 → 车头实际停车距离 1.78m`（≈1:1）。实车要求 [1.0,1.5]，取中值 **1.25**（±0.2 控制摆动仍安全）。

| 规则 | 参数 | 实车当前值 | 推荐值 | 依据 |
|------|------|-----------|--------|------|
| crosswalk | `stop_distance` | **0.10** | **1.25** | 🚨 0.1 停太近 → 扣 20 + 急刹 4.9-6.9 双扣分 |
| crosswalk | `expand_s_distance` | **0.0** | **6.0** | 行人更早被识别 → 提前平缓减速（防急刹） |
| crosswalk | `stop_timeout` | **3.0** | **30.0** | 防行人未走完提前放行 |
| crosswalk | `stop_strict_l_distance` | **1.0** | **4.0** | 🆕 回归源码：SKIP 条件=`stop_deceleration≥4.0 且 l>此值`→1.0 时行人 l>1.0 可能被放行（冲斑马线=硬失败 100）；调回官方默认 4.0 |
| crosswalk | `stop_loose_l_distance` | **1.5** | **5.0** | 🆕 回归源码：l≥loose 仅路径交叉才停；1.5 太小（斑马线宽 3.9m），调回官方默认 5.0 确保全宽必停 |
| crosswalk | `max_stop_deceleration` | 4.0 | 4.0（不动） | — |
| traffic_light | `stop_distance` | **0.10** | **1.25** | 红灯停车 [1.0,1.5] |
| traffic_light | `max_stop_deceleration` | 4.0 | 4.0（不动） | — |
| stop_sign | `stop_distance` | **4.0** | **1.25** | 备选（旧版 4 题制 StopSign_1 [0,2]m）；新版正式图无 stop_sign |

### 3.3 限速插件 `scene_speed_limit`（减速带限速核心）

> 文件：`profiles/HQEV_BDN/modules/planning/traffic_rules/scene_speed_limit/conf/default_conf.pb.txt`（实车已预装 + 已在 traffic_rule_config 注册，**直接改配置即可**）
> 实车当前：四区段全 `enabled: false`、limit_speed 全 1.0、buffer 全 1.0

| 区段 | 参数 | 推荐值 | 覆盖的评分区域 |
|------|------|--------|---------------|
| **crosswalk** | `crosswalk_enabled` | **true** | 7上午/7下午减速带（=Crosswalk_1 位置） |
| | `crosswalk_limit_speed` | **0.8** | 限速 ≤1.0 留 0.2 裕量 |
| | `crosswalk_forward_buffer` | **8.0** | 提前 8m 开始限速（3.0→0.8 平缓减速） |
| | `crosswalk_backward_buffer` | **2.0** | 区域后 2m 恢复 |
| **traffic_light** | `traffic_light_enabled` | **true** | 6上午/6下午减速带（=Signal_1/Signal_6 位置） |
| | `traffic_light_limit_speed` | **0.8** | 同上 |
| | `traffic_light_forward_buffer` | **8.0** | 同上 |
| | `traffic_light_backward_buffer` | **2.0** | 同上 |
| speed_bump | `speed_bump_enabled` | false | ⚠️ 地图无 speed_bump，开了无效 |
| junction | `junction_enabled` | false | 保持关闭 |

> **💡 通用配置**：crosswalk + traffic_light 两组**同时开**（都配 0.8/8.0/2.0）→ 4 个场次一网打尽！不需要按场次切换。

### 3.4 跟车/障碍（新版无此赛题，备选）

| 配置 | 参数 | 实车值 | 备选值 | 何时用 |
|------|------|--------|--------|--------|
| speed_decider | `follow_min_time_sec` | 2.0 | 1.0 | 若现场含跟车题，缩短恢复迟滞 |
| speed_decider | `stop_follow_distance` | 1.5 | 1.5（保持） | 跟车距离 |
| planning.conf | `min_stop_distance_obstacle` | 0.0 | 2.5~3.0 | 若现场障碍停车题要求 ≥2m，且实测停太近 |

---

## 四、🚨 推荐参数配置（一套打 4 场次 —— 现场直接粘贴）

### 4.1 修改清单（共 4 个文件，8 处改动）

**文件 1：`planning_component/conf/planning.conf`**
```text
--default_cruise_speed=2.8        # 5.0 → 2.8（评分 cruise ≤3.0556，留 0.25 裕量）
--planning_upper_speed_limit=5.0  # 保持不动！
```

**文件 2：`traffic_rules/crosswalk/conf/default_conf.pb.txt`**
```text
stop_distance: 1.25          # 0.10 → 1.25
stop_strict_l_distance: 4.0  # 1.0 → 4.0（防 SKIP 提前放行）
stop_loose_l_distance: 5.0   # 1.5 → 5.0（斑马线全宽必停）
expand_s_distance: 6.0       # 0.0 → 6.0
stop_timeout: 30.0           # 3.0 → 30.0
max_stop_deceleration: 4.0   # 保持
```

**文件 3：`traffic_rules/traffic_light/conf/default_conf.pb.txt`**
```text
stop_distance: 1.25        # 0.10 → 1.25
max_stop_deceleration: 4.0 # 保持
```

**文件 4：`traffic_rules/scene_speed_limit/conf/default_conf.pb.txt`**
```text
crosswalk_enabled: true
crosswalk_forward_buffer: 8.0
crosswalk_backward_buffer: 2.0
crosswalk_limit_speed: 0.8

traffic_light_enabled: true
traffic_light_forward_buffer: 8.0
traffic_light_backward_buffer: 2.0
traffic_light_limit_speed: 0.8

# speed_bump / junction 两组保持 false（地图无对应元素，开了无效）
```

**备选文件 5：`traffic_rules/stop_sign/conf/default_conf.pb.txt`**（仅现场确认旧版 4 题制时）
```text
stop_distance: 1.25        # 4.0 → 1.25
```

### 4.2 生效操作

```bash
# 只改 Planning 相关（上述 1-5 全是 planning 模块）
# 方式 A（Dreamview+）：模块栏 Planning 关掉再打开
# 方式 B（命令行）：
bash shiche/car_connect/apply_config.sh planning
# 验证生效：
ssh car "cat /apollo/modules/planning/planning_component/conf/planning.conf | grep default_cruise_speed"
ssh car "cat /apollo/modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt"
```

### 4.3 逐场次验证清单（改完跑 4 个场次各确认一次）

| 场次 | 验证点 1（减速带 ≤1.0） | 验证点 2（停车 [1.0,1.5]） | 验证点 3（巡航 ≤3.0556） |
|------|------------------------|---------------------------|-------------------------|
| 6上午 | Signal_1 处车速 ≤1.0 | Crosswalk_1 车头距 1.0-1.5 | 全程 ≤3.0556 |
| 6下午 | Signal_6 处车速 ≤1.0 | Signal_1 红灯车头距 1.0-1.5 | 全程 ≤3.0556 |
| 7上午 | Crosswalk_1 处车速 ≤1.0 | Signal_1 红灯车头距 1.0-1.5 | 全程 ≤3.0556 |
| 7下午 | Crosswalk_1 处车速 ≤1.0 | Crosswalk_5 车头距 1.0-1.5 | 全程 ≤3.0556 |

---

## 五、评分规则反推的核心洞察（为什么这么配）

### 5.1 苏州 score_history 10 次实战 → 丢分规律

| 规律 | 根因 | 本方案解法 |
|------|------|-----------|
| red_light 10/10 满分 | 停车题停对位置即 100 | traffic_light stop_distance=1.25 |
| **road_speed 从不满分（0~63）** | default_cruise_speed=5.0 持续超速 | **降到 3.0** |
| crosswalk 一半 100 一半 0 | stop_distance=0.1 停太近 + 急刹 | stop_distance=1.25 + expand=6 |
| stop_sign 一半 100 一半 0 | stop_distance=4.0 停太远 | 1.25（备选） |

### 5.2 急刹双扣分机制（日志实证）

- 日志实测：crosswalk 急刹 `stop_deceleration 4.9~6.9 m/s²`（stop_distance=0.1 导致）
- 评分加速度下限 **-3.0**（cruise 题）→ 急刹超 -3.0 每帧扣 1 分
- **本方案**：stop_distance 0.1→1.25 + expand_s_distance 0→6（行人提前触发）→ 车提前平缓减速，减速度 <2 m/s²，加速度题满分

### 5.3 减速带区域 = 交通元素位置（地图解析实证）

- 4 张图 `speed_bump=0`，但减速带评分多边形（Δy 23-32）**精确覆盖**第 1 个信号灯/人行道
- 插件 AddSpeedLimit 基于**地图 overlap**（不依赖感知）→ 100% 稳定触发
- 限速 0.8 < 评分 1.0 留 0.2 裕量；forward_buffer=8m 保证进入区域前已降速

### 5.4 停车距离标定（仿真 800 分实证）

- `stop_distance=1.75 → 车头实际停车 1.78m`（≈1:1）
- 实车要求 [1.0,1.5] → 取 **1.25**（中值，±0.2 摆动安全）
- 停车距离偏大（>1.5）不扣分可继续接近；偏小（<1.0）直接扣 20 → **宁远勿近**

---

## 六、现场 30 分钟操作流程（照着做）

| 时间 | 动作 | 命令/操作 |
|------|------|----------|
| 0-3min | 连接+摸底 | `connect_car.sh` → `check_car.sh` → 确认 HQEV_BDN profile + 插件已装 |
| 3-6min | 备份当前配置 | `backup_car.sh`（出问题 30s 回滚） |
| 6-10min | **改 4 个文件**（§4.1） | 直接粘贴推荐值 |
| 10-12min | 重启 Planning | Dreamview+ 模块栏关开 Planning |
| 12-15min | 验证生效 | `ssh car cat .../planning.conf` 确认 3.0/1.25/0.8 |
| 15-25min | **逐场次实测**（优先 7下午=人行道+减速带） | 4 场次各跑一遍，看评分 + 车行为 |
| 25-28min | 按实测微调 | 停车距离偏差 → 微调 stop_distance ±0.1；减速带超速 → limit_speed 降到 0.7 |
| 28-30min | 固化 + 收尾 | 记录改动、确认最优成绩 |

### 现场快速微调决策树

```
停车距离 < 1.0 → stop_distance 调大 0.1（1.25→1.35）
停车距离 > 1.5 → stop_distance 调小 0.1（1.25→1.15）
减速带超速   → limit_speed 0.8→0.7；或 forward_buffer 8→10
巡航超速     → default_cruise_speed 3.0→2.8
行人走完不走 → stop_timeout 30→60；expand_s_distance 6→8
起步急加速   → 保持 3.0 巡航（已留裕量）
```

---

## 七、失效排查表（改了没效果）

| 现象 | 可能原因 | 处理 |
|------|---------|------|
| 改配置没生效 | 改错文件 / 没重启 / 软链没指向 | 重启 Planning；`ssh car cat` 验证 |
| 停车距离还是 0.1 的效果 | crosswalk 规则没触发（被 6m 障碍 STOP 覆盖） | 查 STOP reason_code（104=CROSSWALK 说明生效；1=HEAD_VEHICLE 说明被障碍 STOP 覆盖） |
| 减速带区域超速 | 插件没生效 / buffer 太小 | 确认 `SCENE_SPEED_LIMIT` 在 traffic_rule_config；limit 0.8→0.7；buffer 8→10 |
| 巡航超速 | default_cruise_speed 没改到 / 地图限速 5.5556 | 确认 planning.conf 生效值 |
| 红灯/人行道不停 | 信号灯被感知为静态障碍（UNKNOWN_UNMOVABLE） | 确认场景注入后感知为真实信号灯；否则查 STOP reason |
| 车停不住冲过 | 接近速度高 + 检测晚 | expand_s_distance 调大；巡航保持 3.0 勿高 |

---

## 八、🚨 现场必须确认的风险点（赛前/开跑前）

1. **场景注入**：比赛时行人/信号灯是否经 scenario_injection 注入（`--enable_scenario_injection=true`）？注入后信号灯是真实信号灯还是静态障碍？→ 决定红灯题能否触发
2. **减速带题期望行为**：是"限速 ≤1.0 通过"还是"停车再走"？→ 决定用插件限速还是别的机制
3. **场次**：现场用哪个场次（presets active=day7_pm）？4 场次都要跑还是抽 1 个？
4. **评分版本**：新版 3 题制（hehai_contest 正式图，减速带 ≤1.0，无 stop_sign）还是旧版 4 题制（训练场，减速带 ≤1.5，含 stop_sign）？
5. **感知状态**：8/4 日志显示人行道/信号灯被感知为 `UNKNOWN_UNMOVABLE` 静态障碍（测试未注入时）——比赛注入后需确认
6. **`min/max_stop_distance_obstacle=0`**：若障碍停车题需要 ≥2m 停车距离，0 可能停太近 → 现场实测后调 2.5-3.0

---

## 九、参数速查卡（现场一页纸）

| 文件 | 参数 | 值 |
|------|------|-----|
| planning.conf | `default_cruise_speed` | **2.8** |
| planning.conf | `planning_upper_speed_limit` | **5.0（别动）** |
| crosswalk conf | `stop_distance` | **1.25** |
| crosswalk conf | `stop_strict_l_distance` | **4.0** |
| crosswalk conf | `stop_loose_l_distance` | **5.0** |
| crosswalk conf | `expand_s_distance` | **6.0** |
| crosswalk conf | `stop_timeout` | **30.0** |
| traffic_light conf | `stop_distance` | **1.25** |
| scene_speed_limit conf | `crosswalk_enabled` / `limit_speed` / `forward_buffer` | **true / 0.8 / 8.0** |
| scene_speed_limit conf | `traffic_light_enabled` / `limit_speed` / `forward_buffer` | **true / 0.8 / 8.0** |
| stop_sign conf（备选） | `stop_distance` | 1.25 |

**配置生效**：只改 Planning 相关 → 重启 Planning 模块（Dreamview+ 关开），不编译。
**回滚**：`backup_car.sh` 备份的配置可直接还原。
