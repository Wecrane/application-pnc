# 11 实车配置实测审计（890 车，2026-08-05）

> **来源**：实车 SSH 登录 + SSHFS 挂载实测（`apollo@192.168.10.6`，工程 `~/apollo-edu_9.1.1`，profile `HQEV_BDN`）。
> **核心结论**：⚠️ **实车是车厂深度定制配置，与本地官方默认差异巨大**——所有调参必须**以实车实测值为基线**，本地文档只作机制参考！
> **连接要点**：USB 手机共享网卡（rndis）会抢占默认路由导致车端 SSH 连不上 → **拔掉 USB 或 `nmcli dev disconnect <usb网卡>`** 即可。

---

## 1. 实车环境确认

| 项 | 值 |
|----|-----|
| 主机名 | `apollo-PURLEY` |
| 用户名 | `apollo` |
| 车机 IP | `192.168.10.6`（热点网段 192.168.10.x） |
| 工程目录 | `~/apollo-edu_9.1.1`（⚠️ **下划线 `_`**，非横杠 `-`） |
| profile 名 | **`HQEV_BDN`**（⚠️ 不是 `default`！） |
| current 软链 | `/apollo_workspace/profiles/HQEV_BDN` |
| 限速插件 | ✅ **已预装**（traffic_rule_config 含 `SCENE_SPEED_LIMIT`，且出现两次 + 有 `SPEED_SETTING`） |
| control 模块 | ✅ 存在（profiles/HQEV_BDN/modules/control） |

**挂载方式**：
```bash
# 先禁用 USB 网卡（否则路由冲突连不上）
nmcli dev disconnect enxba1609a3a1f8   # USB rndis 网卡名（现场用 nmcli dev status 查）
# 挂载（注意下划线工程名 + HQEV_BDN profile）
PROJ_DIR=apollo-edu_9.1.1 bash shiche/car_connect/mount_car.sh
# 挂载点：~/car/profiles、~/car/modules
```

---

## 2. ⚠️ 实车 planning.conf（与本地差异巨大！）

> 文件：`profiles/HQEV_BDN/modules/planning/planning_component/conf/planning.conf`

| 参数 | **实车值** | 本地官方默认 | 备注 |
|------|-----------|------------|------|
| `--planning_upper_speed_limit` | **5.0** | 31.3 | ⚠️ 实车限速极保守 |
| `--default_cruise_speed` | **5.0** | 5.0 | 巡航 5 m/s（18 km/h） |
| `--destination_check_distance` | **20.0** | 4.0 | 提前判定到达终点 |
| `--min_stop_distance_obstacle` | **0.0** | 6.0 | ⚠️ 改成了 0（不设最小停车距离） |
| `--max_stop_distance_obstacle` | **0.0** | 10.0 | ⚠️ 0 |
| `--obstacle_lat_buffer` | 0.4 | 0.4 | 路径边界横向缓冲 |
| `--static_obstacle_nudge_l_buffer` | **0.6** | 0.3 | 静态障碍绕行缓冲加大 |
| `--nonstatic_obstacle_nudge_l_buffer` | **0.15** | 0.4 | 动态障碍绕行缓冲减小 |
| `--yield_distance` | **1.0** | 5.0 | 让行距离大幅减小 |
| `--passed_destination_threshold` | **0.5** | 0.05 | 已过终点阈值增大 |
| `--speed_fallback_distance` | **0.0** | 3.0 | 速度回退距离 0 |
| `--longitudinal_jerk_lower_bound` | **-5** | -4.0 | jerk 放宽 |
| `--longitudinal_jerk_upper_bound` | **5** | 2.0 | jerk 放宽 |
| `--replan_time_threshold` | 7.0 | 7.0 | 重规划时间 |
| `--obstacle_lon_start_buffer` | **0.4** | 3.0 | 纵向起始缓冲减小 |
| `--use_s_curve_speed_smooth` | 开 | 关 | S 曲线平滑 |
| `--enable_parallel_trajectory_smoothing` | true | false | 并行轨迹平滑 |

**解读**：
- 实车把 `min/max_stop_distance_obstacle` 设 0 → 障碍停车距离不再由 6m 兜底（**赛题5 无 6m 覆盖风险**？需现场验证）
- `planning_upper_speed_limit=5.0` → 全局限速 5 m/s，**赛题0 定速巡航若要求 11km/h≈3.06 可直接用，但其它赛题速度也被压到 5 m/s 内**
- 车厂 profile 是"低速园区"调校，跟比赛赛题（最高 11km/h）基本匹配

---

## 3. 实车 TrafficRules 配置（与本地对比）

### 3.1 SceneSpeedLimit（限速插件）— 已预装，全 false
> 文件：`profiles/HQEV_BDN/modules/planning/traffic_rules/scene_speed_limit/conf/default_conf.pb.txt`
```text
crosswalk_enabled: false        crosswalk_limit_speed: 1.0
traffic_light_enabled: false    traffic_light_limit_speed: 1.0
speed_bump_enabled: false       speed_bump_limit_speed: 1.0
junction_enabled: false         junction_limit_speed: 1.0   # ⚠️ 比本地多 junction 区段！
```
> 三组（+junction）全 false，buffer 全 1.0，限速全 1.0。**赛题1/7/8 需置 true + 调值**。
> ⚠️ 实车插件**比本地多一个 `junction_enabled`**（路口区段），现场注意。

### 3.2 Crosswalk（人行道）⭐ 赛题2
> 文件：`.../crosswalk/conf/default_conf.pb.txt`
```text
stop_distance: 0.10            # ⚠️ 0.1m！极近（本地 1.0）
max_stop_deceleration: 4.0
min_pass_s_distance: 1.0
expand_s_distance: 0.0         # ⚠️ 0（本地 2.0）
stop_strict_l_distance: 1.0    # ⚠️ 1.0（本地 4.0/5.0）
stop_loose_l_distance: 1.5     # ⚠️ 1.5（本地 5.0）
start_watch_timer_distance: 10 # ⚠️ 无空格（本地 40.0）
stop_timeout: 3.0              # ⚠️ 3s（本地 10.0）
```
> ⚠️ **实车 crosswalk 停车距离 0.1m**——赛题2 要求 1~1.5m 停车，当前配置会**停得太近**（压线风险），需调大到 1.25 左右。

### 3.3 TrafficLight（红绿灯）⭐ 赛题3/8
```text
enabled: true
stop_distance: 0.10            # ⚠️ 0.1m（本地 1.0）→ 需调大到 1.25
max_stop_deceleration: 4.0
```

### 3.4 StopSign（停止标志）⭐ 赛题4
```text
enabled: true
stop_distance: 4.0             # ⚠️ 4.0m！（本地 1.0）→ 停太远，需调小到 1.25
```

### 3.5 Destination（终点）⭐ 赛题5
```text
stop_distance: 0.2
```

### 3.6 ReferenceLineEnd
```text
stop_distance: 0.5
min_reference_line_remain_length: 5.0   # ⚠️ 5.0（本地 50.0）
```

### 3.7 KeepClear
```text
enable_keep_clear_zone: true
enable_junction: true
min_pass_s_distance: 2.0
align_with_traffic_sign_tolerance: 4.5
```

---

## 4. 实车 Tasks 配置

### 4.1 SpeedDecider（速度决策）⭐ 赛题6
> 文件：`.../tasks/speed_decider/conf/default_conf.pb.txt`
```text
follow_min_obs_lateral_distance: 2.5
follow_min_time_sec: 2.0
is_stop_for_pedestrain: false
keep_clear_last_point_speed: 0.8
yield_distance_buffer: 2.0      # ⚠️ 2.0（本地无此项/5.0）
stop_follow_distance: 1.5       # ⚠️ 1.5（本地 2.0）
follow_distance_scheduler { ... }  # slope 1.25/1.0/0.75/0.5
```

### 4.2 ⚠️ 需要补充实测的（未列全）
- `tasks/rule_based_stop_decider/conf/default_conf.pb.txt`
- `tasks/piecewise_jerk_speed/conf/default_conf.pb.txt`
- `tasks/lane_follow_path/conf/default_conf.pb.txt`
- `tasks/speed_bounds_decider/conf/default_conf.pb.txt`
- `scenarios/traffic_light_protected、stop_sign_unprotected` 等场景配置
- `control/control_component/conf/control.conf` + LQR/PID
> 现场/下次连接时继续拉取补充。

---

## 5. 连接排障经验（本次实战）

### 5.1 USB 手机共享网卡导致连不上车端
- **现象**：插着 USB 手机共享网络 → `ssh 192.168.10.6` 超时
- **根因**：rndis USB 网卡（如 `enxba1609a3a1f8`）抢默认路由（metric 100 < WiFi 600），车端流量走错网关
- **解决**：拔掉 USB 或 `nmcli dev disconnect enxba1609a3a1f8`（等效拔线）

### 5.2 双 AP 同 SSID 陷阱
- 同一 SSID（HQEV890）存在**多个 AP**（50.x / 10.x 不同网段）
- 连到错误的 AP 会拿到 192.168.50.x，访问不到 10.6
- **对策**：连上后用 `ip addr` 确认网段；锁定正确 BSSID（本次正确 AP 网段 = 192.168.10.x）

### 5.3 工程目录名
- 实车是 `apollo-edu_9.1.1`（**下划线**），工具包默认 `apollo-edu-9.1.1`（横杠）是错的
- **必须 `PROJ_DIR=apollo-edu_9.1.1`**

### 5.4 profile 名
- 实车 profile = **`HQEV_BDN`**（车厂定制），不是 `default`
- 改配置 = 改 `profiles/HQEV_BDN/modules/...`；`aem profile use HQEV_BDN` 若需切换

---

## 6. 对 9 赛题的影响（基于实车基线）

| # | 赛题 | 实车现状 | 需调整 |
|---|------|---------|--------|
| 0 | 定速巡航 | `default_cruise_speed=5.0` | 若要求 ≤3.06 需调低到 3.0；上限 5.0 已够 |
| 1 | 减速带 | 插件 speed_bump_enabled=false | 置 true + 限速 1.2 左右 |
| 2 | 人行道停车 | `stop_distance=0.1`（太近！） | **调大到 1.25** + expand_s_distance 4~6 + stop_timeout 调大 |
| 3 | 红绿灯停车 | `stop_distance=0.1`（太近） | **调大到 1.25** |
| 4 | 停止牌停车 | `stop_distance=4.0`（太远） | **调小到 1.25** |
| 5 | 障碍物避让 | min/max_stop_distance=0（无兜底） | 需实测实际停车距离 |
| 6 | 动态跟随 | follow_min_time_sec=2.0 | 可调 1.0 提速；stop_follow_distance=1.5 |
| 7 | 路口减速 | 插件 crosswalk_enabled=false | 置 true + 限速 1.2 |
| 8 | 交通灯减速 | 插件 traffic_light_enabled=false | 置 true + 限速 1.2 |

> ⚠️ **注意**：实车 stop_distance 取值混乱（0.1 / 4.0），可能车厂针对自有场地标定过；比赛现场必须**实测停车距离**再微调。**所有值以现场实测为准。**

---

## 7. 待补充清单（下次连接拉取）

- [ ] rule_based_stop_decider / piecewise_jerk_speed / lane_follow_path / speed_bounds_decider conf
- [ ] 各 scenario conf（traffic_light_protected / stop_sign_unprotected / bare_intersection）
- [ ] control.conf + LQR/PID controller_conf
- [ ] 实车源码版本确认（planning 是官方还是定制、scene_speed_limit 是否本地一致）
- [ ] 是否能用 `aem profile use HQEV_BDN` 生效（实车 profile 机制）
