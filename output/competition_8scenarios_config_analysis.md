# 星火大赛 PnC — 2026 新赛题 8 场景配置分析

> 生成时间：2026-08-01 | 目标：**尽量只改配置文件参数、不改代码**完成赛题
> 约束：本分析只读配置文件与地图数据，未读场景代码

---

## 一、赛题与评分标准速览

| 序号 | 赛题 | 关键要求 | 评分扣分点 |
|------|------|---------|-----------|
| 1 | 减速带场景 | 通过减速带 ≤ 3 m/s | 超速每 1m/s 每帧扣 2 分 |
| 2 | 人行道场景（有人） | 停止在停止线前 **1.5~2.0m**，行人通过后驶离 | 未停 1.5-2.0m 扣 20 分；未避让/超线 0 分 |
| 3 | 红绿灯场景 | 红灯停止在停止线前 **1.5~2.0m**，不超停止线 | 未停 1.5-2.0m 扣 20 分 |
| 4 | 停止标志场景 | 完全停止在停止线前 **1.5~2.0m**，确认安全后通行 | 未停/距离不对扣 20 分；超线 0 分 |
| 5 | 障碍物停车避让 | 静止障碍物前停车，与前车距离 **≥ 2m** | 距离 <2m 扣 20 分；碰撞 0 分；急加减速扣分 |
| 6 | 动态障碍物跟随 | 跟随前车加减速，保持安全距离，避免追尾/急加减速 | 碰撞 0 分；急加减速扣分 |
| 7 | 人行道路口减速通行（无人） | 通过无人人行道 ≤ **5 m/s** | 超速每 1m/s 每帧扣 2 分 |
| 8 | 交通灯路口减速通行 | 绿灯通过信号灯区域 ≤ **5 m/s** | 超速每 1m/s 每帧扣 2 分 |

**统一规律**：
- 停车类（赛题 2/3/4/5）→ 控制 `stop_distance` 相关参数到 1.5~2.0m 区间
- 限速类（赛题 1/7/8）→ 控制对应区域的限速值
- 跟随类（赛题 6）→ 控制跟随距离/时间参数

---

## 二、配置体系（如何改、如何生效）

### 2.1 配置存放位置（两处）

| 位置 | 内容 | 运行期是否读取 |
|------|------|:---:|
| `profiles/default/modules/planning/` | profile 覆盖层（**当前只有 planning.conf**） | ✅ 覆盖源码配置 |
| `modules/planning/<插件>/conf/*.pb.txt` | 各插件源码配置（运行期实际读取） | ✅ 直接读取 |

> 结论：profile 目录是"覆盖层"，源码目录是"默认层"。运行期实际读到的配置 = profile 覆盖后的结果。
> 打包提交时：只改配置 → `tar -zcvf 提交包.tar.gz profiles/default`；改了源码 → 两个目录都打。

### 2.2 配置类型与生效方式

| 类型 | 文件示例 | 改后需编译？ | 生效步骤 |
|------|---------|:---:|---------|
| protobuf 文本 | `default_conf.pb.txt`、`pipeline.pb.txt` | 否 | `aem profile use default` + 重启 Dreamview |
| gflags | `planning.conf` | 否 | `aem profile use default` + 重启 Dreamview |
| C++ 源码 | `planning_gflags.cc` 硬编码 | **是** | `buildtool build -p modules/planning/ -j15` |

### 2.3 配置加载链路

```
PlanningComponent::Init()
  ├── planning.conf (gflags)  ← 全局参数
  ├── traffic_rule_config.pb.txt → TrafficRule 列表（9 个已启用）
  └── public_road_planner_config.pb.txt → Scenario 列表（12 个，LaneFollow 兜底）
        每个插件 Init() → __cxa_demangle → 找到自身 conf/default_conf.pb.txt
```

### 2.4 关键注册文件（`modules/planning/planning_component/conf/`）

| 文件 | 作用 | 当前状态 |
|------|------|---------|
| `planning.conf` | 全局 gflags（限速、减速带等） | `speed_bump_speed_limit=3` 已激活 |
| `traffic_rule_config.pb.txt` | TrafficRule 启用列表 | CROSSWALK/STOP_SIGN/TRAFFIC_LIGHT 等 9 个已启用 |
| `public_road_planner_config.pb.txt` | Scenario 优先级列表 | 12 个场景注册，LANE_FOLLOW 兜底 |

---

## 三、各赛题配置映射

### 赛题 1：减速带 ≤ 3m/s — 已配置 ✅

| 参数 | 文件 | 值 |
|------|------|-----|
| `speed_bump_speed_limit` | `profiles/default/.../planning.conf` | `3` ✅ |
| 地图限速 | `base_map.bin`（27 条车道） | 2.78 m/s（10km/h）✅ |

> 双保险：planning.conf 已设 3m/s，地图减速带车道本身限速 10km/h。

### 赛题 2：人行道（有人）停车 1.5~2.0m ⚠️ 需调整

| 参数 | 文件 | 当前值 | 目标值 |
|------|------|--------|--------|
| `stop_distance` | `traffic_rules/crosswalk/conf/default_conf.pb.txt` | 1.0 | **1.5~2.0 之间（建议 1.75）** |
| `stop_timeout` | 同上 | 10.0 | 行人通过等待上限 |

### 赛题 3：红绿灯停车 1.5~2.0m ⚠️ 需调整

| 参数 | 文件 | 当前值 | 目标值 |
|------|------|--------|--------|
| `stop_distance` | `traffic_rules/traffic_light/conf/default_conf.pb.txt` | 1.0 | **1.5~2.0 之间（建议 1.75）** |

### 赛题 4：停止标志停车 1.5~2.0m ⚠️ 需调整

| 参数 | 文件 | 当前值 | 目标值 |
|------|------|--------|--------|
| `stop_distance` | `traffic_rules/stop_sign/conf/default_conf.pb.txt` | 1.0 | **1.5~2.0 之间（建议 1.75）** |

### 赛题 5：障碍物停车 ≥ 2m — 已达标 ✅

| 参数 | 文件 | 当前值 |
|------|------|--------|
| `stop_follow_distance` | `tasks/speed_decider/conf/default_conf.pb.txt` | 2.0 ✅（正好满足 ≥2m） |
| `static_obstacle_buffer` | `tasks/path_decider/conf/default_conf.pb.txt` | 0.3 |

> 注意：2.0m 是刚好达标，实际停车可能因减速误差略近，需实测后微调（如 2.2~2.5m 留裕量）。

### 赛题 6：动态障碍物跟随 ⚠️ 需调参

| 参数 | 文件 | 当前值 | 说明 |
|------|------|--------|------|
| `follow_min_time_sec` | `tasks/speed_decider/conf/default_conf.pb.txt` | 2.0 | 最小跟随时长 |
| `follow_distance_scheduler` | 同上 | speed→slope 分段 | 不同速度下的目标跟车距离 |
| `follow_min_obs_lateral_distance` | 同上 | 2.5 | 可绕行的最小横向距离 |

### 赛题 7：无人人行道 ≤ 5m/s 🔍 机制待确认

- 地图无 5m/s 限速（仅 10km/h 减速带 + 70km/h 默认）
- crosswalk 规则配置中**无通行限速字段**
- **待确认**：Crosswalk 规则代码中是否有通行限速逻辑（如需看代码才能确定）

### 赛题 8：绿灯路口 ≤ 5m/s 🔍 机制待确认

- 同上，traffic_light 配置中无通行限速字段
- **待确认**：路口通行限速来自何处（TrafficLight 规则 / 场景 / 地图）

---

## 四、下一步行动计划

1. **用户先跑现有场景**，观察各赛题当前表现（哪题已过、哪题扣分）
2. **停车类（赛题 2/3/4）**：把三个 `stop_distance` 从 1.0 → 1.75（或实测后微调）
3. **赛题 5**：实测停车距离，若 <2m 则增大 `stop_follow_distance`
4. **赛题 6**：根据急加减速情况调 `follow_distance_scheduler` 的 slope
5. **赛题 7/8**：确认限速机制后，找到可配置的限速参数（可能需新增 profile 配置或看代码）

## 五、配置生效操作（用户执行）

```bash
# 改完配置后：
aem profile use default          # 重新激活 profile（pb.txt/conf 生效）
aem bootstrap stop && aem bootstrap start --plus   # 重启 Dreamview
# 若改了源码/新增 proto 才需要：
# buildtool build -p modules/planning/ -j15 && aem profile use default
```
