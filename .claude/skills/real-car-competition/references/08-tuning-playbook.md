# 08 调参手册（详细版）—— 调哪些参数、怎么调、怎么验证

> 这是实车赛**调参的最终定稿**。目标：只改 `profiles/` 配置，用**固定参数方案**完成 9 赛题。
> 配套：`references/06-official-config-solutions.md`（官方环境 9 赛题具体取值）、`references/05-tuning-methodology.md`（方法论原理）、`references/10-quick-tuning-guide.md`（全参数表）。
> 🚨 **铁律：所有情况以实车为准！** 本文取值基于仿真+官方资料推断，实车代码/插件/配置/评分可能不同，现场以实车实测 + 现场手册为准。
> ✅ **生效方式（官方确认）**：只改参数 → 改哪个模块就重启哪个模块（`apply_config.sh planning/control`），**不用编译、不用 `aem profile use default`**！

---

## 一、调参铁律（先记住这 6 条）

1. **一次只改一个参数**——多参数同时改无法归因，出问题不知道是谁的锅
2. **留裕量**——限速值 = 赛题上限 − 0.1~0.3；停车距离取区间中值（控制有超调/滑行）
3. **改配置不编译**——pb.txt/conf 改完**重启对应模块**就生效（不用 use default）；只有改 C++ 才编译
4. **先诊断后调参**——先跑一遍看实际行为（停车距离多少、速度多少），再决定调哪个
5. **只改 `profiles/`，非必要不动 `modules/`**——动源码 = 要编译 = 耗时 + 风险
6. **每改必记**——记录参数原值和新值，出问题 30 秒回滚（backup_car.sh）

---

## 二、参数速查表（调哪些参数，一页看完）

| 参数 | 文件 | 作用 | 调大效果 | 调小效果 |
|------|------|------|---------|---------|
| `stop_distance` | crosswalk / traffic_light / stop_sign 各自 conf | 停止线前停车距离 | 停得更远 | 停得更近 |
| `expand_s_distance` | crosswalk conf | 人行道检测扩展区 | 行人更早/更久被识别（等走完） | 行人更早放行 |
| `stop_timeout` | crosswalk conf | 停车等待上限 | 等更久（防提前放行） | 更快放行（省时间） |
| `speed_bump_speed_limit` | planning.conf (gflag) | 减速带限速（±1m 窗口） | 过带更快（贴边） | 更慢更安全 |
| `speed_bump_limit_speed` | scene_speed_limit conf (插件) | 减速带限速（可调缓冲区） | 过带更快 | 更慢更安全 |
| `crosswalk_limit_speed` | scene_speed_limit conf (插件) | 人行道限速 | 通过更快 | 更慢更安全 |
| `traffic_light_limit_speed` | scene_speed_limit conf (插件) | 信号灯区限速 | 通过更快 | 更慢更安全 |
| `*_forward_buffer` | scene_speed_limit conf (插件) | 限速区前缓冲（提前多少开始限速） | 更早减速 | 更晚减速（贴边） |
| `follow_min_time_sec` | speed_decider conf | 跟车判定时间阈值 | 更难判 FOLLOW（恢复慢） | 更快判 FOLLOW（恢复快） |
| `stop_follow_distance` | speed_decider conf | 跟车距离基准 | 跟车更远 | 跟车更近 |
| `follow_distance_scheduler` | speed_decider conf | 速度→跟车距离斜率 | 高速时跟更远 | 高速时跟更近 |
| `follow_min_obs_lateral_distance` | speed_decider conf | 横向多远的障碍不算跟车 | 更宽容（路边物不触发） | 更严格 |
| `min_stop_distance_obstacle` | planning.conf (gflag) | 静态障碍 STOP 距离 | 停更远 | 停更近 |
| `default_cruise_speed` | planning.conf (gflag) | 巡航目标速度 | 更快 | 更慢 |
| `planning_upper_speed_limit` | planning.conf (gflag) | 全局限速上限 | 评测限速更高 | **⚠️ 千万别调小**（自锁陷阱） |
| `longitudinal_acceleration_upper_bound` | planning.conf (gflag) | 加速度上限 | 起步更快 | 起步更平缓 |
| `jerk_weight` / `acc_weight` | piecewise_jerk_speed conf | 速度平滑权重 | 更平缓（防急加减速） | 更激进（更快但可能顿挫） |

---

## 三、9 赛题调参详解（固定方案 + 怎么调）

### 赛题 0：定速巡航（≤11 km/h ≈ 3.06 m/s）

**参数**：`default_cruise_speed`（planning.conf）

| 项 | 值 |
|----|-----|
| 推荐值 | `--default_cruise_speed=3.0`（留 0.06 裕量） |

**怎么调**：
1. 先设 3.0 → 测试看实际车速
2. 实际速度若超 3.06 → 降到 2.8~2.9
3. 若 30 分钟时间紧张且该赛题独立 → 保持实车官方值不动（本地 conf=11.18，以实车 `cat planning.conf` 为准），优先现场确认测评方式

**⚠️ 注意**：这是**全局参数**，会影响所有赛题速度。若定速巡航是独立场景，此参数只影响它；若共用，需权衡（现场确认）。

---

### 赛题 1：减速带通行（≤1.5 m/s）

**参数**（二选一，**只配一个**）：
- 方案 A（官方 gflag）：`speed_bump_speed_limit`（planning.conf）
- 方案 B（插件）：`speed_bump_enabled` + `speed_bump_limit_speed`（scene_speed_limit conf）

| 项 | 方案A（gflag） | 方案B（插件） |
|----|---------------|--------------|
| 推荐值 | `--speed_bump_speed_limit=1.2` | `speed_bump_limit_speed: 1.2`，`speed_bump_forward_buffer: 3.0` |

**怎么调**：
1. 先设 1.2 → 测试过带实际速度
2. 实际超 1.5 → 降到 1.0
3. 若过带太早减速浪费（接近前就慢）→ 调小 forward_buffer 或改用 gflag（±1m 窗口更紧凑）

**⚠️ 两个机制并存会取最小限速**——确认只开一个，避免"看起来配了 1.2 却更慢"。

---

### 赛题 2：人行道（有人）停车 1~1.5 m

**参数**（crosswalk conf）：`stop_distance`、`expand_s_distance`、`stop_timeout`

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `stop_distance` | **1.25** | 区间中值；实际停车距离 = 1.25 + 停车偏移(0.2~0.8)，需标定 |
| `expand_s_distance` | 4.0~6.0 | 行人完全走完才放行（防提前起步） |
| `stop_timeout` | 30.0 | 等待行人时间（10s 可能提前放行） |

**怎么调**（重点：标定）：
1. 先设 1.25 → 测试，看**车头到人行道线**的实际距离
2. 实际 > 1.5 → 减小 stop_distance（车停太远）
3. 实际 < 1.0 → 增大 stop_distance
4. 车提前起步（行人没走完）→ 增大 `expand_s_distance` / `stop_timeout`
5. 车迟迟不走 → 减小 `stop_timeout` 或调小 `expand_s_distance`

**⚠️ 最大风险**：官方代码 speed_decider 对**静止行人**用 6m STOP 会覆盖 crosswalk 的 1.25m → 车可能停 6m。**现场第一件事验证此赛题**。若发生：
- 检查 STOP reason_code（104=CROSSWALK 说明 crosswalk 生效；1=HEAD_VEHICLE 说明被 6m 覆盖）
- 官方无 `pedestrian_stop_distance` gflag → 纯配置无解，只能接受或兜底

---

### 赛题 3：红绿灯停车 1~1.5 m

**参数**（traffic_light conf）：`stop_distance`

| 项 | 值 |
|----|-----|
| 推荐值 | `stop_distance: 1.25` |

**怎么调**：同赛题 2 标定法（先 1.25 → 实测车头到停止线 → 偏近减小/偏远增大）。

---

### 赛题 4：停止标志停车 1~1.5 m（完全停止）

**参数**（stop_sign conf）：`stop_distance`

| 项 | 值 |
|----|-----|
| 推荐值 | `stop_distance: 1.25` |

**怎么调**：同赛题 3。完全停止由 `STOP_SIGN_UNPROTECTED` 场景保证（不用调）。

---

### 赛题 5：障碍物停车避让（≥2 m）

**参数**：`min_stop_distance_obstacle`（planning.conf，默认 6.0）

| 项 | 值 |
|----|-----|
| 推荐值 | **不改**（6m ≥ 2m 天然达标） |

**怎么调**：
- 若现场障碍物是**静止车辆** → 6m 达标，无需改
- 若障碍是**横穿行人** → 官方无横向过滤，行人走完可能仍阻塞 → 需现场观察行为，必要时考虑（见 06 开放问题）
- 若车**绕行**不停车 → obstacle_nudge_decider `obs_passable_buffer` 调小（如 0.7→0.3）

---

### 赛题 6：动态障碍物跟随

**参数**（speed_decider conf）：`follow_min_time_sec`、`stop_follow_distance`、`follow_min_obs_lateral_distance`、`follow_distance_scheduler`

| 参数 | 推荐值 | 调的理由 |
|------|--------|---------|
| `follow_min_time_sec` | 1.0（官方2.0） | 前车起步后更快判 FOLLOW，缩短恢复迟滞 |
| `stop_follow_distance` | 2.0（保持官方） | 跟车距离基准 |
| `follow_min_obs_lateral_distance` | 2.5（保持） | 路边物不误触跟车 |

**怎么调**：
1. 前车起步后车**迟迟不动**（恢复慢）→ 减小 `follow_min_time_sec`（2.0→1.0→0.5）
2. 跟车太近/追尾风险 → 增大 `stop_follow_distance` 或 `follow_distance_scheduler` 斜率
3. 急加速/急减速被扣分 → 调 `piecewise_jerk_speed` 的 `jerk_weight`/`acc_weight`（增大更平缓）
4. 跟车距离过远浪费时间 → 减小 `stop_follow_distance` / 斜率

---

### 赛题 7：人行道路口减速（无人）≤1.5 m/s

**参数**（scene_speed_limit conf 插件）：`crosswalk_enabled` + `crosswalk_limit_speed` + buffer

| 参数 | 推荐值 |
|------|--------|
| `crosswalk_enabled` | true |
| `crosswalk_limit_speed` | 1.2 |
| `crosswalk_forward_buffer` | 5.0 |
| `crosswalk_backward_buffer` | 2.0 |

**怎么调**：
1. 先设 1.2 → 测通过速度
2. 超 1.5 → 降到 1.0
3. 提前减速太早（影响时间）→ 减小 forward_buffer
4. 到人行道跟前才降来不及 → 增大 forward_buffer

---

### 赛题 8：交通灯路口减速 ≤1.5 m/s

**参数**（scene_speed_limit conf 插件）：`traffic_light_enabled` + `traffic_light_limit_speed` + buffer

| 参数 | 推荐值 |
|------|--------|
| `traffic_light_enabled` | true |
| `traffic_light_limit_speed` | 1.2 |
| `traffic_light_forward_buffer` | 8.0 |
| `traffic_light_backward_buffer` | 2.0 |

**怎么调**：同赛题 7。

---

## 四、全局参数（影响所有赛题，谨慎）

| 参数 | 当前建议 | 说明 |
|------|---------|------|
| `planning_upper_speed_limit` | **保持实车官方值（本地 conf=20.0，源码默认 31.3）** | ⚠️ **千万别调小**！评测限速 = min(地图, 此值)，调小 = 自锁陷阱 |
| `default_cruise_speed` | 实车官方值（本地 conf=11.18；除非赛题0需要） | 全局巡航速度 |
| `longitudinal_acceleration_upper_bound` | 官方值（源码默认 4.0） | 起步/加速上限，一般不用动 |

> ⚠️ 以上默认值以**实车 `cat planning.conf` 实际内容为准**。仿真 800 分经验：把 `planning_upper_speed_limit` 调低曾导致起步超速（超冲越线）。**实车保持官方值最稳**。

---

## 五、调参标准流程（七步定式）

```
1. 读要求   明确赛题数值（限速/距离/时间）
2. 诊断     先跑一遍，记录实际行为（停车距离、速度、是否超时）
3. 定位     判断是哪个机制在管（TrafficRule停车 / gflag限速 / speed_decider决策 / 插件限速）
4. 改参     改对应 conf，一次一个，留裕量
5. 生效     重启对应模块：`apply_config.sh planning` 或 `apply_config.sh control`（不用编译/use default）
6. 验证     测试 → 看评分/行为 → 不达标回到 4
7. 固化     记录参数，进入下一赛题
```

---

## 六、失效排查（改了没效果怎么办）

| 现象 | 排查 | 处理 |
|------|------|------|
| 改配置没生效 | 改错文件 / 软链没指向 / 没重启对应模块 | 重启对应模块；`cat /apollo/.../conf/xx.pb.txt` 验证 |
| 停车距离不对 | 被 speed_decider 6m 覆盖（reason=1） | 查 STOP reason_code：`extract_stop_reason`（104=crosswalk / 1=车辆 / 2=终点） |
| 限速没生效 | 插件没预装/没注册 / 两个机制打架 | 现场确认插件已预装（`ls scene_speed_limit`）；只开一个限速机制 |
| 超速 | 限速值贴边 / 全局限速被调低 | 留裕量；`planning_upper_speed_limit` 保持官方 |
| 车不停/提前走 | stop_timeout 太短 / expand_s_distance 太小 | 增大等待时间/检测区 |
| 时间不够 | 恢复迟滞 / 巡航太慢 | follow_min_time_sec 调小；确认 cruise 速度 |

---

## 七、现场 30 分钟调参时间线

| 时间 | 动作 |
|------|------|
| 0-3min | `connect_car.sh` 一键连接 → `check_car.sh` → `backup_car.sh` |
| 3-6min | 确认限速插件是否在车；读官方默认配置 |
| 6-12min | **先配限速类**（插件 1/7/8 一组配置）+ **停车类**（2/3/4 stop_distance=1.25 一组配置）→ apply_config |
| 12-20min | 逐赛题测试（优先**赛题2**验证 6m 风险，再 3/4/1/7/8） |
| 20-27min | 按实测标定微调（停车距离、限速裕量）+ 测 5/6 |
| 27-30min | 复查最优成绩、固化参数、收尾 |

---

## 八、参数改动记录模板（现场用）

```text
赛题: ____
改动文件: ____
原值: ____ → 新值: ____
测试结果: 停车距离/速度 = ____ 达标? ☐是 ☐否
下一步: ____
```
