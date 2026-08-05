# 14 苏州赛区专属资料（🚨 本队主战场，2026-08-05 确认）

> **背景**：本队参加的是**苏州赛区**，场地在 **🏫 河海大学**（Hohai）！
> **🚨 关键拼写**：`hehai` = 河海（拼音）、`hohai` = 河海大学（英文 Hohai）——**都是河海/苏州**！
> **✅ 苏州正式比赛图 = `hehai_contest_*_3`（已拷贝！）**；`hohai_contest_1` = 河海英文名拼写的另一版（未拷，补拷次优先）。
> **🚨 日志（8/3-8/4）就是在 hehai_contest 正式赛道上实测的（车 8/4 已在苏州）**——苏州正式赛道实车数据已有！
> **来源**：实车拷贝 `maps_tools/suzhou.tar.gz`（7/28 旧版评分）+ `grading_console`（8/4 新版评分）+ hehai_contest 地图 + 日志。

---

## 1. 🚨 苏州 4 题制评分体系（旧版 suzhou.tar.gz，训练场用）

> 来源：`suzhou.tar.gz` → `auto_drive_console/metric/scenes.json` + 4 个赛题指标 json。

| # | 赛题 id | scope | 评分目标 | 要求 |
|---|---------|-------|---------|------|
| 1 | **road_speed**（限速巡航） | full（全程） | — | 速度 **≤3.056 m/s**（11km/h）+ 加速度 **[-2.0, 2.0]** + **3 个减速带区域限速 ≤1.5 m/s** |
| 2 | **crosswalk_pedestrian**（人行道） | crosswalk | **Crosswalk_5** | 停在人行道前 **[1.0, 1.5]m**，行人占用期间 |
| 3 | **red_light**（红灯） | red_light | **Signal_1** | 停在停止线前 **[1.0, 1.5]m**，红灯期间 |
| 4 | **stop_sign**（停止牌） | stop_sign | **StopSign_1** | **完全停止**在停止线前 **[0.0, 2.0]m**，不得越线 |

**关键评分参数**：
- 全部 `is_critical: true` + `require_all_time_pass: true` → 全程达标才 PASS
- 停车题超区间：`single_deduction=20`；压线/越线/碰撞：硬失败 100
- 限速题：每超 1m/s 每帧扣 2 分（`deduction_unit=1`）
- 加速度题：超出 [-2, 2] 每 1m/s² 每帧扣 1 分

### 苏州 3 个减速带区域（限速 ≤1.5 m/s，精确多边形）
| 区域 | 坐标范围 | 距场景start(423637.8) |
|------|---------|---------------------|
| 减速带1 | x≈423655-423656, y≈4437966-4437977 | ~17-18m |
| 减速带2 | x≈423685-423691, y≈4437965-4437987 | ~47-53m |
| 减速带3 | x≈423721-423727, y≈4437969-4437990 | ~83-89m |

> ⚠️ **旧版减速带限速 1.5 m/s**（新版正式图是 1.0）——别用错值！
> ⚠️ **旧版加速度下限 -2.0**（新版 cruise 是 -3.0）→ 旧版急刹更容易扣分，停车减速要更平缓！

---

## 2. 🚨 苏州 score_history.json 真实评分数据（10 次，2026-07-28）

> 来源：`suzhou.tar.gz` → `auto_drive_console/records/score_history.json`——**官方评分系统在苏州的实战记录**！

| 时间 | 总分 | road_speed | crosswalk | red_light | stop_sign |
|------|------|-----------|-----------|-----------|-----------|
| 15:03 | **90.8** | 63.0 | 100 | 100 | 100 |
| 15:14 | 54.2 | 17.0 | 100 | 100 | 0 |
| 15:18 | 86.2 | 45.0 | 100 | 100 | 100 |
| 16:19 | 25.0 | 0 | 0 | 100 | 0 |
| 16:29 | 61.0 | 44.0 | 0 | 100 | 100 |
| 17:48 | 33.3 | 0 | 0 | 100 | 0 |
| 17:57 | 27.8 | 11.0 | 0 | 100 | 0 |
| 18:11 | 58.5 | 34.0 | 100 | 100 | 0 |
| 18:17 | 57.2 | 29.0 | 100 | 100 | 0 |
| 18:21 | 59.5 | 38.0 | 100 | 100 | 0 |

### 📊 决定性规律（苏州 4 题）
1. **red_light 10/10 全满分** —— 红灯题最容易，停对位置就 100
2. **road_speed（限速题）从不满分**（0~63）—— ⚠️ **超速是最大丢分源**！
3. **crosswalk 一半 100 一半 0** —— 行人占用时停车时机/距离不稳（与 stop_distance=0.1 急刹相关）
4. **stop_sign 一半 100 一半 0** —— 停止牌题不稳（需完全停止在 0-2m）
5. 最高分 90.8 仍被 road_speed=63 拖累 → **攻克限速题是拿高分关键**

---

## 3. 苏州测试场景（scenario_injection，实车回灌测试用）

> 来源：`suzhou.tar.gz` → `scenario_injection/1.json`（苏州测试场场景，map_dir=测试场星火测试2）。

- **赛道**：start (423637.81, 4437969.97) → end (423813.32, 4437985.18)，沿 x+ 方向约 175m
- **8 个 agent**：7 行人（type=2，速度 0.4-1.0 m/s，start_dist 30-40m）+ 1 车辆（type=0，1 m/s，start_dist 15m）
- **2 个信号灯**：
  - `Signal_1`：绿 5s → **红 15s** → 绿 100s（位置 x=423709.6，距 start ~72m）
  - `Signal_3`：红 50s → 绿 100s（位置 x=423701.5）
- 场景时长 120s

### 苏州赛道布局（距 start 距离）
| 元素 | 位置 | 距start |
|------|------|--------|
| 减速带1 | x≈423655 | ~17m |
| 减速带2 | x≈423688 | ~50m |
| Signal_3 | x=423701.5 | ~64m |
| Signal_1（红灯题） | x=423709.6 | ~72m |
| 减速带3 | x≈423723 | ~85m |
| end | x=423813.3 | ~175m |

---

## 4. 苏州两套评分体系（都是本队要面对的）

> ⚠️ 之前误以为 hehai_contest=北京，实际 **hehai=河海=苏州**！两套评分都是苏州的：
> - **旧版（7/28，suzhou.tar.gz）**：4 题制，用训练场"测试场星火测试2"
> - **新版（8/4，grading_console 升级版）**：presets 4 场次，用**正式图 hehai_contest_*_3**

| 维度 | 新版 8/4（正式图 hehai_contest_*_3） | 旧版 7/28（训练场 测试场星火测试2） |
|------|------|---------|
| 题数 | 3 题/场次（cruise + crosswalk或red_light + 减速带） | **4 题制**（road_speed/crosswalk/red_light/stop_sign） |
| 停止牌 | ❌ 无 | ✅ 有（StopSign_1，0-2m） |
| 减速带限速 | **≤1.0 m/s** | **≤1.5 m/s** |
| 加速度下限 | cruise: [-3, 2] | **[-2.0, 2.0]** |
| 限速题 | cruise（≤3.0556，减速带单独成题） | road_speed（≤3.056 + 3 减速带区域） |
| 地图 | **hehai_contest_*_3（河海正式图，已拷）** | 测试场星火测试2（训练场，未拷） |
| 信号灯时序 | Signal_1 绿3→红25→绿100 | Signal_1 绿5→红15→绿100 |

> 🚨 **现场确认用哪套**：若比赛用正式图 hehai_contest_*_3（新版）→ 无停止牌题、减速带 ≤1.0；若用旧版 4 题制 → 含停止牌、减速带 ≤1.5。两套都要能过！

---

## 5. 🚨 苏州地图资源情况（正式图已拷，其余需补）

### 5.1 ✅ 苏州正式比赛地图 = hehai_contest_*_3（河海，已拷！）

- **用户确认：hehai = 河海（苏州）**；车 8/4 已在苏州，日志就在 hehai_contest 上跑 → **hehai_contest_*_3 就是苏州正式比赛图，且已拷贝！**
- 4 张：`hehai_contest_{6_am,6_pm,7_am,7_pm}_3`（base_map/routing/sim_map 齐全）
- `hohai_contest_1` = 河海大学英文拼写（Hohai）的地图版本，`global_flagfile` 第 60 行出现过（加载在 hehai_contest 之前，可能是更早版本/验证版）→ **未拷贝，补拷次优先**（验证是否与 hehai_contest 同赛道）
- "测试场星火测试2"（423xxx 坐标）= 训练场/另一场地，与正式图（272xxx 坐标）不同坐标系

### 5.2 已拷 vs 待补（2026-08-05 复盘）
- ✅ 已拷：hehai_contest_*_3（苏州正式图，4 张）+ demo
- ❌ 待补：hohai_contest_1、测试场星火测试2、slame-HaiDianJiaXiao-0827 系列（海淀驾校训练场）、test_cefang 等

### 5.3 车上地图完整清单（从已拷 global_flagfile.txt 归纳）

| 地图 | 路径 | 是否已拷 | 价值 |
|------|------|---------|------|
| **hehai_contest_{6_am,6_pm,7_am,7_pm}_3** | data/map_data/ | ✅ | 🚨 **苏州正式比赛图（河海拼音），4 张已拷** |
| **hohai_contest_1** | /apollo/modules/map/data/ | ❌ | 河海英文名版（可能旧版/同赛道），**补拷次优先** |
| **slame-HaiDianJiaXiao-0827** | 同上 | ❌ | 海淀驾校训练场（苏州场景 json 引用） |
| slame-HaiDianJiaXiao-0827_1~_4 | 同上 | ❌ | 海淀 0827 变体 |
| slame-HaiDianJiaXiao-0904_Sroad_area | 同上 | ❌ | 海淀 0904 |
| slam_tf_haijia827 | 同上 | ❌ | 海淀 tf |
| **test_cefang** | 同上 | ❌ | 测试车坊（可能苏州测试场） |
| 海淀大圈 | 同上 | ❌ | 中文名地图 |
| AnnotatedMap_202607291120 | 同上 | ❌ | 标注地图 |
| apollo_virutal_map / sunnyvale_big_loop / demo | 同上 | ❌(demo已拷) | 通用 |

> 💡 `global_flagfile.txt` 的 `--map_dir` 按时间顺序：demo → 海淀 0904/0827 → test_cefang → 海淀大圈 → AnnotatedMap → **hohai_contest_1** → hehai_contest 比赛图。说明实车先加载过河海英文名版（hohai_contest_1），后切到正式比赛图（hehai_contest_*_3，8/3-8/4 日志实证）。

### 5.4 补拷命令（下次连车执行，hohai_contest_1 次优先）
```bash
# 1. 查看车上地图目录确认
ssh car "ls /apollo/modules/map/data/ 2>/dev/null; ls /apollo_workspace/data/map_data/ 2>/dev/null"

# 2. 打包待补地图（hohai_contest_1 + 训练场/测试场）
ssh car "cd /apollo/modules/map/data && tar czf /tmp/car_maps_suzhou.tar.gz \
  hohai_contest_1 测试场星火测试2 slame-HaiDianJiaXiao-0827* slam_tf_haijia827 slame-HaiDianJiaXiao-0904_Sroad_area \
  test_cefang 海淀大圈 AnnotatedMap_202607291120 2>/dev/null; ls -lh /tmp/car_maps_suzhou.tar.gz"

# 3. 下载到本地
scp car:/tmp/car_maps_suzhou.tar.gz ~/application-pnc/shiche/car_890/

# 4. 清理车端残留
ssh car "rm -f /tmp/car_maps_suzhou.tar.gz"
```

> ✅ **hehai_contest_*_3（正式图）已拷**，可直接解析赛道几何；hohai_contest_1 补拷后验证是否同赛道。赛道几何见 **13 文档 §4**（Crosswalk_1/5、Signal_1、减速带精确位置）。

### 5.5 本地可用资源
- 本地 `data/map_data/Xh_2026_contest/`（2026 星火大赛官方地图）**坐标系一致**（left=423223, top=4438490，覆盖苏州场景坐标），但**元素位置不同**（Signal_1/StopSign_1/Crosswalk_5 不在场景沿线）→ 仿真赛道图，非实车测试场图
- 备赛时可用 `Xh_2026_contest` 练机制（含 StopSign_1 可练停止牌），但**赛道细节以现场为准**

---

## 6. 苏州调参重点（基于评分体系 + 真实数据）

| 赛题 | 官方要求 | 关键动作 |
|------|---------|---------|
| 全局铁律 | planning.conf `default_cruise_speed`/`planning_upper_speed_limit` ∈ [1.5, 5.1] | 实车 5.0/5.0 ✅ 别动 |
| road_speed 限速 | ≤3.056 m/s + 3 减速带 ≤1.5 | ⚠️ **default_cruise_speed 5.0 会超速扣分** → 降到 ≤3.0；减速带区域限速 1.2~1.4（留裕量） |
| crosswalk | Crosswalk_5 前 [1.0,1.5]m | `stop_distance` 0.1 → **1.25**（急刹 4.9-6.9 也同时解决） |
| red_light | Signal_1 前 [1.0,1.5]m | `stop_distance` 0.1 → **1.25** |
| stop_sign | StopSign_1 前 [0,2]m 完全停止 | `stop_distance` 4.0 → **1.0~1.5**（当前 4.0 可能停太远）；需确认实车停止牌场景触发 |
| 加速度 | [-2, 2] | 停车减速平缓（<2 m/s²），急刹双扣分 |

### 🎯 苏州 4 题得分策略
1. **保 red_light + stop_sign 满分**（停车题，位置调对即 100）
2. **稳 crosswalk**：stop_distance 1.25 + 平缓减速
3. **攻克 road_speed**（最大丢分源）：巡航 ≤3.0 + 3 个减速带限速 1.2~1.4
4. 目标：4 题全 PASS = 100 分（苏州历史最高 90.8，说明 100 可期但需攻克限速题）

---

## 7. 配套文件速查

- 苏州评分指标：`shiche/car_890/maps_tools/suzhou.tar.gz` → `auto_drive_console/metric/`（scenes/road_speed/crosswalk_pedestrian/red_light/stop_sign/dkit_vehicle_param）
- 苏州评分历史：同包 `records/score_history.json`（10 次实战）
- 苏州注入测试日志：同包 `records/scenario_injection.log`（⚠️ 曾报 "Traffic light not found on map" 注入失败）
- 苏州测试场景：同包 `scenario_injection/1.json`（8 agent + Signal_1/Signal_3）
- 改造 msg_adapter 源码：同包 `modules/perception/msg_adapter/`（场景注入核心，需编译感知）
- 评分引擎：同包 `grading_system_py/`（两套评分共用，含 stop_sign_stop_handler）
