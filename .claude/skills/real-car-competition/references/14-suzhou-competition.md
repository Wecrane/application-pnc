# 14 苏州赛区专属资料（🚨 本队主战场，2026-08-05 确认）

> **背景**：本队参加的是**苏州赛区**，场地在 **🏫 河海大学**！评分体系、赛题、地图均与北京不同，以本文档为准。
> **来源**：实车拷贝 `maps_tools/suzhou.tar.gz`（苏州赛区完整包）+ 本地 `Xh_2026_contest` 地图交叉验证。
> **🚨 正式比赛地图 = `hohai_contest_1`（河海大学赛道）**，见 §5——该地图**未拷贝**，是补拷最高优先！

---

## 1. 🚨 苏州 4 题制评分体系（与北京 3 题制不同）

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

> ⚠️ **苏州减速带限速 1.5 m/s**（北京是 1.0）——别用错值！
> ⚠️ 苏州加速度下限 **-2.0**（北京 cruise 是 -3.0）→ 急刹更容易扣分，停车减速要更平缓！

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

## 4. 苏州 vs 北京（差异速查）

| 维度 | 苏州 | 北京海驾 |
|------|------|---------|
| 题数 | **4 题制**（road_speed/crosswalk/red_light/stop_sign） | 3 题制（cruise/crosswalk或red_light/减速带） |
| 停止牌 | ✅ 有（StopSign_1，0-2m） | ❌ 无 |
| 减速带限速 | **≤1.5 m/s** | ≤1.0 m/s |
| 加速度下限 | **-2.0** | -3.0 |
| 限速题名称 | road_speed（含 3 个减速带区域） | cruise（减速带单独成题） |
| 地图 | 测试场星火测试2（苏州测试场） | hehai_contest_*_3 |
| 信号灯时序 | Signal_1 绿5→红15→绿100 | Signal_1 绿3→红25→绿100 |

---

## 5. 🚨 苏州地图资源情况（⚠️ 实车地图未拷贝，需补拷！）

### 5.1 🚨 苏州正式比赛地图 = hohai_contest_1（河海大学）

- **用户确认：苏州赛区场地在河海大学** → 正式比赛地图是 **`hohai_contest_1`**（河海赛道）
- 证据：实车 `global_flagfile.txt` 第 60 行 `--map_dir=/apollo/modules/map/data/hohai_contest_1`，且**加载顺序在 hehai 比赛图之前** → 这辆车实测过河海赛道
- ⚠️ `hohai`（河海）≠ `hehai`（海驾）——是两个不同地图！别混淆
- `hohai_contest_1` **未拷贝** → **补拷最高优先**
- 苏州评分包（suzhou.tar.gz）场景注入用 "测试场星火测试2"（训练/测试场地），正式赛道以 hohai_contest_1 为准

### 5.2 确认遗漏（2026-08-05 复盘）
- 从实车拷贝时**只打包了 `data/map_data`（hehai_contest 4 张比赛图）**，未打容器地图目录 `/apollo/modules/map/data/`！
- 苏州/海驾/河海地图**全部遗漏**，需下次连车补拷。

### 5.3 车上地图完整清单（从已拷 global_flagfile.txt 归纳）

| 地图 | 路径 | 是否已拷 | 价值 |
|------|------|---------|------|
| **hohai_contest_1** | /apollo/modules/map/data/ | ❌ | 🚨 **苏州正式比赛图（河海大学）**，最高优先 |
| **slame-HaiDianJiaXiao-0827** | 同上 | ❌ | 苏州场景 json 引用的地图（localization 实载） |
| slame-HaiDianJiaXiao-0827_1~_4 | 同上 | ❌ | 海驾 0827 变体 |
| slame-HaiDianJiaXiao-0904_Sroad_area | 同上 | ❌ | 海驾 0904 |
| slam_tf_haijia827 | 同上 | ❌ | 海驾 tf |
| **test_cefang** | 同上 | ❌ | 测试车坊（可能河海测试场） |
| 海淀大圈 | 同上 | ❌ | 中文名地图 |
| AnnotatedMap_202607291120 | 同上 | ❌ | 标注地图 |
| apollo_virutal_map / sunnyvale_big_loop / demo | 同上 | ❌(demo已拷) | 通用 |
| hehai_contest_{6_am,6_pm,7_am,7_pm}_3 | data/map_data/ | ✅ | 北京比赛图（非本队） |

> 💡 `global_flagfile.txt` 的 `--map_dir` 按时间顺序记录了实车加载过的地图：demo → 海驾 0904/0827 → test_cefang → 海淀大圈 → AnnotatedMap → **hohai_contest_1** → hehai_contest 比赛图。hohai 出现 1 次且在比赛图之前 → 河海赛道是实车近期实测的正式赛道。

### 5.4 补拷命令（下次连车执行，hohai_contest_1 最优先）
```bash
# 1. 查看车上地图目录确认
ssh car "ls /apollo/modules/map/data/ 2>/dev/null; ls /apollo_workspace/data/map_data/ 2>/dev/null"

# 2. 打包河海(正式比赛)+苏州/海驾相关地图
ssh car "cd /apollo/modules/map/data && tar czf /tmp/car_maps_suzhou.tar.gz \
  hohai_contest_1 slame-HaiDianJiaXiao-0827* slam_tf_haijia827 slame-HaiDianJiaXiao-0904_Sroad_area \
  test_cefang 海淀大圈 AnnotatedMap_202607291120 2>/dev/null; ls -lh /tmp/car_maps_suzhou.tar.gz"

# 3. 下载到本地
scp car:/tmp/car_maps_suzhou.tar.gz ~/application-pnc/shiche/car_890/

# 4. 清理车端残留
ssh car "rm -f /tmp/car_maps_suzhou.tar.gz"
```

> ⚠️ 拿到 hohai_contest_1 后：解析赛道几何（减速带/人行道/信号灯/停止牌位置），与苏州 4 题制评分目标（Crosswalk_5/Signal_1/StopSign_1）对照，确认正式赛道布局。

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
- 评分引擎：同包 `grading_system_py/`（与北京相同，含 stop_sign_stop_handler）
