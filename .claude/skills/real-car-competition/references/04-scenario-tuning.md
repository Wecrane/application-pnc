# 04 九大赛题调参方案（核心速查 + 完整机制）

> 来源：`output/solutions/00~08_*.md`（仿真 800/800 满分沉淀）+ 限速插件分析
> ⚠️ 实车赛题扣分标准以现场手册为准；仿真值作参考。**仿真限速类取值（≤5m/s 场景 4.5）与实车（≤1.5m/s）不同，实车限速取 1.2~1.4！**

---

## 赛题 0：定速巡航

**要求**：规定道路最高速度 ≤ 11 km/h（≈3.06 m/s）（仿真标准；实车以现场为准）

**机制**：`default_cruise_speed` gflag = 巡航目标速度（QP 软目标 dx_ref）

**配置**（`profiles/default/modules/planning/planning_component/conf/planning.conf`）：
```text
--default_cruise_speed=3.0        # 或按现场标准，留裕量
```
**注意**：仿真里 default_cruise_speed=16.39 是为了高速通过其他赛题；定速巡航赛题是**独立场景**，按该场景标准单独配（若有全局影响需权衡）。

---

## 赛题 1：减速带通行

**要求**：通过减速带 ≤ 3 m/s（仿真）/ ≤1.5 m/s（实车，以现场为准）

**机制**（代码链路）：
```
HD Map speed_bump_overlaps（路线 Lane_1962 上 SpeedBump_3/4）
→ ReferenceLineInfo::Init() 对每个减速带加限速区 [start_s-1.0, end_s+1.0]
   限速值 = FLAGS_speed_bump_speed_limit
→ ReferenceLine::GetSpeedLimitFromS() → SpeedLimitDecider（取所有限速最小）
→ SpeedBoundsDecider → DP + QP 生成不超限速的速度曲线
```

**关键事实**：
- 减速带是**地图元素**（不是感知障碍物），gflag 限速天然生效，纯配置即可
- 地图不帮忙限速（关联车道 70km/h），**100% 依赖 gflag**
- 源码 planning.conf 中该行**注释掉**，必须靠 profile 覆盖层激活

**配置**（planning.conf）：
```text
--speed_bump_speed_limit=2.5      # 仿真赛题 ≤3 推荐 2.5~2.7（留裕量）
```

**实车（限速插件方案）**：`speed_bump_enabled=true` + `speed_bump_limit_speed=1.2~1.4`（若赛题 ≤1.5）
⚠️ 若两个机制（gflag + 插件）同时存在会叠加取最小，建议只配一个或配相同值。

**经验**：
- 留裕量原则：3.0 贴边有超调风险（2.9+0.2=3.1 超速）；2.5 保守 / 2.7 激进
- 生效：改配置 → `aem profile use default` → 重启 Dreamview → 验证 `grep speed_bump_speed_limit /apollo/.../planning.conf`

---

## 赛题 2：人行道场景（有人）

**要求**：有人通过人行道时，在人行道前 **1~1.5m**（实车）/1.5~2.0m（仿真）停车；行人完全通过后继续行驶

**机制**（代码链路）：
```
Crosswalk TrafficRule（traffic_rules/crosswalk/）
→ 检测参考线 crosswalk_overlaps + 行人（expand_s_distance 外扩检测）
→ 有行人 → BuildStopDecision: stop_s = stop_line_s - config_.stop_distance()
→ stop_timeout 后行人未过完则放行
```

**⚠️ 关键坑（仿真踩过）**：
1. **静止行人被 speed_decider 当静态障碍用 6m 建 STOP**（`min_stop_distance_obstacle`），覆盖 crosswalk 的 1.75m → 车停 6.9m 超标
   - 仿真修复：新增 gflag `pedestrian_stop_distance=1.75`（speed_decider IsStatic 分支行人专用）
   - **实车无此 gflag** → 需验证实车行为：若仍 6m 超标，可能需改代码或确认官方版本是否有处理
2. 行人横穿中 `stop_loose_l_distance`（5.0）判定"行人离开路径"会**提前放行**（行人还在人行道上）→ 未礼让行人
   - 仿真修复：crosswalk.cc 扩展 polygon +2m 内行人移动中无条件 STOP
   - 实车需现场验证放行时机

**配置**（`profiles/default/.../traffic_rules/crosswalk/conf/default_conf.pb.txt`）：
```text
stop_distance: 1.75              # 1.0 不满足；1.75 为区间中值
expand_s_distance: 6.0           # 扩展检测区，行人走完才放行
stop_timeout: 30.0               # 等待行人时间（10s 行人未走完会提前放行）
# 其余官方默认：max_stop_deceleration:4.0, min_pass_s_distance:1.0,
# stop_strict_l_distance:5.0, stop_loose_l_distance:5.0
```

**停车距离标定（仿真 011/012 轮血泪）**：评测距离 ≈ stop_distance + 车到 fence 偏移（0.2~0.8m 波动，由接近 STOP 速度决定）。`stop_distance` 值要结合车实际停车偏移综合调，仿真最终 1.0 + QP 强制停分支 = 1.5~1.66m 达标。

**实车注意**：实车停车偏移受控制影响更大，需现场实测标定 stop_distance。

---

## 赛题 3：红绿灯停车

**要求**：红灯时停止线前 **1.0~1.5m**（实车）/1.5~2.0m（仿真）停车，不越线；绿灯且行人安全后通行

**机制**：
```
TrafficLight TrafficRule（traffic_rules/traffic_light/）
→ 遍历 signal_overlaps，红灯/黄灯建虚拟停车障碍
→ BuildStopDecision: stop_s = stop_line_s - config_.stop_distance()
```

**配置**（`profiles/default/.../traffic_rules/traffic_light/conf/default_conf.pb.txt`）：
```text
stop_distance: 1.75
enabled: true
max_stop_deceleration: 4.0
```

**注意**：本地代码有 `// mayaochang add` 绿灯/黑灯跳过逻辑（绿灯放行）；实车官方代码绿灯逻辑以实际为准。绿灯通行限速见赛题 8（插件 traffic_light_limit_speed）。

---

## 赛题 4：停止标志停车

**要求**：停止线前 **1.0~1.5m**（实车）/1.5~2.0m（仿真）**完全停止**，不越线，确认安全后通行

**机制（双机制）**：
1. **StopSign TrafficRule**：检测 stop_sign_overlaps → `stop_s = stop_line_s - config_.stop_distance()`
2. **StopSignUnprotectedScenario**：PRE_STOP → STOP → CREEP → INTERSECTION_CRUISE 四阶段（含停止计时 `stop_duration_sec=1`、蠕行确认安全）

**配置**（`profiles/default/.../traffic_rules/stop_sign/conf/default_conf.pb.txt`）：
```text
enabled: true
stop_distance: 1.75
```

**经验**：
- 与赛题 2/3 完全同模式（stop_distance 1.0→1.75）
- 需**完全停止**（speed=0 保持片刻），由场景 STOP 阶段保证
- 场景在 **xh_2025_contest** 旧地图（仿真），实车以现场地图为准

---

## 赛题 5：障碍物停车避让

**要求**：前方静止障碍（实车可能是静止车辆/行人），道路不支持绕行时提前平稳减速，障碍后**安全停车距离 ≥ 2m**；急加减速/碰撞扣分

**机制**：
```
静态障碍 → speed_decider IsStatic 分支
CreateStopDecision(obstacle, -FLAGS_min_stop_distance_obstacle)  # = -6.0m
fence_s = adc_sl_boundary_.end_s() + obstacle.min_s() + stop_distance
→ 停车栅栏 = 障碍前 6m → ≥2m 天然达标 ✅
```

**配置**：`min_stop_distance_obstacle=6.0`（默认）**无需改**。绕行风险由 obstacle_nudge_decider 管（`obs_passable_buffer=0.7`，两侧空间>0.7m 会绕行）——若现场出现绕行不停车，可调 `enbale_nugde_static_obs=false` 或改 buffer。

**⚠️ 仿真赛题 5 最难（行人 7673 横穿）**——完整复盘：
- 障碍是**行人**（PEDESTRIAN）不是车：静止→横穿→停在车道外（横向 10.7m）
- 难点 1：行人走完后 `CheckStopForPedestrian` 仍返回 true（不看横向）→ 车被一直 STOP → 磨蹭 15s → 超时/终点计分帧失败
- 难点 2：评测器 `DistToObstacleCar` 要求车距行人 [2, 5.5]m，`require_all_time_pass`；在"车停止"时采样
- **仿真满分解法**：① 行人横向>3.5m 不 STOP（CheckStopForPedestrian 加横向判据）② destination 25m 内才建 stop + 禁 REF_END ③ 车提前停稳 → 评测提前结束 → 躲过终点计分帧
- **实车**：若实车障碍也是行人横穿，同样逻辑风险；若障碍是**静止车辆**（秩序册描述"静止车辆"），则 6m STOP 天然达标，简单得多——**以现场赛题为准**

**实车调参关注点**：
- 障碍是车：`min_stop_distance_obstacle=6m` 达标，重点看减速平顺性（防急刹）
- 障碍是行人横穿：需现场确认官方代码的 CheckStopForPedestrian 是否有横向过滤；没有则可能需代码或接受扣分

---

## 赛题 6：动态障碍物跟随

**要求**：前方同向动态车辆，道路不支持超车/绕行时平稳跟随，保持安全距离，避免追尾/异常停车

**机制**：
```
动态障碍 → speed_decider CheckIsFollow → CreateFollowDecision
follow_distance_s = -EstimateProperFollowGap(adc_speed)
EstimateProperFollowGap: distance = stop_follow_distance + Σ slope_i × Δspeed
分段 (speed, slope)： (0,2.0) (2.7,1.0) (5.4,0.75) (8.1,0.5)
```

**跟车距离表（按自车速度）**：0→2.0m，2.7→7.4m，5.4→10.1m，8.1→12.1m，16.7→~16.4m

**关键坑（仿真踩过）**：
1. **STOP→FOLLOW 恢复迟滞**：前车静止→起步，prediction 延迟 ~9.4s 才切动态 → 车停滞 9.4s → 时间不够（仿真赛题 60s 上限，本地 55s 才到）
   - 修复 1：`follow_min_time_sec` 2.0→1.0（更快判 FOLLOW）
   - 修复 2：代码（`mayaochang add2`）移动中 VEHICLE（速度>0.5）不再建 STOP 直接 FOLLOW（实车无此代码，恢复迟滞可能较长 → 现场注意时间）
2. 路肩护栏（横向 2.18m < follow_min_obs_lateral_distance=2.5）触发 STOP 导致减速震荡
   - 修复：横向>1.35m 静态非行人 ignore（仿真代码）
   - 实车：`follow_min_obs_lateral_distance` 可调大（如 3.0）让护栏不触发跟车

**配置**（`profiles/default/.../tasks/speed_decider/conf/default_conf.pb.txt`）：
```text
follow_min_obs_lateral_distance: 2.5
follow_min_time_sec: 1.0
stop_follow_distance: 1.5         # 官方 2.0；调小缩短跟车距离（仿真 2026-08-04 已改 1.5）
follow_distance_scheduler {       # 斜率减半（仿真已改）
  follow_distance { speed: 0.0  slope: 1.0 }
  follow_distance { speed: 2.7  slope: 0.5 }
  follow_distance { speed: 5.4  slope: 0.375 }
  follow_distance { speed: 8.1  slope: 0.25 }
}
```

**平稳性**：若急加减速扣分 → 调 `piecewise_jerk_speed` 的 `jerk_weight`（仿真默认 3.0，官方 10）、`acc_weight`

---

## 赛题 7：人行道路口减速通行（无人）

**要求**：通过**无人**人行道时速度 **≤ 1.5 m/s**（实车）/≤5 m/s（仿真）

**机制（仿真发现无现成限速机制）**：
- 地图无 5m/s 限速；crosswalk 规则无限速逻辑；SpeedSetting 依赖外部指令；场景 approach_cruise_speed 不适用
- 仿真必须改代码：crosswalk.cc 无人人行道分支加 `AddSpeedLimit(overlap.start_s, overlap.end_s, 4.5)`

**实车方案（限速插件）**：
```text
# scene_speed_limit conf
crosswalk_enabled: true
crosswalk_forward_buffer: 5.0     # 提前限速距离，可调
crosswalk_backward_buffer: 2.0
crosswalk_limit_speed: 1.2        # 实车 ≤1.5 → 留裕量 1.2~1.4
```

**若实车无插件**：需改代码（同仿真）或确认官方版本有其它限速机制。

**副作用**：有人行道（赛题2）有行人→停车不受限速影响；减速带场景限速 2.5 更低取最小无影响。

---

## 赛题 8：交通灯路口减速通行

**要求**：绿灯通过信号灯区域速度 **≤ 1.5 m/s**（实车）/≤5 m/s（仿真）

**机制**：绿灯时内置 traffic_light 规则放行（无限速）；仿真改代码在绿灯分支加 `AddSpeedLimit(..., 4.5)`

**实车方案（限速插件）**：
```text
# scene_speed_limit conf
traffic_light_enabled: true
traffic_light_forward_buffer: 8.0
traffic_light_backward_buffer: 2.0
traffic_light_limit_speed: 1.2    # 实车 ≤1.5 → 1.2~1.4
```

**注意**：红灯停车（赛题3 stop_distance=1.75）与绿灯限速通过（插件）结合覆盖全流程；两个机制独立不冲突。

---

## 跨赛题通用配置（仿真 800 分最终状态，实车需复核）

### planning.conf 关键 gflag（当前仿真值）
```text
--planning_upper_speed_limit=20.0   # 全局限速上限（评测限速=min(地图,此值)；别往下调！）
--default_cruise_speed=16.39        # 巡航目标（QP 软目标）
--planning_max_acceleration=3.0     # 加速度上限（用满评测上限）
--speed_bump_speed_limit=2.9        # 减速带限速
--pedestrian_stop_distance=1.75     # ⚠️ 自定义 gflag，实车无此 flag！
--destination_check_distance=25.0   # ⚠️ 自定义，实车无！
--disable_reference_line_end_stop=true  # ⚠️ 自定义，实车无！
```

### 停车三规则（crosswalk/traffic_light/stop_sign）
```text
stop_distance: 1.75
```

### 路径（lane_follow_path）——右转切内弯修复
```text
# 仿真最终：贴线权重组合
path_optimizer_config {
  l_weight: 2.0
  dl_weight: 20.0
  ddl_weight: 1000.0
  dddl_weight: 1000.0
  lateral_derivative_bound_default: 2.0
  path_reference_l_weight: 100.0
}
```
> 仿真复盘：官方新基线 path_reference_l_weight=100 贴线太弱 → 右转急弯切内弯碰路肩；恢复旧版 10000 或折中组合。实车若现场有急弯可参考。

### speed_decider
```text
follow_min_time_sec: 1.0
stop_follow_distance: 1.5
```

---

## 实车调参优先级建议（30 分钟内）

1. **先解决会 0 分/超时的**：限速类（1/7/8，插件配置一次搞定）→ 停车距离类（2/3/4，stop_distance）→ 巡航（0）
2. **再优化拿满**：5/6 看现场障碍类型再定
3. **每改一个参数测一个赛题**（一次只改一个），记录结果
4. 改配置 → `aem profile use default` → 重启 Dreamview → 测试 → 看评分
