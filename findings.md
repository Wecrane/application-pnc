# 研究发现：赛题代码分析

> 创建时间：2026-06-13
> 与 task_plan.md 的 Phase 对应

---

## Phase 1 发现：赛题详细拆解

### 1.1 评分标准反推

#### 评分体系架构

Apollo 星火大赛的评分基于 `grading_metrics_default.conf` 配置文件。主要评分维度：

| 维度 | 指标 | 扣分条件 |
|------|------|----------|
| 碰撞检测 | collision | 车辆与任何障碍物碰撞 |
| 车道偏离 | lane_departure | 车辆中心超出车道边界 |
| 速度违规 | speed_violation | 超速或低于最低速度 |
| 红灯违规 | red_light | 闯红灯 |
| 停止标志违规 | stop_sign | 未在停止线前停下 |
| 完成度 | checkpoint | 是否到达终点 |
| 时间效率 | time | 完成时间 |
| 横向加速度 | lateral_accel | 横向加速度超限 |
| 纵向加加速度 | lon_jerk | 加加速度超限 |

#### 各场景评分权重推测

基于场景难度和静态/动态障碍物配置：

| 赛题 | 碰撞风险 | 车道偏离风险 | 超速风险 | 时间压力 | 综合难度 |
|------|----------|------------|----------|----------|----------|
| 交通灯 | 低 | 低 | 中 | 低 | ⭐⭐ |
| 变道 | 中(侧碰) | 低 | 中 | 中 | ⭐⭐ |
| S 弯 | 高(锥桶) | 高(窄弯) | 中 | 高 | ⭐⭐⭐ |
| U 型弯 | 高(大货) | 高(急转弯) | 高 | 中 | ⭐⭐⭐⭐ |
| 施工区域 | 高(密锥桶) | 高(窄通道) | 中 | 中 | ⭐⭐⭐ |
| 站点接驳 | 中(泊车) | 低 | 中 | 中 | ⭐⭐⭐ |
| 环岛让行 | 高(动态车) | 中 | 中 | 高 | ⭐⭐⭐⭐ |

### 1.2 各场景起终点精确分析

#### U 型弯场景_1 (07b)

```
起点: (424040.27, 4438479.61), heading=π (180°, 正西)
终点: (424041.15, 4438467.46)

Route lanes:
  Lane_1498 (s=357.5→492.9, 135m) → Lane_885 (s=357.5→492.9, 135m)
  → Lane_1955 (s=0→13.1, 13m, 179° U弯)
  → Lane_1953 (s=0→7.3, 7m)
  → Lane_1949 (s=0→5.0, 5m)
  → Lane_1952 (s=0→123.9, 124m) [终点]

关键观察: 起终点 x 差仅 0.9m (几乎同列), y 差 -12.1m (南行相邻车道)
         这是典型的"掉头返回"场景
```

#### 环岛让行场景 (080)

```
起点: (423377.61, 4438035.97), heading=0 (正东)
终点: (423544.07, 4438146.10)

Route path:
  Waypoint1(起点西) → Waypoint2(环岛南) → Waypoint3(终点东北)
  
交通流分析:
  BatchRoute1: 间距10m, 速度10m/s → 时间间隔1s
  BatchRoute2: 间距30m, 速度10m/s → 时间间隔3s
  
环岛周长约 150m (近似), BatchRoute1 车流密度:
  150m / 10m = 15辆同时在环岛上
  ADC 切入间隙 = 10m / 10m/s = 1s (极为紧张)
```

---

## Phase 2 发现：架构深度剖析

### 2.1 完整 Scenario 优先级与互斥图

```
优先级从高到低:
1.  EMERGENCY_PULL_OVER       [紧急] 触发: emergency command
2.  EMERGENCY_STOP             [紧急] 触发: emergency command
3.  BUS_BAY_TRANSFER           [公交] 触发: bus bay overlap
4.  BARE_INTERSECTION          [路口] 触发: bare intersection overlap
5.  STOP_SIGN_UNPROTECTED      [停车] 触发: stop sign overlap
6.  YIELD_SIGN                 [让行] 触发: yield sign overlap
7.  CONTEST_ROUNDABOUT         [环岛] 触发: 几何+PNC junction ← 赛题七
    ↓ 禁止切回: U_TURN, STATION_SHUTTLE
    ↓ 阻止切入: LANE_CHANGE
8.  CONTEST_CONSTRUCTION_ZONE  [施工] 触发: 锥桶≥20 ← 赛题五
    ↓ 互斥: U_TURN, S_CURVE
9.  CONTEST_LANE_CHANGE        [变道] 触发: ref_line>1 ← 赛题二
    ↓ 被阻止切入: 处于环岛时
10. CONTEST_S_CURVE            [S弯]  触发: 高曲率+锥桶≥4 ← 赛题三
    ↓ 互斥: U_TURN, CONSTRUCTION_ZONE
11. CONTEST_U_TURN             [U弯]  触发: 原始heading变化>2.0rad ← 赛题四
    ↓ 互斥: S_CURVE, CONSTRUCTION_ZONE
    ↓ 禁止被抢: ROUNDABOUT
12. TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN  [左转] 触发: 交通灯overlap
13. TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN [右转] 触发: 交通灯overlap
14. TRAFFIC_LIGHT_PROTECTED   [直行] 触发: 交通灯overlap ← 赛题一
15. PULL_OVER                  [靠边] 触发: pull over command
16. PARK_AND_GO                [泊车] 触发: parking command
17. LANE_FOLLOW                [默认] 永远命中 ← 兜底
```

### 2.2 IsTransferable 调用时序

每帧 Planning 循环中：

```
Frame 到来
  └─ ScenarioManager::Update(frame)
       ├─ 遍历 scenarios_[0..N-1] (按配置顺序)
       │   ├─ scenarios_[i]->IsTransferable(current_scenario, frame)
       │   │   ├─ current_scenario != nullptr?
       │   │   │   └─ 检查是否可以"抢走"当前场景
       │   │   ├─ IsReferenceLineReady(frame)?
       │   │   │   └─ planning_command 存在? lane_follow_command? ref_line 非空?
       │   │   └─ 场景特定检测 (见 Phase 3 详细分析)
       │   └─ 命中 → scenario_manager 切换到新场景
       └─ ...
  └─ current_scenario->Stage::Process(frame)
       └─ 执行 Task 流水线
       └─ StillInScenario(frame)?
            ├─ true → RUNNING (继续)
            └─ false → FinishScenario() → 下次循环匹配新场景
```

---

## Phase 3 发现：代码实现逐行分析

### 3.1 IsContestUTurn 的两阶段检测

#### 方法A (Lane Turn 属性检测)

```cpp
// 问题: 地图中 U_TURN lane 被 routing graph_creator 的 IsValidUTurn()
//       排除 (半径太小), 所以 routing 路径中可能不包含 U_TURN lane
// 优点: 如果 routing 中包含, 这是最快最准的检测
for (double s = adc_end_s; s < adc_end_s + 35m; s += 2m) {
    if (reference_line_info.GetPathTurnType(s) == U_TURN) return true;
}
```

#### 方法B (原始几何 heading 检测)

```cpp
// 核心洞察: 不依赖参考线平滑 (平滑会稀释 heading 变化)
// 直接读取 lane 的 central_curve 原始点

// 对路由中每条 lane:
for (const auto& seg : reference_line_info.Lanes()) {
    // 1. 提取 central_curve 的所有点
    // 2. 逐对计算 heading 变化:
    sum_abs_dh = Σ|Δheading[i]|
    net_h = |heading_last - heading_first|
    
    // 3. 单向性检查:
    ratio = net_h / sum_abs_dh
    
    // U弯: net_h ≈ 3.14 rad, sum_abs ≈ 3.14, ratio ≈ 1.0
    // S弯: net_h ≈ 0.10 rad, sum_abs ≈ 3.00, ratio ≈ 0.03
    if (net_h > 2.0 && ratio > 0.5) return true;
}
```

**数学原理**：
- 单向转弯: 所有 Δheading 同号 → net ≈ sum_abs → ratio≈1.0
- S形弯道: 左右方向抵消 → net≈0, sum_abs≈大 → ratio≈0
- 阈值 0.5 是保守选择: 允许一定程度的方向变化, 但排除明显的双向弯道

### 3.2 IsContestRoundaboutEntry 的层级检测

```
Layer 1: 排除 U 弯
  └─ HasNearUTurnFeature():
       kappa > 0.08 AND heading_change > 2.2 rad → 这是 U 弯, 不是环岛

Layer 2: PNC 路口近场检测
  └─ 80m 内有 pnc_junction_overlap?
       AND 距离入口 ≤ 18m (激活窗口)
  
Layer 3: 短距离曲率-转角检测 (55m)
  └─ max_abs_kappa > 0.025
       AND 0.65 < heading_change < 1.8 rad
  
Layer 4: 长距离几何检测 (180m) [仅当有 PNC junction 时]
  └─ max_abs_kappa > 0.018
       AND 0.75 < heading_change < 2.2 rad

命中条件: Layer3 OR (Layer2 AND Layer4)
软命中: max_abs_kappa > 0.015 AND heading_change > 0.325

环岛 vs U 弯区分:
  环岛: 0.65 < Δheading < 1.8 (37°~103°)
  U弯:  Δheading > 2.2 (126°)
  缓冲区: 1.8~2.2 (避免误判)
```

### 3.3 U 弯大半径转弯参考的几何原理

#### 为什么需要大半径转弯？

```
问题场景:
  左侧参考线曲率 κ = 0.24 rad/m (13m转180°)
  若车紧贴参考线 (l=0): 实际转弯半径 = 1/0.24 = 4.17m
  车辆 min_turn_radius = 2.5m → 可以转, 但极紧

  若车向右侧偏移 l = -3.2m:
    实际转弯半径 = 4.17 + 3.2 = 7.37m
    实际曲率 = 1/7.37 = 0.136 rad/m
    → 转弯轻松很多! 曲率降低 43%
```

#### 偏移量计算

```
dominant_kappa > 0 (左转弯): outer_l = -3.2m (右侧大半径)
dominant_kappa < 0 (右转弯): outer_l = +3.2m (左侧大半径)

内侧阻塞时: target_l = outer_l × 0.66 = ±2.11m (保守等待)
释放后: target_l = outer_l × 1.0 = ±3.2m (最大偏移)
```

---

## Phase 4 发现：配置参数推导

### 4.1 关键参数选择依据

#### u_turn_heading_change_threshold = 2.0 rad (114.6°)

```
推导: Lane_1955 实测 heading 变化 ≈ 3.14 rad (180°)
      需要远小于 3.14 来提前检测
      需要大于 1.8 (环岛上限) 来区分环岛
      → 2.0 = (1.8 + 3.14) / 2 ≈ 2.47, 取保守值 2.0
      
      实际运行时 override 为 2.7 rad (155°): 更保守, 需更大转向才判定
```

#### roundabout_min_heading_change = 0.65 rad (37°)

```
推导: 环岛入口通常是 90°转入, 但因地图误差和参考线平滑,
      实际检测到的 heading 变化可能只有 40~60°。
      0.65 rad 是多个环岛场景统计后的下界。
```

#### roundabout_max_heading_change = 1.8 rad (103°)

```
推导: 超过 103°的转弯可能不是环岛 (环岛通常是 90°转入后沿弯道走)
      设置上界与 U 弯下界 (2.0) 之间留 0.2 rad 死区
```

### 4.2 硬编码常量的物理推导

| 常量 | 值 | 推导来源 |
|------|-----|----------|
| kUTurnExitMaxLateralOffset | 0.8m | 车道半宽约 1.75m, 0.8m 约为中心 ±1σ |
| kOuterLaneOffset | 3.2m | 车道宽度 3.5m - 安全边距 0.3m |
| kCloseFrontS | 6.0m | 紧急制动距离 (5m/s → 0, 减速度 2m/s² → 6.25m) |
| kInnerLaneHalfWidth | 1.8m | 车道宽度 3.5m / 2 ≈ 1.75m, 加 5cm 余量 |
| kRoundaboutExitPastEntryDistance | 8.0m | 车辆长度 4m × 2 = 8m (确保完全通过入口) |

---

## Phase 5 发现：综合评估

### 5.1 代码质量评估

#### 优点
1. **模块化清晰**: Scenario/Stage/Task 三层分离, 职责明确
2. **检测鲁棒**: 多方法冗余检测 (U 弯: 方法A+B, 环岛: 4层)
3. **日志完备**: 每个关键节点有带标签的结构化日志
4. **防御性编程**: latch 过渡、防重入、互斥保护

#### 待改进
1. **硬编码坐标**: 施工区域 ROI 使用固定 XY 范围, 不通用
2. **魔法数字多**: 大量硬编码阈值分散在各文件中
3. **跨文件常量不一致**: proto 默认值和配置文件值不一致
4. **缺少单元测试**: 检测函数没有独立测试

### 5.2 潜在 Bug 分析

| 风险点 | 影响 | 触发条件 |
|--------|------|----------|
| U 弯检测 ratio 阈值 0.5 | 漏检 | 参考线包含短直段 + U弯段 (ratio<0.5) |
| 环岛防重入 100m 限制 | 无法重入 | 两个环岛间距<100m |
| 施工区域跨车道统计 | 重复计数 | 同一锥桶出现在多个 ref_line 的 SL 中 |
| U 弯退出 heading 阈值 2.7 | 过早退出 | 掉头过程中 heading 短暂反转 |

### 5.3 性能瓶颈
- IsContestUTurn 方法B 遍历所有路由 lane 的 central_curve 点 (O(N×M))
- IsContestRoundaboutEntry 每个参考线都计算曲率 (O(ref_lines × points))
