# kClearLateral 放行阈值调大影响评估（停车避让赛题）

> 研究日期：2026-08-01　|　纯数据分析，不改代码
> 目标：评估 `HandlePedestrianStop::kClearLateral` 从 3.5m 调大到 8m / 11m 对其它 7 个赛题的影响
> 数据源：8 个场景 json + follow_0002/0003/0004/0005 回放 + 记忆文件

---

## 1. 结论速览

| 阈值 | 停车避让 7673（目标） | 人行道 3060 | 其它 6 场景 | 结论 |
|------|----------------------|------------|-------------|------|
| **8m** | ✅ 横向 8m 放行（晚 ~4s） | ✅ 安全（走完 8.73m > 8 同步释放） | ✅ 零影响 | **推荐** |
| **11m** | ⚠️ 等更久（晚 ~7s） | ❌ **卡死风险**（走完 8.73m < 11 永不释放） | ✅ 零影响 | **不可行** |

**核心前提**：当前 HEAD（c904b46，`boundary.IsEmpty() → IGNORE`）下**只调 kClearLateral 对 7673 无效**——
放行主路径是 ST boundary 空（行人横向 >~1.9m）→ `AppendIgnoreDecision` → 车起步，kClearLateral 根本没机会参与。
必须配套 add9 式"boundary 空时对 fence 行人调 HandlePedestrianStop"的逻辑，kClearLateral 才真正决定放行时机。
本评估 = **在配套该逻辑的前提下的影响评估**（也是用户方案的真实前提）。

---

## 2. 8 场景行人/障碍物横向分布表

| 赛题 | json | 地图 | 对象 | 类型 | 初始位置 | 移动方式 | 横向距离（相对车道中心） | 触发 HandlePedestrianStop？ |
|------|------|------|------|------|----------|----------|--------------------------|------------------------------|
| 1 减速带 | 6a6c3d8b | Xh_2026 | 无 | - | - | - | - | 否 |
| **2 人行道** | 67e4bc95 | xh_2025 | 行人 3060 | PED | (423662.30, 4437623.24) | 0.5m/s 沿 +y 横穿 15.8m → (423662.01, 4437639.07) | **7.1m → 0 → 8.73m（穿过车道）** | **是**（横向<1.9m 时 boundary 非空，实测 stop×19） |
| 3 红绿灯 | 6a6c3f21 | Xh_2026 | 无 | - | - | - | - | 否 |
| 4 停止标志 | 67d938f8 | xh_2025 | 无 | - | - | - | - | 否 |
| **5 停车避让** | 6a699aef | Xh_2026 | 行人 7673 + 6 护栏 | PED + type2 | (423446.67, 4437610.95)=**车道中央** | ≥50s 触发，3m/s 沿轨迹走 32m → (423475.75, 4437599.76) | **0 → 11.2m（离开车道）** | **是（目标场景）** |
| 6 车辆跟随 | 6a699d15 | Xh_2026 | 车辆 2600 + 6 护栏 | VEH + type2 | (423319.33, 4437610.74) | 0 → 5.5m/s 同向行驶 | ~0.2–0.4m（同车道前车） | **否**（VEHICLE 不进入该分支） |
| 7 人行道路口 | 6a6c3ea5 | Xh_2026 | 无 | - | - | - | - | 否 |
| 8 交通灯路口 | 69d420a3 | Xh_2026 | 无 | - | - | - | - | 否 |

**要点**：
- 8 个赛题中**只有 2 个含行人**（人行道 3060、停车避让 7673）；车辆跟随的 2600 是 VEHICLE。
- 人行道 3060 虽起点在车道外（横向 7.1m），但**横穿会经过车道中心**（横向 7.1→0→8.73m），不是"从不进车道近处"。
- 6 个护栏（type=2 UNKNOWN_UNMOVABLE）在主循环顶层被无条件 IGNORE（add4b），不产生纵向决策。

---

## 3. HandlePedestrianStop 调用前提链（决定影响范围）

```
speed_decider::MakeObjectDecision 主循环
  ├─ boundary.IsEmpty() → AppendIgnoreDecision → continue   ← 横向 >~1.9m 的行人在这里被放行
  ├─ UNKNOWN_UNMOVABLE(type=2) → IGNORE                    ← 护栏
  ├─ IsStatic 分支 → PEDESTRIAN 用 -FLAGS_pedestrian_stop_distance(1.75) STOP（与 kClearLateral 无关）
  └─ 动态分支（非 IsStatic）
       ├─ CheckIsFollow → PEDESTRIAN → HandlePedestrianStop  ← kClearLateral 生效点
       └─ else          → PEDESTRIAN → HandlePedestrianStop  ← kClearLateral 生效点
```

- **ST boundary 非空的前提**：行人（或其 6s 预测轨迹）与 ADC box 重叠。
  `CheckOverlap`：ADC box = 车长 ×（车宽 2.11 + 2×0.4 l_buffer = 2.91m，半宽 1.455m）+ 行人半宽 → **行人横向 < ~1.9m 才非空**。
- 所以 HandlePedestrianStop **只作用于"横向 <~1.9m 的动态行人"**。
- 8 场景中满足"动态行人 + boundary 非空"的只有：**7673**（横向 0→1.9m 期间）和 **3060**（横穿车道横向 0→1.9m 期间，实测 3060_0 stop×19 = HandlePedestrianStop 的 ped_fixed 栅栏）。

---

## 4. 人行道场景（3060）专项分析

### 4.1 横向轨迹与 crosswalk 关键节点

| 节点 | 行人的 y | 横向（车道 y=4437630.34） | 说明 |
|------|----------|---------------------------|------|
| 起点（站在人行道南侧） | 4437623.24 | **7.1m** | 0.5m/s 沿 +y 横穿 |
| 进入车道（横向 1.9m） | 4437628.4 | 1.9m | boundary 开始非空，HandlePedestrianStop 介入 |
| 车道中心 | 4437630.34 | 0m | 横穿中点 |
| 出车道（横向 1.9m） | 4437632.2 | 1.9m | boundary 重新为空 |
| crosswalk polygon 北缘 | 4437636.57 | 6.23m | add5 v2 扩展区边界 |
| **crosswalk 放行时刻**（出扩展 +2m 区） | **4437638.57** | **8.23m** | 行人仍移动，crosswalk 放行 |
| 终点（走完停下） | 4437639.07 | **8.73m** | 行人完全离开 |

### 4.2 车在 crosswalk 规则下的行为（0005 验证）

- crosswalk STOP(104) 栅栏 423665.73，车停 x=423669.81，**车前端距人行道 1.91m（1.5~2.0m 达标）**。
- 主决策 = CROSSWALK + DESTINATION，**无 ped_fixed**（crosswalk 在 MergeLongitudinalDecision 中胜出）。
- crosswalk 放行逻辑（add5 v2）：行人在扩展 polygon 内且仍移动 → 无条件 STOP；行人停下 → 默认逻辑（横向>5m 放行）。

### 4.3 kClearLateral 与 3060 的交互

| kClearLateral | 3060 在横向 3.5~8.23m 时段 | 3060 走完（横向 8.73m） | 人行道结果 |
|---------------|---------------------------|------------------------|-----------|
| 3.5（当前） | ped_fixed STOP（与 crosswalk 并存，crosswalk 胜出） | 早已释放 | ✅ 达标 |
| **8** | ped_fixed STOP（与 crosswalk 并存，行为一致） | **8.73 > 8 → 释放** | ✅ **安全** |
| **11** | ped_fixed STOP | **8.73 < 11 → 永不释放** | ❌ **卡死**（crosswalk 放行后只剩 HandlePedestrianStop 栅栏，车被卡到超时） |

**结论**：kClearLateral=8m 对人行道安全；11m 会卡死人行道场景（3060 走完横向 8.73m < 11m）。

---

## 5. 停车避让场景（7673）时间线

- 行人 50s 触发移动，3m/s 沿 31.16m 轨迹走（横向速度 ≈ 1.08m/s）。
- 横向到达时间（从触发起，忽略加速段）：3.5m ≈ 3.2s｜8m ≈ 7.4s｜11m ≈ 10.2s。
- 2209 回放：当前 kClearLateral=3.5 时，车 t≈55.4s 起步（行人横向 3.47m 仍以 2.1m/s 移动）→ **"起步偏早"正是要修的问题**。
- kClearLateral=8：车 ≈58s 起步（行人横向 8m，已完全离开车道+裕量）。
- kClearLateral=11：车 ≈61s 起步（行人横向 11.2m=终点），收益边际小，等待更长。

---

## 6. 其它场景安全性

| 场景 | 是否受影响 | 原因 |
|------|-----------|------|
| 1 减速带 | ✅ 无 | 无对象 |
| 3 红绿灯 | ✅ 无 | 无对象（Signal_5 绿灯限速由 traffic_light 规则管，与 kClearLateral 无关） |
| 4 停止标志 | ✅ 无 | 无对象（StopSign_1/Crosswalk_30 地图元素，无人不触发） |
| 6 车辆跟随 | ✅ 无 | 2600 是 VEHICLE，不进入 HandlePedestrianStop；护栏 type=2 无条件 IGNORE |
| 7 人行道路口 | ✅ 无 | 无对象 |
| 8 交通灯路口 | ✅ 无 | 无对象 |

---

## 7. 安全阈值建议

1. **推荐 kClearLateral = 8.0m**：
   - 对 7673：行人横向 8m 时已完全离开车道（车道半宽 ~2.3m + 行人半宽 1m ≈ 3.3m 即够），8m 有充分裕量。
   - 对人行道 3060：crosswalk 放行时刻行人横向 8.23m > 8 → 同步释放，不卡死；横穿中段的额外 ped_fixed STOP 与 crosswalk 行为一致（0005 验证主决策不变、停车距离不变）。
   - 其余 6 场景零影响。
2. **上限 8.2m**（3060 在 crosswalk 放行时刻的横向），留裕量取 **8.0m**。
3. **11m 不可行**：人行道 3060 走完仅 8.73m < 11，HandlePedestrianStop 永不释放 → 卡死/超时。
4. **配套前提**：
   - 必须加 add9 式"boundary 空时对 fence 行人调 HandlePedestrianStop"逻辑（否则只调阈值对 7673 无效）。
   - fence 记录须保留"曾进入车道近处（横向<1.9m、boundary 非空、v>0.5）"过滤——防误伤；虽然 3060 会横穿车道（会短暂进入近处），但 fence 语义是按"横穿行人"记录，放行条件 obs_l≥8 使 3060 在 8.73m 释放，行为与 crosswalk 一致，无新增风险。

---

## 8. 与记忆文件的修正

- 旧记忆（pedestrian-7673-dual-id-analysis.md）称"人行道行人 3060 横向 6.6m 起步、**从不进车道近处** → add9 不影响"。
- **实测修正**（follow_0002/0004/0005）：3060 起点横向 7.1m，但**横穿会经过车道中心（横向→0）**，横向 <1.9m 时段 boundary 非空，实测 `3060_0: stop×19`（= HandlePedestrianStop ped_fixed 栅栏）。
- 结论不变的是：**在 kClearLateral=8 时人行道仍安全**（8.73>8 释放）；但 **11 会卡死**——这是旧分析未覆盖的新发现。
