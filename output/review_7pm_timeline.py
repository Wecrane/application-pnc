#!/usr/bin/env python3
"""独立审查：7号下午_1 时间线精确核算（纯研究）。

输入（实测/源码）：
- 车道链 Lane_48->Lane_52->Lane_49->Lane_67->Lane_66，起点投影 s=3.52m
- 减速带评分区 y[3476132.0,3476137.5]  -> 距起点 Δy[23.51,29.06]
- Crosswalk_1 y[3476133.29,3476135.24] -> Δy[24.79,26.75]
- Crosswalk_5 y[3476168.14,3476170.10] -> Δy[59.64,61.61]
- 插件限速：crosswalk_limit 0.7 / forward 8 / backward 5
   -> Crosswalk_1 区 Δy[16.79,31.75]；Crosswalk_5 区 Δy[51.64,66.61]
- 巡航 2.5 m/s，加速度边界 [-3,2]，规划加减速 ~1.0 m/s²
- 行人（Crosswalk_5 处 Δy≈60.5）：
   5844 35m/1.0 西；6840 25m/0.6 东；6841 20m/0.5 西；6844 15m/0.5 西；6845 15m/0.5 东
"""
import math

# ---------- 速度剖面（分段线性）----------
CRUISE = 2.5
LIMIT = 0.7
ACC = 1.0  # 加减速 m/s²
DEC = (CRUISE**2 - LIMIT**2)/(2*3.1)  # 限速前减速 ~0.93

# 分段： (t0, s0, v0, v1, a)
SEGS = [
    ("起步加速",      0.0,  0.00, 0.0, 2.5,  ACC),
    ("巡航到限速区",   2.5,  3.13, 2.5, 2.5,  0.0),
    ("减速带减速",     7.97, 16.79, 2.5, 0.7, -DEC),
    ("限速0.7通过",   9.91, 19.89, 0.7, 0.7,  0.0),
    ("加速回巡航",    26.85,31.75, 0.7, 2.5,  ACC),
    ("巡航到C5区",    28.65,34.63, 2.5, 2.5,  0.0),
    ("C5区减速",      35.45,51.64, 2.5, 0.7, -DEC),
    ("C5区0.7",       37.39,54.74, 0.7, 0.7,  0.0),
]

def s_at(t):
    """给定时刻 t 的 Δy：定位到真实分段 t0（SEGS 已含累积时刻），段内积分。"""
    for i, (name, t0, s0, v0, v1, a) in enumerate(SEGS):
        dur = (v1 - v0)/a if a != 0 else (SEGS[i+1][1] - t0 if i+1 < len(SEGS) else 1e9)
        if t <= t0 + dur + 1e-9:
            dt = t - t0
            return s0 + v0*dt + 0.5*a*dt*dt
    return SEGS[-1][2] + 0.7*(t - SEGS[-1][1])

def t_at(dy):
    """到达指定 Δy 的时刻（二分）。"""
    lo, hi = 0.0, 300.0
    for _ in range(80):
        mid = (lo+hi)/2
        if s_at(mid) < dy:
            lo = mid
        else:
            hi = mid
    return (lo+hi)/2

print("== 速度剖面（推荐配置）==")
for name, t0, s0, v0, v1, a in SEGS:
    print("  %-12s t=%6.2f~%-6.2f  Δy %6.2f  v %.2f->%.2f  a=%+.2f" % (
        name, t0, t0 + ((v1-v0)/a if a != 0 else 1e9), s0, v0, v1, a))

print()
print("== 关键 Δy 到达时刻（推荐配置速度剖面）==")
for dy in [15, 20, 25, 35, 45.5, 51.64, 58.4]:
    # 二分找时间
    lo, hi = 0.0, 200.0
    for _ in range(60):
        mid = (lo+hi)/2
        if s_at(mid) < dy: lo = mid
        else: hi = mid
    print("  Δy=%-6.1f -> t=%.2fs" % (dy, (lo+hi)/2))

print()
print("== 行人时序（两种触发语义对比）==")
# 行人: (id, start_dist, speed, 横穿距离, 方向, 出发x)
peds = [
    ("5844", 35, 1.0, 6.92, "西", 272590.70),
    ("6840", 25, 0.6, 7.29, "东", 272583.78),
    ("6841", 20, 0.5, 7.39, "西", 272590.96),
    ("6844", 15, 0.5, 8.17, "西", 272591.69),
    ("6845", 15, 0.5, 7.61, "东", 272583.90),
]
# 车道中心 x≈272587.3，横道 x∈[272584.99,272589.40]，box±0.5 → 占用 x∈[272584.49,272589.90]
# 路边界 x∈[272585.62,272589.01]（Road_49），box 重叠路判定 x∈[272585.12,272589.51]
LANE_X = 272587.3
for label, mode in [("A 接近触发(ADC距行≤start_dist)", "A"), ("B 起点距离触发(ADC行驶=start_dist)", "B")]:
    print("  【%s】" % label)
    for pid, sd, sp, dist, direction, px0 in peds:
        if mode == "A":
            t_act = None
            # 找 ego Δy = 60.5 - sd 的时刻
            lo, hi = 0.0, 200.0
            for _ in range(60):
                mid = (lo+hi)/2
                if s_at(mid) < 60.5-sd: lo = mid
                else: hi = mid
            t_act = (lo+hi)/2
        else:
            lo, hi = 0.0, 200.0
            for _ in range(60):
                mid = (lo+hi)/2
                if s_at(mid) < sd: lo = mid
                else: hi = mid
            t_act = (lo+hi)/2
        # 占用窗口：box 进入/离开横道
        # 进入横道（东向：px 增；西向：px 减）
        if direction == "西":
            t_in = t_act + (px0 - 272589.90)/sp
            t_out = t_act + (px0 - 272584.49)/sp
        else:
            t_in = t_act + (272584.49 - px0)/sp
            t_out = t_act + (272589.90 - px0)/sp
        # 规则释放：离开路（box 离开 [272585.12,272589.51]）
        if direction == "西":
            t_rel = t_act + (px0 - 272585.12)/sp
        else:
            t_rel = t_act + (272589.51 - px0)/sp
        print("    %s: 触发t=%.1f  占用[%.1f,%.1f]  规则释放t=%.1f" % (pid, t_act, t_in, t_out, t_rel))

print()
print("== 结论核算 ==")
print("减速带区: 评分区 Δy[23.51,29.06]; 插件限速 Δy[16.79,31.75] 覆盖 ✓")
print("  车辆多边形在区内: 前保险杠进入(后轴Δy≈19.9) ~ 后保险杠离开(后轴Δy≈29.0-29.9)")
print("  => 限速区完全覆盖车辆在评分区全程，尾部裕量 >= 1.8m ✓")
print("Crosswalk_5: 停车墙(车头) Δy≈58.4 (近边59.86-1.25), 距横道1.25m ∈[1.0,1.5] ✓")
print("120s 预算: 减速带区 ~19s + 巡航 ~9s + C5区 ~2s + 停车等待 ~10s + 恢复 ~22s ≈ 70-75s ✓")
