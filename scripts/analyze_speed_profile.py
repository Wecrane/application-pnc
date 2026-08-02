#!/usr/bin/env python3
"""解析 planning.log.INFO 的 planning_start_point，提取速度/加速度曲线，分析加速/巡航/刹车三段。
用法: python3 scripts/analyze_speed_profile.py <logfile>
"""
import re
import sys
import math

LOG = sys.argv[1] if len(sys.argv) > 1 else 'data/log/planning.log.INFO.20260803-012349.243978'

pat = re.compile(
    r'I\d{4} (\d\d):(\d\d):(\d\d)\.(\d{6}) .*?planning_start_point x,y,the,k: '
    r'([\d.\-]+), y: ([\d.\-]+),([\d.\-]+)')

frames = []  # (t, x, y)
for line in open(LOG, errors='ignore'):
    m = pat.search(line)
    if m:
        h, mi, s, us = int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))
        t = h * 3600 + mi * 60 + s + us / 1e6
        x, y = float(m.group(5)), float(m.group(6))
        frames.append((t, x, y))

if not frames:
    print('no frames'); sys.exit(1)

# 累计路径距离
dist = [0.0]
for i in range(1, len(frames)):
    dx = frames[i][1] - frames[i - 1][1]
    dy = frames[i][2] - frames[i - 1][2]
    dist.append(dist[-1] + math.hypot(dx, dy))

# 速度（前向差分）与加速度（对速度差分）
ts = [f[0] for f in frames]
v = [0.0] * len(frames)
a = [0.0] * len(frames)
for i in range(1, len(frames)):
    dt = ts[i] - ts[i - 1]
    if dt > 0:
        v[i] = (dist[i] - dist[i - 1]) / dt
for i in range(2, len(frames)):
    dt = ts[i] - ts[i - 2]
    if dt > 0:
        a[i] = (v[i] - v[i - 2]) / dt

print(f'帧数={len(frames)} 时长={ts[-1]-ts[0]:.1f}s 总距离={dist[-1]:.1f}m')
print(f'起点=({frames[0][1]:.2f},{frames[0][2]:.2f}) 终点=({frames[-1][1]:.2f},{frames[-1][2]:.2f})')
print()

# ---- 分段分析 ----
# 停止帧（速度<0.1 的最后一个）
stop_idx = None
for i in range(len(frames) - 1, -1, -1):
    if v[i] < 0.1:
        stop_idx = i
    else:
        break
stop_s = dist[stop_idx] if stop_idx else None
print(f'停车位置: s={stop_s:.1f}m (idx={stop_idx}) 车停在 ({frames[stop_idx][1]:.2f},{frames[stop_idx][2]:.2f})')

# 加速段: 起点到最大速度之前, v 持续上升
vmax = max(v)
vmax_idx = v.index(vmax)
print(f'最大速度 vmax={vmax:.2f} m/s @ t={ts[vmax_idx]-ts[0]:.1f}s s={dist[vmax_idx]:.1f}m')

# 加速段(0->90% vmax)
target = 0.9 * vmax
acc_idx = None
for i in range(1, vmax_idx):
    if v[i] >= target:
        acc_idx = i
        break
if acc_idx:
    dt = ts[acc_idx] - ts[0]
    print(f'加速段: 0 -> {target:.1f}m/s 用时 {dt:.1f}s, 平均加速度={v[acc_idx]/dt:.2f} m/s², 距离={dist[acc_idx]:.1f}m')
    # 加速段峰值加速度
    acc_peak = max(a[1:acc_idx + 1])
    print(f'  加速段峰值加速度 a_max={acc_peak:.2f} m/s²')

# 巡航段: 90%vmax 持续段
cruise_start = None
cruise_end = None
for i in range(len(frames)):
    if v[i] >= 0.9 * vmax:
        if cruise_start is None:
            cruise_start = i
        cruise_end = i
    else:
        if cruise_start is not None and cruise_end - cruise_start > 5:
            break
        cruise_start = None
if cruise_start is not None and cruise_end is not None and cruise_end - cruise_start > 5:
    print(f'巡航段: s={dist[cruise_start]:.1f}~{dist[cruise_end]:.1f}m, 速度≈{vmax:.1f}m/s, 持续 {ts[cruise_end]-ts[cruise_start]:.1f}s')

# 刹车段: vmax 之后到停止
if stop_idx and vmax_idx < stop_idx:
    dec = a[vmax_idx:stop_idx + 1]
    dec_min = min(dec)
    dec_min_idx = vmax_idx + dec.index(dec_min)
    # 刹车起点: 从 vmax 后第一个 a<0 的帧
    brake_start = None
    for i in range(vmax_idx, stop_idx):
        if a[i] < -0.3:
            brake_start = i
            break
    if brake_start:
        brake_v = v[brake_start]
        brake_dist = dist[stop_idx] - dist[brake_start]
        print(f'刹车段: 开始于 s={dist[brake_start]:.1f}m (v={brake_v:.1f}m/s), 停止 s={dist[stop_idx]:.1f}m')
        print(f'  刹车距离={brake_dist:.1f}m, 用时={ts[stop_idx]-ts[brake_start]:.1f}s')
        print(f'  刹车段峰值减速度 a_min={dec_min:.2f} m/s² @ s={dist[dec_min_idx]:.1f}m')
        # 理论最晚刹车点: v²/(2*6)
        if brake_v > 0:
            late = dist[stop_idx] - brake_v * brake_v / (2 * 6.0)
            print(f'  理论最晚刹车点(6m/s²): s={late:.1f}m (即距离停车点 {brake_v*brake_v/(2*6):.1f}m 处)')
            print(f'  实际刹车点比最晚刹车点早 {brake_dist - brake_v*brake_v/(2*6):.1f}m')

print()
print('--- 分段速度/加速度表 (每1s采样) ---')
print(f'{"t(s)":>6} {"s(m)":>8} {"v(m/s)":>8} {"a(m/s²)":>8}')
last = -1
for i in range(len(frames)):
    t_rel = ts[i] - ts[0]
    if int(t_rel) != last:
        last = int(t_rel)
        print(f'{t_rel:6.1f} {dist[i]:8.1f} {v[i]:8.2f} {a[i]:8.2f}')

print()
print('--- 关键事件(每3s, 含位置) ---')
last = -1
for i in range(len(frames)):
    t_rel = ts[i] - ts[0]
    if int(t_rel) % 3 == 0 and int(t_rel) != last:
        last = int(t_rel)
        print(f't={t_rel:5.1f}s s={dist[i]:7.1f}m v={v[i]:6.2f} a={a[i]:6.2f} pos=({frames[i][1]:.1f},{frames[i][2]:.1f})')
