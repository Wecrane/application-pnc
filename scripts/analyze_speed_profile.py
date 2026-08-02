#!/usr/bin/env python3
"""解析 planning.log.INFO 的 planning_start_point，提取速度/加速度曲线，分析加速/巡航/刹车三段。
用法: python3 scripts/analyze_speed_profile.py <logfile>
"""
import re
import sys
import math

LOG = sys.argv[1] if len(sys.argv) > 1 else 'data/log/planning.log.INFO.20260803-014016.264810'

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

ts = [f[0] for f in frames]
# 累计路径距离
dist = [0.0]
for i in range(1, len(frames)):
    dist.append(dist[-1] + math.hypot(frames[i][1] - frames[i - 1][1],
                                      frames[i][2] - frames[i - 1][2]))

# 平滑差分速度（±2帧=0.2s 窗口）与加速度（±2帧）
def v_at(i, w=2):
    j0 = max(0, i - w); j1 = min(len(frames) - 1, i + w)
    dt = ts[j1] - ts[j0]
    return (dist[j1] - dist[j0]) / dt if dt > 0 else 0.0

v = [v_at(i) for i in range(len(frames))]

def a_at(i, w=2):
    j0 = max(0, i - w); j1 = min(len(frames) - 1, i + w)
    dt = ts[j1] - ts[j0]
    return (v_at(j1) - v_at(j0)) / dt if dt > 0 else 0.0

a = [a_at(i) for i in range(len(frames))]

print(f'帧数={len(frames)} 时长={ts[-1]-ts[0]:.1f}s 总距离={dist[-1]:.1f}m')
print(f'起点=({frames[0][1]:.2f},{frames[0][2]:.2f}) 终点=({frames[-1][1]:.2f},{frames[-1][2]:.2f})')
print()

# ---- 分段分析 ----
# 停止帧（最后一段速度<0.1）
stop_idx = None
for i in range(len(frames) - 1, -1, -1):
    if v[i] < 0.1:
        stop_idx = i
    else:
        break
if stop_idx:
    print(f'停车位置: s={dist[stop_idx]:.1f}m 车停在 ({frames[stop_idx][1]:.2f},{frames[stop_idx][2]:.2f})')

# 全局最大速度
vmax_v, vmax_i = max((v[i], i) for i in range(len(frames)))
print(f'最大速度 vmax={vmax_v:.2f} m/s @ t={ts[vmax_i]-ts[0]:.1f}s s={dist[vmax_i]:.1f}m')

# 分段：找所有加速段(a>+0.5) 和刹车段(a<-0.5)
def segments(cond):
    segs = []
    cur = None
    for i in range(len(frames)):
        if cond(a[i]):
            if cur is None:
                cur = [i, i]
            cur[1] = i
        else:
            if cur:
                segs.append(cur); cur = None
    if cur:
        segs.append(cur)
    return segs

acc_segs = segments(lambda x: x > 0.5)
dec_segs = segments(lambda x: x < -0.5)

print()
print('--- 加速段 (a>+0.5) ---')
for s_, e_ in acc_segs:
    if e_ - s_ < 2:
        continue
    dv = v[e_] - v[s_]
    dur = ts[e_] - ts[s_]
    peak = max(a[s_:e_ + 1])
    print(f'  t={ts[s_]-ts[0]:5.1f}~{ts[e_]-ts[0]:5.1f}s s={dist[s_]:6.1f}~{dist[e_]:6.1f}m '
          f'v={v[s_]:5.2f}->{v[e_]:5.2f} 平均a={dv/dur:.2f} 峰值a={peak:.2f}')

print()
print('--- 刹车段 (a<-0.5) ---')
for s_, e_ in dec_segs:
    if e_ - s_ < 2:
        continue
    dv = v[e_] - v[s_]
    dur = ts[e_] - ts[s_]
    peak = min(a[s_:e_ + 1])
    print(f'  t={ts[s_]-ts[0]:5.1f}~{ts[e_]-ts[0]:5.1f}s s={dist[s_]:6.1f}~{dist[e_]:6.1f}m '
          f'v={v[s_]:5.2f}->{v[e_]:5.2f} 平均a={dv/dur:.2f} 峰值a={peak:.2f}')

print()
print('--- 速度/加速度表 (每1s采样) ---')
print(f'{"t(s)":>6} {"s(m)":>8} {"v(m/s)":>8} {"a(m/s²)":>8}')
last = -1
for i in range(len(frames)):
    t_rel = ts[i] - ts[0]
    if int(t_rel) != last:
        last = int(t_rel)
        print(f'{t_rel:6.1f} {dist[i]:8.1f} {v[i]:8.2f} {a[i]:8.2f}')
