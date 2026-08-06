#!/usr/bin/env python3
# 7号下午 planning 日志深度分析
import re, sys
from collections import defaultdict

LOG = sys.argv[1] if len(sys.argv) > 1 else 'data/log/planning.log.INFO.20260805-183747.28213'
START_Y = 3476108.50

def w2s(w):
    h, m, s = w.split(':')
    return int(h)*3600 + int(m)*60 + float(s)

rows = []          # (t, x, y, dy, v, a)
stop_events = []   # (t, stop_s, end_s)
speedlimits = []   # (t, line)
for line in open(LOG, errors='ignore'):
    m = re.search(r'(\d\d:\d\d:\d\d\.\d+)', line)
    if 'planning_start_point' in line:
        mm = re.search(r'planning_start_point x,y,the,k: (.*)', line)
        if mm:
            nums = [float(x) for x in re.findall(r'[-+]?\d+\.\d+', mm.group(1))]
            if m and len(nums) >= 6:
                rows.append((w2s(m.group(1)), nums[0], nums[1], nums[1]-START_Y, nums[4], nums[5]))
    if 'stop_s:' in line and m:
        sm = re.search(r'stop_s: ([\d\.]+) end_s: ([\d\.]+)', line)
        if sm:
            stop_events.append((w2s(m.group(1)), float(sm.group(1)), float(sm.group(2))))
    if re.search(r'AddSpeedLimit|SpeedLimit', line) and m:
        speedlimits.append((w2s(m.group(1)), line.strip()[:120]))

if not rows:
    print("无 planning_start_point 记录")
    sys.exit(0)

t0 = rows[0][0]
print(f"=== 轨迹概览: {len(rows)} 帧, 时长 {(rows[-1][0]-t0):.1f}s ===")
print(f"起点: x={rows[0][1]:.2f} y={rows[0][2]:.2f}")
print(f"终点: x={rows[-1][1]:.2f} y={rows[-1][2]:.2f} Δy={rows[-1][3]:.1f}m")
print(f"最大速度: {max(r[4] for r in rows):.3f} m/s (t={rows[[r[4] for r in rows].index(max(r[4] for r in rows))][0]-t0:.1f}s)")
print(f"最大加速度: {max(r[5] for r in rows):.2f} / 最小: {min(r[5] for r in rows):.2f} m/s²")

# 速度剖面：每 5s 采样
print("\n=== 速度剖面 (每2s) ===")
prev_i = 0
for i, r in enumerate(rows):
    if r[0]-t0 >= prev_i*2 or i == len(rows)-1:
        print(f"t={r[0]-t0:6.1f}s  Δy={r[3]:6.1f}m  v={r[4]:.2f}  a={r[5]:+.2f}")
        prev_i += 1

# STOP 事件
print(f"\n=== STOP 事件 ({len(stop_events)}) — 每2s采样 ===")
seen = set()
for t, s, e in stop_events:
    k = int(t-t0)//2
    if k not in seen and t-t0 > 3:
        seen.add(k)
        print(f"t={t-t0:6.1f}s  stop_s={s:8.1f}  end_s={e:8.1f}")

# 限速
print(f"\n=== SpeedLimit 事件 ({len(speedlimits)}) — 前10条 ===")
for t, l in speedlimits[:10]:
    print(f"t={t-t0:6.1f}s  {l}")

# 减速带区/横道关键位置速度
print("\n=== 关键区域速度 ===")
for lo, hi, name in [(16.8,31.8,'减速带区'), (51.6,66.8,'Crosswalk_5区'), (0,103,'全程')]:
    seg = [r for r in rows if lo <= r[3] <= hi]
    if seg:
        print(f"{name} Δy[{lo},{hi}]: 帧数={len(seg)} 最大v={max(r[4] for r in seg):.2f} 平均v={sum(r[4] for r in seg)/len(seg):.2f}")
