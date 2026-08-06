#!/usr/bin/env python3
# 实车 7pm 深度分析：完整时间线 + 停车距离 + 放行时机
import re, sys
LOG = sys.argv[1]
START_Y = 3476108.50
FRONT = 3.62  # front_edge_to_center

def w2s(w):
    h, m, s = w.split(':')
    return int(h)*3600 + int(m)*60 + float(s)

rows = []
for line in open(LOG, errors='ignore'):
    if 'planning_start_point' in line:
        mm = re.search(r'planning_start_point x,y,the,k: (.*)', line)
        if mm:
            nums = [float(x) for x in re.findall(r'[-+]?\d+\.\d+', mm.group(1))]
            m = re.search(r'(\d\d:\d\d:\d\d\.\d+)', line)
            if m and len(nums) >= 6:
                # nums: x,y,theta,k,v,a
                rows.append((w2s(m.group(1)), nums[1], nums[1]-START_Y, nums[4], nums[5], nums[2]))
if not rows: print("无轨迹"); sys.exit(0)
t0 = rows[0][0]

# 找第一段完整运行（从速度>0开始到终点停车）
# 打印时间线每2s
print(f"=== {LOG.split('/')[-1]} 轨迹 (每3s) ===")
print(f"{'t':>7} {'Δy(后轴)':>9} {'车头Δy':>9} {'v':>6} {'a':>6}")
for i, (t, y, dy, v, a, th) in enumerate(rows):
    rel = t - t0
    if rel >= 0 and (int(rel*10) % 30 == 0 or i == len(rows)-1):
        head = dy + FRONT
        print(f"{rel:7.1f} {dy:9.2f} {head:9.2f} {v:6.2f} {a:+6.2f}")

# 停车点（v<0.1 持续1s），含车头距横道
print("\n=== 停车点 (含车头距 Crosswalk_5 前缘 59.64m) ===")
CW5_FRONT = 59.64
i = 0
stops = []
while i < len(rows):
    if rows[i][3] < 0.1:
        j = i
        while j < len(rows) and rows[j][3] < 0.1:
            j += 1
        if rows[j-1][0]-rows[i][0] >= 1.0:
            t, y, dy, v, a, th = rows[i]
            head = dy + FRONT
            dist = CW5_FRONT - head  # 车头距横道前缘（正=在横道前）
            stops.append((t-t0, rows[j-1][0]-t0, dy, head, dist))
            print(f"  停 t={t-t0:6.1f}-{rows[j-1][0]-t0:6.1f}s ({rows[j-1][0]-t0-(t-t0):.1f}s) 后轴Δy={dy:6.1f} 车头Δy={head:6.1f} 距横道前缘={dist:+.2f}m")
        i = j
    else:
        i += 1

# Crosswalk_5 停车前后各 5s 的详细轨迹（放行时机验证）
print("\n=== Crosswalk_5 附近停车-放行详细 (Δy 50-70) ===")
for t, y, dy, v, a, th in rows:
    rel = t-t0
    if 50 <= dy <= 70:
        head = dy + FRONT
        print(f"  t={rel:7.1f} 后轴Δy={dy:6.2f} 车头Δy={head:6.2f} v={v:5.2f} a={a:+5.2f}")
