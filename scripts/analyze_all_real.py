#!/usr/bin/env python3
# 批量分析所有实车日志：统一指标对比
import re, glob

START_Y = 3476108.50

def w2s(w):
    h, m, s = w.split(':')
    return int(h)*3600 + int(m)*60 + float(s)

files = sorted(glob.glob('shiche/car_890/selected_logs/planning.log.INFO.*') +
               glob.glob('shiche/car_890/replay_logs/log/planning.log.INFO.*') +
               glob.glob('shiche/car_890/scripts_biglogs/data/log/planning.log.INFO.*'))
files = list(dict.fromkeys(files))  # 去重

print(f"{'日志':<40} {'帧数':>6} {'时长s':>6} {'maxV':>6} {'超速帧':>6} {'急刹帧':>6} {'减速带maxV':>9} {'CW5停Δy':>10}")
for f in files:
    rows = []
    maxv = 0; over_cnt = 0; hard_cnt = 0; zone_max = 0; cw5_stop = None
    for line in open(f, errors='ignore'):
        if 'planning_start_point' in line:
            mm = re.search(r'planning_start_point x,y,the,k: (.*)', line)
            if mm:
                nums = [float(x) for x in re.findall(r'[-+]?\d+\.\d+', mm.group(1))]
                m = re.search(r'(\d\d:\d\d:\d\d\.\d+)', line)
                if m and len(nums) >= 6:
                    rows.append((w2s(m.group(1)), nums[1], nums[1]-START_Y, nums[4], nums[5]))
    if not rows: continue
    t0 = rows[0][0]
    for t, y, dy, v, a in rows:
        if v > maxv: maxv = v
        if v > 3.0556: over_cnt += 1
        if a < -3.0 or a > 2.0: hard_cnt += 1
        if 23.5 <= dy <= 29.1 and v > zone_max: zone_max = v
    # Crosswalk_5 前停车（Δy 55-62 内 v<0.1）
    i = 0
    while i < len(rows):
        if rows[i][3] < 0.1 and 55 <= rows[i][2] <= 62:
            j = i
            while j < len(rows) and rows[j][3] < 0.1:
                j += 1
            if rows[j-1][0]-rows[i][0] >= 0.8:
                cw5_stop = f"{rows[i][2]:.1f}m/{rows[j-1][0]-rows[i][0]:.1f}s"
                break
        i += 1
    dur = rows[-1][0]-t0
    name = f.split('/')[-1].replace('planning.log.INFO.','')
    print(f"{name:<40} {len(rows):>6} {dur:>6.0f} {maxv:>6.2f} {over_cnt:>6} {hard_cnt:>6} {zone_max:>9.2f} {cw5_stop or '-':>10}")
