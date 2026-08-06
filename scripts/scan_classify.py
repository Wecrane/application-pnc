#!/usr/bin/env python3
# 全面扫描所有 planning.log，分类场次（7pm/6am/6pm/7am/其他）
import re, glob, os

START_Y = 3476108.50
SZ_X = (272589.0, 272591.5)   # 苏州图 x 范围
SZ_Y = (3476105.0, 3476110.0) # 苏州图起点 y 范围

def w2s(w):
    h, m, s = w.split(':')
    return int(h)*3600 + int(m)*60 + float(s)

files = sorted(glob.glob('data/log/planning.log.INFO.*') +
               glob.glob('output/cloud_0*/**/planning.log.INFO.*', recursive=True) +
               glob.glob('shiche/car_890/selected_logs/planning.log.INFO.*') +
               glob.glob('shiche/car_890/replay_logs/log/planning.log.INFO.*') +
               glob.glob('shiche/car_890/scripts_biglogs/data/log/planning.log.INFO.*'))

print(f"{'日志':<46} {'帧':>6} {'秒':>6} {'起点x':>11} {'起点y':>11} {'场次':>6}")
for f in files:
    name = f.split('/')[-1].replace('planning.log.INFO.','')
    start = None; n = 0; dur = 0
    has_signal_stop = 0   # 红灯 STOP
    cw5_zone_maxv = 0     # Crosswalk_5 区 (55-62) 速度
    sb_zone_maxv = 0      # 减速带区 (23.5-29.1) 速度
    maxv = 0
    for line in open(f, errors='ignore'):
        if 'planning_start_point' in line:
            mm = re.search(r'planning_start_point x,y,the,k: (.*)', line)
            if not mm: continue
            nums = [float(x) for x in re.findall(r'[-+]?\d+\.\d+', mm.group(1))]
            m = re.search(r'(\d\d:\d\d:\d\d\.\d+)', line)
            if m and len(nums) >= 6:
                t = w2s(m.group(1)); x, y = nums[0], nums[1]; dy = y - START_Y
                if start is None: start = (x, y)
                if n == 0: t0 = t
                n += 1; dur = t - t0
                if nums[4] > maxv: maxv = nums[4]
                if 23.5 <= dy <= 29.1 and nums[4] > sb_zone_maxv: sb_zone_maxv = nums[4]
                if 55.0 <= dy <= 62.0 and nums[4] > cw5_zone_maxv: cw5_zone_maxv = nums[4]
        if 'STOP_REASON_SIGNAL' in line or 'red_light' in line.lower():
            has_signal_stop += 1

    if start is None:
        print(f"{name:<46} {'—':>6} {'—':>6} 无轨迹")
        continue
    sx, sy = start
    is_sz = SZ_X[0] <= sx <= SZ_X[1] and SZ_Y[0] <= sy <= SZ_Y[1]
    if not is_sz:
        print(f"{name:<46} {n:>6} {dur:>6.0f} {sx:>11.2f} {sy:>11.2f} {'其他':>6}")
        continue
    # 苏州图：判定场次
    if has_signal_stop > 5:
        field = '6pm/7am(红等)'
    elif cw5_zone_maxv > 0.5 and sb_zone_maxv > 2.0:
        field = '7pm(限速失效)'
    elif cw5_zone_maxv > 0.5:
        field = '6am/7pm(停车)'
    elif sb_zone_maxv < 1.0:
        field = '7pm/7am(限速区)'
    else:
        field = '苏州?'
    print(f"{name:<46} {n:>6} {dur:>6.0f} {sx:>11.2f} {sy:>11.2f} {field:>6}  减速带区maxV={sb_zone_maxv:.2f} CW5区maxV={cw5_zone_maxv:.2f} 总maxV={maxv:.2f} 信号STOP={has_signal_stop}")
