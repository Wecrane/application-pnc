#!/usr/bin/env python3
"""017轮 8赛题 速度优化分析脚本
提取每个赛题 planning 日志的: 速度曲线/加速度/停车段/起步加速度
"""
import re, glob, os, json

BASE = '/home/skye/下载/017/extracted'
RESULT = {}

def parse_planning(logpath):
    rows = []
    for line in open(logpath, errors='ignore'):
        if 'planning_start_point' not in line:
            continue
        m = re.search(r'(\d\d):(\d\d):(\d\d)\.(\d+)', line)
        if not m:
            continue
        h, mi, s, us = map(int, m.groups())
        w = h*3600 + mi*60 + s + us/1e6
        nums = re.findall(r'[-+]?\d+\.\d+', line.split('x,y,the,k:')[-1])
        if len(nums) >= 6:
            rows.append((w, float(nums[0]), float(nums[4]), float(nums[5])))  # wall, x, v, a
    return rows

def analyze(rows):
    if not rows:
        return None
    T0 = rows[0][0]
    # 相对时间
    data = [(r[0]-T0, r[1], r[2], r[3]) for r in rows]
    dur = data[-1][0]
    vs = [r[2] for r in data]
    accs = [r[3] for r in data]
    max_v = max(vs)
    max_a = max(accs)
    min_a = min(accs)
    # 停车段: v<0.05 连续>0.5s
    stops = []
    in_stop = False
    for i, r in enumerate(data):
        if r[2] < 0.05:
            if not in_stop:
                s0 = r[0]
                in_stop = True
        else:
            if in_stop:
                if r[0] - s0 > 0.5:
                    stops.append((s0, r[0], r[0]-s0))
                in_stop = False
    if in_stop and data[-1][0] - s0 > 0.5:
        stops.append((s0, data[-1][0], data[-1][0]-s0))
    # 起步段: 前5s 速度/加速度 (起步开始到第一个v>1)
    start_a = []
    for r in data:
        if r[0] < 6:
            start_a.append(r[3])
    # 起步耗时: 从0到巡航(16) 或到第一个高峰
    t_to_cruise = None
    for r in data:
        if r[2] >= 15.5:
            t_to_cruise = r[0]
            break
    # 最大加速度时刻
    amax_t = data[accs.index(max_a)][0]
    return {
        'duration': round(dur, 1),
        'max_v': round(max_v, 2),
        'max_a': round(max_a, 2),
        'min_a': round(min_a, 2),
        'amax_t': round(amax_t, 1),
        'stops': [(round(s,1), round(e,1), round(d,1)) for s,e,d in stops],
        'start_a_max': round(max(start_a), 2) if start_a else None,
        't_to_15.5': round(t_to_cruise, 1) if t_to_cruise else None,
        'data_len': len(data),
    }

for d in sorted(glob.glob(os.path.join(BASE, '*/'))):
    name = os.path.basename(d.rstrip('/'))
    pls = glob.glob(os.path.join(d, 'mods/planning.log.INFO*'))
    if not pls:
        print(f'{name}: 无planning日志')
        continue
    rows = parse_planning(pls[0])
    res = analyze(rows)
    RESULT[name] = res
    # 打印摘要
    print(f'=== {name} ===')
    if res:
        print(f'  时长={res["duration"]}s 最大v={res["max_v"]} 最大a={res["max_a"]} 最小a={res["min_a"]}')
        print(f'  起步a_max={res["start_a_max"]} 到15.5m/s耗时={res["t_to_15.5"]}s 最大a时刻={res["amax_t"]}s')
        print(f'  停车段: {res["stops"]}')
    print()

with open(os.path.join(BASE, 'speed_summary.json'), 'w') as f:
    json.dump(RESULT, f, indent=2, ensure_ascii=False)
print('已保存 speed_summary.json')
