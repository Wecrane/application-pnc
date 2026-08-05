#!/usr/bin/env python3
"""赛题7 (scn7) 云端日志分析脚本 - 只读分析"""
import re, json, sys

BASE = '/home/skye/application-pnc/output/scn7_analysis'
OUT = []

def log(*a):
    s = ' '.join(str(x) for x in a)
    OUT.append(s)
    print(s)

# ============ 1. sim_engine.log 评分解析 ============
log('='*70)
log('## 1. sim_engine.log 评分解析')
data = open(f'{BASE}/sim_engine.log', errors='ignore').read()

# 找所有 metric_result 块
blocks = re.findall(r'metric_result \{.*?\n\}', data, re.S)
for b in blocks:
    m = re.search(r'name: "(\w+)"', b)
    if not m: continue
    name = m.group(1)
    desc = re.search(r'description: "([^"]*)"', b)
    isp = re.search(r'is_metric_pass: (\w+)', b)
    ms = re.search(r'metric_score: ([\d.]+)', b)
    log(f'\n--- {name} ---')
    log(f'  desc: {desc.group(1) if desc else ""}')
    log(f'  is_metric_pass: {isp.group(1) if isp else ""}  metric_score: {ms.group(1) if ms else ""}')
    # 失败帧
    fails = []
    # 逐帧解析
    for fm in re.finditer(r'detailed_result \{(.*?)\n  \}', b, re.S):
        seg = fm.group(1)
        ts_m = re.search(r'timestamp: ([\d.eE+-]+)', seg)
        if not ts_m: continue
        ts = float(ts_m.group(1))
        if 'is_pass: false' in seg:
            dbg = re.search(r'description: "([^"]*)"', seg)
            dscore = re.search(r'delta_score: ([\d.]+)', seg)
            fails.append((ts, dbg.group(1) if dbg else '', dscore.group(1) if dscore else ''))
    for ts, d, ds in fails:
        log(f'  FAIL t={ts:.2f}  delta_score={ds}  desc={d}')

# 最终 score 行
score_lines = [l for l in data.split('\n') if '"score"' in l and 'updateGrading' in l]
if score_lines:
    log(f'\n最终评分: {score_lines[-1][:200]}')

# ============ 2. replay-engine.log 场景配置 ============
log('='*70)
log('## 2. replay-engine.log 场景配置')
rd = open(f'{BASE}/replay-engine.log', errors='ignore').read()
log(f'replay-engine.log 行数: {len(rd.splitlines())}')

# 找人行道/行人/限速相关配置
for kw in ['crosswalk', 'Crosswalk', 'pedestrian', 'speed_limit', 'speed_limit_regions', 'scenario', 'map_path', 'region']:
    lines = [l for l in rd.split('\n') if kw in l]
    log(f'\n关键词 [{kw}] 命中 {len(lines)} 行')
    for l in lines[:15]:
        log(f'  {l.strip()[:250]}')

# ============ 3. planning.log 车轨迹分析 ============
log('='*70)
log('## 3. planning.log 车轨迹分析')
pd = open(f'{BASE}/planning.log', errors='ignore').read()
log(f'planning.log 行数: {len(pd.splitlines())}')

def w2s(w):
    try:
        h, m, s = w.split(':')
        return int(h)*3600 + int(m)*60 + float(s)
    except:
        return None

# planning_start_point
rows = []
for line in pd.split('\n'):
    if 'planning_start_point' in line:
        m = re.search(r'(\d\d:\d\d:\d\d\.\d+)', line)
        if m:
            nums = re.findall(r'[-+]?\d+\.\d+', line.split('x,y,the,k:')[-1]) if 'x,y,the,k:' in line else re.findall(r'[-+]?\d+\.\d+', line)
            if len(nums) >= 6:
                rows.append((w2s(m.group(1)), float(nums[0]), float(nums[1]), float(nums[4]), float(nums[5])))
if rows:
    t0 = rows[0][0]
    log(f'\nplanning_start_point 帧数: {len(rows)}, 起始 wall={t0:.2f}')
    # 每隔若干帧采样打印
    step = max(1, len(rows)//40)
    log(f'时间轴(相对t0) | x | y | v(m/s) | v(km/h) | a')
    prev = None
    for i, (t, x, y, v, a) in enumerate(rows):
        if i % step == 0 or i == len(rows)-1:
            log(f'  {t-t0:8.2f}s | {x:.2f} | {y:.2f} | {v:6.2f} | {v*3.6:6.1f} | {a:6.2f}')
    # 最大速度
    maxv = max(r[3] for r in rows)
    log(f'\n最大速度: {maxv:.2f} m/s = {maxv*3.6:.1f} km/h')
else:
    log('  planning_start_point 未找到!')

# 找 Crosswalk 相关日志
cw_lines = [l for l in pd.split('\n') if 'Crosswalk' in l or 'crosswalk' in l]
log(f'\nCrosswalk/crosswalk 日志行数: {len(cw_lines)}')
for l in cw_lines[:10]:
    log(f'  {l.strip()[:250]}')

# 找 STOP fence / stop_s
stop_lines = [l for l in pd.split('\n') if 'stop_s:' in l or 'BuildStopDecision' in l]
log(f'\nstop_s/BuildStopDecision 行数: {len(stop_lines)}')
for l in stop_lines[:20]:
    log(f'  {l.strip()[:250]}')

# 找限速
spd_lines = [l for l in pd.split('\n') if 'print_curr_speed_limit' in l or 'speed_limit' in l.lower()]
log(f'\n限速相关行数: {len(spd_lines)}')
for l in spd_lines[:10]:
    log(f'  {l.strip()[:250]}')

# 行人
ped_lines = [l for l in pd.split('\n') if 'pedestrian' in l.lower() or '3060' in l or 'PEDESTRIAN' in l]
log(f'\n行人相关行数: {len(ped_lines)}')
for l in ped_lines[:10]:
    log(f'  {l.strip()[:250]}')

with open(f'{BASE}/analysis_result.txt', 'w') as f:
    f.write('\n'.join(OUT))
log('\n\n=== 结果已写入 analysis_result.txt ===')
