#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析云端障碍物停车避让回放：自车-行人7673距离、决策、停车点。

用法: python3 analyze_cloud_ped.py <decoded.json> [行人id]
"""
import json
import sys
import math

src = sys.argv[1] if len(sys.argv) > 1 else 'output/cloud_replay_decoded.json'
PED = sys.argv[2] if len(sys.argv) > 2 else '7673'

frames = json.load(open(src))
print('frames:', len(frames))

# 找行人轨迹和自车
def get_ped(f):
    for o in f.get('object', []):
        if o.get('id') == PED:
            return o
    return None

def get_adc(f):
    return f.get('autoDrivingCar', {})

rows = []
prev_ped = None
for f in frames:
    t = f.get('timestamp', 0) / 1000.0  # 秒
    adc = get_adc(f)
    ped = get_ped(f)
    adc_x, adc_y = adc.get('positionX', 0), adc.get('positionY', 0)
    adc_v = adc.get('speed', 0)
    ped_x = ped_y = ped_v = None
    if ped:
        ped_x = ped.get('positionX')
        ped_y = ped.get('positionY')
        ped_v = ped.get('speed', 0)
    # 距离（中心）
    dist = math.hypot(adc_x - ped_x, adc_y - ped_y) if ped else None
    # 横向距离（y 差 = 行人相对自车的横向偏移）
    lat = (ped_y - adc_y) if ped else None
    # 纵向距离（x 差 = 行人相对自车前方距离）
    lon = (ped_x - adc_x) if ped else None
    # 决策
    decisions = []
    if ped:
        for d in ped.get('decision', []):
            decisions.append(d.get('type'))
    rows.append((t, adc_x, adc_v, ped_x, ped_y, ped_v, dist, lat, lon, decisions, ped))

# 输出关键时间线
print('\n=== 关键时间线（每秒采样）===')
print('%-7s %-11s %-6s %-11s %-8s %-7s %-8s %-7s %-7s %s' % (
    't(s)', 'ADC_x', 'ADC_v', 'PED_x', 'PED_v', 'dist', 'lat(y)', 'lon(x)', 'dec', 'PED_y'))
for i in range(0, len(rows), 10):
    t, adc_x, adc_v, ped_x, ped_y, ped_v, dist, lat, lon, dec, _ = rows[i]
    if t > 100:
        break
    print('%-7.1f %-11.2f %-6.2f %-11.2f %-8.2f %-7.2f %-8.2f %-7.2f %s %s' % (
        t, adc_x, adc_v, ped_x, ped_v, dist if dist else -1,
        lat if lat else -1, lon if lon else -1, ','.join(dec) if dec else '-', ped_y if ped_y else ''))

# 找关键事件
print('\n=== 关键事件 ===')
# 停车瞬间（ADC 速度首次 <0.1）
stop_t = None
for t, adc_x, adc_v, ped_x, ped_y, ped_v, dist, lat, lon, dec, _ in rows:
    if adc_v < 0.1 and adc_x > 423400:
        stop_t = t
        break
print('首次停车时间: t=%.1fs (x=%.2f)' % (stop_t, adc_x))

# 行人起步（速度首次 >0.1）
for t, adc_x, adc_v, ped_x, ped_y, ped_v, dist, lat, lon, dec, _ in rows:
    if ped_v and ped_v > 0.1:
        print('行人起步: t=%.1fs (x=%.2f y=%.2f v=%.2f) 自车x=%.2f' % (t, ped_x, ped_y, ped_v, adc_x))
        break

# 自车起步（停后速度再次 >0.5）
started = False
for t, adc_x, adc_v, ped_x, ped_y, ped_v, dist, lat, lon, dec, _ in rows:
    if stop_t and t > stop_t + 0.5 and adc_v > 0.5:
        print('自车起步: t=%.1fs (x=%.2f v=%.2f) 行人x=%.2f y=%.2f v=%.2f dist=%.2f lat=%.2f lon=%.2f' % (
            t, adc_x, adc_v, ped_x, ped_y, ped_v, dist, lat, lon))
        started = True
        break

# 停车距离（停车瞬间 自车-行人）
for t, adc_x, adc_v, ped_x, ped_y, ped_v, dist, lat, lon, dec, _ in rows:
    if stop_t and abs(t - stop_t) < 0.15:
        print('停车瞬间: t=%.1fs ADC(%.2f,%.2f) PED(%.2f,%.2f) dist=%.2f lat=%.2f lon=%.2f' % (
            t, adc_x, rows[rows.index((t, adc_x, adc_v, ped_x, ped_y, ped_v, dist, lat, lon, dec, _))][1],
            ped_x, ped_y, dist, lat, lon))
        break

# 最小距离
min_dist = None; min_row = None
for r in rows:
    if r[6] is not None and (min_dist is None or r[6] < min_dist):
        min_dist = r[6]; min_row = r
print('全程最小距离: %.2fm @ t=%.1fs (ADC_x=%.2f PED_x=%.2f)' % (min_dist, min_row[0], min_row[1], min_row[3]))

# 全程 STOP/FOLLOW 统计
from collections import Counter
dec_counter = Counter()
for r in rows:
    for d in r[9]:
        dec_counter[d] += 1
print('行人决策统计:', dict(dec_counter))
