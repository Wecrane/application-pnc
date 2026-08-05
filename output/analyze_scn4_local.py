#!/usr/bin/env python3
"""分析本地 scn4 日志：对齐车轨迹与 QP stop/v_up 时间线。"""
import re
import sys

LOG = sys.argv[1] if len(sys.argv) > 1 else 'data/log/planning.log.INFO.20260803-154102.1320055'
T0 = float(sys.argv[2]) if len(sys.argv) > 2 else 9.0
T1 = float(sys.argv[3]) if len(sys.argv) > 3 else 17.5


def ts(s):
    hh, mm, ss = s.split(':')
    return int(hh) * 3600 + int(mm) * 60 + float(ss)


# 车轨迹
pt = re.compile(r'(\d{2}:\d{2}:\d{2}\.\d+).*?planning_start_point x,y,the,k: ([\d.-]+), y: ([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)')
car = {}
for line in open(LOG, encoding='utf-8', errors='ignore'):
    m = pt.search(line)
    if m:
        t = ts(m.group(1))
        y = float(m.group(3))
        v = float(m.group(5))
        car[t] = (y, v)

# pjs2
pj = re.compile(r'(\d{2}:\d{2}:\d{2}\.\d+).*?\[pjs2\] init_v=([\d.-]+) stop_s=([\d.e+-]+) s_up\[0/40/80\]=([\d.-]+)/([\d.-]+)/([\d.-]+) dx_ref\[0/40\]=([\d.-]+)/([\d.-]+) v_up\[0\]=([\d.-]+) ref_len=([\d.-]+) sl\[0/60\]=([\d.-]+)/([\d.-]+) npts=(\d+) stops=(.*)')
rows = []
for line in open(LOG, encoding='utf-8', errors='ignore'):
    m = pj.search(line)
    if m:
        g = m.groups()
        rows.append((ts(g[0]), float(g[1]), float(g[2]), float(g[3]), float(g[4]),
                     float(g[5]), float(g[6]), float(g[7]), float(g[8]), float(g[9]),
                     float(g[10]), float(g[11]), float(g[12]), g[13]))

t0 = rows[0][0]
print(f'{"时间":>6} {"车v":>6} {"车y":>11} {"v_up0":>7} {"stop_s":>9} {"s_up0/40/80":>24} {"sl0/60":>9} stops')
for r in rows:
    t, iv, ss, s0, s40, s80, dx0, dx40, vup, refl, sl0, sl60, npts, stops = r
    dt = t - t0
    if dt < T0 or dt > T1:
        continue
    cy, cv = car.get(t, (0.0, 0.0))
    print(f'{dt:6.2f} {cv:6.2f} {cy:11.1f} {vup:7.2f} {ss:9.2f} {s0:8.1f}/{s40:8.1f}/{s80:8.1f} {sl0:6.1f}/{sl60:5.1f} {stops[:45]}')
