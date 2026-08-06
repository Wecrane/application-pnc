#!/usr/bin/env python3
# 全面分析：所有苏州人行道题日志（7pm/6am）→ 指标 + 7pm评分复现
import sys, glob, os
ROOT = '/home/skye/application-pnc'
sys.path.insert(0, f'{ROOT}/scripts')
sys.path.insert(0, f'{ROOT}/shiche/car_890/xh_bak/.XH_2026_bak')
sys.path.insert(0, f'{ROOT}/shiche/car_890/grading_console')
from local_grading_replay import (parse_planning_log, PedestrianSim, LightMap,
    build_config, make_adc, make_ped, START_Y, FRONT)
from grading_system_py.main import GradingEngine, parse_config_file, GradingWorld
from grading_system_py.common.map_interface import set_map_interface

MAP = 'data/map_data/hehai_contest_7_pm_3/base_map.txt'
SCEN = 'shiche/car_890/grading_console/auto_drive_console/7号下午_1.json'
METRIC = 'shiche/car_890/grading_console/auto_drive_console/metric'

LOGS = [
    ('本地183108', 'data/log/planning.log.INFO.20260805-183108.20391'),
    ('本地183747', 'data/log/planning.log.INFO.20260805-183747.28213'),
    ('实车192840', 'shiche/car_890/replay_logs/log/planning.log.INFO.20260804-192840.7964'),
    ('实车193620', 'shiche/car_890/replay_logs/log/planning.log.INFO.20260804-193620.19039'),
    ('实车100527', 'shiche/car_890/scripts_biglogs/data/log/planning.log.INFO.20260804-100527.12072'),
    ('实车155058', 'shiche/car_890/selected_logs/planning.log.INFO.20260803-155058.4778'),
    ('实车162903', 'shiche/car_890/selected_logs/planning.log.INFO.20260803-162903.25829'),
    ('实车140922', 'shiche/car_890/selected_logs/planning.log.INFO.20260804-140922.31291'),
    ('实车164819', 'shiche/car_890/selected_logs/planning.log.INFO.20260804-164819.31648'),
    ('实车183119', 'shiche/car_890/selected_logs/planning.log.INFO.20260804-183119.24704'),
    ('实车190503', 'shiche/car_890/selected_logs/planning.log.INFO.20260804-190503.3864'),
]

set_map_interface(LightMap(MAP))
engine = GradingEngine(parse_config_file(build_config(METRIC, '7号下午')))

print("="*120)
print("苏州人行道题日志 全面分析（7pm 评分配置，含完整指标）")
print("="*120)
for tag, log in LOGS:
    rows = parse_planning_log(log)
    if len(rows) < 50:
        print(f"\n### {tag} ({log.split('/')[-1]})：帧数过少({len(rows)})，跳过")
        continue
    # 截断
    end = None
    for i in range(len(rows)):
        if rows[i][2] - START_Y >= 96 and abs(rows[i][4]) < 0.1:
            if i+5 < len(rows) and all(abs(rows[j][4]) < 0.1 for j in range(i, min(i+5, len(rows)))):
                end = min(i+10, len(rows)-1); break
    trunc = end is not None
    if end: rows = rows[:end]
    t0 = rows[0][0]
    # ---- 指标 ----
    maxv = max(r[4] for r in rows)
    over = sum(1 for r in rows if r[4] > 3.0556)
    hard = sum(1 for r in rows if r[5] < -3.0 or r[5] > 2.0)
    sb = [r for r in rows if 23.5 <= r[2]-START_Y <= 29.1]
    sb_max = max((r[4] for r in sb), default=0)
    # 停车点
    stops = []
    i = 0
    while i < len(rows):
        if abs(rows[i][4]) < 0.1 and 49 <= rows[i][2]-START_Y <= 62:
            j = i
            while j < len(rows) and abs(rows[j][4]) < 0.1 and 49 <= rows[j][2]-START_Y <= 62:
                j += 1
            if j-i >= 5:
                mid = rows[(i+j)//2]
                hd = 59.64 - (mid[2]-START_Y + FRONT)
                stops.append((rows[j-1][0]-rows[i][0], mid[2]-START_Y, hd))
            i = j
        else: i += 1
    stop_str = "; ".join(f"{d:.1f}s@{dy:.1f}m/距{hd:.2f}m" for d, dy, hd in stops[:3])
    # ---- 评分 ----
    pedsim = PedestrianSim(SCEN)
    eng2 = GradingEngine(parse_config_file(build_config(METRIC, '7号下午')))
    for ii, r in enumerate(rows):
        t = r[0]-t0
        w = GradingWorld()
        w.timestamp_sec = t
        w.auto_driving_car = make_adc(r)
        for pid, px, py, spd, hd in pedsim.update(t, r[2]-START_Y):
            w.object.append(make_ped(pid, px, py, spd, hd))
        w.last_frame = (ii == len(rows)-1)
        eng2.feed_frame(w)
    s = eng2.get_summary()['metrics']
    sc = lambda n: s.get(n, {}).get('score', 0)
    total = 100 - sum(max(0, 100-sc(n)) for n in s)
    print(f"\n### {tag}  ({log.split('/')[-1]}) 截断={trunc}")
    print(f"  指标: {len(rows)}帧 {rows[-1][0]-t0:.0f}s | maxV={maxv:.2f} 超速帧={over} 急刹帧={hard} "
          f"| 减速带区maxV={sb_max:.2f} | 停车: {stop_str or '无'}")
    cw = s.get('CrosswalkStop', {})
    cw_pass = cw.get('is_pass')
    print(f"  评分: Collision={sc('Collision'):.0f} CrosswalkStop={sc('CrosswalkStop'):.0f}{'(✅)' if cw_pass else ''} "
          f"Accel={sc('AccelerationLimit'):.0f} Cruise={sc('CruiseSpeedLimit'):.0f} "
          f"RoadSpeed={sc('RoadSpeedLimit'):.0f} | 总分≈{total:.0f}")
    ff = cw.get('first_failure')
    if ff: print(f"        Crosswalk首败: t={ff['time']:.1f}s {ff['code']}")
print("\n" + "="*120)
