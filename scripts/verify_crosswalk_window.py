#!/usr/bin/env python3
# 正确验证：从头逐帧推进行人重放，记录"停车→放行"瞬间的行人位置
import sys, re, math
sys.path.insert(0, '/home/skye/application-pnc/scripts')
from local_grading_replay import parse_planning_log, PedestrianSim, START_Y

LOG = 'shiche/car_890/replay_logs/log/planning.log.INFO.20260804-192840.7964'
SCEN = 'shiche/car_890/grading_console/auto_drive_console/7号下午_1.json'
CW5_FRONT = 59.64
CW5_X_MIN = 272584.99   # 评分认定行人出横道
CW5_X_MAX = 272589.40
FRONT = 3.62
STOP_EPS = 0.1

rows = parse_planning_log(LOG)
end = None
for i in range(len(rows)):
    if rows[i][2] - START_Y >= 96 and abs(rows[i][4]) < 0.1:
        if i+5 < len(rows) and all(abs(rows[j][4]) < 0.1 for j in range(i, min(i+5, len(rows)))):
            end = min(i+10, len(rows)-1); break
rows = rows[:end]
t0 = rows[0][0]
pedsim = PedestrianSim(SCEN)

# 从头逐帧推进，找停车段 + 放行帧，记录行人位置
prev_v = 0.0
in_stop = False
stop_start = None
release_info = []   # (停车段时长, 放行t, 放行时各行人x, 是否全出横道)
for i, (t, x, y, th, v, a) in enumerate(rows):
    rel = t - t0
    adc_dy = y - START_Y
    peds = pedsim.update(rel, adc_dy)
    moving = abs(v) >= STOP_EPS
    # 停车段检测：v<eps 且 Δy 49-62
    stopped_near = (abs(v) < STOP_EPS and 49 <= adc_dy <= 62)
    if stopped_near and not in_stop:
        in_stop = True
        stop_start = i
    elif not stopped_near and in_stop:
        # 停车段结束 = 放行帧
        dur = rows[i-1][0] - rows[stop_start][0]
        ped_pos = {pid: px for pid, px, py, spd, hd in peds}
        all_clear = all(px < CW5_X_MIN or px > CW5_X_MAX for px in ped_pos.values())
        in_cw = {pid: (CW5_X_MIN <= px <= CW5_X_MAX) for pid, px in ped_pos.items()}
        release_info.append((dur, rel, ped_pos, all_clear, in_cw, v))
        in_stop = False
    prev_v = v

print(f"192840 共 {len(release_info)} 次停车放行\n")
for idx, (dur, rt, ped_pos, all_clear, in_cw, rv) in enumerate(release_info):
    clear_str = "是 (无窗口)" if all_clear else "否 (有窗口风险!)"
    print(f"=== 第{idx+1}次放行 t={rt:.1f}s (停车{dur:.1f}s) 放行瞬间车v={rv:.2f} ===")
    print(f"  放行时行人是否全部离开横道: {'[OK]' if all_clear else '[!!]'} {clear_str}")
    for pid, px in ped_pos.items():
        status = "仍在横道" if in_cw.get(pid) else "已出横道"
        print(f"    {pid}: x={px:.2f}  {status}")
    print()
