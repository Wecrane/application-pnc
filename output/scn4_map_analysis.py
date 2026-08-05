#!/usr/bin/env python3
"""赛题4 地图限速分析：解析 Xh_2026_contest base_map 中赛题4路线车道的限速分布"""
import sys

sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/679551712d2357b63e6e0ce858ebf90e/execroot/application-pnc/bazel-out/k8-opt/bin/external/apollo_src')
sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/679551712d2357b63e6e0ce858ebf90e/execroot/application-pnc/bazel-out/k8-opt/bin')

from modules.common_msgs.map_msgs import map_pb2

hdmap = map_pb2.Map()
with open('/home/skye/application-pnc/data/map_data/Xh_2026_contest/base_map.bin', 'rb') as f:
    hdmap.ParseFromString(f.read())

print(f'车道数: {len(hdmap.lane)}, 路口: {len(hdmap.junction)}, 信号灯: {len(hdmap.signal)}, 人行道: {len(hdmap.crosswalk)}, 停车位: {len(hdmap.parking_space)}')

# 起点/终点
sx, sy = 423254.57576769503, 4438082.2017030129
ex, ey = 423258.2215546447, 4438364.4334788956


def lane_start_end(lane):
    pts = lane.central_curve.segment[0].line_segment.point
    return pts[0].x, pts[0].y, pts[-1].x, pts[-1].y


cand_s, cand_e = [], []
for lane in hdmap.lane:
    lx0, ly0, lx1, ly1 = lane_start_end(lane)
    d_s = min(((lx0 - sx) ** 2 + (ly0 - sy) ** 2) ** 0.5, ((lx1 - sx) ** 2 + (ly1 - sy) ** 2) ** 0.5)
    d_e = min(((lx0 - ex) ** 2 + (ly0 - ey) ** 2) ** 0.5, ((lx1 - ex) ** 2 + (ly1 - ey) ** 2) ** 0.5)
    if d_s < 3:
        cand_s.append((lane.id.id, round(d_s, 2), round(lane.speed_limit, 3), lane.turn, round(lane.length, 1)))
    if d_e < 3:
        cand_e.append((lane.id.id, round(d_e, 2), round(lane.speed_limit, 3), lane.turn, round(lane.length, 1)))

print('\n=== 起点附近车道 (id, 距起点, 限速m/s, turn, 长度) ===')
for c in sorted(cand_s)[:10]:
    print(c)
print('=== 终点附近车道 ===')
for c in sorted(cand_e)[:10]:
    print(c)

# 从起点车道出发，沿 successor 追溯路线（限速变化）
print('\n=== 起点车道 successor 链 (限速分布) ===')
lane_by_id = {l.id.id: l for l in hdmap.lane}


def chain(start_id, max_depth=30):
    seen = set()
    cur = start_id
    path = []
    for _ in range(max_depth):
        if cur in seen or cur not in lane_by_id:
            break
        seen.add(cur)
        l = lane_by_id[cur]
        lx0, ly0, lx1, ly1 = lane_start_end(l)
        path.append((cur, round(l.speed_limit, 2), l.turn, round(l.length, 1), round(lx1, 1), round(ly1, 1)))
        # 选后继中 y 最大的（向北）
        succs = [s.id for s in l.successor_id if s.id in lane_by_id]
        if not succs:
            break
        succs.sort(key=lambda sid: lane_by_id[sid].central_curve.segment[0].line_segment.point[-1].y, reverse=True)
        cur = succs[0]
    return path


for cid, d, sp, turn, ln in sorted(cand_s)[:5]:
    print(f'\n--- 起点车道 {cid} ---')
    for row in chain(cid):
        print(f'  {row[0]}: 限速={row[1]}m/s({row[1]*3.6:.1f}km/h) turn={row[2]} 长={row[3]}m 末端=({row[4]},{row[5]})')
