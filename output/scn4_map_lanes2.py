#!/usr/bin/env python3
"""赛题4：路线车道与路口限速分析"""
import sys

sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/679551712d2357b63e6e0ce858ebf90e/execroot/application-pnc/bazel-out/k8-opt/bin/external/apollo_src')
sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/679551712d2357b63e6e0ce858ebf90e/execroot/application-pnc/bazel-out/k8-opt/bin')

from modules.common_msgs.map_msgs import map_pb2

hdmap = map_pb2.Map()
with open('/home/skye/application-pnc/data/map_data/Xh_2026_contest/base_map.bin', 'rb') as f:
    hdmap.ParseFromString(f.read())

print('=== 2.78 m/s (10km/h) 车道位置 ===')
for l in hdmap.lane:
    if abs(l.speed_limit - 2.78) < 0.01:
        pts = l.central_curve.segment[0].line_segment.point
        print(f'  {l.id.id}: len={l.length:.0f}m 起=({pts[0].x:.1f},{pts[0].y:.1f}) 终=({pts[-1].x:.1f},{pts[-1].y:.1f})')

print()
print('=== 赛题4路线车道 ===')
lane_by_id = {l.id.id: l for l in hdmap.lane}
for lid in ['Lane_1870', 'Lane_1960', 'Lane_1959', 'Lane_1871', 'Lane_1965', 'Lane_1909', 'Lane_1910', 'Lane_1964', 'Lane_714']:
    if lid in lane_by_id:
        l = lane_by_id[lid]
        pts = l.central_curve.segment[0].line_segment.point
        print(f'  {lid}: speed={l.speed_limit}m/s({l.speed_limit*3.6:.0f}km/h) len={l.length:.0f}m '
              f'起=({pts[0].x:.1f},{pts[0].y:.1f}) 终=({pts[-1].x:.1f},{pts[-1].y:.1f}) '
              f'pred={[p.id for p in l.predecessor_id]} succ={[p.id for p in l.successor_id]}')

# 路口 Junction_9 的多边形与关联
print()
print('=== Junction_9 详情 ===')
for j in hdmap.junction:
    if j.id.id == 'Junction_9':
        print(f'  id={j.id.id} type={j.type}')
        for p in j.polygon.point:
            print(f'    point=({p.x:.1f},{p.y:.1f})')
        # 关联 overlap
        for o in hdmap.overlap:
            for obj in o.object:
                if obj.id.id == 'Junction_9':
                    print(f'  overlap {o.id.id}: 关联对象 {[x.id.id for x in o.object]}')
