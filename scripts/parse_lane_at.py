#!/usr/bin/env python3
"""查询地图中指定 x 范围内所有车道中心线点 (用于算行人横向)
用法: python3 scripts/parse_lane_at.py <base_map.bin> <x_min> <x_max>
"""
import sys, os
sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/679551712d2357b63e6e0ce858ebf90e/execroot/application-pnc/bazel-out/k8-opt/bin/external/apollo_src')
sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/679551712d2357b63e6e0ce858ebf90e/execroot/application-pnc/bazel-out/k8-opt/bin')

from modules.common_msgs.map_msgs import map_pb2

bm = map_pb2.Map()
with open(sys.argv[1], 'rb') as f:
    bm.ParseFromString(f.read())
x0, x1 = float(sys.argv[2]), float(sys.argv[3])

for lane in bm.lane:
    pts = []
    for seg in lane.central_curve.segment:
        for ls in seg.line_segment.point:
            pts.append((ls.x, ls.y))
    for (x, y) in pts:
        if x0 <= x <= x1:
            # 打印车道、该点 x、y（中心线）
            print(f"{lane.id.id:12s} x={x:.2f} y={y:.2f}")
            break
