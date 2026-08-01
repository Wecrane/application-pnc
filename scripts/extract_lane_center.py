#!/usr/bin/env python3
"""提取指定 lane 的 centerline 点 — 容器内运行
用法：python3 scripts/extract_lane_center.py <base_map.bin> <lane_id>...
输出: lane_id -> [(x,y), ...] 每5个点
"""
import sys, os

for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/apollo/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo/bazel-out/k8-opt/bin",
           "/home/skye/application-pnc/.cache/bazel/679551712d2357b63e6e0ce858ebf90e/execroot/application-pnc/bazel-out/k8-opt/bin/external/apollo_src",
           "/home/skye/application-pnc/.cache/bazel/679551712d2357b63e6e0ce858ebf90e/execroot/application-pnc/bazel-out/k8-opt/bin"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except (AttributeError, ValueError):
    try:
        import io
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")
    except Exception:
        pass

from modules.common_msgs.map_msgs import map_pb2

map_file = sys.argv[1]
want = set(sys.argv[2:])

m = map_pb2.Map()
with open(map_file, 'rb') as f:
    m.ParseFromString(f.read())

for lane in m.lane:
    if not any(w in lane.id.id for w in want):
        continue
    pts = []
    for seg in lane.central_curve.segment:
        for pt in seg.line_segment.point:
            pts.append((round(pt.x, 3), round(pt.y, 3)))
    print(f"=== Lane_{lane.id.id} turn={lane.turn} points={len(pts)} ===")
    # 每5个点打印，含首尾
    step = max(1, len(pts)//15)
    for i in range(0, len(pts), step):
        print(f"  p{i}: ({pts[i][0]}, {pts[i][1]})")
    print(f"  last: ({pts[-1][0]}, {pts[-1][1]})")
