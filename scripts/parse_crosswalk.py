#!/usr/bin/env python3
"""解析地图中 Crosswalk 的位置 — 在 Apollo 容器内运行
用法：python3 scripts/parse_crosswalk.py [地图目录] [CrosswalkID可选]
"""
import sys, os

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

for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/apollo/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo/bazel-out/k8-opt/bin"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from modules.common_msgs.map_msgs import map_pb2

MAP_DIR = sys.argv[1] if len(sys.argv) > 1 else "data/map_data/xh_2025_contest"
FILTER = sys.argv[2] if len(sys.argv) > 2 else None

m = map_pb2.Map()
with open(os.path.join(MAP_DIR, "base_map.bin"), "rb") as f:
    m.ParseFromString(f.read())

print(f"Map: {MAP_DIR}")
for cw in m.crosswalk:
    if FILTER and FILTER not in cw.id.id:
        continue
    # polygon 中心
    pts = [(p.x, p.y) for p in cw.polygon.point]
    cx = sum(p[0] for p in pts) / len(pts) if pts else 0
    cy = sum(p[1] for p in pts) / len(pts) if pts else 0
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    print(f"Crosswalk {cw.id.id}: center=({cx:.3f},{cy:.3f}) "
          f"x[{min(xs):.3f},{max(xs):.3f}] y[{min(ys):.3f},{max(ys):.3f}]")
