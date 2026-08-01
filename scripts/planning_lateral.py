#!/usr/bin/env python3
"""提取规划轨迹(ADCTrajectory)在转弯段的横向偏移 — 容器内运行
判断规划路径本身是否切内弯
用法：python3 scripts/planning_lateral.py <record> <base_map> <start_frame> <end_frame>
"""
import sys, os, math

for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/opt/apollo/neo/src/cyber/python",
           "/opt/apollo/neo/src"]:
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

from cyber.python.cyber_py3 import record
from modules.common_msgs.planning_msgs import planning_pb2
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.map_msgs import map_pb2

rf = sys.argv[1]
map_file = sys.argv[2]
f0, f1 = int(sys.argv[3]), int(sys.argv[4])

# lane centerlines
m = map_pb2.Map()
with open(map_file, 'rb') as f:
    m.ParseFromString(f.read())
lanes = {}
for lane in m.lane:
    if any(w in lane.id.id for w in ['1353', '697']):
        pts = []
        for seg in lane.central_curve.segment:
            for pt in seg.line_segment.point:
                pts.append((pt.x, pt.y))
        lanes[lane.id.id] = pts

def nearest_off(px, py):
    best = 1e9; bl = None; bi = 0
    for lid, lpts in lanes.items():
        for i, (lx, ly) in enumerate(lpts):
            d = (px-lx)**2 + (py-ly)**2
            if d < best:
                best = d; bl = lid; bi = i
    return math.sqrt(best), bl, bi

ego = []
pcount = 0
for msg in record.RecordReader(rf).read_messages():
    if msg.topic == "/apollo/localization/pose":
        pose = localization_pb2.LocalizationEstimate()
        pose.ParseFromString(msg.message)
        p = pose.pose.position
        ego.append((p.x, p.y))
    elif msg.topic == "/apollo/planning":
        pcount += 1
        if pcount % 40 != 0:
            continue
        traj = planning_pb2.ADCTrajectory()
        traj.ParseFromString(msg.message)
        if not ego:
            continue
        ex, ey = ego[-1]
        # 精确定位转弯段：x∈[424405,424428] 且 y∈[4437631,4437642]
        if not (424405 < ex < 424428 and 4437631 < ey < 4437642):
            continue
            continue
        # 取规划轨迹前 10 个点（车前路径），算相对 lane 中心偏移
        print(f"=== plan#{pcount} ego=({ex:.2f},{ey:.2f}) ===")
        for j, tp in enumerate(traj.trajectory_point[:10]):
            pp = tp.path_point
            off, lid, _ = nearest_off(pp.x, pp.y)
            print(f"  plan[{j}] ({pp.x:.2f},{pp.y:.2f}) -> {lid} off={off:.2f}m")
