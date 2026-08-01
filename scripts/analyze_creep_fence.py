#!/usr/bin/env python3
"""蠕动段 STOP 栅栏每帧原始值 + 车轨迹
用法: python3 scripts/analyze_creep_fence.py <record文件>
"""
import sys, os

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except (AttributeError, ValueError):
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/apollo/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo/bazel-out/k8-opt/bin"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from cyber.python.cyber_py3 import record
from modules.common_msgs.perception_msgs import perception_obstacle_pb2
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.planning_msgs import planning_pb2

rf = sys.argv[1]
reader = record.RecordReader(rf)

t0 = None
rows = []  # (t, veh_x, veh_v, stop_x, reason)
last_veh = None
last_plan = None

for msg in reader.read_messages():
    ts = msg.timestamp / 1e9
    if t0 is None:
        t0 = ts
    t = ts - t0
    if t < 15 or t > 45:
        continue
    if msg.topic == "/apollo/localization/pose":
        p = localization_pb2.LocalizationEstimate()
        p.ParseFromString(msg.message)
        v = p.pose.linear_velocity
        sp = (v.x ** 2 + v.y ** 2) ** 0.5
        last_veh = (t, p.pose.position.x, sp)
    elif msg.topic == "/apollo/planning":
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(msg.message)
        sx, rc = None, None
        if tp.HasField("decision") and tp.decision.main_decision.HasField("stop"):
            sx = tp.decision.main_decision.stop.stop_point.x
            rc = tp.decision.main_decision.stop.reason_code
        if last_veh is not None:
            rows.append((t, last_veh[1], last_veh[2], sx, rc))

print("t      veh_x      veh_v   stop_x     reason")
prev_stop = None
for t, vx, vv, sx, rc in rows:
    # 每 ~0.3s 采样
    if int(t * 3) == int(t * 3 - 0.01):
        continue
    key = None if sx is None else round(sx, 3)
    mark = ""
    if key != prev_stop:
        mark = "  <== 变化"
        prev_stop = key
    print(f"{t:6.2f} {vx:9.3f} {vv:7.3f} {str(sx if sx is None else round(sx,3)):>10} {str(rc):>6}{mark}")
