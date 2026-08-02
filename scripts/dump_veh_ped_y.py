#!/usr/bin/env python3
"""dump 车经过指定x范围时的 y + 指定行人的位置
用法: python3 scripts/dump_veh_ped_y.py <record> <ped_id> <x_min> <x_max>
"""
import sys, os
for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/apollo/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo/bazel-out/k8-opt/bin"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from cyber.python.cyber_py3 import record
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.perception_msgs import perception_obstacle_pb2

rf, ped_id, x0, x1 = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
r = record.RecordReader(rf)
base = None
ped = None
for m in r.read_messages():
    ts = m.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if m.topic == "/apollo/localization/pose":
        p = localization_pb2.LocalizationEstimate()
        p.ParseFromString(m.message)
        x = p.pose.position.x
        y = p.pose.position.y
        if x0 <= x <= x1:
            print(f"t={t:.1f} veh x={x:.2f} y={y:.2f} ped={ped}")
    elif m.topic == "/apollo/perception/obstacles":
        o = perception_obstacle_pb2.PerceptionObstacles()
        o.ParseFromString(m.message)
        for ob in o.perception_obstacle:
            if ob.id == ped_id:
                ped = f"({ob.position.x:.1f},{ob.position.y:.1f})"
