#!/usr/bin/env python3
"""dump 指定行人的 stop 决策 stop_point 变化
用法: python3 scripts/dump_ped_stop.py <record> <ped_id_prefix> <t_start> <t_end>
"""
import sys, os
for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/apollo/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo/bazel-out/k8-opt/bin"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from cyber.python.cyber_py3 import record
from modules.common_msgs.planning_msgs import planning_pb2

rf = sys.argv[1]
prefix = sys.argv[2]
t0, t1 = float(sys.argv[3]), float(sys.argv[4])
r = record.RecordReader(rf)
base = None
n = 0
for m in r.read_messages():
    ts = m.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if m.topic == "/apollo/planning" and t0 <= t <= t1:
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(m.message)
        tags = []
        for obj in tp.decision.object_decision.decision:
            if obj.id.startswith(prefix):
                for d in obj.object_decision:
                    if d.HasField("stop"):
                        sp = d.stop.stop_point
                        tags.append(f"stop(sp={sp.x:.1f},{sp.y:.1f},d={d.stop.distance_s:.1f})")
                    elif d.HasField("ignore"):
                        tags.append("ignore")
                    elif d.HasField("yield"):
                        tags.append("yield")
                    elif d.HasField("follow"):
                        tags.append("follow")
        if tags:
            n += 1
            if n % 2 == 1:
                print(f"t={t:.2f} " + " ".join(tags))
        else:
            n += 1
            if n % 2 == 1:
                print(f"t={t:.2f} (no-decision)")
