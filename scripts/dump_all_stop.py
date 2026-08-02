#!/usr/bin/env python3
"""dump 所有对象 stop 决策的 tag + stop_point
用法: python3 scripts/dump_all_stop.py <record> <t_start> <t_end>
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
t0, t1 = float(sys.argv[2]), float(sys.argv[3])
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
        n += 1
        if n % 3 != 1:
            continue
        line = []
        for obj in tp.decision.object_decision.decision:
            for d in obj.object_decision:
                if d.HasField("stop"):
                    sp = d.stop.stop_point
                    line.append(f"{obj.id}:stop({sp.x:.1f})")
                elif d.HasField("ignore"):
                    line.append(f"{obj.id}:ign")
                elif d.HasField("yield"):
                    line.append(f"{obj.id}:yld")
                elif d.HasField("follow"):
                    line.append(f"{obj.id}:fol")
        if line:
            print(f"t={t:.2f} " + " ".join(line))
