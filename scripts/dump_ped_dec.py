#!/usr/bin/env python3
"""dump 指定 id 的纵向决策类型 + stop point
用法: python3 scripts/dump_ped_dec.py <record> <id_exact> <t_start> <t_end>
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
exact_id = sys.argv[2]
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
        n += 1
        if n % 5 != 1:
            continue
        for obj in tp.decision.object_decision.decision:
            if obj.id == exact_id or obj.id.startswith(exact_id + "_"):
                tags = []
                for d in obj.object_decision:
                    if d.HasField("stop"):
                        sp = d.stop.stop_point
                        tags.append(f"STOP({sp.x:.1f},rc={d.stop.reason_code})")
                    elif d.HasField("yield"):
                        tags.append("YIELD")
                    elif d.HasField("ignore"):
                        tags.append("IGNORE")
                    elif d.HasField("follow"):
                        tags.append("FOLLOW")
                if tags:
                    print(f"t={t:.2f} id={obj.id} " + " ".join(tags))
