#!/usr/bin/env python3
"""dump 指定 id 的完整纵向决策（stop point + distance_s + reason）。"""
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
for m in r.read_messages():
    ts = m.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if m.topic == "/apollo/planning" and t0 <= t <= t1:
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(m.message)
        for d in tp.decision.object_decision.decision:
            oid = d.id
            if oid == "7673" or oid == "7673_0":
                for od in d.object_decision:
                    kind = od.WhichOneof("object_tag")
                    if kind == "stop":
                        s = od.stop
                        print("t=%.2f %s tag=stop dist_s=%.2f stop@(%.2f,%.2f) reason=%s" % (
                            t, oid, s.distance_s, s.stop_point.x, s.stop_point.y, s.reason_code))
                    elif kind:
                        print("t=%.2f %s tag=%s" % (t, oid, kind))
        if tp.decision.HasField("main_decision"):
            md = tp.decision.main_decision
            if md.HasField("stop"):
                print("  MAIN_STOP stop@(%.2f) reason=%s" % (md.stop.stop_point.x, md.stop.reason_code))
