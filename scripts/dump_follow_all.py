#!/usr/bin/env python3
"""dump 所有 follow/yield 决策 + 障碍物类型, 找'跟车线'(FOLLOW)来源
用法: python3 scripts/dump_follow_all.py <record> <t_start> <t_end>
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
from modules.common_msgs.perception_msgs import perception_obstacle_pb2

rf = sys.argv[1]
t0, t1 = float(sys.argv[2]), float(sys.argv[3])
r = record.RecordReader(rf)
base = None
n = 0
# 障碍物类型缓存
types = {}
for m in r.read_messages():
    ts = m.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if m.topic == "/apollo/perception/obstacles":
        o = perception_obstacle_pb2.PerceptionObstacles()
        o.ParseFromString(m.message)
        for ob in o.perception_obstacle:
            types[ob.id] = ob.type
    elif m.topic == "/apollo/planning" and t0 <= t <= t1:
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(m.message)
        n += 1
        if n % 3 != 1:
            continue
        tags = []
        for obj in tp.decision.object_decision.decision:
            for d in obj.object_decision:
                if d.HasField("follow"):
                    tid = obj.id
                    tname = types.get(tid, "?")
                    tags.append(f"{tid}(t{tname}):FOL")
                elif d.HasField("yield"):
                    tid = obj.id
                    tname = types.get(tid, "?")
                    tags.append(f"{tid}(t{tname}):YLD")
                elif d.HasField("stop"):
                    tags.append(f"{obj.id}:STOP")
        if tags:
            print(f"t={t:.2f} " + " ".join(tags))
