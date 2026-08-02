#!/usr/bin/env python3
"""dump 车(x,y,v) + 行人(x,y,v) + 7673决策 (0.5s采样)
用法: python3 scripts/dump_ego_ped_full.py <record> <ped_id>
"""
import sys, os
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
ped_id = sys.argv[2]
r = record.RecordReader(rf)
base = None
rows = []
ped = None
veh = None
last_dec = None
for m in r.read_messages():
    ts = m.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if m.topic == "/apollo/perception/obstacles":
        o = perception_obstacle_pb2.PerceptionObstacles()
        o.ParseFromString(m.message)
        for ob in o.perception_obstacle:
            if str(ob.id) == ped_id:
                sp = (ob.velocity.x ** 2 + ob.velocity.y ** 2) ** 0.5
                ped = (ob.position.x, ob.position.y, sp)
    elif m.topic == "/apollo/localization/pose":
        p = localization_pb2.LocalizationEstimate()
        p.ParseFromString(m.message)
        v = p.pose.linear_velocity
        veh = (p.pose.position.x, p.pose.position.y, (v.x ** 2 + v.y ** 2) ** 0.5)
    elif m.topic == "/apollo/planning":
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(m.message)
        decs = []
        for obj_dec in tp.decision.object_decision.decision:
            if obj_dec.id.startswith(ped_id):
                for dec in obj_dec.object_decision:
                    dk = dec.WhichOneof("object_tag")
                    if dk in ("stop", "follow", "yield", "ignore"):
                        decs.append(dk)
        dstr = ",".join(decs) if decs else "-"
        if veh is not None and (int(t * 2) == int(t * 2 - 0.01) or dstr != last_dec):
            rows.append((t, ped, veh, dstr))
            last_dec = dstr

print("t    ped(x,y,v)                       veh(x,y,v)      dec")
for t, p_, v_, d_ in rows:
    pstr = f"({p_[0]:.2f},{p_[1]:.2f},{p_[2]:.2f})" if p_ else "(-)"
    vstr = f"({v_[0]:.2f},{v_[1]:.2f},{v_[2]:.2f})" if v_ else "(-)"
    print(f"{t:6.1f} {pstr:>30} {vstr:>26} {d_}")
