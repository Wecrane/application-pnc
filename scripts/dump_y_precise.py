#!/usr/bin/env python3
"""高分辨率 dump 车Y + 规划轨迹横向 + 7673决策, 找右拐(横向偏移突变)
用法: python3 scripts/dump_y_precise.py <record> <t_start> <t_end>
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
from modules.common_msgs.planning_msgs import planning_pb2

rf = sys.argv[1]
t0, t1 = float(sys.argv[2]), float(sys.argv[3])
r = record.RecordReader(rf)
base = None
veh = None
for m in r.read_messages():
    ts = m.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if m.topic == "/apollo/localization/pose":
        p = localization_pb2.LocalizationEstimate()
        p.ParseFromString(m.message)
        v = p.pose.linear_velocity
        veh = (p.pose.position.x, p.pose.position.y, (v.x ** 2 + v.y ** 2) ** 0.5)
    elif m.topic == "/apollo/planning" and t0 <= t <= t1:
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(m.message)
        # 7673 决策
        decs = []
        for obj_dec in tp.decision.object_decision.decision:
            if obj_dec.id.startswith("7673"):
                for dec in obj_dec.object_decision:
                    dk = dec.WhichOneof("object_tag")
                    if dk in ("stop", "yield", "ignore", "follow"):
                        decs.append(dk)
        dstr = ",".join(decs) if decs else "-"
        # 规划轨迹前 3 点 y 偏移(相对车道中心 4437610.7)
        yoff = []
        for p_ in tp.trajectory_point[:3]:
            yoff.append(f"{p_.path_point.y - 4437610.7:+.3f}")
        if veh:
            print(f"t={t:.2f} veh_y={veh[1]:.3f} traj_yoff={','.join(yoff)} dec={dstr}")
