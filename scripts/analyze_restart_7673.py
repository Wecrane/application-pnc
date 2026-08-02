#!/usr/bin/env python3
"""分析起步阶段高精度数据: 车辆x/y/航向/速度 + 行人7673位置 + 决策 + 轨迹横向(相对车)
用法: python3 scripts/analyze_restart_7673.py <record> <t_start> <t_end>
输出: t, veh(x,y,v,yaw), ped(x,y,v), ped横向(lat=ped_y-veh_y), 纵向(lon=ped_x-veh_x), traj点y相对车y, dec
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

rf, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
r = record.RecordReader(rf)
base = None
veh = None
ped = None
for m in r.read_messages():
    ts = m.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if m.topic == "/apollo/localization/pose":
        p = localization_pb2.LocalizationEstimate()
        p.ParseFromString(m.message)
        v = p.pose.linear_velocity
        veh = (p.pose.position.x, p.pose.position.y,
               (v.x ** 2 + v.y ** 2) ** 0.5, p.pose.heading)
    elif m.topic == "/apollo/perception/obstacles":
        o = perception_obstacle_pb2.PerceptionObstacles()
        o.ParseFromString(m.message)
        for ob in o.perception_obstacle:
            if str(ob.id) == "7673":
                sp = (ob.velocity.x ** 2 + ob.velocity.y ** 2) ** 0.5
                ped = (ob.position.x, ob.position.y, sp)
    elif m.topic == "/apollo/planning" and t0 <= t <= t1:
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(m.message)
        decs = []
        for od in tp.decision.object_decision.decision:
            if od.id.startswith("7673"):
                for d in od.object_decision:
                    k = d.WhichOneof("object_tag")
                    if k in ("stop", "follow", "yield", "ignore"):
                        decs.append(k)
        dstr = ",".join(decs) if decs else "-"
        if veh and ped:
            lat = ped[1] - veh[1]
            lon = ped[0] - veh[0]
            toffs = []
            for p_ in tp.trajectory_point[:4]:
                toffs.append(f"{p_.path_point.y - veh[1]:+.3f}")
            print(f"t={t:.2f} veh(x={veh[0]:.2f},y={veh[1]:.3f},v={veh[2]:.2f},yaw={veh[3]:.3f}) "
                  f"ped(x={ped[0]:.2f},y={ped[1]:.3f},v={ped[2]:.2f}) lat={lat:+.2f} lon={lon:.1f} "
                  f"traj_yoff_veh={','.join(toffs)} dec={dstr}")
