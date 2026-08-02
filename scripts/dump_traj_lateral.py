#!/usr/bin/env python3
"""dump 规划轨迹前N点的横向(y)偏移, 用于观察起步后是否朝障碍物拐
用法: python3 scripts/dump_traj_lateral.py <record> <t_start> <t_end> <lane_y_ref>
输出: t, 轨迹点1~3的y
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
lane_y = float(sys.argv[4])
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
        if n % 4 != 1:
            continue
        pts = []
        for p in tp.trajectory_point[:4]:
            pts.append(f"({p.path_point.x:.2f},{p.path_point.y:.2f})")
        if pts:
            print(f"t={t:.2f} traj={','.join(pts)}")
