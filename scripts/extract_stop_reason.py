#!/usr/bin/env python3
"""从 record 提取 planning STOP 决策原因 — 容器内运行
用法：python3 scripts/extract_stop_reason.py <record文件>
"""
import sys, os, collections

for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/apollo/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo/bazel-out/k8-opt/bin"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from cyber.python.cyber_py3 import record
from modules.common_msgs.planning_msgs import planning_pb2

rf = sys.argv[1]
reader = record.RecordReader(rf)
reasons = collections.Counter()
n = 0
shown = {}
for msg in reader.read_messages():
    if msg.topic != "/apollo/planning":
        continue
    try:
        traj = planning_pb2.ADCTrajectory()
        traj.ParseFromString(msg.message)
        if traj.HasField("decision"):
            md = traj.decision.main_decision
            if md.HasField("stop"):
                r = md.stop.reason_code
                reasons[r] += 1
                if r not in shown:
                    st = md.stop.stop_point
                    shown[r] = (st.x, st.y)
                    print(f"STOP reason_code={r} point=({st.x:.3f},{st.y:.3f})")
        n += 1
    except Exception:
        pass
print("total_planning_frames:", n)
print("stop_reason_counts:", dict(reasons))
