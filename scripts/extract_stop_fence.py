#!/usr/bin/env python3
"""提取指定帧区间的对象级 STOP 栅栏 — 容器内运行
用法：python3 scripts/extract_stop_fence.py <record> <start_frame> <end_frame>
"""
import sys, os, collections

for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/apollo/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo/bazel-out/k8-opt/bin"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except (AttributeError, ValueError):
    try:
        import io
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")
    except Exception:
        pass

from cyber.python.cyber_py3 import record
from modules.common_msgs.planning_msgs import planning_pb2
from modules.common_msgs.localization_msgs import localization_pb2

rf = sys.argv[1]
f0, f1 = int(sys.argv[2]), int(sys.argv[3])
reader = record.RecordReader(rf)

frame = 0
ego_traj = []
for msg in reader.read_messages():
    if msg.topic == "/apollo/localization/pose":
        pose = localization_pb2.LocalizationEstimate()
        pose.ParseFromString(msg.message)
        p = pose.pose.position
        v = pose.pose.linear_velocity
        ego = (round(p.x, 2), round(p.y, 2), round((v.x**2+v.y**2)**0.5, 2))
        ego_traj.append(ego)
    elif msg.topic == "/apollo/planning":
        frame = len(ego_traj)
        if frame < f0 or frame > f1:
            continue
        traj = planning_pb2.ADCTrajectory()
        traj.ParseFromString(msg.message)
        line = []
        md = traj.decision.main_decision
        if md.HasField("stop"):
            line.append(f"MAIN-STOP({md.stop.reason_code})@({md.stop.stop_point.x:.2f},{md.stop.stop_point.y:.2f})")
        elif md.HasField("cruise"):
            line.append("MAIN-CRUISE")
        # 对象级 STOP
        for obj_dec in traj.decision.object_decision.decision:
            for dec in obj_dec.object_decision:
                dk = dec.WhichOneof("object_tag")
                if dk == "stop":
                    st = dec.stop
                    line.append(f"obs{obj_dec.id}-STOP({st.reason_code})@({st.stop_point.x:.2f},{st.stop_point.y:.2f})")
        if line and frame % 20 == 0:
            print(f"f{frame} ego={ego} | " + " | ".join(line[:6]))
