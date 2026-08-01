#!/usr/bin/env python3
"""人行道场景行人横穿 vs 车辆停车时序分析
用法: python3 scripts/analyze_crosswalk_ped.py <record文件>
"""
import sys, os

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except (AttributeError, ValueError):
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

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
reader = record.RecordReader(rf)

pedtl = []      # (frame, x, y, speed)
traj = []       # (x, y, v) vehicle
stops = []      # (frame, reason_code)

frame = 0
for msg in reader.read_messages():
    if msg.topic == "/apollo/perception/obstacles":
        frame += 1
        if frame % 40 == 0:
            o = perception_obstacle_pb2.PerceptionObstacles()
            o.ParseFromString(msg.message)
            for ob in o.perception_obstacle:
                if str(ob.id) == "3060":
                    sp = (ob.velocity.x ** 2 + ob.velocity.y ** 2) ** 0.5
                    pedtl.append((frame, round(ob.position.x, 2), round(ob.position.y, 2), round(sp, 2)))
    elif msg.topic == "/apollo/localization/pose":
        p = localization_pb2.LocalizationEstimate()
        p.ParseFromString(msg.message)
        v = p.pose.linear_velocity
        traj.append((round(p.pose.position.x, 2), round(p.pose.position.y, 2),
                     round((v.x ** 2 + v.y ** 2) ** 0.5, 2)))
    elif msg.topic == "/apollo/planning":
        t = planning_pb2.ADCTrajectory()
        t.ParseFromString(msg.message)
        if t.HasField("decision") and t.decision.main_decision.HasField("stop"):
            stops.append((len(traj), t.decision.main_decision.stop.reason_code))

print("=== 行人3060 时间线 (frame, x, y, speed) ===")
for t in pedtl:
    print("  ", t)

print()
print("=== 车辆轨迹 (frame, x, y, v) 采样每200 ===")
for i in range(0, len(traj), 200):
    t = traj[i]
    print("  f%d: x=%.2f y=%.2f v=%.2f" % (i, t[0], t[1], t[2]))

print()
print("=== 车辆 STOP 决策 (frame, reason) 去重 ===")
prev = None
for f, r in stops:
    if r != prev:
        print("  f%d reason=%d" % (f, r))
        prev = r

print()
print("traj_points:", len(traj), " last:", traj[-1])
print("ped_samples:", len(pedtl))
