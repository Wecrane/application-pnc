#!/usr/bin/env python3
"""人行道场景: 用时间戳对齐 行人横穿进度 vs 车辆停车/起步 时序
用法: python3 scripts/analyze_crosswalk_timing.py <record文件>
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

# 用相对时间(第一个消息时间戳为0)
t0 = None
ped_events = []   # (t, x, y, speed) 行人每5秒采样
veh_events = []   # (t, x, v) 车速度变化关键帧
stop_events = []  # (t, reason) STOP 决策变化

last_veh = None
last_stop = None
last_ped_sample = -1

for msg in reader.read_messages():
    ts = msg.timestamp / 1e9
    if t0 is None:
        t0 = ts
    t = ts - t0

    if msg.topic == "/apollo/perception/obstacles":
        o = perception_obstacle_pb2.PerceptionObstacles()
        o.ParseFromString(msg.message)
        for ob in o.perception_obstacle:
            if str(ob.id) == "3060":
                sp = (ob.velocity.x ** 2 + ob.velocity.y ** 2) ** 0.5
                if int(t) >= last_ped_sample + 5:
                    ped_events.append((round(t, 1), round(ob.position.x, 2),
                                       round(ob.position.y, 2), round(sp, 2)))
                    last_ped_sample = int(t)
    elif msg.topic == "/apollo/localization/pose":
        p = localization_pb2.LocalizationEstimate()
        p.ParseFromString(msg.message)
        v = p.pose.linear_velocity
        sp = (v.x ** 2 + v.y ** 2) ** 0.5
        veh = (round(t, 1), round(p.pose.position.x, 2), round(sp, 2))
        if last_veh is None or abs(sp - last_veh[2]) > 0.5 or t - last_veh[0] > 5:
            veh_events.append(veh)
            last_veh = veh
    elif msg.topic == "/apollo/planning":
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(msg.message)
        rc = None
        if tp.HasField("decision") and tp.decision.main_decision.HasField("stop"):
            rc = tp.decision.main_decision.stop.reason_code
        elif tp.HasField("decision") and tp.decision.main_decision.HasField("mission_complete"):
            rc = "COMPLETE"
        elif tp.HasField("decision") and tp.decision.main_decision.HasField("cruise"):
            rc = "CRUISE"
        if rc is not None and rc != last_stop:
            stop_events.append((round(t, 1), rc))
            last_stop = rc

print("=== 行人3060 关键帧 (t, x, y, speed) ===")
for e in ped_events:
    print("  t=%6.1f x=%.2f y=%.2f v=%.2f" % e)

print()
print("=== 车辆关键帧 (t, x, v) ===")
for e in veh_events:
    print("  t=%6.1f x=%.2f v=%.2f" % e)

print()
print("=== 主决策变化 (t, reason) ===")
for e in stop_events:
    print("  t=%6.1f %s" % e)

print()
print("人行道 Crosswalk_62: x in [423659.5, 423664.0]; 行人目标 y=4437639.07")
print("车起点 x=423781.44, 终点 x=423567.77")
