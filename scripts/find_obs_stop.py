#!/usr/bin/env python3
"""找指定障碍的 STOP/决策时间线（时间戳）
用法: python3 scripts/find_obs_stop.py <record> <obs_id_prefix>
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
from modules.common_msgs.planning_msgs import planning_pb2
from modules.common_msgs.localization_msgs import localization_pb2

rf = sys.argv[1]
target = sys.argv[2]
reader = record.RecordReader(rf)

base = None
veh_t = None
events = []

for msg in reader.read_messages():
    ts = msg.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if msg.topic == "/apollo/localization/pose":
        p = localization_pb2.LocalizationEstimate()
        p.ParseFromString(msg.message)
        v = p.pose.linear_velocity
        veh_t = (t, (v.x ** 2 + v.y ** 2) ** 0.5)
    elif msg.topic == "/apollo/planning":
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(msg.message)
        for obj_dec in tp.decision.object_decision.decision:
            if not obj_dec.id.startswith(target):
                continue
            for dec in obj_dec.object_decision:
                dk = dec.WhichOneof("object_tag")
                if dk in ("stop", "follow", "yield", "ignore", "overtake"):
                    vtxt = f"veh_v={veh_t[1]:.2f}" if veh_t else ""
                    events.append((t, dk, vtxt))
        # 主决策
        if tp.HasField("decision") and tp.decision.main_decision.HasField("stop"):
            md = tp.decision.main_decision.stop
            if md.reason_code != 2:  # 非 DESTINATION
                events.append((t, f"MAIN_STOP(rc={md.reason_code})",
                               f"pt=({md.stop_point.x:.2f},{md.stop_point.y:.2f})"))

# 去重连续相同
print(f"=== obs{target} 决策 + 非终点主STOP (t, 决策, 车辆速度) ===")
prev = None
for t, dk, extra in events:
    key = dk
    if key != prev:
        print(f"  t={t:6.2f} {dk:>20} {extra}")
        prev = key
