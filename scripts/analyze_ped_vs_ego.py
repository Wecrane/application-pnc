#!/usr/bin/env python3
"""行人 vs 车起步精确时序
用法: python3 scripts/analyze_ped_vs_ego.py <record> <ped_id>
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
ped_id = sys.argv[2]
reader = record.RecordReader(rf)

base = None
rows = []  # (t, ped_x, ped_y, ped_speed, veh_x, veh_v, obs_dec)
ped = None
veh = None
last_dec = None

for msg in reader.read_messages():
    ts = msg.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if msg.topic == "/apollo/perception/obstacles":
        o = perception_obstacle_pb2.PerceptionObstacles()
        o.ParseFromString(msg.message)
        for ob in o.perception_obstacle:
            if str(ob.id) == ped_id:
                sp = (ob.velocity.x ** 2 + ob.velocity.y ** 2) ** 0.5
                ped = (round(ob.position.x, 2), round(ob.position.y, 2), round(sp, 3))
    elif msg.topic == "/apollo/localization/pose":
        p = localization_pb2.LocalizationEstimate()
        p.ParseFromString(msg.message)
        v = p.pose.linear_velocity
        veh = (round(p.pose.position.x, 2), round((v.x ** 2 + v.y ** 2) ** 0.5, 3))
    elif msg.topic == "/apollo/planning":
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(msg.message)
        # 对象级: 该行人(含 _0 动态)决策
        decs = []
        for obj_dec in tp.decision.object_decision.decision:
            if obj_dec.id.startswith(ped_id):
                for dec in obj_dec.object_decision:
                    dk = dec.WhichOneof("object_tag")
                    if dk in ("stop", "follow", "yield", "ignore"):
                        decs.append(dk)
        dstr = ",".join(decs) if decs else "-"
        if veh is not None and (int(t * 2) == int(t * 2 - 0.01) or dstr != last_dec):
            rows.append((round(t, 1), ped, veh, dstr))
            last_dec = dstr

print("t    ped(x,y,v)                veh(x,v)      obs决策")
for t, p_, v_, d_ in rows:
    pstr = f"({p_[0]},{p_[1]},{p_[2]:.2f})" if p_ else "(-)"
    vstr = f"({v_[0]},{v_[1]:.2f})" if v_ else "(-)"
    print(f"{t:5.1f} {pstr:>24} {vstr:>16} {d_}")
