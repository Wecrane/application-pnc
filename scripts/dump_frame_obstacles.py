#!/usr/bin/env python3
"""闪烁帧障碍物感知 dump
用法: python3 scripts/dump_frame_obstacles.py <record> <start_frame> <end_frame>
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
from modules.common_msgs.planning_msgs import planning_pb2

rf = sys.argv[1]
t0_, t1_ = float(sys.argv[2]), float(sys.argv[3])
reader = record.RecordReader(rf)
base = None

TYPE = {0: "UNK", 1: "UNK_MOV", 2: "UNK_UNMOV", 3: "PED", 4: "BIKE", 5: "VEH", 6: "VIRT"}

frame = 0
for msg in reader.read_messages():
    ts = msg.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if msg.topic == "/apollo/perception/obstacles":
        frame += 1
        if t0_ <= t <= t1_:
            o = perception_obstacle_pb2.PerceptionObstacles()
            o.ParseFromString(msg.message)
            print(f"--- perception frame {frame} ---")
            for ob in o.perception_obstacle:
                sp = (ob.velocity.x ** 2 + ob.velocity.y ** 2) ** 0.5
                print(f"  obs{ob.id} type={TYPE.get(ob.type, ob.type)} "
                      f"pos=({ob.position.x:.2f},{ob.position.y:.2f}) vel=({ob.velocity.x:.2f},{ob.velocity.y:.2f}) speed={sp:.2f}")
    elif msg.topic == "/apollo/planning" and t0_ <= t <= t1_:
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(msg.message)
        # 对象级纵向决策
        decs = []
        for obj_dec in tp.decision.object_decision.decision:
            for dec in obj_dec.object_decision:
                dk = dec.WhichOneof("object_tag")
                if dk in ("stop", "follow", "yield", "ignore", "overtake"):
                    decs.append(f"{obj_dec.id}:{dk}")
        if decs:
            print(f"  [plan frame {frame}] " + ", ".join(decs))
