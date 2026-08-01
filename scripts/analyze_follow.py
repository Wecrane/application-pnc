#!/usr/bin/env python3
"""深度分析障碍物跟随回放 — 容器内运行
提取：障碍物完整时间线(VEHICLE)、车辆速度/位置采样、planning 决策(STOP/FOLLOW/YIELD)
用法：python3 scripts/analyze_follow.py <record文件>
"""
import sys, os, collections

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except (AttributeError, ValueError):
    try:
        import io
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")
    except (AttributeError, ValueError, io.UnsupportedOperation):
        pass

for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/apollo/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo/bazel-out/k8-opt/bin"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from cyber.python.cyber_py3 import record
from modules.common_msgs.perception_msgs import perception_obstacle_pb2
from modules.common_msgs.planning_msgs import planning_pb2
from modules.common_msgs.localization_msgs import localization_pb2

rf = sys.argv[1]
reader = record.RecordReader(rf)

# 障碍物时间线: id -> list of (frame, type, x, y, vx, vy)
obs_tl = collections.defaultdict(list)
# 车辆轨迹采样
traj = []
# planning 决策序列
decisions = []  # (frame, type, obstacle_id, reason)
# planning 障碍决策(带 id)
obs_decisions = collections.defaultdict(list)  # frame -> list

frame = 0
for msg in reader.read_messages():
    if msg.topic == "/apollo/perception/obstacles":
        frame += 1
        obs = perception_obstacle_pb2.PerceptionObstacles()
        obs.ParseFromString(msg.message)
        for ob in obs.perception_obstacle:
            oid = str(ob.id)
            if oid in obs_tl and len(obs_tl[oid]) > 200:
                continue  # 已采样足够
            obs_tl[oid].append((frame, ob.type,
                                round(ob.position.x, 2), round(ob.position.y, 2),
                                round(ob.velocity.x, 2), round(ob.velocity.y, 2)))
    elif msg.topic == "/apollo/localization/pose":
        pose = localization_pb2.LocalizationEstimate()
        pose.ParseFromString(msg.message)
        p = pose.pose.position
        v = pose.pose.linear_velocity
        sp = (v.x**2 + v.y**2)**0.5
        traj.append((round(p.x, 2), round(p.y, 2), round(sp, 2)))
    elif msg.topic == "/apollo/planning":
        traj_p = planning_pb2.ADCTrajectory()
        traj_p.ParseFromString(msg.message)
        if traj_p.HasField("decision"):
            md = traj_p.decision.main_decision
            d = None
            if md.HasField("stop"):
                d = ("STOP", md.stop.reason_code, md.stop.stop_point.x, md.stop.stop_point.y)
            elif md.HasField("mission_complete"):
                d = ("COMPLETE", 0, 0, 0)
            elif md.HasField("cruise"):
                d = ("CRUISE", 0, 0, 0)
            if d:
                decisions.append((len(traj), d[0], d[1], d[2], d[3]))
            # 对象级纵向决策
            for obj_dec in traj_p.decision.object_decision.decision:
                for dec in obj_dec.object_decision:
                    dk = dec.WhichOneof("object_tag")
                    if dk in ("stop", "follow", "yield", "overtake"):
                        obs_decisions[obj_dec.id].append((len(traj), dk))

print("=== 障碍物时间线（按出现帧排序，采样） ===")
for oid in sorted(obs_tl, key=lambda k: obs_tl[k][0][0]):
    tl = obs_tl[oid]
    t0 = tl[0]
    # 首尾 + 关键变化
    print(f"obs{oid} type={t0[1]} first_frame={t0[0]} pos=({t0[2]},{t0[3]}) vel=({t0[4]},{t0[5]})")
    last = tl[-1]
    print(f"   last_frame={last[0]} pos=({last[2]},{last[3]}) vel=({last[4]},{last[5]})")
    # 采样中间点（均匀 6 个）
    n = len(tl)
    for i in [n//7, 2*n//7, 3*n//7, 4*n//7, 5*n//7, 6*n//7]:
        s = tl[i]
        print(f"   frame{s[0]}: pos=({s[2]},{s[3]}) vel=({s[4]},{s[5]})")

print()
print("=== 车辆轨迹采样（每 ~500 帧） ===")
for i in range(0, len(traj), 500):
    t = traj[i]
    print(f"  frame{i}: x={t[0]} y={t[1]} v={t[2]}")

print()
print("=== 规划主决策序列（去重） ===")
prev = None
for d in decisions:
    key = (d[1], d[2], d[3], d[4])
    if key != prev:
        print(f"  frame{d[0]}: {d[1]} obs={d[2]} point=({d[3]},{d[4]})")
        prev = key

print()
print("=== 对象级纵向决策统计 ===")
for oid, dl in sorted(obs_decisions.items()):
    cnt = collections.Counter(x[1] for x in dl)
    print(f"  obs{oid}: {dict(cnt)} first_frame={dl[0][0]}")

print()
print("total_perception_frames:", frame)
print("traj_points:", len(traj), "min_v:", min(t[2] for t in traj), "last:", traj[-1])
