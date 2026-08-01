#!/usr/bin/env python3
"""护栏/静态障碍物深度分析 — 容器内运行
提取：UNKNOWN/UNKNOWN_UNMOVABLE 类障碍物(护栏)的时间线、速度波动、planning 决策交替
用法：python3 scripts/analyze_barrier.py <record文件>
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

rf = sys.argv[1]
reader = record.RecordReader(rf)

TYPE_NAME = {0: "UNKNOWN", 1: "UNKNOWN_MOVABLE", 2: "UNKNOWN_UNMOVABLE",
             3: "PEDESTRIAN", 4: "BICYCLE", 5: "VEHICLE", 6: "VIRTUAL", 7: "CIPV"}

# 所有障碍物类型统计
type_cnt = collections.Counter()
# 护栏类障碍物(id): (frame, type, x, y, vx, vy, vel) 全采样
barriers = collections.defaultdict(list)
# 对象级决策: id -> list of (frame, decision)
obs_dec = collections.defaultdict(list)
# 车辆轨迹(估算护栏横向位置用)
traj = []
# 每次出现的连续段(闪烁检测): id -> [(start_frame, end_frame, n_frames)]
segments = collections.defaultdict(list)

cur_seg = {}
frame = 0
for msg in reader.read_messages():
    if msg.topic == "/apollo/perception/obstacles":
        frame += 1
        seen = set()
        obs = perception_obstacle_pb2.PerceptionObstacles()
        obs.ParseFromString(msg.message)
        for ob in obs.perception_obstacle:
            oid = str(ob.id)
            type_cnt[ob.type] += 1
            seen.add(oid)
            v = (ob.velocity.x ** 2 + ob.velocity.y ** 2) ** 0.5
            if ob.type in (0, 1, 2):  # UNKNOWN / 静态类
                barriers[oid].append((frame, ob.type, round(ob.position.x, 2),
                                      round(ob.position.y, 2),
                                      round(ob.velocity.x, 2), round(ob.velocity.y, 2),
                                      round(v, 3)))
        # 闪烁检测：上一帧存在但本帧消失
        for oid in list(cur_seg):
            if oid not in seen:
                segments[oid].append(cur_seg.pop(oid))
        for oid in seen:
            if oid not in cur_seg:
                cur_seg[oid] = [frame, frame, 1]
            else:
                cur_seg[oid][1] = frame
                cur_seg[oid][2] += 1
    elif msg.topic == "/apollo/localization/pose":
        from modules.common_msgs.localization_msgs import localization_pb2
        pose = localization_pb2.LocalizationEstimate()
        pose.ParseFromString(msg.message)
        p = pose.pose.position
        v = pose.pose.linear_velocity
        traj.append((p.x, p.y, (v.x ** 2 + v.y ** 2) ** 0.5))
    elif msg.topic == "/apollo/planning":
        traj_p = planning_pb2.ADCTrajectory()
        traj_p.ParseFromString(msg.message)
        f = len(traj)
        for obj_dec in traj_p.decision.object_decision.decision:
            for dec in obj_dec.object_decision:
                dk = dec.WhichOneof("object_tag")
                if dk in ("stop", "follow", "yield", "overtake", "ignore"):
                    obs_dec[obj_dec.id].append((f, dk))

# 收尾剩余段
for oid, seg in cur_seg.items():
    segments[oid].append(seg)

print("=== 障碍物类型分布 ===")
for t, c in sorted(type_cnt.items()):
    print(f"  type={t} ({TYPE_NAME.get(t,'?')}): {c} 帧次")

print()
print("=== 护栏类障碍物(UNKNOWN/STATIC)时间线 ===")
for oid in sorted(barriers, key=lambda k: barriers[k][0][0]):
    tl = barriers[oid]
    t0 = tl[0]
    # 速度>0.5 的帧数(是否闪烁抖动)
    mv = sum(1 for s in tl if s[6] > 0.5)
    print(f"obs{oid} type={TYPE_NAME.get(t0[1], t0[1])} first={t0[0]} last={tl[-1][0]} "
          f"n={len(tl)} 速度>0.5帧={mv}")
    # 段(闪烁)
    segs = segments.get(oid, [])
    if len(segs) > 1:
        print(f"   出现{len(segs)}段(闪烁!): " + "; ".join(f"[{s[0]}-{s[1]}]({s[2]}帧)" for s in segs))
    # 采样: 首 + 3个中 + 尾
    n = len(tl)
    idxs = [0, n // 4, n // 2, 3 * n // 4, n - 1]
    for i in idxs:
        s = tl[i]
        print(f"   frame{s[0]}: pos=({s[2]},{s[3]}) vel=({s[4]},{s[5]}) speed={s[6]}")

print()
print("=== 护栏类障碍物决策序列(去重, 找 STOP/FOLLOW 交替) ===")
for oid in sorted(obs_dec):
    dl = obs_dec[oid]
    # 去重连续相同
    prev = None
    seq = []
    for f, d in dl:
        if d != prev:
            seq.append((f, d))
            prev = d
    # 只显示有交替的
    stops = sum(1 for _, d in seq if d == "stop")
    follows = sum(1 for _, d in seq if d in ("follow", "yield"))
    if stops + follows >= 3:
        print(f"obs{oid}: STOP={stops} FOLLOW/YIELD={follows}")
        print("   序列: " + " ".join(f"{d}@{f}" for f, d in seq))

print()
print("traj_points:", len(traj))
print("车辆速度波动(抽搐): 采样每300帧")
for i in range(0, len(traj), 300):
    t = traj[i]
    print(f"  frame{i}: x={t[0]:.2f} v={t[2]:.2f}")
