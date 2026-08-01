#!/usr/bin/env python3
"""人行道停车段高分辨率分析: 蠕动速度 + STOP栅栏变化
用法: python3 scripts/analyze_crosswalk_creep.py <record文件>
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

t0 = None
veh_samples = []   # (t, x, v) 全部 localization 采样
stop_fences = []   # (t, stop_point_x, reason) 主 STOP
ped_pos = {}       # t -> (x, y, speed)

for msg in reader.read_messages():
    ts = msg.timestamp / 1e9
    if t0 is None:
        t0 = ts
    t = ts - t0
    if msg.topic == "/apollo/localization/pose":
        p = localization_pb2.LocalizationEstimate()
        p.ParseFromString(msg.message)
        v = p.pose.linear_velocity
        sp = (v.x ** 2 + v.y ** 2) ** 0.5
        veh_samples.append((round(t, 2), round(p.pose.position.x, 3), round(sp, 3)))
    elif msg.topic == "/apollo/planning":
        tp = planning_pb2.ADCTrajectory()
        tp.ParseFromString(msg.message)
        if tp.HasField("decision") and tp.decision.main_decision.HasField("stop"):
            st = tp.decision.main_decision.stop
            stop_fences.append((round(t, 2), round(st.stop_point.x, 3),
                                st.reason_code, round(st.stop_point.y, 3)))
    elif msg.topic == "/apollo/perception/obstacles":
        o = perception_obstacle_pb2.PerceptionObstacles()
        o.ParseFromString(msg.message)
        for ob in o.perception_obstacle:
            if str(ob.id) == "3060":
                sp = (ob.velocity.x ** 2 + ob.velocity.y ** 2) ** 0.5
                ped_pos[round(t, 1)] = (round(ob.position.x, 2), round(ob.position.y, 2), round(sp, 3))

# 找停车段: v<0.2 的连续区间
print("=== 停车段高分辨率 (v<0.3 每帧) ===")
in_stop = False
stop_start = None
for t, x, v in veh_samples:
    if v < 0.3 and not in_stop:
        in_stop = True
        stop_start = t
    if in_stop and (v >= 0.3 or t - stop_start > 12):
        # 输出这段
        seg = [s for s in veh_samples if stop_start <= s[0] <= (t if v >= 0.3 else stop_start + 12)]
        if len(seg) > 5:
            print(f"--- 停车段 t={seg[0][0]}~{seg[-1][0]}s, {len(seg)}帧 ---")
            for st_ in seg[::4]:
                print(f"  t={st_[0]:6.2f} x={st_[1]:.3f} v={st_[2]:.3f}")
        in_stop = False
# 结尾段
if in_stop:
    seg = [s for s in veh_samples if s[0] >= stop_start]
    print(f"--- 停车段(尾) t={seg[0][0]}~{seg[-1][0]}s ---")
    for st_ in seg[::4]:
        print(f"  t={st_[0]:6.2f} x={st_[1]:.3f} v={st_[2]:.3f}")

print()
print("=== STOP 栅栏变化 (t, x, y, reason) 采样 ===")
prev = None
for t, x, y, rc in stop_fences:
    key = (round(x, 1), rc)
    if key != prev:
        print(f"  t={t:6.2f} stop_point=({x:.3f},{y:.3f}) reason={rc}")
        prev = key

print()
print("=== 行人3060 每2秒 ===")
for t in sorted(ped_pos):
    if int(t) % 2 == 0:
        x, y, sp = ped_pos[t]
        print(f"  t={t:5.1f} x={x:.2f} y={y:.2f} v={sp:.3f}")
