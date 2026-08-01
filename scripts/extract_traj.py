#!/usr/bin/env python3
"""提取指定帧区间的车辆高分辨率轨迹 — 容器内运行
用法：python3 scripts/extract_traj.py <record> <start_frame> <end_frame> [step]
"""
import sys, os, math

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
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.perception_msgs import perception_obstacle_pb2

rf = sys.argv[1]
f0, f1 = int(sys.argv[2]), int(sys.argv[3])
step = int(sys.argv[4]) if len(sys.argv) > 4 else 10
reader = record.RecordReader(rf)

pts = []
for msg in reader.read_messages():
    if msg.topic == "/apollo/localization/pose":
        pose = localization_pb2.LocalizationEstimate()
        pose.ParseFromString(msg.message)
        p = pose.pose.position
        v = pose.pose.linear_velocity
        h = pose.pose.heading
        sp = (v.x**2 + v.y**2)**0.5
        pts.append((p.x, p.y, sp, h))

# 输出指定区间
print("frame x y v heading")
for i in range(f0, min(f1, len(pts)), step):
    x, y, v, h = pts[i]
    print(f"{i} {x:.3f} {y:.3f} {v:.3f} {h:.4f}")

# 转弯曲率估计（用三点）
print("\n=== 转弯曲率(每20帧三点法) ===")
for i in range(f0+20, min(f1, len(pts))-20, 20):
    x1, y1 = pts[i-20][0], pts[i-20][1]
    x2, y2 = pts[i][0], pts[i][1]
    x3, y3 = pts[i+20][0], pts[i+20][1]
    # 三点曲率
    try:
        ax, ay = x1-x2, y1-y2
        bx, by = x3-x2, y3-y2
        cross = ax*by - ay*bx
        a = math.hypot(ax, ay); b = math.hypot(bx, by)
        if cross != 0 and a*b != 0:
            # 曲率 = 2*cross/(a*b*|chord|) 近似
            chord = math.hypot(x3-x1, y3-y1)
            curvature = 2*abs(cross)/(a*b*chord) if chord else 0
            print(f"f{i}: ({x2:.2f},{y2:.2f}) v={pts[i][2]:.2f} curvature={curvature:.4f}")
    except Exception:
        pass
