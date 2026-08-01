#!/usr/bin/env python3
"""解析 Apollo record 录制数据，提取车辆轨迹/行人/决策 — 在 Apollo 容器内运行

用法（容器内）：
    python3 scripts/analyze_record.py [record文件或目录] [--channel 通道名]

默认分析 /apollo/planning + /apollo/localization/pose + /apollo/perception/obstacles
输出：output/record_analysis.txt（车辆位置/速度序列、行人信息）
"""

import sys
import os
import json

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

# 容器内 protobuf python 产物路径
for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/apollo/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo/bazel-out/k8-opt/bin"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

# 尝试导入 cyber record 与 proto
try:
    from cyber.python.cyber_py3 import record
    HAS_CYBER = True
except Exception as e:
    HAS_CYBER = False
    print(f"[warn] cyber_py3 import failed: {e}", file=sys.stderr)

try:
    from modules.common_msgs.localization_msgs import localization_pb2
    from modules.common_msgs.perception_msgs import perception_obstacle_pb2
    from modules.common_msgs.planning_msgs import planning_pb2
    HAS_PROTO = True
except Exception as e:
    HAS_PROTO = False
    print(f"[warn] proto import failed: {e}", file=sys.stderr)


def list_records(path):
    """返回 record 文件列表（支持文件或目录，按分段顺序）。"""
    if os.path.isdir(path):
        files = sorted(f for f in os.listdir(path) if f.endswith(".record"))
        return [os.path.join(path, f) for f in files]
    return [path]


def read_messages(record_file, channel):
    """用 cyber record 读取指定通道消息（PyBagMessage: topic/message/data_type/timestamp）。"""
    reader = record.RecordReader(record_file)
    for msg in reader.read_messages():
        if msg.topic == channel:
            yield msg


def analyze(path):
    rec_files = list_records(path)
    print(f"Records: {rec_files}")

    # 1. vehicle position/speed (localization)
    print("\n=== Vehicle trajectory (localization/pose) ===")
    positions = []
    for rf in rec_files:
        try:
            for msg in read_messages(rf, "/apollo/localization/pose"):
                try:
                    loc = localization_pb2.LocalizationEstimate()
                    loc.ParseFromString(msg.message)
                    pose = loc.pose
                    x = pose.position.x
                    y = pose.position.y
                    v = (pose.linear_velocity.x ** 2 +
                         pose.linear_velocity.y ** 2) ** 0.5
                    positions.append({"x": x, "y": y, "v": v})
                except Exception:
                    pass
        except Exception as e:
            print(f"[warn] {rf} localization: {e}", file=sys.stderr)

    if positions:
        print(f"Total frames: {len(positions)}")
        for i in range(0, len(positions), 50):
            p = positions[i]
            print(f"  frame{i}: x={p['x']:.3f} y={p['y']:.3f} v={p['v']:.3f} m/s")
        min_v = min(positions, key=lambda p: p["v"])
        print(f"\nMin speed point: x={min_v['x']:.3f} y={min_v['y']:.3f} v={min_v['v']:.3f} m/s")
        xs = [p["x"] for p in positions]
        ys = [p["y"] for p in positions]
        print(f"Traj range: x[{min(xs):.2f},{max(xs):.2f}] y[{min(ys):.2f},{max(ys):.2f}]")

    # 2. perception obstacles (pedestrians)
    print("\n=== Obstacles (perception/obstacles) ===")
    obs_seen = {}
    for rf in rec_files:
        try:
            for msg in read_messages(rf, "/apollo/perception/obstacles"):
                try:
                    obs = perception_obstacle_pb2.PerceptionObstacles()
                    obs.ParseFromString(msg.message)
                    for ob in obs.perception_obstacle:
                        oid = ob.id
                        t = ob.type
                        pos = {"x": ob.position.x, "y": ob.position.y}
                        vel = {"x": ob.velocity.x, "y": ob.velocity.y}
                        obs_seen.setdefault(oid, {"type": t, "pos": pos, "vel": vel})
                except Exception:
                    pass
        except Exception as e:
            print(f"[warn] {rf} obstacles: {e}", file=sys.stderr)
    # 类型映射
    type_names = {0: "UNKNOWN", 1: "UNKNOWN_MOVABLE", 2: "UNKNOWN_UNMOVABLE",
                  3: "PEDESTRIAN", 4: "BICYCLE", 5: "VEHICLE"}
    for oid, info in list(obs_seen.items())[:10]:
        print(f"  obs{oid}: type={type_names.get(info['type'], info['type'])} "
              f"pos=({info['pos']['x']:.2f},{info['pos']['y']:.2f}) "
              f"vel=({info['vel']['x']:.2f},{info['vel']['y']:.2f})")

    # 3. planning decisions (ADCTrajectory tail)
    print("\n=== Planning (ADCTrajectory, sample) ===")
    for rf in rec_files:
        try:
            n = 0
            last = None
            for msg in read_messages(rf, "/apollo/planning"):
                try:
                    traj = planning_pb2.ADCTrajectory()
                    traj.ParseFromString(msg.message)
                    if len(traj.trajectory_point) > 0:
                        last_pt = traj.trajectory_point[-1]
                        last = {"x": last_pt.path_point.x,
                                "y": last_pt.path_point.y,
                                "v": last_pt.v}
                        n += 1
                except Exception:
                    pass
            print(f"  {rf}: planning_frames={n} last_pt={last}")
        except Exception as e:
            print(f"[warn] {rf} planning: {e}", file=sys.stderr)

    # 保存 JSON
    out = "output/record_analysis.json"
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"positions": positions[:], "obstacles": obs_seen}, f,
                  ensure_ascii=False, indent=1)
    print(f"\nJSON written: {out}")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else \
        "/home/skye/.apollo/resources/records"
    if not HAS_CYBER:
        print("ERROR: cyber python record 不可用，无法解析", file=sys.stderr)
        sys.exit(2)
    analyze(path)


if __name__ == "__main__":
    main()
