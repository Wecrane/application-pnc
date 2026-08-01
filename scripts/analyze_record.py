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
    pass

# 尝试导入 cyber record
try:
    from cyber.python.cyber_py3 import record
    HAS_CYBER = True
except Exception as e:
    HAS_CYBER = False
    print(f"[warn] cyber_py3 import failed: {e}", file=sys.stderr)


def list_records(path):
    """返回 record 文件列表（支持文件或目录，按分段顺序）。"""
    if os.path.isdir(path):
        files = sorted(f for f in os.listdir(path) if f.endswith(".record"))
        return [os.path.join(path, f) for f in files]
    return [path]


def read_messages(record_file, channel):
    """用 cyber record 读取指定通道消息。"""
    reader = record.RecordReader(record_file)
    for msg in reader.read_messages():
        if msg.channel_name == channel:
            yield msg


def analyze(path):
    rec_files = list_records(path)
    print(f"Records: {rec_files}")

    # 1. 车辆位置/速度（localization）
    print("\n=== 车辆轨迹（localization/pose）===")
    positions = []
    for rf in rec_files:
        try:
            for msg in read_messages(rf, "/apollo/localization/pose"):
                try:
                    d = json.loads(msg.data)
                    pose = d.get("pose", {})
                    x = pose.get("position", {}).get("x")
                    y = pose.get("position", {}).get("y")
                    # 速度（m/s）
                    vx = pose.get("linear_velocity", {}).get("x", 0)
                    vy = pose.get("linear_velocity", {}).get("y", 0)
                    v = (vx * vx + vy * vy) ** 0.5
                    positions.append({"x": x, "y": y, "v": v})
                except Exception:
                    pass
        except Exception as e:
            print(f"[warn] {rf} localization: {e}", file=sys.stderr)

    if positions:
        # 采样输出（每 50 帧一条）
        print(f"总帧数: {len(positions)}")
        for i in range(0, len(positions), 50):
            p = positions[i]
            print(f"  frame{i}: x={p['x']:.3f} y={p['y']:.3f} v={p['v']:.3f} m/s")
        # 停车点：速度最低处
        min_v = min(positions, key=lambda p: p["v"])
        print(f"\n最低速点: x={min_v['x']:.3f} y={min_v['y']:.3f} v={min_v['v']:.3f} m/s")
        # 轨迹范围
        xs = [p["x"] for p in positions]
        ys = [p["y"] for p in positions]
        print(f"轨迹范围: x[{min(xs):.2f},{max(xs):.2f}] y[{min(ys):.2f},{max(ys):.2f}]")

    # 2. 感知障碍物（行人）
    print("\n=== 感知障碍物（perception/obstacles）===")
    obs_seen = {}
    for rf in rec_files:
        try:
            for msg in read_messages(rf, "/apollo/perception/obstacles"):
                try:
                    d = json.loads(msg.data)
                    for ob in d.get("perception_obstacle", []):
                        oid = ob.get("id")
                        t = ob.get("type", 0)
                        pos = ob.get("position", {})
                        vel = ob.get("velocity", {})
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
              f"pos=({info['pos'].get('x',0):.2f},{info['pos'].get('y',0):.2f}) "
              f"vel=({info['vel'].get('x',0):.2f},{info['vel'].get('y',0):.2f})")

    # 3. 规划决策（ADCTrajectory 尾部，看是否 STOP）
    print("\n=== 规划（planning/ADCTrajectory，抽样）===")
    for rf in rec_files:
        try:
            n = 0
            last = None
            for msg in read_messages(rf, "/apollo/planning"):
                try:
                    d = json.loads(msg.data)
                    tp = d.get("trajectory_point", [])
                    if tp:
                        last_pt = tp[-1]
                        last = {"x": last_pt.get("path_point", {}).get("x"),
                                "y": last_pt.get("path_point", {}).get("y"),
                                "v": last_pt.get("v")}
                        n += 1
                except Exception:
                    pass
            print(f"  {rf}: planning帧数={n} 末帧轨迹点={last}")
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
