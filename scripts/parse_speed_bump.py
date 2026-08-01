#!/usr/bin/env python3
"""解析 base_map.bin 中的减速带（SpeedBump）信息 — 在 Apollo 容器内运行

用途：确认赛题地图中减速带的位置、与车道的 overlap、车道限速，
     判断减速带限速机制（speed_bump_speed_limit gflag + 地图车道限速）是否覆盖主车路线。

用法（容器内）：
    python3 scripts/parse_speed_bump.py [base_map.bin路径] [--json]

默认路径：data/map_data/Xh_2026_contest/base_map.bin
输出：终端打印 + 可选 JSON 到 output/speed_bump_analysis.json
"""

import sys
import os
import json

# Container locale may be ASCII; force UTF-8 output to avoid Unicode errors.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except (AttributeError, ValueError):
    try:
        import io
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                      errors="replace")
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8",
                                      errors="replace")
    except (AttributeError, ValueError, io.UnsupportedOperation):
        pass

# ---------------------------------------------------------------------------
# 1. 尝试定位 map_pb2 的 python 编译产物（容器内多路径候选）
# ---------------------------------------------------------------------------
CANDIDATE_PATHS = [
    # 工作区内 bazel 产物（buildtool build 后生成）
    "/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
    "/apollo_workspace/bazel-out/k8-opt/bin",
    "/apollo/bazel-out/k8-opt/bin/external/apollo_src",
    "/apollo/bazel-out/k8-opt/bin",
    # 工作区 .cache 下 execroot
    "/apollo_workspace/.cache/bazel",
    # 已安装包路径
    "/opt/apollo/lib/python3/site-packages",
    "/opt/apollo/cyber/python",
]

def setup_proto_paths():
    """把可能包含 modules.common_msgs 的父目录加入 sys.path。"""
    # 先清掉默认的 /apollo_workspace（host 路径在容器里可能不存在）
    for base in ["/apollo_workspace", "/apollo"]:
        for extra in [
            "/bazel-out/k8-opt/bin/external/apollo_src",
            "/bazel-out/k8-opt/bin",
        ]:
            p = base + extra
            if os.path.isdir(p):
                sys.path.insert(0, p)
    # 通用递归查找：在常见根下找 map_pb2.py
    for root in ["/apollo_workspace/bazel-out", "/apollo/bazel-out",
                 "/opt/apollo", "/apollo_workspace/.cache/bazel"]:
        if not os.path.isdir(root):
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            if "map_pb2.py" in filenames:
                # 找到包含 modules 的父目录
                idx = dirpath.find("/modules/common_msgs/map_msgs")
                if idx != -1:
                    parent = dirpath[:idx]
                    if parent not in sys.path:
                        sys.path.insert(0, parent)
                    # 也加 external/apollo_src 的上级
                    if "/external/apollo_src" in parent:
                        sys.path.insert(0, parent.replace("/external/apollo_src", ""))
                # 找到即返回，避免遍历太深
                return True
    return False


def load_map(base_map_path):
    """加载 Map protobuf，返回 (Map, 使用的解析方式)。"""
    # 方式 A：python map_pb2
    try:
        setup_proto_paths()
        from modules.common_msgs.map_msgs import map_pb2
        m = map_pb2.Map()
        with open(base_map_path, "rb") as f:
            m.ParseFromString(f.read())
        return m, "python-map_pb2"
    except Exception as e:
        print(f"[warn] python map_pb2 import failed: {e}", file=sys.stderr)

    # 方式 B：protoc --decode_raw 兜底（输出原始字段，人工对照 proto 字段号）
    import subprocess
    try:
        raw = subprocess.run(
            ["protoc", "--decode_raw"],
            input=open(base_map_path, "rb").read(),
            capture_output=True,
        )
        if raw.returncode == 0:
            return raw.stdout.decode("utf-8", errors="replace"), "protoc-decode_raw"
    except Exception as e:
        print(f"[warn] protoc --decode_raw failed: {e}", file=sys.stderr)

    return None, "failed"


def analyze_python(m):
    """用 map_pb2 解析减速带信息。"""
    result = {
        "map_speed_bumps": [],
        "speed_bump_lanes": {},   # lane_id -> {speed_limit, in_overlap_with}
        "all_lane_speed_limits": {},  # 全地图车道限速统计
    }

    # 收集所有车道及其限速
    for lane in m.lane:
        result["all_lane_speed_limits"][lane.id.id] = lane.speed_limit

    # 1) 每个 SpeedBump 的 overlap
    for sb in m.speed_bump:
        info = {"id": sb.id.id, "overlaps": []}
        for ov_id in sb.overlap_id:
            info["overlaps"].append(ov_id.id)
        result["map_speed_bumps"].append(info)

    # 2) 遍历 overlap，找出 SpeedBump <-> Lane 的对应
    for ov in m.overlap:
        obj_ids = [o.id.id for o in ov.object]
        sb_ids = [i for i in obj_ids if str(i).startswith("SpeedBump")]
        lane_ids = [i for i in obj_ids if str(i).startswith("Lane_")]
        if sb_ids and lane_ids:
            for lid in lane_ids:
                result["speed_bump_lanes"].setdefault(lid, {
                    "speed_limit": result["all_lane_speed_limits"].get(lid),
                    "speed_bumps": [],
                })["speed_bumps"].extend(sb_ids)

    # 3) 统计：减速带车道限速分布
    bump_lane_limits = {}
    for lid, info in result["speed_bump_lanes"].items():
        sl = info["speed_limit"]
        bump_lane_limits.setdefault(sl, 0)
        bump_lane_limits[sl] += 1
    result["speed_bump_lane_speed_limit_distribution"] = bump_lane_limits

    # 4) 全图限速分布
    dist = {}
    for sl in result["all_lane_speed_limits"].values():
        dist[sl] = dist.get(sl, 0) + 1
    result["all_lane_speed_limit_distribution"] = dist

    return result


def main():
    base_map = sys.argv[1] if len(sys.argv) > 1 else \
        "data/map_data/Xh_2026_contest/base_map.bin"
    want_json = "--json" in sys.argv

    if not os.path.exists(base_map):
        print(f"ERROR: map file not found: {base_map}", file=sys.stderr)
        print("Usage: python3 scripts/parse_speed_bump.py [base_map.bin path] [--json]")
        sys.exit(1)

    m, mode = load_map(base_map)
    if m is None:
        print("Parse failed: neither python map_pb2 nor protoc is available.",
              file=sys.stderr)
        sys.exit(2)

    print(f"=== Parse mode: {mode} ===")
    print(f"Map: {base_map}")

    if mode == "python-map_pb2":
        result = analyze_python(m)

        print("\n--- Speed bumps in map ---")
        for sb in result["map_speed_bumps"]:
            print(f"  {sb['id']}: overlaps={sb['overlaps']}")

        print("\n--- Lanes overlapped with speed bumps (with speed limit) ---")
        for lid, info in sorted(result["speed_bump_lanes"].items()):
            print(f"  {lid}: speed_limit={info['speed_limit']} m/s "
                  f"({info['speed_limit']*3.6:.1f} km/h), "
                  f"speed_bumps={info['speed_bumps']}")

        print("\n--- Speed bump lane speed limit distribution ---")
        for sl, cnt in sorted(result["speed_bump_lane_speed_limit_distribution"].items()):
            print(f"  {sl} m/s ({sl*3.6:.1f} km/h): {cnt} lanes")

        print("\n--- All lane speed limit distribution ---")
        for sl, cnt in sorted(result["all_lane_speed_limit_distribution"].items()):
            print(f"  {sl} m/s ({sl*3.6:.1f} km/h): {cnt} lanes")

        print("\n--- Are speed bump lanes on main route (has pred/succ) ---")
        connected = 0
        for lid, info in sorted(result["speed_bump_lanes"].items()):
            lane = next((l for l in m.lane if l.id.id == lid), None)
            if lane is None:
                continue
            npre = len(lane.predecessor_id)
            nsucc = len(lane.successor_id)
            if npre > 0 and nsucc > 0:
                connected += 1
            else:
                print(f"  ! {lid}: pred/succ {npre}/{nsucc} (maybe not on main route)")
        print(f"  Total speed bump lanes: {len(result['speed_bump_lanes'])}, "
              f"main-route connected: {connected}")

        if want_json:
            out = "output/speed_bump_analysis.json"
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, "w") as f:
                json.dump(result, f, ensure_ascii=False, indent=1)
            print(f"\nJSON written to: {out}")
    else:
        # protoc --decode_raw fallback: print first 3000 lines of raw output
        lines = m.splitlines()
        print(f"Total {len(lines)} lines of raw output, showing first 3000:")
        print("\n".join(lines[:3000]))


if __name__ == "__main__":
    main()
