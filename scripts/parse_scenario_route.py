#!/usr/bin/env python3
"""解析减速带通行场景的路线，确认路线上经过哪些 SpeedBump — 在 Apollo 容器内运行

用法（容器内）：
    python3 scripts/parse_scenario_route.py [场景json路径]

默认场景：~/.apollo/resources/scenario_sets/6a6c99a3abf06e2d1d590840/scenarios/6a6c3d8babf06e0e5f59083c.json
          （xh_2026_gsxx_减速带通行）
输出：起点/终点所在车道、路线 lane 序列、路线上经过的减速带及其 s 位置
"""

import sys
import os
import json
import math

# 强制 UTF-8（容器内 locale 可能为 ASCII）
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

BASE_MAP = "data/map_data/Xh_2026_contest/base_map.bin"


def setup_proto_paths():
    for base in ["/apollo_workspace", "/apollo"]:
        for extra in [
            "/bazel-out/k8-opt/bin/external/apollo_src",
            "/bazel-out/k8-opt/bin",
        ]:
            p = base + extra
            if os.path.isdir(p) and p not in sys.path:
                sys.path.insert(0, p)
    for root in ["/apollo_workspace/bazel-out", "/apollo/bazel-out",
                 "/opt/apollo", "/apollo_workspace/.cache/bazel"]:
        if not os.path.isdir(root):
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            if "map_pb2.py" in filenames:
                idx = dirpath.find("/modules/common_msgs/map_msgs")
                if idx != -1:
                    parent = dirpath[:idx]
                    if parent not in sys.path:
                        sys.path.insert(0, parent)
                    if "/external/apollo_src" in parent:
                        sys.path.insert(0, parent.replace("/external/apollo_src", ""))
                return True
    return False


def load_map(path):
    setup_proto_paths()
    from modules.common_msgs.map_msgs import map_pb2
    m = map_pb2.Map()
    with open(path, "rb") as f:
        m.ParseFromString(f.read())
    return m


def point_to_lane_dist(px, py, lane):
    """计算点到 lane 中央曲线的最短距离。"""
    min_d = float("inf")
    best_s = 0.0
    pts = [(p.x, p.y) for p in lane.central_curve.segment]
    # central_curve.segment 是 LineSegment 列表，需要展开
    coords = []
    for seg in lane.central_curve.segment:
        if seg.HasField("line_segment"):
            coords.append((seg.line_segment.start.x, seg.line_segment.start.y))
            coords.append((seg.line_segment.end.x, seg.line_segment.end.y))
        elif seg.HasField("arc"):
            pass
    if not coords:
        return float("inf"), 0.0
    # 用折线近似
    acc = 0.0
    accs = [0.0]
    for i in range(len(coords) - 1):
        dx = coords[i + 1][0] - coords[i][0]
        dy = coords[i + 1][1] - coords[i][1]
        seg_len = math.hypot(dx, dy)
        acc += seg_len
        accs.append(acc)
        # 点到线段距离
        t = ((px - coords[i][0]) * dx + (py - coords[i][1]) * dy) / (seg_len * seg_len + 1e-9)
        t = max(0.0, min(1.0, t))
        cx = coords[i][0] + t * dx
        cy = coords[i][1] + t * dy
        d = math.hypot(px - cx, py - cy)
        if d < min_d:
            min_d = d
            best_s = accs[i] + t * seg_len
    return min_d, best_s


def find_nearest_lane(m, px, py):
    best = None
    best_d = float("inf")
    for lane in m.lane:
        d, _ = point_to_lane_dist(px, py, lane)
        if d < best_d:
            best_d = d
            best = lane
    return best, best_d


def main():
    scenario_json = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.expanduser("~/.apollo/resources/scenario_sets/"
                           "6a6c99a3abf06e2d1d590840/scenarios/"
                           "6a6c3d8babf06e0e5f59083c.json")

    if not os.path.exists(BASE_MAP):
        print(f"ERROR: base_map not found: {BASE_MAP}", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(scenario_json):
        print(f"ERROR: scenario json not found: {scenario_json}", file=sys.stderr)
        sys.exit(1)

    m = load_map(BASE_MAP)
    print(f"Map loaded: {BASE_MAP}")

    with open(scenario_json) as f:
        sc = json.load(f)
    ac = sc.get("scenario", {}).get("autoCarInfo", {})
    start = ac.get("start", {})
    end = ac.get("end", {})
    sx, sy = start.get("x"), start.get("y")
    ex, ey = end.get("x"), end.get("y")
    print(f"Scenario: {sc.get('descriptionEnTokens', 'unknown')}")
    print(f"Start: ({sx}, {sy})  heading={start.get('heading')}")
    print(f"End:   ({ex}, {ey})")

    start_lane, sd = find_nearest_lane(m, sx, sy)
    end_lane, ed = find_nearest_lane(m, ex, ey)
    if start_lane is None or end_lane is None:
        print("ERROR: cannot find nearest lane for start/end", file=sys.stderr)
        sys.exit(1)
    print(f"\nStart nearest lane: {start_lane.id.id} (dist={sd:.2f}m)")
    print(f"End nearest lane:   {end_lane.id.id} (dist={ed:.2f}m)")

    # 建 lane 索引
    lane_by_id = {l.id.id: l for l in m.lane}

    # BFS 从 start_lane 沿 successor 找 end_lane
    from collections import deque
    start_id = start_lane.id.id
    end_id = end_lane.id.id
    parent = {}
    visited = {start_id}
    q = deque([start_id])
    found = False
    while q:
        cur = q.popleft()
        if cur == end_id:
            found = True
            break
        lane = lane_by_id.get(cur)
        if lane is None:
            continue
        for succ in lane.successor_id:
            if succ.id not in visited:
                visited.add(succ.id)
                parent[succ.id] = cur
                q.append(succ.id)
    if not found:
        print("WARN: no path from start lane to end lane via successors "
              "(may need to search both directions or via junctions)")
        # 反向：从 end 向 start 找（可能起点终点同一条 lane 或相邻）
        # 先检查是否同一条 lane
        if start_id == end_id:
            found = True

    if found:
        path = [end_id]
        while path[-1] in parent and path[-1] != start_id:
            path.append(parent[path[-1]])
        path.reverse()
        if start_id != end_id:
            pass
        print(f"\nRoute lanes ({len(path)}):")
        print("  " + " -> ".join(path[:50]))
        if len(path) > 50:
            print(f"  ... ({len(path)} lanes total)")

        # 找路径上经过的减速带
        print("\nSpeed bumps on route:")
        sb_found = []
        lane2sb = {}
        # 构建 lane -> speed bump 映射
        for ov in m.overlap:
            obj_ids = [o.id.id for o in ov.object]
            sb_ids = [i for i in obj_ids if str(i).startswith("SpeedBump")]
            lane_ids = [i for i in obj_ids if str(i).startswith("Lane_")]
            if sb_ids and lane_ids:
                for lid in lane_ids:
                    lane2sb.setdefault(lid, set()).update(sb_ids)
        for lid in path:
            if lid in lane2sb:
                for sb in sorted(lane2sb[lid]):
                    sb_found.append((lid, sb))
        if sb_found:
            for lid, sb in sb_found:
                print(f"  lane {lid} -> {sb}")
        else:
            print("  (none - route has no SpeedBump map element)")
    else:
        print("\nNo path found. Checking if start/end share route manually...")
        # 输出起点终点 lane 的相邻信息供排查
        for lid in (start_id, end_id):
            l = lane_by_id.get(lid)
            if l:
                succs = [s.id for s in l.successor_id]
                preds = [p.id for p in l.predecessor_id]
                print(f"  lane {lid}: pred={preds} succ={succs}")


if __name__ == "__main__":
    main()
