#!/usr/bin/env python3
"""批量解析 2026 星火大赛全部 8 个场景的路线与地图元素 — 在 Apollo 容器内运行

功能：
  1. 自动扫描 ~/.apollo/resources/scenario_sets/*/scenarios/*.json（全国总决赛仿真赛场景集）
  2. 自动建立 mapId -> base_map.bin 映射（扫描 data/map_data/*/metaInfo.json）
  3. 对每个场景：起点/终点 -> 最近车道 -> 沿 successor 推算路线
  4. 输出路线经过的地图元素：SpeedBump / TrafficLight / Crosswalk / StopSign / Junction

用法（容器内）：
    python3 scripts/parse_scenarios.py                 # 解析全部 8 个场景
    python3 scripts/parse_scenarios.py <场景json路径>  # 解析单个场景
    python3 scripts/parse_scenarios.py --json          # 额外输出 output/scenarios_analysis.json
"""

import sys
import os
import json
import math
from collections import deque

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

SCENARIO_DIR = os.path.expanduser(
    "~/.apollo/resources/scenario_sets/*/scenarios")
MAP_DATA_DIR = "data/map_data"


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


def discover_maps():
    """扫描 data/map_data/*/metaInfo.json -> mapId 到 base_map.bin 的映射。"""
    mapping = {}
    if os.path.isdir(MAP_DATA_DIR):
        for d in os.listdir(MAP_DATA_DIR):
            meta = os.path.join(MAP_DATA_DIR, d, "metaInfo.json")
            if os.path.exists(meta):
                try:
                    with open(meta, encoding="utf-8") as f:
                        info = json.load(f)
                    for mid, _ in info.items():
                        mapping[mid] = os.path.join(MAP_DATA_DIR, d, "base_map.bin")
                except Exception:
                    pass
    return mapping


def discover_scenarios():
    """扫描场景集目录下的所有场景 json。"""
    import glob
    files = sorted(glob.glob(os.path.join(SCENARIO_DIR, "*.json")))
    return files


def point_to_lane_dist(px, py, lane):
    min_d = float("inf")
    best_s = 0.0
    coords = []
    for seg in lane.central_curve.segment:
        if seg.HasField("line_segment"):
            # LineSegment 是 repeated PointENU point
            for p in seg.line_segment.point:
                coords.append((p.x, p.y))
        elif seg.HasField("arc"):
            # 弧线段：用起终点近似
            if seg.arc.HasField("start"):
                coords.append((seg.arc.start.x, seg.arc.start.y))
            if seg.arc.HasField("end"):
                coords.append((seg.arc.end.x, seg.arc.end.y))
    if not coords:
        return float("inf"), 0.0
    acc = 0.0
    accs = [0.0]
    for i in range(len(coords) - 1):
        dx = coords[i + 1][0] - coords[i][0]
        dy = coords[i + 1][1] - coords[i][1]
        seg_len = math.hypot(dx, dy)
        acc += seg_len
        accs.append(acc)
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


def build_element_index(m):
    """lane -> [elements] 与 全局元素 id 列表。"""
    lane2elem = {}
    elem_types = {
        "SpeedBump": [], "TrafficLight": [], "Crosswalk": [],
        "StopSign": [], "Junction": [], "YieldSign": [], "ParkingSpace": [],
    }
    # 各元素对象
    for sb in m.speed_bump:
        elem_types["SpeedBump"].append(sb.id.id)
    for tl in m.signal:
        elem_types["TrafficLight"].append(tl.id.id)
    for cw in m.crosswalk:
        elem_types["Crosswalk"].append(cw.id.id)
    for ss in m.stop_sign:
        elem_types["StopSign"].append(ss.id.id)
    for jn in m.junction:
        elem_types["Junction"].append(jn.id.id)
    # overlap -> lane 关联
    prefix_map = {
        "SpeedBump": "SpeedBump", "TrafficLight": "SignalLight",
        "Crosswalk": "Crosswalk", "StopSign": "StopSign",
        "Junction": "Junction", "YieldSign": "YieldSign",
        "ParkingSpace": "ParkingSpace",
    }
    for ov in m.overlap:
        obj_ids = [o.id.id for o in ov.object]
        lane_ids = [i for i in obj_ids if str(i).startswith("Lane_")]
        if not lane_ids:
            continue
        for pfx in prefix_map.values():
            hits = [i for i in obj_ids if str(i).startswith(pfx)]
            for h in hits:
                for lid in lane_ids:
                    lane2elem.setdefault(lid, set()).add((pfx, h))
    return lane2elem, elem_types


def analyze_scenario(path, map_by_id):
    with open(path, encoding="utf-8") as f:
        sc = json.load(f)
    desc = sc.get("descriptionEnTokens", ["unknown"])
    name = desc[0] if desc else sc.get("id")
    map_id = sc.get("mapId")
    base_map = map_by_id.get(map_id)
    result = {"id": sc.get("id"), "name": name, "mapId": map_id}

    if not base_map or not os.path.exists(base_map):
        result["error"] = f"map not found for mapId {map_id}"
        return result
    result["map"] = os.path.basename(os.path.dirname(base_map))

    m = load_map(base_map)
    ac = sc.get("scenario", {}).get("autoCarInfo", {})
    start = ac.get("start", {})
    end = ac.get("end", {})
    sx, sy = start.get("x"), start.get("y")
    ex, ey = end.get("x"), end.get("y")
    result["start"] = [sx, sy]
    result["end"] = [ex, ey]

    start_lane, sd = find_nearest_lane(m, sx, sy)
    end_lane, ed = find_nearest_lane(m, ex, ey)
    if start_lane is None or end_lane is None:
        result["error"] = "cannot find nearest lane"
        return result
    result["start_lane"] = start_lane.id.id
    result["end_lane"] = end_lane.id.id

    lane_by_id = {l.id.id: l for l in m.lane}
    start_id, end_id = start_lane.id.id, end_lane.id.id

    # 正向 BFS：从 start 沿 successor 找 end
    parent = {}
    visited = {start_id}
    queue = deque([start_id])
    route = None
    while queue:
        cur = queue.popleft()
        if cur == end_id:
            route = []
            node = cur
            while node != start_id:
                route.append(node)
                node = parent[node]
            route.append(start_id)
            route.reverse()
            break
        lane = lane_by_id.get(cur)
        if lane is None:
            continue
        for succ in lane.successor_id:
            if succ.id not in visited:
                visited.add(succ.id)
                parent[succ.id] = cur
                queue.append(succ.id)

    # 正向失败则反向：从 end 沿 predecessor 找 start
    if route is None:
        parent = {}
        visited = {end_id}
        queue = deque([end_id])
        while queue:
            cur = queue.popleft()
            if cur == start_id:
                route = []
                node = cur
                while node != end_id:
                    route.append(node)
                    node = parent[node]
                route.append(end_id)
                # 此时 route 已是 start->end 正向
                break
            lane = lane_by_id.get(cur)
            if lane is None:
                continue
            for pred in lane.predecessor_id:
                if pred.id not in visited:
                    visited.add(pred.id)
                    parent[pred.id] = cur
                    queue.append(pred.id)

    result["_debug"] = {
        "start_id": start_id,
        "end_id": end_id,
        "n_parent": len(parent),
        "route_type": str(type(route).__name__),
        "route_len": len(route) if route else -1,
        "route_head": [str(x) for x in route[:5]] if route else None,
    }

    if route is None:
        result["error"] = "no path between start and end lanes"
        return result

    # 去重保持顺序（应对环）
    seen = set()
    clean = []
    for lid in route:
        if lid not in seen:
            seen.add(lid)
            clean.append(lid)
    result["route_lanes"] = clean

    # 收集路线元素
    lane2elem, elem_types = build_element_index(m)
    route_elems = {"SpeedBump": [], "TrafficLight": [], "Crosswalk": [],
                   "StopSign": [], "Junction": [], "YieldSign": [],
                   "ParkingSpace": []}
    for lid in clean:
        for pfx, eid in lane2elem.get(lid, set()):
            if eid not in route_elems[pfx]:
                route_elems[pfx].append(eid)
    result["route_elements"] = route_elems
    # 全图元素数量
    result["map_element_counts"] = {k: len(v) for k, v in elem_types.items()}
    return result


def main():
    want_json = "--json" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]

    # 输出同时打印到终端并写入文件（UTF-8），方便复制
    os.makedirs("output", exist_ok=True)
    log_file = open("output/scenarios_analysis.txt", "w", encoding="utf-8")

    def log(msg=""):
        print(msg)
        log_file.write(msg + "\n")

    map_by_id = discover_maps()
    if not map_by_id:
        log("WARN: no map data found under data/map_data/")
    log(f"Discovered maps: {len(map_by_id)}")
    for mid, path in map_by_id.items():
        log(f"  {mid} -> {path}")

    if args:
        files = args
    else:
        files = discover_scenarios()
    if not files:
        log("ERROR: no scenario json found")
        log_file.close()
        sys.exit(1)

    log(f"\nScenarios to analyze: {len(files)}\n")
    all_results = []
    for f in files:
        log("=" * 70)
        r = analyze_scenario(f, map_by_id)
        all_results.append(r)
        log(f"Scenario: {r.get('name')}  (id={r.get('id')})")
        log(f"  map: {r.get('map')}  mapId: {r.get('mapId')}")
        if r.get("error"):
            log(f"  ERROR: {r['error']}")
            continue
        log(f"  start({r['start'][0]:.2f},{r['start'][1]:.2f}) lane={r.get('start_lane')} "
            f"-> end({r['end'][0]:.2f},{r['end'][1]:.2f}) lane={r.get('end_lane')}")
        dbg = r.get("_debug")
        if dbg:
            log(f"  [dbg] start_id={dbg['start_id']!r} end_id={dbg['end_id']!r} "
                f"n_parent={dbg['n_parent']} route_type={dbg['route_type']} "
                f"route_head={dbg['route_head']}")
        lanes = r.get("route_lanes", [])
        log(f"  route lanes ({len(lanes)}): {' -> '.join(lanes[:40])}"
            + (" ..." if len(lanes) > 40 else ""))
        elems = r.get("route_elements", {})
        nonempty = {k: v for k, v in elems.items() if v}
        if nonempty:
            for k, v in nonempty.items():
                log(f"  route {k}: {v}")
        else:
            log("  route elements: (none)")
        log(f"  map total: {r.get('map_element_counts')}")

    log(f"\n>>> 完整输出已写入: output/scenarios_analysis.txt")

    if want_json:
        out = "output/scenarios_analysis.json"
        with open(out, "w", encoding="utf-8") as f:
            json.dump(all_results, f, ensure_ascii=False, indent=1)
        log(f">>> JSON 已写入: {out}")

    log_file.close()


if __name__ == "__main__":
    main()
