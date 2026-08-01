#!/usr/bin/env python3
"""计算车辆轨迹在转弯车道的横向偏移 — 宿主机运行
用法：python3 scripts/lane_lateral_offset.py <record> <base_map> <start_frame> <end_frame>
输出: 车辆轨迹点 + 最近lane中心 + 横向偏移
"""
import sys, os, math

for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/opt/apollo/neo/src/cyber/python",
           "/opt/apollo/neo/src"]:
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
from modules.common_msgs.map_msgs import map_pb2

rf = sys.argv[1]
map_file = sys.argv[2]
f0, f1 = int(sys.argv[3]), int(sys.argv[4])

# 载入 lane centerlines（转弯相关车道）+ 宽度
m = map_pb2.Map()
with open(map_file, 'rb') as f:
    m.ParseFromString(f.read())
lanes = {}
lane_widths = {}
for lane in m.lane:
    if any(w in lane.id.id for w in ['1353', '697', '817']):
        pts = []
        for seg in lane.central_curve.segment:
            for pt in seg.line_segment.point:
                pts.append((pt.x, pt.y))
        lanes[lane.id.id] = pts
        w_max = 0.0
        for s in list(lane.left_sample) + list(lane.right_sample):
            if s.width > w_max:
                w_max = s.width
        if w_max > 0:
            lane_widths[lane.id.id] = w_max
print("loaded lanes:", list(lanes.keys()))
print("lane widths:", {k: round(v,2) for k,v in lane_widths.items()})

# 车辆轨迹
pts = []
for msg in record.RecordReader(rf).read_messages():
    if msg.topic == "/apollo/localization/pose":
        pose = localization_pb2.LocalizationEstimate()
        pose.ParseFromString(msg.message)
        p = pose.pose.position
        v = pose.pose.linear_velocity
        pts.append((p.x, p.y, (v.x**2+v.y**2)**0.5))

# 对每个车辆点找最近 lane 中心，算横向偏移
def nearest_offset(px, py, lane_pts):
    best_d = 1e9; best_i = 0
    for i, (lx, ly) in enumerate(lane_pts):
        d = (px-lx)**2 + (py-ly)**2
        if d < best_d:
            best_d = d; best_i = i
    return math.sqrt(best_d), best_i

def signed_offset(px, py, lane_pts, idx):
    """带符号横向偏移：+ = 车道法线右侧(内弯侧取决于方向)，返回相对lane方向"""
    if idx >= len(lane_pts)-1 or idx <= 0:
        return 0.0
    # lane 切线方向
    dx = lane_pts[min(idx+1,len(lane_pts)-1)][0] - lane_pts[max(idx-1,0)][0]
    dy = lane_pts[min(idx+1,len(lane_pts)-1)][1] - lane_pts[max(idx-1,0)][1]
    L = math.hypot(dx, dy)
    if L == 0:
        return 0.0
    # 法线（右侧）
    nx, ny = dy/L, -dx/L
    # 车辆相对 lane 中心在法线方向的投影
    vx = px - lane_pts[idx][0]
    vy = py - lane_pts[idx][1]
    return vx*nx + vy*ny

print("\nframe  车x       车y      v     最近lane  横向偏移(有符号)  车道半宽")
for i in range(f0, min(f1, len(pts)), 10):
    px, py, v = pts[i]
    best_lane = None; best_off = 1e9; best_pt = None; best_idx = 0
    for lid, lpts in lanes.items():
        off, idx = nearest_offset(px, py, lpts)
        if off < best_off:
            best_off = off; best_lane = lid; best_pt = lpts[idx]; best_idx = idx
    s_off = signed_offset(px, py, lanes[best_lane], best_idx)
    half_w = lane_widths.get(best_lane, 1.75)/2 if best_lane in lane_widths else 1.75
    # 车半宽 1.055，判断是否超出
    exceed = "!!!" if abs(s_off) + 1.055 > half_w else ""
    print(f"{i:5d} {px:9.2f} {py:9.2f} {v:5.2f}  {best_lane:10s} {s_off:7.2f}m  {half_w:.2f}m {exceed}")
