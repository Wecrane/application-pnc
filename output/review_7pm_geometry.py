#!/usr/bin/env python3
"""独立审查：7号下午_1 场景几何精确核算（纯研究，不改任何配置）。

解析 base_map.txt：
1. 提取车道（central_curve / predecessor / successor / length）
2. 提取 crosswalk 多边形
3. 找自车车道链（start->end），算累计 s
4. 投影 Crosswalk_1 / Crosswalk_5 / 评分减速带区 到 s
5. 计算自车在关键位置的 x 坐标，验证是否压入横道
"""
import json
import math
import re
import sys
from collections import OrderedDict

MAP = "/home/skye/application-pnc/data/map_data/hehai_contest_7_pm_3/base_map.txt"
SCEN = "/home/skye/application-pnc/shiche/car_890/grading_console/auto_drive_console/7号下午_1.json"
METRIC = "/home/skye/application-pnc/shiche/car_890/grading_console/auto_drive_console/metric/7号下午_1号赛道_crosswalk_road_speed.json"

START = (272589.88041370973, 3476108.4951679576)
END = (272594.16429722705, 3476211.376412994)


def parse_blocks(text, block_name):
    """提取顶层名为 block_name 的块，返回 (content, start_idx) 列表。"""
    pat = re.compile(r'(?m)^%s\s*\{' % re.escape(block_name))
    blocks = []
    for m in pat.finditer(text):
        i = m.end()
        depth = 1
        while i < len(text) and depth > 0:
            c = text[i]
            if c == '{':
                depth += 1
            elif c == '}':
                depth -= 1
            i += 1
        blocks.append(text[m.start():i])
    return blocks


def get_field(block, field):
    """从块内取 field 的值（string / number / {id:"..."} message）。"""
    m = re.search(r'(?m)^\s*%s\s*:\s*"([^"]*)"' % re.escape(field), block)
    if m:
        return m.group(1)
    m = re.search(r'(?m)^\s*%s\s*:\s*(-?[\d.]+(?:[eE][+-]?\d+)?)' % re.escape(field), block)
    if m:
        return float(m.group(1))
    m = re.search(r'(?m)^\s*%s\s*\{\s*id\s*:\s*"([^"]*)"' % re.escape(field), block)
    if m:
        return m.group(1)
    return None


def get_points(block, struct):
    """提取 struct 内所有 point{x,y}。struct 形如 'central_curve' 或 'polygon'。"""
    m = re.search(r'(?m)^\s*%s\s*\{' % re.escape(struct), block)
    if not m:
        return []
    i = m.end()
    depth = 1
    while i < len(block) and depth > 0:
        if block[i] == '{':
            depth += 1
        elif block[i] == '}':
            depth -= 1
        i += 1
    sub = block[m.start():i]
    pts = []
    for pm in re.finditer(r'point\s*\{([^}]*)\}', sub):
        s = pm.group(1)
        x = re.search(r'x\s*:\s*(-?[\d.]+(?:[eE][+-]?\d+)?)', s)
        y = re.search(r'y\s*:\s*(-?[\d.]+(?:[eE][+-]?\d+)?)', s)
        if x and y:
            pts.append((float(x.group(1)), float(y.group(1))))
    return pts


def poly_length(pts):
    return sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
               for i in range(len(pts)-1))


def main():
    with open(MAP) as f:
        text = f.read()

    lanes = []
    for blk in parse_blocks(text, 'lane'):
        lid = get_field(blk, 'id')
        if not lid:
            continue
        pts = get_points(blk, 'central_curve')
        pred = get_field(blk, 'predecessor_id')
        succ = get_field(blk, 'successor_id')
        length = get_field(blk, 'length') or 0.0
        # 车道边界
        lb = get_points(blk, 'left_boundary')
        rb = get_points(blk, 'right_boundary')
        lanes.append({
            'id': lid, 'pts': pts, 'pred': pred, 'succ': succ,
            'length': length, 'lb': lb, 'rb': rb,
        })
    by_id = {l['id']: l for l in lanes}

    cws = {}
    for blk in parse_blocks(text, 'crosswalk'):
        cid = get_field(blk, 'id')
        pts = get_points(blk, 'polygon')
        cws[cid] = pts

    print("== 车道数:", len(lanes), " Crosswalk 数:", len(cws))

    def dist_pt_line(p, a, b):
        vx, vy = b[0]-a[0], b[1]-a[1]
        wx, wy = p[0]-a[0], p[1]-a[1]
        L2 = vx*vx + vy*vy
        if L2 == 0:
            return math.hypot(wx, wy)
        t = max(0.0, min(1.0, (wx*vx + wy*vy)/L2))
        return math.hypot(p[0]-(a[0]+t*vx), p[1]-(a[1]+t*vy))

    def lane_contains(lane, p, tol=1.5):
        """点是否在车道（中心线+半宽）附近。"""
        pts = lane['pts']
        if len(pts) < 2:
            return False
        d = min(dist_pt_line(p, pts[i], pts[i+1]) for i in range(len(pts)-1))
        return d < tol

    # 起点所在车道
    start_lanes = [l for l in lanes if lane_contains(l, START, tol=3.0)]
    print("\n起点 (%.2f, %.2f) 附近车道:" % START)
    for l in start_lanes[:8]:
        print("  ", l['id'], "首点", tuple(round(v,2) for v in l['pts'][0]),
              "末点", tuple(round(v,2) for v in l['pts'][-1]), "len", round(l['length'],2))

    # 终点所在车道
    end_lanes = [l for l in lanes if lane_contains(l, END, tol=3.0)]
    print("\n终点 (%.2f, %.2f) 附近车道:" % END)
    for l in end_lanes[:8]:
        print("  ", l['id'], "首点", tuple(round(v,2) for v in l['pts'][0]),
              "末点", tuple(round(v,2) for v in l['pts'][-1]), "len", round(l['length'],2))

    # ---- 车道链追踪（按 successor 图推导）----
    # 起点 Lane_48，依次 successor：Lane_48->Lane_52->Lane_49->Lane_67->Lane_66(终点)
    chain = []
    cur = 'Lane_48'
    seen = set()
    while cur and cur not in seen:
        seen.add(cur)
        chain.append(cur)
        nxt = by_id.get(cur)
        cur = nxt['succ'] if nxt and nxt['succ'] else None
    # 若终点车道未在链上，追加
    if end_lanes and chain[-1] != end_lanes[0]['id']:
        chain.append(end_lanes[0]['id'])

    print("\n== 车道链:", " -> ".join(chain))
    cum = 0.0
    lane_s = {}
    for lid in chain:
        ln = by_id[lid]
        pts = ln['pts']
        # 车道内 s 累计（用 polyline 长度）
        L = sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
                for i in range(len(pts)-1))
        lane_s[lid] = (cum, cum + L)
        print("  %-8s s=[%.2f, %.2f]  x首=%.2f x末=%.2f  L=%.2f" % (
            lid, cum, cum+L, pts[0][0], pts[-1][0], L))
        cum += L
    print("  总长 = %.2f m" % cum)

    # ---- 元素投影到链 ----
    ref_pts = []
    for lid in chain:
        ref_pts.extend(by_id[lid]['pts'])

    def proj_s(p):
        """点在折线上的 s（最近点投影）。"""
        best, best_t = None, None
        for i in range(len(ref_pts)-1):
            a, b = ref_pts[i], ref_pts[i+1]
            vx, vy = b[0]-a[0], b[1]-a[1]
            wx, wy = p[0]-a[0], p[1]-a[1]
            L2 = vx*vx + vy*vy
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, (wx*vx + wy*vy)/L2))
            d = math.hypot(p[0]-(a[0]+t*vx), p[1]-(a[1]+t*vy))
            seg_s = sum(math.hypot(ref_pts[j+1][0]-ref_pts[j][0],
                                   ref_pts[j+1][1]-ref_pts[j][1])
                        for j in range(i))
            ss = seg_s + t * math.sqrt(L2)
            if best is None or d < best:
                best, best_t = d, ss
        return best_t, best

    print("\n== 关键元素 s 投影（沿车道链，起自起点投影）")
    s_start, d_start = proj_s(START)
    print("  起点投影 s=%.2f (d=%.2f)" % (s_start, d_start))
    for name, poly in cws.items():
        ss = [proj_s(pt)[0] for pt in poly]
        print("  %-12s Δs=[%.2f, %.2f] 宽=%.2f" % (name, min(ss), max(ss), poly_length(poly)))
    # 评分减速带区（7号下午 metric）
    region = [(272593.2476553972, 3476137.559967942),
              (272593.140862391, 3476132.006728918),
              (272585.7721433669, 3476132.113521961),
              (272585.4517632103, 3476137.3463817965)]
    ss = [proj_s(pt)[0] for pt in region]
    print("  %-12s Δs=[%.2f, %.2f]" % ("评分区", min(ss), max(ss)))


if __name__ == '__main__':
    main()
