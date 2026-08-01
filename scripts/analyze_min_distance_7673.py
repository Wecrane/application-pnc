#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
研究任务: 计算所有回放中自车与行人7673/护栏的最小距离
验证 "停车后与障碍物小于2米扣20分" 假设

数据源: output/*.txt (多个版本回放)
自车: 4.933 x 2.11 m, heading≈0 (东向), center(x,y)
行人7673: 5 x 2 x 1.8 m, heading≈-21° (沿轨迹走向)
护栏: unknownUnmovable, 长 x 宽 = 30/15 x 0.3 m, heading≈0
"""
import math, re, os, glob

OUT = '/home/skye/application-pnc/output'

# ---------- 常量 ----------
VEH_LEN, VEH_WID = 4.933, 2.11
PED_LEN, PED_WID = 5.0, 2.0
PED_HEADING = math.radians(-21.05)   # 轨迹方向 (423446.67,4437610.95)->(423475.75,4437599.76)
# 护栏: id -> (cx, cy, length, width, heading)
GUARDRAILS = {
    '3098': (423395.25, 4437613.01, 30.0, 0.3, 1.35e-05),
    '4717': (423416.12, 4437613.35, 15.0, 0.3, -0.0077),
    '4857': (423298.30, 4437613.23, 30.0, 0.3, -0.0058),
    '5530': (423438.43, 4437612.88, 30.0, 0.3, -1.5e-10),
    '6324': (423329.67, 4437613.14, 30.0, 0.3, -0.0036),
    '6503': (423362.17, 4437613.03, 30.0, 0.3, 0.0020),
}
# 无 y 数据的版本默认自车 y
DEFAULT_VEH_Y = 4437610.77

# ---------- 几何工具 ----------
def rect_pts(cx, cy, h, L, W):
    c, s = math.cos(h), math.sin(h)
    hx, hy = L / 2.0, W / 2.0
    pts = []
    for lx, ly in [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]:
        pts.append((cx + lx * c - ly * s, cy + lx * s + ly * c))
    return pts

def orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

def seg_intersect(a, b, c, d):
    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    return o1 * o2 < 0 and o3 * o4 < 0

def point_in_poly(p, poly):
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xint = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < xint:
                inside = not inside
    return inside

def dist_point_seg(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

def rect_rect_dist(p1, p2):
    """两个凸多边形(矩形)的最短距离, 相交则返回0"""
    for i in range(4):
        for j in range(4):
            if seg_intersect(p1[i], p1[(i + 1) % 4], p2[j], p2[(j + 1) % 4]):
                return 0.0
    if point_in_poly(p1[0], p2) or point_in_poly(p2[0], p1):
        return 0.0
    best = float('inf')
    for poly, other in ((p1, p2), (p2, p1)):
        for i in range(4):
            a, b = poly[i], poly[(i + 1) % 4]
            for p in other:
                best = min(best, dist_point_seg(p, a, b))
    return best

# ---------- 解析 ----------
def parse_ped_veh(path):
    """格式: t (ped_x,ped_y,ped_v) (veh_x,veh_v) dec  -> 无 veh_y"""
    rows = []
    pat = re.compile(r'^\s*([-\d.]+)\s+\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+\(([-\d.]+),([-\d.]+)\)\s+(\S+)')
    with open(path) as f:
        for line in f:
            m = pat.match(line)
            if m:
                t, px, py, pv, vx, vv, dec = m.groups()
                rows.append((float(t), float(px), float(py), float(pv),
                             float(vx), float(vv), dec))
    return rows

def parse_ego_ped(path):
    """格式: t (ped_x,ped_y,ped_v) (veh_x,veh_y,veh_v) dec"""
    rows = []
    pat = re.compile(r'^\s*([-\d.]+)\s+\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+(\S+)')
    with open(path) as f:
        for line in f:
            m = pat.match(line)
            if m:
                t, px, py, pv, vx, vy, vv, dec = m.groups()
                rows.append((float(t), float(px), float(py), float(pv),
                             float(vx), float(vy), float(vv), dec))
    return rows

def parse_y_precise(path):
    """格式: t=... veh_y=... traj_yoff=... dec=..."""
    rows = []
    pat = re.compile(r't=([-\d.]+)\s+veh_y=([-\d.]+)')
    with open(path) as f:
        for line in f:
            m = pat.search(line)
            if m:
                rows.append((float(m.group(1)), float(m.group(2))))
    return rows

def interp_y(ytab, t):
    if not ytab:
        return None
    if t <= ytab[0][0]:
        return ytab[0][1]
    if t >= ytab[-1][0]:
        return ytab[-1][1]
    # 二分
    lo, hi = 0, len(ytab) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ytab[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, y0 = ytab[lo]
    t1, y1 = ytab[hi]
    if t1 == t0:
        return y0
    return y0 + (y1 - y0) * (t - t0) / (t1 - t0)

# ---------- 分析 ----------
def analyze(version, rows, veh_y_fn):
    """rows: (t, px, py, pv, vx, vy_or_None, vv, dec)"""
    results = {
        'version': version,
        'n': len(rows),
        't_range': (rows[0][0], rows[-1][0]) if rows else None,
        'min_ped': None,      # (t, dist, veh_x, veh_y, ped_x, ped_y, vv, dec)
        'min_ped_center': None,
        'stop': None,          # 停车段: (t_stop, veh_x, veh_y, min_dist_to_ped_while_stopped, min_dist_to_5530_while_stopped, min_dist_to_nearest_guardrail_while_stopped, guardrail_id)
        'min_guardrail': {},   # guardrail_id -> (t, dist, veh_x, veh_y, vv)
        'pass_ped': None,      # 起步后(2209)过行人最近
        'stopped_seg': None,
    }
    # 停车状态跟踪
    stopped = False
    stop_seg_start = None
    stop_info = {}
    ped_boxes_cache = {}

    for i, r in enumerate(rows):
        t, px, py, pv, vx, vv, dec = r[0], r[1], r[2], r[3], r[4], r[6], r[7]
        vy = veh_y_fn(t) if veh_y_fn else None
        if vy is None:
            vy = DEFAULT_VEH_Y
        # 行人朝向: 用速度方向近似; 无速度方向则用轨迹方向
        # (数据中只有速度标量, 用轨迹方向 -21.05°)
        ph = PED_HEADING
        veh_rect = rect_pts(vx, vy, 0.0, VEH_LEN, VEH_WID)
        ped_rect = rect_pts(px, py, ph, PED_LEN, PED_WID)
        d = rect_rect_dist(veh_rect, ped_rect)
        dc = math.hypot(vx - px, vy - py)  # 中心距
        rec = (t, d, vx, vy, px, py, vv, dec)
        if results['min_ped'] is None or d < results['min_ped'][1]:
            results['min_ped'] = rec
        if results['min_ped_center'] is None or dc < results['min_ped_center'][1]:
            results['min_ped_center'] = (t, dc, vx, vy, px, py, vv, dec)
        # 护栏
        for gid, (gx, gy, gl, gw, gh) in GUARDRAILS.items():
            g_rect = rect_pts(gx, gy, gh, gl, gw)
            gd = rect_rect_dist(veh_rect, g_rect)
            if gid not in results['min_guardrail'] or gd < results['min_guardrail'][gid][1]:
                results['min_guardrail'][gid] = (t, gd, vx, vy, vv, dec)
        # 停车段: v<0.05
        if vv < 0.05:
            if not stopped:
                stopped = True
                stop_seg_start = t
            # 停车中到行人的距离(记最小)
            key = 'stop_ped_min'
            if key not in stop_info or d < stop_info[key][0]:
                stop_info[key] = (d, t, vx, vy, px, py)
            # 停车中到最近护栏
            best_g = None
            for gid, (gx, gy, gl, gw, gh) in GUARDRAILS.items():
                g_rect = rect_pts(gx, gy, gh, gl, gw)
                gd = rect_rect_dist(veh_rect, g_rect)
                if best_g is None or gd < best_g[1]:
                    best_g = (gid, gd, t, vx, vy)
            if 'stop_g_min' not in stop_info or best_g[1] < stop_info['stop_g_min'][1]:
                stop_info['stop_g_min'] = best_g
        else:
            if stopped:
                stopped = False
                results['stopped_seg'] = (stop_seg_start, t, dict(stop_info))
                stop_info = {}
    if stopped:
        results['stopped_seg'] = (stop_seg_start, rows[-1][0], dict(stop_info))

    # 停车位置(取第一个停车段的停车点)
    if results['stopped_seg']:
        _, _, info = results['stopped_seg']
        # 停车点 = 停车段中 veh_x 最小帧的 (x,y)
        seg_rows = [(r[4], veh_y_fn(r[0]) if veh_y_fn else DEFAULT_VEH_Y, r[0])
                    for r in rows if r[0] >= results['stopped_seg'][0] and r[0] <= results['stopped_seg'][1] and r[6] < 0.05]
        if seg_rows:
            vx0, vy0, t0 = min(seg_rows, key=lambda x: x[0])
        else:
            vx0 = vy0 = t0 = None
        stop_ped_d = info.get('stop_ped_min', (None,))[0]
        stop_g = info.get('stop_g_min', (None,))
        results['stop'] = {
            't0': t0, 'veh_x': vx0, 'veh_y': vy0,
            'ped_min': stop_ped_d,
            'g_id': stop_g[0], 'g_dist': stop_g[1] if len(stop_g) > 1 else None,
            'g_t': stop_g[2] if len(stop_g) > 2 else None,
        }
    return results

def load_version(name):
    """返回 (rows_normalized, veh_y_fn) rows: (t,px,py,pv,vx,vy,vv,dec)"""
    if name == '2209':
        # 2209_ped_vs_ego (veh无y) + 2209_y_precise (veh_y)
        base = parse_ped_veh(f'{OUT}/2209_ped_vs_ego.txt')
        ytab = parse_y_precise(f'{OUT}/2209_y_precise.txt')
        rows = [(t, px, py, pv, vx, None, vv, dec) for (t, px, py, pv, vx, vv, dec) in base]
        return rows, (lambda t, yt=ytab: interp_y(yt, t))
    elif name in ('2054', '2054b', '2109'):
        f = {'2054': 'ego_ped_2054.txt', '2054b': 'ego_ped_2054b.txt', '2109': 'ego_ped_2109.txt'}[name]
        raw = parse_ego_ped(f'{OUT}/{f}')
        rows = [(t, px, py, pv, vx, vy, vv, dec) for (t, px, py, pv, vx, vy, vv, dec) in raw]
        return rows, None  # 自带 y
    elif name in ('1925', '2005', '2012', '2012b', '2014', '2026'):
        f = {'1925': 'ped_ego_1925.txt', '2005': 'ped_ego_2005.txt',
             '2012': 'ped_ego_2012.txt', '2012b': 'ped_ego_2012b.txt',
             '2014': 'ped_ego_2014.txt', '2026': 'ped_ego_2026.txt'}[name]
        base = parse_ped_veh(f'{OUT}/{f}')
        rows = [(t, px, py, pv, vx, None, vv, dec) for (t, px, py, pv, vx, vv, dec) in base]
        return rows, None  # 默认 y
    return None, None

def main():
    versions = ['1925', '2005', '2012', '2012b', '2014', '2026', '2054', '2054b', '2109', '2209']
    all_res = []
    for v in versions:
        rows, yfn = load_version(v)
        if not rows:
            print(f'{v}: 无数据')
            continue
        res = analyze(v, rows, yfn)
        all_res.append(res)
        # ---- 输出 ----
        r = res
        print('=' * 100)
        print(f'版本 {v}: {r["n"]} 帧, t∈{r["t_range"]}')
        mp = r['min_ped']
        print(f'  自车-行人7673 最小边缘距离: {mp[1]:.3f} m @t={mp[0]:.2f} '
              f'(veh=({mp[2]:.2f},{mp[3]:.2f}) ped=({mp[4]:.2f},{mp[5]:.2f}) v={mp[6]:.2f} dec={mp[7]})')
        mpc = r['min_ped_center']
        print(f'  自车-行人7673 最小中心距: {mpc[1]:.3f} m @t={mpc[0]:.2f} (v={mpc[6]:.2f} dec={mpc[7]})')
        if r['stop']:
            s = r['stop']
            print(f'  停车点: t={s["t0"]} veh=({s["veh_x"]:.2f},{s["veh_y"]:.3f}) | '
                  f'停车中距行人最小 {s["ped_min"]:.3f} m | '
                  f'停车中距最近护栏 {s["g_id"]} = {s["g_dist"]:.3f} m')
        else:
            print('  未检测到停车段')
        # 护栏
        for gid in ['5530', '4717', '3098', '6503', '6324', '4857']:
            if gid in r['min_guardrail']:
                g = r['min_guardrail'][gid]
                print(f'    护栏{gid} 全程最小边缘距离: {g[1]:.3f} m @t={g[0]:.2f} (v={g[4]:.2f} dec={g[5]})')
        print()
    # ---- 汇总表 ----
    print('=' * 100)
    print('汇总: 停车点距护栏5530 / 距行人 / 全程距行人最小')
    print(f'{"版本":<8}{"停车veh_x":>10}{"停车veh_y":>12}{"停→护栏5530":>12}{"停→最近护栏":>12}{"停→行人":>10}{"全程→行人最小":>14}')
    for r in all_res:
        s = r['stop'] or {}
        vx = f'{s.get("veh_x", float("nan")):.2f}' if s.get('veh_x') else '-'
        vy = f'{s.get("veh_y", float("nan")):.3f}' if s.get('veh_y') else '-'
        g5530 = r['min_guardrail'].get('5530')
        g5530s = f'{g5530[1]:.3f}' if g5530 else '-'
        # 停车中距护栏5530(如果最近护栏是5530)
        gid_stop = s.get('g_id')
        gdist_stop = s.get('g_dist')
        if gid_stop == '5530':
            g5530_stop = f'{gdist_stop:.3f}' if gdist_stop else '-'
        else:
            g5530_stop = f'>{gdist_stop:.3f}' if gdist_stop else '-'
        gnearest = f'{s.get("g_id")}:{s.get("g_dist"):.3f}' if s.get('g_dist') is not None else '-'
        pd = f'{s.get("ped_min", float("nan")):.2f}' if s.get('ped_min') is not None else '-'
        mp = r['min_ped']
        mpd = f'{mp[1]:.3f}' if mp else '-'
        print(f'{r["version"]:<8}{vx:>10}{vy:>12}{g5530_stop:>12}{gnearest:>12}{pd:>10}{mpd:>14}')
    # ---- 2209 起步通过细节 ----
    print()
    print('=' * 100)
    print('2209 起步通过行人阶段详细 (来自 2209_restart_precise.txt, t=52-58)')
    pat = re.compile(r't=([-\d.]+) veh\(x=([-\d.]+),y=([-\d.]+),v=([-\d.]+),yaw=([-\d.-]+)\) ped\(x=([-\d.]+),y=([-\d.]+),v=([-\d.]+)\) lat=([-\d.]+) lon=([-\d.]+)')
    veh_rect_cache = {}
    best = None
    with open(f'{OUT}/2209_restart_precise.txt') as f:
        for line in f:
            m = pat.search(line)
            if not m:
                continue
            t, vx, vy, vv, yaw, px, py, pv, lat, lon = map(float, m.groups())
            v_rect = rect_pts(vx, vy, yaw, VEH_LEN, VEH_WID)
            p_rect = rect_pts(px, py, PED_HEADING, PED_LEN, PED_WID)
            d = rect_rect_dist(v_rect, p_rect)
            dc = math.hypot(vx - px, vy - py)
            if best is None or d < best[1]:
                best = (t, d, vx, vy, px, py, vv, lat, lon)
    if best:
        t, d, vx, vy, px, py, vv, lat, lon = best
        print(f'  2209 起步后 自车-行人7673 最小边缘距离: {d:.3f} m @t={t:.2f} '
              f'veh=({vx:.2f},{vy:.3f},v={vv:.2f}) ped=({px:.2f},{py:.3f},v={pv:.2f}) lat={lat:.2f} lon={lon:.2f}')
    # 也把 2209_ped_vs_ego 在起步后(>55s)的最小距离算出来(覆盖更长时段)
    print('  (补充: 用 2209_ped_vs_ego 全程含起步后)')
    rows, yfn = load_version('2209')
    tail = [r for r in rows if r[0] > 55.0]
    bt = None
    for r in tail:
        t, px, py, pv, vx, vy, vv, dec = r
        vy = yfn(t)
        d = rect_rect_dist(rect_pts(vx, vy, 0, VEH_LEN, VEH_WID),
                           rect_pts(px, py, PED_HEADING, PED_LEN, PED_WID))
        if bt is None or d < bt[1]:
            bt = (t, d, vx, vy, px, py, vv, dec)
    if bt:
        t, d, vx, vy, px, py, vv, dec = bt
        print(f'  2209 t>55s 最小边缘距离: {d:.3f} m @t={t:.2f} veh=({vx:.2f},{vy:.3f},v={vv:.2f}) ped=({px:.2f},{py:.2f}) dec={dec}')

if __name__ == '__main__':
    main()
