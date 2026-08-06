#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本地评分复现器：实车/本地 planning.log → 官方 metric 评分
===========================================================
输入: 实车 planning.log（adc 轨迹）+ 场景 json（行人重放）+ base_map.txt（横道几何）+ metric json
输出: 7号下午 4 项官方评分（cruise速度 / 加速度 / 人行道停车 / 减速带限速）

用法:
  python3 scripts/local_grading_replay.py \
      --log  <planning.log.INFO.xxx> \
      --scenario shiche/car_890/grading_console/auto_drive_console/7号下午_1.json \
      --map data/map_data/hehai_contest_7_pm_3/base_map.txt \
      --metric shiche/car_890/grading_console/auto_drive_console/metric \
      --field 7号下午
"""
import argparse, json, math, re, sys, os

ROOT = '/home/skye/application-pnc'
sys.path.insert(0, f'{ROOT}/shiche/car_890/xh_bak/.XH_2026_bak')
sys.path.insert(0, f'{ROOT}/shiche/car_890/grading_console')

from grading_system_py.main import GradingEngine, parse_config_file, GradingWorld, Object
from grading_system_py.common.map_interface import (
    set_map_interface, StubMapInterface, CrosswalkInfo, Polygon2d, Vec2d,
)

START_Y = 3476108.50
FRONT = 3.62   # front_edge_to_center (dkit)
BACK = 0.87    # back_edge_to_center = 4.49 - 3.62
W_HALF = 1.007  # width/2 = 2.014/2

# ============================================================
# 1. 轻量地图：从 base_map.txt 解析 crosswalk polygon
# ============================================================
class LightMap(StubMapInterface):
    def __init__(self, base_map_txt):
        self.crosswalks = []
        text = open(base_map_txt, encoding='utf-8', errors='ignore').read()
        for block in re.findall(r'crosswalk\s*\{.*?\n\}', text, re.S):
            mid = re.search(r'id:\s*"([^"]+)"', block)
            if not mid:
                continue
            pts = re.findall(r'point\s*\{\s*x:\s*([-\d.]+)\s+y:\s*([-\d.]+)', block)
            if len(pts) >= 3:
                poly = Polygon2d([Vec2d(float(x), float(y)) for x, y in pts])
                self.crosswalks.append(CrosswalkInfo(mid.group(1), poly))
        print(f"[Map] 解析 {len(self.crosswalks)} 个 crosswalk: "
              f"{[c.crosswalk_id for c in self.crosswalks]}")

    def find_crosswalk_for_polygon(self, x, y, radius, polygon):
        loc = Vec2d(x, y)
        for cw in self.crosswalks:
            if cw.polygon.distance_to(loc) <= radius and cw.polygon.has_overlap(polygon):
                return cw
        return None


# ============================================================
# 2. 解析 planning.log 的 adc 轨迹
# ============================================================
def w2s(w):
    h, m, s = w.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)

def parse_planning_log(log_path):
    rows = []
    for line in open(log_path, errors='ignore'):
        if 'planning_start_point' not in line:
            continue
        mm = re.search(r'planning_start_point x,y,the,k: (.*)', line)
        if not mm:
            continue
        nums = [float(x) for x in re.findall(r'[-+]?\d+\.\d+', mm.group(1))]
        m = re.search(r'(\d\d:\d\d:\d\d\.\d+)', line)
        if m and len(nums) >= 6:
            # x,y,theta,k,v,a
            rows.append((w2s(m.group(1)), nums[0], nums[1], nums[2], nums[4], nums[5]))
    return rows


# ============================================================
# 3. 行人重放（场景 json agent 定义 → 每帧位置）
# ============================================================
class PedestrianSim:
    """按语义A（车距横道 <= start_distance 时触发横穿）重放 5 个行人"""
    def __init__(self, scenario_json, start_y=START_Y):
        scen = json.load(open(scenario_json, encoding='utf-8'))
        self.agents = []
        for a in scen['scenario'].get('agent', []):
            sp = a['start_position']
            tp = a['tracked_point'][0]
            self.agents.append({
                'id': a['id'],
                'speed': sp['speed'],
                'start_distance': a.get('start_distance', 0),
                'sx': sp['x'], 'sy': sp['y'],
                'ex': tp['x'], 'ey': tp['y'],
                'heading': sp.get('heading', 0),
                'trigger_dy': (sp['y'] - start_y) - a.get('start_distance', 0),
                'trigger_t': None,
            })
        print(f"[Ped] 重放 {len(self.agents)} 个行人: "
              f"{[a['id'] for a in self.agents]} 触发Δy={[round(a['trigger_dy'],1) for a in self.agents]}")

    def update(self, t, adc_dy):
        out = []
        for a in self.agents:
            if a['trigger_t'] is None:
                if adc_dy >= a['trigger_dy']:
                    a['trigger_t'] = t
                else:
                    continue  # 未触发，不出现在世界
            total = math.hypot(a['ex'] - a['sx'], a['ey'] - a['sy'])
            d = a['speed'] * (t - a['trigger_t'])
            if d >= total:
                x, y, spd = a['ex'], a['ey'], 0.0
            else:
                ratio = d / total if total > 0 else 0
                x = a['sx'] + (a['ex'] - a['sx']) * ratio
                y = a['sy'] + (a['ey'] - a['sy']) * ratio
                spd = a['speed']
            out.append((a['id'], x, y, spd, a['heading']))
        return out


# ============================================================
# 4. 主流程
# ============================================================
def build_config(metric_dir, field):
    """合并 3 个 metric json（crosswalk/cruise/crosswalk_road_speed）"""
    files = {
        '6号上午': ['crosswalk', 'cruise', 'traffic_road_speed'],
        '6号下午': ['red_light', 'cruise', 'traffic_road_speed'],
        '7号上午': ['red_light', 'cruise', 'crosswalk_road_speed'],
        '7号下午': ['crosswalk', 'cruise', 'crosswalk_road_speed'],
    }[field]
    metrics = []
    use_score = True
    for f in files:
        p = os.path.join(metric_dir, f'{field}_1号赛道_{f}.json')
        data = json.load(open(p, encoding='utf-8'))
        metrics.extend(data.get('metric', []))
        use_score = use_score and data.get('use_score', True)
    cfg_path = '/tmp/grading_config_merged.json'
    json.dump({'use_score': use_score, 'metric': metrics}, open(cfg_path, 'w', encoding='utf-8'))
    print(f"[Config] 合并 {len(metrics)} 个 metric: {[m['name'] for m in metrics]}")
    return cfg_path


def make_adc(r):
    t, x, y, th, v, a = r
    obj = Object()
    obj.id = 'adc'
    obj.position_x, obj.position_y = x, y
    obj.heading = th
    obj.speed = v
    obj.speed_acceleration = a
    obj.length, obj.width = 4.49, 2.014
    obj.front_edge_to_center, obj.back_edge_to_center = FRONT, BACK
    obj.left_edge_to_center = obj.right_edge_to_center = W_HALF
    obj.type = 'VEHICLE'
    return obj


def make_ped(pid, x, y, spd, heading):
    obj = Object()
    obj.id = pid
    obj.position_x, obj.position_y = x, y
    obj.heading = heading
    obj.speed = spd
    obj.length, obj.width = 1.0, 1.0
    obj.front_edge_to_center = obj.back_edge_to_center = 0.5
    obj.left_edge_to_center = obj.right_edge_to_center = 0.5
    obj.type = 'PEDESTRIAN'
    return obj


def adjust_stop_distance(rows, front_dy, target_dist):
    """把 Crosswalk 前"真静止段"（v<0.1 且位置几乎不变）平移到车头距横道=target_dist"""
    n = len(rows)
    i = 0
    while i < n:
        dy = rows[i][2] - START_Y
        if abs(rows[i][4]) < 0.1 and front_dy - 10 <= dy <= front_dy + 1:
            j = i
            while j < n and abs(rows[j][4]) < 0.1 and front_dy - 10 <= (rows[j][2] - START_Y) <= front_dy + 1:
                j += 1
            seg = rows[i:j]
            dy_min = min(r[2] for r in seg)
            dy_max = max(r[2] for r in seg)
            # 只处理"真静止"：持续>=1s 且段内位置漂移 <0.3m（排除蠕行段）
            if j - i >= 5 and (dy_max - dy_min) < 0.3:
                mid = seg[len(seg) // 2]
                y_rel = mid[2] - START_Y
                head_dist = front_dy - (y_rel + FRONT)   # 正=车头在横道前
                delta = (front_dy - target_dist - FRONT) - y_rel
                for k in range(i, j):
                    t, x, y, th, v, a = rows[k]
                    rows[k] = (t, x, y + delta, th, v, a)
                print(f"[Adjust] 静止段 Δy={y_rel:.2f} 原车头距横道={head_dist:.2f}m "
                      f"→ {target_dist:.2f}m (平移 {delta:+.2f}m)")
                break
        i += 1
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--map', required=True)
    ap.add_argument('--metric', required=True)
    ap.add_argument('--field', default='7号下午')
    ap.add_argument('--stop-distance-target', type=float, default=0,
                    help='把 Crosswalk 停车段平移到"车头距横道=该值"(如1.25)，模拟新配置效果')
    ap.add_argument('--crosswalk-front-dy', type=float, default=59.64,
                    help='目标横道前缘 Δy（7pm Crosswalk_5=59.64）')
    args = ap.parse_args()

    # 地图
    set_map_interface(LightMap(args.map))
    # 配置
    cfg_path = build_config(args.metric, args.field)
    engine = GradingEngine(parse_config_file(cfg_path))

    # 数据
    rows = parse_planning_log(args.log)
    print(f"[Log] {len(rows)} 帧, {rows[-1][0]-rows[0][0]:.0f}s")
    if len(rows) < 50:
        print("[ERROR] 帧数过少"); return

    # 截断到"首次到达终点停稳"（跳过测试流程的倒车段）
    end_idx = None
    for i in range(len(rows)):
        if rows[i][2] - START_Y >= 96 and abs(rows[i][4]) < 0.1:
            if i + 5 < len(rows) and all(abs(rows[j][4]) < 0.1
                                         for j in range(i, min(i + 5, len(rows)))):
                end_idx = min(i + 10, len(rows) - 1)
                break
    if end_idx is not None:
        rows = rows[:end_idx]
        print(f"[Log] 截断到首次终点停稳: {len(rows)} 帧, {rows[-1][0]-rows[0][0]:.0f}s")

    # 可选：平移 Crosswalk 停车段到目标停车距离（零仿真验证新配置效果）
    if args.stop_distance_target > 0:
        rows = adjust_stop_distance(rows, args.crosswalk_front_dy,
                                    args.stop_distance_target)

    t0 = rows[0][0]
    pedsim = PedestrianSim(args.scenario)

    # 逐帧评分
    n = len(rows)
    for i, r in enumerate(rows):
        t = r[0] - t0
        adc_dy = r[2] - START_Y
        world = GradingWorld()
        world.timestamp_sec = t
        world.auto_driving_car = make_adc(r)
        for pid, x, y, spd, hd in pedsim.update(t, adc_dy):
            world.object.append(make_ped(pid, x, y, spd, hd))
        world.speed_limit = None
        world.last_frame = (i == n - 1)
        engine.feed_frame(world)

    summary = engine.get_summary()
    print("\n==================== 本地评分结果 ====================")
    total_deduct = 0.0
    for name, m in summary['metrics'].items():
        deduct = 100.0 - m['score']
        total_deduct += max(0, deduct)
        flag = '✅' if m['is_pass'] else '❌'
        print(f"  {flag} {name:<20} score={m['score']:6.1f} 扣分={deduct:5.1f} "
              f"失败帧={m['fail_count']:4d}/{m['total_frames']}")
        if m['first_failure']:
            ff = m['first_failure']
            print(f"       首个失败 t={ff['time']:.1f}s code={ff['code']} {ff['description'][:70]}")
    print("---------------------------------------------------------")
    print(f"  场景总分 ≈ {100 - max(0, total_deduct):.1f} (100 - Σ各metric扣分)")
    print("=========================================================")


if __name__ == '__main__':
    main()
