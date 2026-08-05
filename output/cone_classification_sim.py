#!/usr/bin/env python3
"""
施工区域锥桶左右分类算法仿真
==============================
精确复刻 C++ construction_zone_helper.cc 中的分类逻辑，
模拟车辆前进过程中逐帧扫描锥桶，输出详细分类结果。

用法:
    python3 cone_classification_sim.py           # 完整仿真（默认参数）
    python3 cone_classification_sim.py --step    # 逐帧步进模式
    python3 cone_classification_sim.py --help    # 查看所有参数
"""

import math
import copy
import argparse
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
from enum import Enum


# ============================================================
# 1. 数据结构（对应 C++ 中的 ConstructionZoneState 等）
# ============================================================

class NudgeType(Enum):
    LEFT_NUDGE = 0    # 需要向左侧绕行（锥桶在右侧）
    RIGHT_NUDGE = 1   # 需要向右侧绕行（锥桶在左侧）
    UNDEFINED = 2

@dataclass
class SLPolygon:
    """模拟 C++ SLPolygon，只保留分类所需的最小字段"""
    id: str
    s_min: float
    s_max: float
    l_min: float
    l_max: float
    x: float = 0.0
    y: float = 0.0
    nudge: NudgeType = NudgeType.UNDEFINED

    @property
    def s_center(self) -> float:
        return (self.s_min + self.s_max) * 0.5

    @property
    def l_center(self) -> float:
        return (self.l_min + self.l_max) * 0.5

    def set_nudge_from_to_left(self, to_left: bool):
        """
        C++ 语义:
          to_left=true  → cone 属于左墙 → SetNudgeInfo(RIGHT_NUDGE)，车辆向右绕
          to_left=false → cone 属于右墙 → SetNudgeInfo(LEFT_NUDGE)，车辆向左绕
        """
        self.nudge = NudgeType.RIGHT_NUDGE if to_left else NudgeType.LEFT_NUDGE

    def classified_side(self) -> str:
        """返回锥桶被分到的墙侧: 'L' 或 'R'"""
        # RIGHT_NUDGE → to_left=true → 左墙 → 'L'
        # LEFT_NUDGE → to_left=false → 右墙 → 'R'
        if self.nudge == NudgeType.RIGHT_NUDGE:
            return 'L'
        elif self.nudge == NudgeType.LEFT_NUDGE:
            return 'R'
        else:
            return '?'

@dataclass
class ConstructionZoneState:
    """对应 C++ ConstructionZoneState 结构体"""
    active: bool = False
    cone_history: List[Tuple[float, float]] = field(default_factory=list)
    left_wall: List[Tuple[float, float]] = field(default_factory=list)     # (s, l)
    right_wall: List[Tuple[float, float]] = field(default_factory=list)    # (s, l)
    cone_wall_memory: Dict[int, bool] = field(default_factory=dict)        # hash -> to_left
    classified_left_xy: List[Tuple[float, float]] = field(default_factory=list)
    classified_right_xy: List[Tuple[float, float]] = field(default_factory=list)
    no_cone_counter: int = 0
    low_cone_counter: int = 0
    farthest_cone_s: float = -1.0
    farthest_cone_x: float = -1.0
    farthest_cone_y: float = 0.0
    max_left_bound: float = 0.0
    max_right_bound: float = 0.0

    def reset(self):
        self.active = False
        self.cone_history.clear()
        self.left_wall.clear()
        self.right_wall.clear()
        self.cone_wall_memory.clear()
        self.classified_left_xy.clear()
        self.classified_right_xy.clear()
        self.no_cone_counter = 0
        self.low_cone_counter = 0
        self.farthest_cone_s = -1.0
        self.farthest_cone_x = -1.0
        self.farthest_cone_y = 0.0
        self.max_left_bound = 0.0
        self.max_right_bound = 0.0


# ============================================================
# 2. 辅助函数（精确复刻 C++ 的实现）
# ============================================================

def hash_xy(x: float, y: float) -> int:
    """对应 C++ HashXY: round(x) << 32 | round(y)"""
    ix = int(round(x))
    iy = int(round(y))
    # 使用 int64 兼容 Python 的任意精度整数
    return (ix << 32) | (iy & 0xFFFFFFFF)


def interpolate_wall_l(wall: List[Tuple[float, float]], s: float) -> float:
    """
    对应 C++ InterpolateWallL。
    在墙的 (s, l) 点列表中线性插值给定 s 处的 l 值。
    wall 必须按 s 排序。如果 s 超出范围超过 20m，返回 NaN。
    """
    if not wall:
        return float('nan')

    # 二分查找第一个 s >= target 的点
    lo, hi = 0, len(wall)
    while lo < hi:
        mid = (lo + hi) // 2
        if wall[mid][0] < s:
            lo = mid + 1
        else:
            hi = mid

    if lo == len(wall):
        # s 在所有点之后
        if s - wall[-1][0] > 20.0:
            return float('nan')
        return wall[-1][1]

    if lo == 0:
        # s 在所有点之前
        if wall[0][0] - s > 20.0:
            return float('nan')
        return wall[0][1]

    prev = wall[lo - 1]
    curr = wall[lo]
    if curr[0] - prev[0] > 20.0:
        return float('nan')

    ratio = (s - prev[0]) / (curr[0] - prev[0])
    return prev[1] + ratio * (curr[1] - prev[1])


def nearest_xy_distance(x: float, y: float, points: List[Tuple[float, float]]) -> float:
    """对应 C++ NearestXYDistance"""
    best = float('inf')
    for px, py in points:
        best = min(best, math.hypot(px - x, py - y))
    return best


def nearest_xy_distance_except_near(
        x: float, y: float, points: List[Tuple[float, float]], min_dist: float) -> float:
    """对应 C++ NearestXYDistanceExceptNear"""
    best = float('inf')
    for px, py in points:
        dist = math.hypot(px - x, py - y)
        if dist > min_dist:
            best = min(best, dist)
    return best


def find_two_recent_points_by_x(
        points: List[Tuple[float, float]], x: float) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """找到 x 方向落在当前点后方、且最靠近当前点的两个历史点。"""
    prev = None
    last = None
    for point in points:
        if point[0] >= x - 1.0:
            continue
        if last is None or point[0] > last[0]:
            if last is not None:
                prev = last
            last = point
        elif prev is None or point[0] > prev[0]:
            prev = point
    if prev is None or last is None:
        return None
    return prev, last


def should_prefer_left_chain_continuation(
        x: float, y: float, l: float, road_right: float, state: ConstructionZoneState) -> Tuple[bool, str]:
    """左墙连续性复核：允许强证据把 memory/单点距离的 R 翻回 L。"""
    recent = find_two_recent_points_by_x(state.classified_left_xy, x)
    if recent is None:
        return False, ""
    prev_left, last_left = recent

    gap_x = x - last_left[0]
    if gap_x < 2.0 or gap_x > 14.0:
        return False, ""
    if l - road_right < 3.2:
        return False, ""

    last_dist = math.hypot(x - last_left[0], y - last_left[1])
    right_dist = nearest_xy_distance_except_near(x, y, state.classified_right_xy, 1.0)
    prev_gap_x = last_left[0] - prev_left[0]
    if abs(prev_gap_x) < 1.0:
        return False, ""

    slope = (last_left[1] - prev_left[1]) / prev_gap_x
    predicted_y = last_left[1] + slope * gap_x
    y_error = abs(y - predicted_y)
    straight_extension = (
        last_dist < 11.5 and y_error < 1.6
        and (math.isinf(right_dist) or last_dist + 0.8 < right_dist)
    )
    left_tail_turn = (
        len(state.classified_left_xy) >= 4 and last_dist < 8.5
        and y + 0.5 >= last_left[1]
        and (math.isinf(right_dist) or last_dist + 0.8 < right_dist)
    )
    if not straight_extension and not left_tail_turn:
        return False, ""
    why = "leftChain" if straight_extension else "leftTurn"
    return True, f"{why} dL={last_dist:.2f} dR={right_dist:.2f} yErr={y_error:.2f}"


def is_global_tail_right_cone(
        x: float, y: float, l: float, road_right: float, state: ConstructionZoneState) -> bool:
    if state.farthest_cone_x <= 0.0:
        return False
    recent = find_two_recent_points_by_x(state.classified_left_xy, x)
    if len(state.classified_left_xy) < 8 or recent is None:
        return False
    _, last_left = recent
    kGlobalTailDist = 10.0
    kRightRoadDist = 7.0
    kLeftTailDrop = 2.8
    return (math.hypot(x - state.farthest_cone_x, y - state.farthest_cone_y) < kGlobalTailDist
            and abs(l - road_right) < kRightRoadDist
            and x > last_left[0] + 1.0
            and last_left[1] - y > kLeftTailDrop)


def add_unique_xy(points: List[Tuple[float, float]], x: float, y: float):
    """对应 C++ AddUniqueXY：去重（0.75m 内视为重复）后添加"""
    for px, py in points:
        if math.hypot(px - x, py - y) < 0.75:
            return
    points.append((x, y))


def remove_near_xy(points: List[Tuple[float, float]], x: float, y: float):
    """对应 C++ RemoveNearXY：删除 0.75m 内的点"""
    i = 0
    while i < len(points):
        if math.hypot(points[i][0] - x, points[i][1] - y) < 0.75:
            points.pop(i)
        else:
            i += 1


def prune_wall_behind(adc_back_s: float, cleanup_dist: float, wall: List[Tuple[float, float]]):
    """对应 C++ PruneWallBehind"""
    i = 0
    while i < len(wall):
        if wall[i][0] < adc_back_s - cleanup_dist:
            wall.pop(i)
        else:
            i += 1


def hash_to_xy(h: int) -> Tuple[float, float]:
    """对应 C++ HashToXY: 从 hash 反解出 (x, y)"""
    ix = (h >> 32) & 0xFFFFFFFF
    iy = h & 0xFFFFFFFF
    # 处理有符号 int32
    if ix >= 0x80000000:
        ix -= 0x100000000
    if iy >= 0x80000000:
        iy -= 0x100000000
    return float(ix), float(iy)


def prune_construction_zone_wall_state(
        cones: List[SLPolygon],
        adc_back_s: float,
        state: ConstructionZoneState
):
    """
    对应 C++ PruneConstructionZoneWallState。
    1. 清理墙壁上落后于车辆的旧点
    2. 根据当前锥桶 XY 包围盒清理 memory、classified_left_xy、classified_right_xy
    """
    kCleanupBehindDist = 30.0
    prune_wall_behind(adc_back_s, kCleanupBehindDist, state.left_wall)
    prune_wall_behind(adc_back_s, kCleanupBehindDist, state.right_wall)

    if not cones:
        return

    kMemoryKeepXYPadding = 160.0
    min_x = float('inf')
    max_x = -float('inf')
    min_y = float('inf')
    max_y = -float('inf')
    for cone in cones:
        min_x = min(min_x, cone.x)
        max_x = max(max_x, cone.x)
        min_y = min(min_y, cone.y)
        max_y = max(max_y, cone.y)
    min_x -= kMemoryKeepXYPadding
    max_x += kMemoryKeepXYPadding
    min_y -= kMemoryKeepXYPadding
    max_y += kMemoryKeepXYPadding

    def in_current_zone_xy(x: float, y: float) -> bool:
        return min_x <= x <= max_x and min_y <= y <= max_y

    # 清理 cone_wall_memory
    keys_to_remove = []
    for h in state.cone_wall_memory:
        hx, hy = hash_to_xy(h)
        if not in_current_zone_xy(hx, hy):
            keys_to_remove.append(h)
    for h in keys_to_remove:
        del state.cone_wall_memory[h]

    # 清理 classified_left_xy
    i = 0
    while i < len(state.classified_left_xy):
        if not in_current_zone_xy(state.classified_left_xy[i][0], state.classified_left_xy[i][1]):
            state.classified_left_xy.pop(i)
        else:
            i += 1

    # 清理 classified_right_xy
    i = 0
    while i < len(state.classified_right_xy):
        if not in_current_zone_xy(state.classified_right_xy[i][0], state.classified_right_xy[i][1]):
            state.classified_right_xy.pop(i)
        else:
            i += 1


def merge_construction_zone_wall_point(
        to_left: bool, s: float, l: float, state: ConstructionZoneState) -> bool:
    """
    对应 C++ MergeConstructionZoneWallPoint。
    如果已有墙点在 s 方向上 0.8m 内，合并（0.7 旧 + 0.3 新）；
    否则追加新点并排序。
    返回是否合并。
    """
    kWallMergeDist = 0.8
    wall = state.left_wall if to_left else state.right_wall
    for i, (ws, wl) in enumerate(wall):
        if abs(ws - s) < kWallMergeDist:
            wall[i] = (ws, wl * 0.7 + l * 0.3)
            return True
    wall.append((s, l))
    wall.sort(key=lambda p: p[0])
    return False


def extrapolate_wall_l(wall: List[Tuple[float, float]], s: float, num_pts: int = 5) -> float:
    """
    用墙壁最后 num_pts 个点做线性回归，外推 s 处的 l 值。
    用于捕捉墙壁的趋势（左墙下降、右墙上升等）。
    返回 NaN 如果墙点数不足或回归失败。
    """
    if len(wall) < num_pts:
        return float('nan')
    pts = wall[-num_pts:]
    n = len(pts)
    sum_s = sum(p[0] for p in pts)
    sum_l = sum(p[1] for p in pts)
    sum_ss = sum(p[0] * p[0] for p in pts)
    sum_sl = sum(p[0] * p[1] for p in pts)
    denom = n * sum_ss - sum_s * sum_s
    if abs(denom) < 1e-12:
        return float('nan')
    slope = (n * sum_sl - sum_s * sum_l) / denom
    intercept = (sum_l - slope * sum_s) / n
    return intercept + slope * s


# ============================================================
# 3. 核心分类函数（精确复刻 C++ ComputeConstructionZoneBoundary）
# ============================================================

def classify_cones(
        cones: List[SLPolygon],
        road_left: float,
        road_right: float,
        state: ConstructionZoneState,
        adc_back_s: float = -1e9,
        verbose: bool = True
) -> Dict[str, str]:
    """
    精确复刻 C++ ComputeConstructionZoneBoundary 的分类逻辑。

    参数:
        cones: 锥桶列表（SLPolygon），已按 s 排序
        road_left: 道路左边界（正数，如 5.0）
        road_right: 道路右边界（负数，如 -3.0）
        state: 施工区状态（跨帧保持）
        adc_back_s: 车辆后方 s（用于剪枝）
        verbose: 是否打印调试信息

    返回:
        {cone_id: "L" | "R"} 的分类结果
    """
    if not cones:
        return {}

    state.max_left_bound = road_left
    state.max_right_bound = road_right

    # ---- 步骤 1: Prune - 清理过期数据（完整复刻 C++ PruneConstructionZoneWallState）----
    prune_construction_zone_wall_state(cones, adc_back_s, state)

    # ---- 步骤 2: Seed - 首次初始化墙壁（仅当两面墙都为空时） ----
    seed_walls(cones, road_left, road_right, state, verbose)

    # ---- 步骤 2.5: Tail right seed（尾端右侧种子） ----
    has_tail_right_seed = False
    tail_right_seed_x = 0.0
    tail_right_seed_y = 0.0
    if len(state.left_wall) >= 8 and len(state.right_wall) >= 8 and len(cones) <= 6:
        best_seed_x = -float('inf')
        for cone in cones:
            l = cone.l_center
            if l - road_right < 3.5 and cone.x > best_seed_x:
                best_seed_x = cone.x
                tail_right_seed_x = cone.x
                tail_right_seed_y = cone.y
                has_tail_right_seed = True

    def in_tail_right_cluster(x: float, y: float) -> bool:
        kTailRightClusterDist = 18.0
        return (has_tail_right_seed and x + 1.0 >= tail_right_seed_x
                and math.hypot(x - tail_right_seed_x, y - tail_right_seed_y) < kTailRightClusterDist)

    # ---- 步骤 3: 主分类循环 ----
    frame_wall_choice: Dict[str, bool] = {}  # cone_id -> to_left

    for i, cone in enumerate(cones):
        s = cone.s_center
        l = cone.l_center
        cx, cy = cone.x, cone.y

        to_left = False
        why = ""
        from_memory = False

        pl = float('nan')  # projected left wall l at s
        pr = float('nan')  # projected right wall l at s

        h = hash_xy(cx, cy)

        # --- 3a. 检查 tail cluster / memory ---
        if in_tail_right_cluster(cx, cy):
            to_left = False
            why = "tailR"
            from_memory = True
        elif h in state.cone_wall_memory:
            to_left = state.cone_wall_memory[h]
            why = "mem"
            from_memory = True
            # Memory override: 如果 XY 分类结果强烈相反，修正
            if state.classified_left_xy and state.classified_right_xy:
                kMemoryOverrideDist = 25.0
                kMemoryOverrideMargin = 2.0
                nl = nearest_xy_distance(cx, cy, state.classified_left_xy)
                nr = nearest_xy_distance(cx, cy, state.classified_right_xy)
                if to_left and nr < kMemoryOverrideDist and nr + kMemoryOverrideMargin < nl:
                    to_left = False
                    why = "memFixR"
                elif not to_left and nl < kMemoryOverrideDist and nl + kMemoryOverrideMargin < nr:
                    to_left = True
                    why = "memFixL"

        if not from_memory and is_global_tail_right_cone(cx, cy, l, road_right, state):
            to_left = False
            why = "globalTail"
            from_memory = True

        # --- 3b. 非 memory 路径：基于墙壁插值判断 ---
        if not from_memory:
            pl = interpolate_wall_l(state.left_wall, s)
            pr = interpolate_wall_l(state.right_wall, s)

            if math.isnan(pl) and math.isnan(pr):
                # 两面墙都无法覆盖此 s：用道路边界做 fallback
                to_left = abs(l - road_left) < abs(l - road_right)
                why = "fallback"

            elif math.isnan(pl):
                # 只有右墙存在
                d_to_rw = abs(l - pr)
                d_to_road_l = abs(l - road_left)
                if d_to_rw > 5.0 and d_to_rw > d_to_road_l:
                    to_left = True
                    why = "farL"
                elif d_to_rw > 5.0:
                    to_left = False
                    why = "newL"
                elif state.classified_left_xy and not math.isnan(cx):
                    nl = nearest_xy_distance(cx, cy, state.classified_left_xy)
                    nr = nearest_xy_distance(cx, cy, state.classified_right_xy)
                    to_left = nl < nr and nl < 15.0
                    why = f"nnL={nl:.2f} nnR={nr:.2f}"
                else:
                    to_left = False
                    why = "noL"

            elif math.isnan(pr):
                # 只有左墙存在
                d_to_lw = abs(l - pl)
                d_to_road_r = abs(l - road_right)
                if d_to_lw > 5.0 and d_to_lw > d_to_road_r:
                    to_left = False
                    why = "farR"
                elif d_to_lw > 5.0:
                    to_left = True
                    why = "newR"
                elif state.classified_right_xy and not math.isnan(cx):
                    nl = nearest_xy_distance(cx, cy, state.classified_left_xy)
                    nr = nearest_xy_distance(cx, cy, state.classified_right_xy)
                    to_left = not (nr < nl and nr < 15.0)
                    why = f"nnL={nl:.2f} nnR={nr:.2f}"
                else:
                    to_left = True
                    why = "noR"

            else:
                # 两面墙都存在
                dl = abs(l - pl)
                dr = abs(l - pr)
                
                # 先走标准 XY/距离判断
                if (abs(dl - dr) < 1.5 or min(dl, dr) > 3.0) and not math.isnan(cx):
                    nl = nearest_xy_distance(cx, cy, state.classified_left_xy)
                    nr = nearest_xy_distance(cx, cy, state.classified_right_xy)
                    to_left = nl < nr
                    why = f"nnL={nl:.2f} nnR={nr:.2f}"
                else:
                    to_left = dl < dr
                    why = f"dL={dl:.2f} dR={dr:.2f}"
                
                # 趋势/lGap 修正：仅当标准判断为 R 且左墙稀疏时尝试翻转为 L
                if not to_left and len(state.left_wall) >= 2:
                    # 方法1: 墙壁趋势外推
                    pl_trend = extrapolate_wall_l(state.left_wall, s, num_pts=min(5, len(state.left_wall)))
                    if not math.isnan(pl_trend):
                        d_trend = abs(l - pl_trend)
                        if d_trend < dr - 0.5 and d_trend < dl:
                            to_left = True
                            why = f"trend dT={d_trend:.2f} dR={dr:.2f}"
                    # 方法2: lGap 兜底 — l 高于右墙且（显著高于 或 左墙插值滞后）
                    if not to_left and l > pr + 0.5 and (l > pr + 0.8 or pl > l + 2.0):
                        last_lw_s = state.left_wall[-1][0]
                        if s - last_lw_s < 15.0:
                            to_left = True
                            why = f"lGap l={l:.2f} pr={pr:.2f}"
                            # XY 复核：若锥桶明显更靠近右墙 XY 参考集，撤销 lGap 翻转
                            if state.classified_left_xy and state.classified_right_xy and not math.isnan(cx):
                                nl = nearest_xy_distance(cx, cy, state.classified_left_xy)
                                nr = nearest_xy_distance(cx, cy, state.classified_right_xy)
                                if nr < nl and nr < 12.0:
                                    to_left = False
                                    why = "lGapXY"

        if not to_left and why not in ("tailR", "globalTail"):
            prefer_left, chain_why = should_prefer_left_chain_continuation(cx, cy, l, road_right, state)
            if prefer_left:
                to_left = True
                why = chain_why

        if is_global_tail_right_cone(cx, cy, l, road_right, state):
            to_left = False
            why = "globalTail"

        # --- 3c. 记录分类结果 ---
        frame_wall_choice[cone.id] = to_left

        # 更新 memory 和 XY 分类集
        state.cone_wall_memory[h] = to_left
        if to_left:
            remove_near_xy(state.classified_right_xy, cx, cy)
            add_unique_xy(state.classified_left_xy, cx, cy)
        else:
            remove_near_xy(state.classified_left_xy, cx, cy)
            add_unique_xy(state.classified_right_xy, cx, cy)

        # 合并入墙壁
        merged = merge_construction_zone_wall_point(to_left, s, l, state)

        if verbose:
            pl_str = f"{pl:.2f}" if not math.isnan(pl) else "nan"
            pr_str = f"{pr:.2f}" if not math.isnan(pr) else "nan"
            print(f"  [{i:2d}] cone={cone.id:6s} xy=({cx:12.3f},{cy:14.6f}) "
                  f"s={s:8.2f} l={l:7.2f} "
                  f"pL={pl_str:>7s} pR={pr_str:>7s} -> {'L' if to_left else 'R'} "
                  f"|{why:25s}|{'m' if merged else 'n'}")

    # ---- 步骤 4: Apply Nudge（设置绕行方向） ----
    apply_nudge_choices(state, frame_wall_choice, cones)

    return {cid: "L" if to_l else "R" for cid, to_l in frame_wall_choice.items()}


def seed_walls(
        cones: List[SLPolygon],
        road_left: float,
        road_right: float,
        state: ConstructionZoneState,
        verbose: bool
):
    """
    对应 C++ SeedConstructionZoneWalls。
    当 left_wall 和 right_wall 均为空时，用滑动窗口 + 墙壁插值分类法初始化。
    每个锥桶根据已构建的墙壁插值做判断，而非用中位数一刀切。
    """
    if state.left_wall or state.right_wall or not cones:
        return

    # C++ 中先构建 sorted_cones 并排序
    sorted_cones = [(c.s_center, c.l_center) for c in cones]
    sorted_cones.sort(key=lambda p: p[0])

    if verbose:
        print(f"\n  [SEED] cones={len(sorted_cones)} road=[{road_right:.2f},{road_left:.2f}]")

    left_count = 0
    right_count = 0

    for s, l in sorted_cones:
        left_wall_l = interpolate_wall_l(state.left_wall, s)
        right_wall_l = interpolate_wall_l(state.right_wall, s)

        to_left = False
        if math.isnan(left_wall_l) and math.isnan(right_wall_l):
            to_left = abs(l - road_left) < abs(l - road_right)
        elif math.isnan(left_wall_l):
            dist_to_right_wall = abs(l - right_wall_l)
            dist_to_road_left = abs(l - road_left)
            to_left = dist_to_right_wall > 5.0 and dist_to_right_wall > dist_to_road_left
        elif math.isnan(right_wall_l):
            dist_to_left_wall = abs(l - left_wall_l)
            dist_to_road_right = abs(l - road_right)
            to_left = not (dist_to_left_wall > 5.0 and dist_to_left_wall > dist_to_road_right)
        else:
            to_left = abs(l - left_wall_l) < abs(l - right_wall_l)

        if to_left:
            state.left_wall.append((s, l))
            left_count += 1
        else:
            state.right_wall.append((s, l))
            right_count += 1

    if verbose:
        print(f"\n  [SEED] sliding: nL={left_count} nR={right_count}")


def apply_nudge_choices(
        state: ConstructionZoneState,
        frame_wall_choice: Dict[str, bool],
        cones: List[SLPolygon]
):
    """
    对应 C++ ApplyConstructionZoneNudgeChoices。
    C++ 代码：
      frame_choice->second ? RIGHT_NUDGE : LEFT_NUDGE
    即 to_left=true → RIGHT_NUDGE, to_left=false → LEFT_NUDGE
    """
    for cone in cones:
        if cone.id in frame_wall_choice:
            cone.set_nudge_from_to_left(frame_wall_choice[cone.id])
            continue

        # fallback: 用墙壁插值决定
        s = cone.s_center
        l = cone.l_center
        left_wall_l = interpolate_wall_l(state.left_wall, s)
        right_wall_l = interpolate_wall_l(state.right_wall, s)

        if math.isnan(left_wall_l) and math.isnan(right_wall_l):
            # 墙壁不可用：fallback 到道路边界
            cone.set_nudge_from_to_left(
                abs(l - state.max_left_bound) < abs(l - state.max_right_bound))
            continue
        if math.isnan(left_wall_l):
            # 只有右墙
            # C++: |l - right_wall_l| > 5.0 ? RIGHT_NUDGE : LEFT_NUDGE
            # >5.0 → 锥桶远离右墙 → 属于左墙(to_left=true) → RIGHT_NUDGE
            # ≤5.0 → 锥桶靠近右墙 → 属于右墙(to_left=false) → LEFT_NUDGE
            cone.set_nudge_from_to_left(abs(l - right_wall_l) > 5.0)
        elif math.isnan(right_wall_l):
            # 只有左墙
            # C++: |l - left_wall_l| > 5.0 ? LEFT_NUDGE : RIGHT_NUDGE
            # >5.0 → 锥桶远离左墙 → 属于右墙(to_left=false) → LEFT_NUDGE
            # ≤5.0 → 锥桶靠近左墙 → 属于左墙(to_left=true) → RIGHT_NUDGE
            cone.set_nudge_from_to_left(abs(l - left_wall_l) <= 5.0)
        else:
            # 两面墙都有
            # C++: |l - left_wall_l| < |l - right_wall_l| ? RIGHT_NUDGE : LEFT_NUDGE
            # 靠近左墙 → RIGHT_NUDGE, 靠近右墙 → LEFT_NUDGE
            cone.set_nudge_from_to_left(abs(l - left_wall_l) < abs(l - right_wall_l))


# ============================================================
# 4. 坐标系转换（XY ↔ SL）
# ============================================================

class ReferenceLine:
    """
    模拟 Apollo ReferenceLine 的简化版。
    支持 XY → SL 转换和 SL → XY 转换。
    """

    def __init__(self, waypoints: List[Tuple[float, float]]):
        """
        waypoints: 参考线离散点列表 [(x1, y1), (x2, y2), ...]
        """
        self.waypoints = waypoints
        self._cumulative_s = self._compute_cumulative_s()

    def _compute_cumulative_s(self) -> List[float]:
        """计算各点的累积弧长 s"""
        s = [0.0]
        for i in range(1, len(self.waypoints)):
            dx = self.waypoints[i][0] - self.waypoints[i-1][0]
            dy = self.waypoints[i][1] - self.waypoints[i-1][1]
            s.append(s[-1] + math.hypot(dx, dy))
        return s

    def xy_to_sl(self, x: float, y: float) -> Tuple[float, float]:
        """
        将 XY 坐标转换为 SL 坐标。
        找到参考线上最近的点，s = 该点的累积弧长，l = 有符号侧向距离。
        l > 0 表示在参考线左侧（沿前进方向看），l < 0 表示右侧。
        """
        best_s = 0.0
        best_l = 0.0
        best_dist = float('inf')

        for i in range(len(self.waypoints) - 1):
            x0, y0 = self.waypoints[i]
            x1, y1 = self.waypoints[i + 1]
            s0 = self._cumulative_s[i]
            s1 = self._cumulative_s[i + 1]

            dx = x1 - x0
            dy = y1 - y0
            seg_len_sq = dx*dx + dy*dy
            if seg_len_sq < 1e-12:
                dist = math.hypot(x - x0, y - y0)
                if dist < best_dist:
                    best_dist = dist
                    best_s = s0
                    best_l = 0.0  # 简化处理
                continue

            # 投影参数 t (0~1)
            t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / seg_len_sq))
            proj_x = x0 + t * dx
            proj_y = y0 + t * dy

            dist = math.hypot(x - proj_x, y - proj_y)
            if dist < best_dist:
                best_dist = dist
                best_s = s0 + t * (s1 - s0)
                # 计算侧向距离的符号: 用叉积
                cross = dx * (y - y0) - dy * (x - x0)
                best_l = dist if cross >= 0 else -dist

        return best_s, best_l

    def sl_to_xy(self, s: float, l: float) -> Tuple[float, float]:
        """SL → XY 转换"""
        if s <= self._cumulative_s[0]:
            idx = 0
        elif s >= self._cumulative_s[-1]:
            idx = len(self.waypoints) - 2
        else:
            idx = 0
            for i in range(len(self._cumulative_s) - 1):
                if self._cumulative_s[i] <= s <= self._cumulative_s[i + 1]:
                    idx = i
                    break

        x0, y0 = self.waypoints[idx]
        x1, y1 = self.waypoints[idx + 1]
        s0 = self._cumulative_s[idx]
        s1 = self._cumulative_s[idx + 1]

        if s1 - s0 < 1e-12:
            t = 0.0
        else:
            t = (s - s0) / (s1 - s0)

        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-12:
            return x0, y0

        # 单位切向和法向
        tx, ty = dx / seg_len, dy / seg_len
        nx, ny = -ty, tx  # 左侧法向量

        base_x = x0 + t * dx
        base_y = y0 + t * dy
        return base_x + l * nx, base_y + l * ny


# ============================================================
# 5. 数据定义
# ============================================================

# 右侧锥桶 XY 坐标
RIGHT_CONES_XY = [
    (424021.5081874608, 4437609.564033156),
    (424021.58892594645, 4437611.097432596),
    (424024.8938826922, 4437614.0025508),
    (424029.9963587621, 4437615.215937046),
    (424038.2014380921, 4437615.309241912),
    (424044.2208290404, 4437615.293310558),
    (424049.97183790227, 4437615.278089542),
    (424054.9146612163, 4437614.11479745),
    (424058.1737533108, 4437613.514118136),
    (424064.0674058977, 4437611.470032134),
    (424071.4867390141, 4437611.545780973),
    (424077.54330521764, 4437611.490181547),
    (424086.47728839214, 4437611.510623975),
    (424094.8744905271, 4437611.577417894),
    (424100.1446800422, 4437609.973929453),
    (424176.7036145943, 4437609.867318109),
    (424181.1567227988, 4437611.5298220385),
    (424185.2529026116, 4437612.718872837),
    (424188.38741257903, 4437613.5062305145),
    (424190.7548643562, 4437614.144347228),
]

# 左侧锥桶 XY 坐标
LEFT_CONES_XY = [
    (424082.31661391834, 4437618.565910922),
    (424082.2503579958, 4437616.896812546),
    (424084.7725151632, 4437617.459829637),
    (424090.7832433672, 4437616.802558058),
    (424100.10377283517, 4437615.774480514),
    (424108.2530313211, 4437615.76277789),
    (424116.0840767474, 4437615.727100059),
    (424123.0143233872, 4437615.725076706),
    (424129.2551353437, 4437614.005899727),
    (424135.05028225295, 4437612.899806708),
    (424141.56676398445, 4437612.1634288225),
    (424148.2166306297, 4437611.947261906),
    (424155.15953626035, 4437611.873756877),
    (424162.3279693907, 4437611.824366805),
    (424169.23204768007, 4437614.55088499),
    (424175.4971678164, 4437617.128851979),
]

# 左侧道路边界
LEFT_ROAD_BOUNDARY = [
    (424041.63782661274, 4437619.852442451),
    (424239.6124002871, 4437619.085553092),
]

# 右侧道路边界
RIGHT_ROAD_BOUNDARY = [
    (424021.1458139795, 4437608.1566763185),
    (424256.69846271095, 4437607.548892385),
]


def create_reference_line() -> ReferenceLine:
    """
    根据道路边界估算参考线（道路中心线）。
    用左右边界的中心来近似，并外推到覆盖全部锥桶。
    """
    # 使用左右边界对应位置的中点构建道路中心线
    left_start, left_end = LEFT_ROAD_BOUNDARY[0], LEFT_ROAD_BOUNDARY[1]
    right_start, right_end = RIGHT_ROAD_BOUNDARY[0], RIGHT_ROAD_BOUNDARY[1]

    # 道路中心线的方向向量
    start_cx = (left_start[0] + right_start[0]) / 2
    start_cy = (left_start[1] + right_start[1]) / 2
    end_cx = (left_end[0] + right_end[0]) / 2
    end_cy = (left_end[1] + right_end[1]) / 2

    # 方向向量
    dx = end_cx - start_cx
    dy = end_cy - start_cy
    length = math.hypot(dx, dy)
    if length < 1e-6:
        # 退化情况
        return ReferenceLine([(start_cx, start_cy), (end_cx, end_cy)])

    ux, uy = dx / length, dy / length  # 单位方向

    # 外推 50m 到后方和前方，确保覆盖所有锥桶
    extend_back = 50.0
    extend_front = 50.0

    ext_start_x = start_cx - ux * extend_back
    ext_start_y = start_cy - uy * extend_back
    ext_end_x = end_cx + ux * extend_front
    ext_end_y = end_cy + uy * extend_front

    # 生成密集的 waypoints（0.5m 分辨率）
    ext_length = math.hypot(ext_end_x - ext_start_x, ext_end_y - ext_start_y)
    num_pts = max(2, int(ext_length / 0.5) + 1)
    waypoints = []
    for i in range(num_pts):
        t = i / (num_pts - 1)
        x = ext_start_x + t * (ext_end_x - ext_start_x)
        y = ext_start_y + t * (ext_end_y - ext_start_y)
        waypoints.append((x, y))

    return ReferenceLine(waypoints)


# ============================================================
# 5.5. 从 C++ 诊断日志加载真实数据
# ============================================================

def load_cpp_diag_data(log_file_path: str) -> dict:
    """
    从 C++ 诊断日志中提取参考线、锥桶 SL/XY、道路边界等真实数据。

    C++ 日志格式 (grep '[CONE_DIAG]' planning.INFO):
      [CONE_DIAG] REFLINE|total_s=...|n_pts=...|waypoints=[(x1,y1),(x2,y2),...]
      [CONE_DIAG] ADC|adc_back_s=...|adc_front_s=...|adc_x=...|adc_y=...|adc_heading=...
      [CONE_DIAG] ROAD|max_left_bound=...|max_right_bound=...|adc_lane_width=...|offset_to_map=...
      [CONE_DIAG] CONE|id=...|s_min=...|s_max=...|l_min=...|l_max=...|cx=...|cy=...
      [CONE_DIAG] CONEXY|id=...|x=...|y=...

    返回 dict，字段对应 Python 中的变量名。
    """
    result = {
        'ref_line_waypoints': [],   # [(x, y), ...]
        'ref_total_s': 0.0,
        'adc_back_s': 0.0,
        'adc_front_s': 0.0,
        'adc_x': 0.0,
        'adc_y': 0.0,
        'adc_heading': 0.0,
        'max_left_bound': 0.0,
        'max_right_bound': 0.0,
        'adc_lane_width': 0.0,
        'offset_to_map': 0.0,
        'adc_s': 0.0,
        'adc_l': 0.0,
        'cones': [],                # [{id, s_min, s_max, l_min, l_max, cx, cy}, ...]
        'cone_xy': {},              # {id: (x, y), ...}
    }

    import re

    with open(log_file_path, 'r') as f:
        for line in f:
            if '[CONE_DIAG] REFLINE' in line:
                # 提取 waypoints
                m = re.search(r'waypoints=\[(.*)\]', line)
                if m:
                    pts_str = m.group(1)
                    for pt in re.finditer(r'\(([\d.-]+),([\d.-]+)\)', pts_str):
                        result['ref_line_waypoints'].append(
                            (float(pt.group(1)), float(pt.group(2))))
                m = re.search(r'total_s=([\d.]+)', line)
                if m:
                    result['ref_total_s'] = float(m.group(1))

            elif '[CONE_DIAG] ADC' in line:
                for key in ['adc_back_s', 'adc_front_s', 'adc_x', 'adc_y', 'adc_heading']:
                    m = re.search(rf'{key}=([\d.-]+)', line)
                    if m:
                        result[key] = float(m.group(1))

            elif '[CONE_DIAG] ROAD' in line:
                for key in ['max_left_bound', 'max_right_bound', 'adc_lane_width',
                             'offset_to_map', 'adc_s', 'adc_l']:
                    m = re.search(rf'{key}=([\d.-]+)', line)
                    if m:
                        result[key] = float(m.group(1))

            elif '[CONE_DIAG] CONE|' in line and 'CONEXY' not in line:
                cone = {}
                for key in ['id', 's_min', 's_max', 'l_min', 'l_max', 'cx', 'cy']:
                    if key == 'id':
                        m = re.search(r'id=(\S+?)\|', line)
                        if m:
                            cone[key] = m.group(1)
                    else:
                        m = re.search(rf'{key}=([\d.-]+)', line)
                        if m:
                            cone[key] = float(m.group(1))
                if 'id' in cone:
                    result['cones'].append(cone)

            elif '[CONE_DIAG] CONEXY' in line:
                m = re.search(r'id=(\S+?)\|x=([\d.-]+)\|y=([\d.-]+)', line)
                if m:
                    result['cone_xy'][m.group(1)] = (float(m.group(2)), float(m.group(3)))

    return result


def create_reference_line_from_cpp(waypoints: List[Tuple[float, float]]) -> ReferenceLine:
    """用从 C++ 日志提取的 waypoints 创建 ReferenceLine"""
    if not waypoints or len(waypoints) < 2:
        raise ValueError("需要至少 2 个 waypoints")
    return ReferenceLine(list(waypoints))


# ============================================================
# 6. 仿真引擎
# ============================================================

def run_simulation(
        step_size: float = 5.0,
        look_forward: float = 60.0,
        look_backward: float = 10.0,
        step_mode: bool = False,
        verbose: bool = True,
        do_plot: bool = True
):
    """
    模拟车辆驶过施工区域，逐帧分类锥桶。

    参数:
        step_size: 每帧车辆前进距离 (m)
        look_forward: 车辆前方感知距离 (m)
        look_backward: 车辆后方保留距离 (m)
        step_mode: 是否逐帧暂停等待用户输入
        verbose: 是否打印详细信息
    """
    print("=" * 80)
    print("  施工区域锥桶左右分类算法仿真")
    print("=" * 80)
    print(f"  参数: step_size={step_size}m, look_forward={look_forward}m, look_backward={look_backward}m")
    print()

    # 构建参考线
    ref_line = create_reference_line()
    print(f"  参考线: {len(ref_line.waypoints)} 个点, 总长约 {ref_line._cumulative_s[-1]:.1f}m")
    print(f"  起点: ({ref_line.waypoints[0][0]:.3f}, {ref_line.waypoints[0][1]:.3f})")
    print(f"  终点: ({ref_line.waypoints[-1][0]:.3f}, {ref_line.waypoints[-1][1]:.3f})")
    print()

    # 构建所有锥桶（含 ground truth 标签）
    all_cones = []
    for i, (x, y) in enumerate(RIGHT_CONES_XY):
        s, l = ref_line.xy_to_sl(x, y)
        all_cones.append(SLPolygon(
            id=f"R_{i:02d}", s_min=s-0.1, s_max=s+0.1,
            l_min=l-0.1, l_max=l+0.1, x=x, y=y
        ))
    for i, (x, y) in enumerate(LEFT_CONES_XY):
        s, l = ref_line.xy_to_sl(x, y)
        all_cones.append(SLPolygon(
            id=f"L_{i:02d}", s_min=s-0.1, s_max=s+0.1,
            l_min=l-0.1, l_max=l+0.1, x=x, y=y
        ))

    # 按 s 排序所有锥桶
    all_cones.sort(key=lambda c: c.s_center)

    # 计算道路边界在 SL 中的 l 值
    left_road_s, left_road_l = ref_line.xy_to_sl(*LEFT_ROAD_BOUNDARY[0])
    right_road_s, right_road_l = ref_line.xy_to_sl(*RIGHT_ROAD_BOUNDARY[0])
    # 从道路边界计算 road_left / road_right
    # C++ 中 max_left_bound 是正值，max_right_bound 是负值
    # 使用道路边界在 SL 中的 l 值
    left_boundary_ls = []
    right_boundary_ls = []
    for x, y in LEFT_ROAD_BOUNDARY:
        _, l = ref_line.xy_to_sl(x, y)
        left_boundary_ls.append(l)
    for x, y in RIGHT_ROAD_BOUNDARY:
        _, l = ref_line.xy_to_sl(x, y)
        right_boundary_ls.append(l)

    road_left = max(left_boundary_ls)  # 正值（左边界）
    road_right = min(right_boundary_ls)  # 负值（右边界）

    print(f"  road_left_bound (from map)  = {road_left:.2f}m")
    print(f"  road_right_bound (from map) = {road_right:.2f}m")
    print()

    # 所有锥桶的 s 范围
    all_s = [c.s_center for c in all_cones]
    min_cone_s = min(all_s)
    max_cone_s = max(all_s)
    print(f"  锥桶 s 范围: [{min_cone_s:.1f}, {max_cone_s:.1f}]")
    print()

    # 仿真状态
    state = ConstructionZoneState()
    state.max_left_bound = road_left
    state.max_right_bound = road_right

    adc_s = min_cone_s - look_backward - 10.0  # 车辆起始位置（在第一个锥桶后方）

    frame = 0
    while adc_s < max_cone_s + look_backward + 20.0:
        frame += 1
        adc_back_s = adc_s  # 简化为车辆后沿

        # 收集当前帧可见的锥桶（按 s 排序）
        visible_cones = [
            c for c in all_cones
            if adc_back_s - look_backward <= c.s_center <= adc_back_s + look_forward
        ]

        if not visible_cones:
            if verbose:
                print(f"\n{'='*60}")
                print(f"  Frame {frame}: adc_s={adc_s:.1f}m, 可见锥桶=0 (空帧)")
            adc_s += step_size
            continue

        print(f"\n{'='*60}")
        print(f"  Frame {frame}: adc_s={adc_s:.1f}m, 可见锥桶={len(visible_cones)}")
        print(f"  锥桶 s 范围: [{visible_cones[0].s_center:.1f}, {visible_cones[-1].s_center:.1f}]")
        print(f"  left_wall={len(state.left_wall)}pts, right_wall={len(state.right_wall)}pts")

        # 浅拷贝锥桶列表（C++ 中会修改 cone 的 nudge 属性）
        cones_frame = copy.deepcopy(visible_cones)

        # 执行分类
        results = classify_cones(
            cones_frame,
            road_left=road_left,
            road_right=road_right,
            state=state,
            adc_back_s=adc_back_s,
            verbose=verbose
        )

        # 统计正确率
        correct = 0
        wrong = 0
        for cone in cones_frame:
            gt = "L" if cone.id.startswith("L_") else "R"
            pred = cone.classified_side()
            if gt == pred:
                correct += 1
            else:
                wrong += 1

        print(f"\n  >>> 本帧正确率: {correct}/{correct+wrong} = {correct/(correct+wrong)*100:.1f}%")

        if step_mode:
            input("\n  按 Enter 继续下一帧...")

        adc_s += step_size

    # ============================================================
    # 最终汇总
    # ============================================================
    print(f"\n{'='*80}")
    print("  最终汇总")
    print(f"{'='*80}")
    print(f"  总帧数: {frame}")
    print(f"  left_wall 点数: {len(state.left_wall)}")
    print(f"  right_wall 点数: {len(state.right_wall)}")
    print(f"  memory 条目: {len(state.cone_wall_memory)}")
    print(f"  classified_left_xy: {len(state.classified_left_xy)}")
    print(f"  classified_right_xy: {len(state.classified_right_xy)}")
    print()

    # 使用逐帧累积的最终状态来做最终分类
    print("  使用逐帧累积状态做最终分类（所有锥桶）:")
    print(f"  {'ID':<8s} {'X':>14s} {'Y':>16s} {'S':>8s} {'L':>8s} {'GT':>4s} {'分类':>4s} {'结果':>6s}")
    print(f"  {'-'*70}")
    # 注意: state 已经在逐帧仿真中累积了所有状态，直接用它再做一次全量分类
    final_cones = copy.deepcopy(all_cones)
    final_results = classify_cones(final_cones, road_left, road_right, state, verbose=False)

    total_correct = 0
    total_wrong = 0
    for cone in final_cones:
        gt = "L" if cone.id.startswith("L_") else "R"
        pred = cone.classified_side()
        ok = "✓" if gt == pred else "✗"
        if gt == pred:
            total_correct += 1
        else:
            total_wrong += 1
        print(f"  {cone.id:<8s} {cone.x:>14.3f} {cone.y:>16.6f} {cone.s_center:>8.2f} {cone.l_center:>8.2f} {gt:>4s} {pred:>4s} {ok:>6s}")

    print(f"\n  总计: 正确={total_correct}, 错误={total_wrong}")
    print(f"  正确率: {total_correct/(total_correct+total_wrong)*100:.1f}%")

    return state, final_cones, ref_line


def run_simulation_cpp_mode(
        json_path: str,
        step_size: float = 5.0,
        look_forward: float = 60.0,
        look_backward: float = 10.0,
        step_mode: bool = False,
        verbose: bool = True,
        do_plot: bool = True
):
    """
    使用 C++ 诊断日志提取的真实数据运行仿真。
    参考线、道路边界、锥桶 SL/XY 全部来自 C++ 运行时。
    """
    import json

    with open(json_path) as f:
        data = json.load(f)

    # 构建参考线
    waypoints = data['ref_line_waypoints']
    ref_line = ReferenceLine(waypoints)

    # 道路边界
    road_left = data['road_left']
    road_right = data['road_right']

    print("=" * 80)
    print("  施工区域锥桶左右分类算法仿真 (C++ 真实数据模式)")
    print("=" * 80)
    print(f"  参数: step_size={step_size}m, look_forward={look_forward}m, look_backward={look_backward}m")
    print(f"  数据来源: {json_path}")
    print()
    print(f"  参考线: {len(ref_line.waypoints)} 个点, 总长约 {ref_line._cumulative_s[-1]:.1f}m")
    print(f"  起点: ({ref_line.waypoints[0][0]:.3f}, {ref_line.waypoints[0][1]:.3f})")
    print(f"  终点: ({ref_line.waypoints[-1][0]:.3f}, {ref_line.waypoints[-1][1]:.3f})")
    print(f"  road_left_bound  = {road_left:.2f}m")
    print(f"  road_right_bound = {road_right:.2f}m")
    print()

    # 用 JSON 中的锥桶构建 SLPolygon（SL 值来自 C++，GT 来自匹配）
    all_cones = []
    for c in data['cones']:
        gt = c['gt']  # 'L', 'R', or '?'
        cone_id = f"{gt}_{c['id']}"  # 如 "R_83846"
        all_cones.append(SLPolygon(
            id=cone_id,
            s_min=c['s'] - 0.1, s_max=c['s'] + 0.1,
            l_min=c['l'] - 0.1, l_max=c['l'] + 0.1,
            x=c['x'], y=c['y']
        ))

    all_cones.sort(key=lambda c: c.s_center)
    all_s = [c.s_center for c in all_cones]
    min_cone_s = min(all_s)
    max_cone_s = max(all_s)
    print(f"  锥桶 s 范围: [{min_cone_s:.1f}, {max_cone_s:.1f}]")
    print(f"  锥桶数量: {len(all_cones)} (L={sum(1 for c in data['cones'] if c['gt']=='L')}, R={sum(1 for c in data['cones'] if c['gt']=='R')})")
    print()

    # 仿真
    state = ConstructionZoneState()
    state.max_left_bound = road_left
    state.max_right_bound = road_right

    adc_s = min_cone_s - look_backward - 10.0
    frame = 0
    while adc_s < max_cone_s + look_backward + 20.0:
        frame += 1
        adc_back_s = adc_s

        visible_cones = [
            c for c in all_cones
            if adc_back_s - look_backward <= c.s_center <= adc_back_s + look_forward
        ]

        if not visible_cones:
            if verbose:
                print(f"\n{'='*60}")
                print(f"  Frame {frame}: adc_s={adc_s:.1f}m, 可见锥桶=0 (空帧)")
            adc_s += step_size
            continue

        print(f"\n{'='*60}")
        print(f"  Frame {frame}: adc_s={adc_s:.1f}m, 可见锥桶={len(visible_cones)}")
        print(f"  锥桶 s 范围: [{visible_cones[0].s_center:.1f}, {visible_cones[-1].s_center:.1f}]")
        print(f"  left_wall={len(state.left_wall)}pts, right_wall={len(state.right_wall)}pts")

        cones_frame = copy.deepcopy(visible_cones)
        classify_cones(cones_frame, road_left, road_right, state,
                      adc_back_s=adc_back_s, verbose=verbose)

        correct = 0
        wrong = 0
        for cone in cones_frame:
            gt = "L" if cone.id.startswith("L_") else "R"
            pred = cone.classified_side()
            if gt == pred:
                correct += 1
            else:
                wrong += 1

        print(f"\n  >>> 本帧正确率: {correct}/{correct+wrong} = {correct/(correct+wrong)*100:.1f}%" if (correct+wrong)>0 else "\n  >>> 本帧正确率: N/A")

        if step_mode:
            input("\n  按 Enter 继续下一帧...")

        adc_s += step_size

    # 最终汇总
    print(f"\n{'='*80}")
    print("  最终汇总")
    print(f"{'='*80}")
    print(f"  总帧数: {frame}")
    print(f"  left_wall 点数: {len(state.left_wall)}")
    print(f"  right_wall 点数: {len(state.right_wall)}")
    print(f"  memory 条目: {len(state.cone_wall_memory)}")
    print()

    print("  使用逐帧累积状态做最终分类（所有锥桶）:")
    print(f"  {'ID':<20s} {'X':>10s} {'Y':>12s} {'S':>8s} {'L':>8s} {'GT':>4s} {'分类':>4s} {'结果':>6s}")
    print(f"  {'-'*76}")
    final_cones = copy.deepcopy(all_cones)
    classify_cones(final_cones, road_left, road_right, state, verbose=False)

    total_correct = 0
    total_wrong = 0
    for cone in final_cones:
        gt = "L" if cone.id.startswith("L_") else "R"
        pred = cone.classified_side()
        ok = "✓" if gt == pred else "✗"
        if gt == pred:
            total_correct += 1
        else:
            total_wrong += 1
        print(f"  {cone.id:<20s} {cone.x:>10.2f} {cone.y:>12.2f} {cone.s_center:>8.2f} {cone.l_center:>8.2f} {gt:>4s} {pred:>4s} {ok:>6s}")

    print(f"\n  总计: 正确={total_correct}, 错误={total_wrong}")
    if total_correct + total_wrong > 0:
        print(f"  正确率: {total_correct/(total_correct+total_wrong)*100:.1f}%")

    return state, final_cones, ref_line


# ============================================================
# 7. 单帧测试模式
# ============================================================

def run_single_frame(visible_cone_count: int = 10):
    """
    单帧测试：只显示前 N 个锥桶的分类结果。
    用于快速验证算法在特定场景下的行为。
    """
    print("=" * 80)
    print("  单帧测试模式")
    print("=" * 80)

    ref_line = create_reference_line()

    all_cones = []
    for i, (x, y) in enumerate(RIGHT_CONES_XY):
        s, l = ref_line.xy_to_sl(x, y)
        all_cones.append(SLPolygon(id=f"R_{i:02d}", s_min=s-0.1, s_max=s+0.1,
                                    l_min=l-0.1, l_max=l+0.1, x=x, y=y))
    for i, (x, y) in enumerate(LEFT_CONES_XY):
        s, l = ref_line.xy_to_sl(x, y)
        all_cones.append(SLPolygon(id=f"L_{i:02d}", s_min=s-0.1, s_max=s+0.1,
                                    l_min=l-0.1, l_max=l+0.1, x=x, y=y))
    all_cones.sort(key=lambda c: c.s_center)

    # 从道路边界计算 road_left / road_right
    left_boundary_ls = []
    right_boundary_ls = []
    for x, y in LEFT_ROAD_BOUNDARY:
        _, l = ref_line.xy_to_sl(x, y)
        left_boundary_ls.append(l)
    for x, y in RIGHT_ROAD_BOUNDARY:
        _, l = ref_line.xy_to_sl(x, y)
        right_boundary_ls.append(l)
    road_left = max(left_boundary_ls)
    road_right = min(right_boundary_ls)

    visible = all_cones[:visible_cone_count]
    state = ConstructionZoneState()
    state.max_left_bound = road_left
    state.max_right_bound = road_right

    print(f"  可见锥桶: {len(visible)}")
    print(f"  road_left={road_left:.2f}, road_right={road_right:.2f}")
    print()

    classify_cones(visible, road_left, road_right, state, verbose=True)


# ============================================================
# 8. 可视化
# ============================================================

def plot_results(
        all_cones: List[SLPolygon],
        state: ConstructionZoneState,
        road_left: float,
        road_right: float,
        ref_line: ReferenceLine,
        frame_results: List[Dict] = None,
        save_path: str = None
):
    """
    绘制锥桶分类结果的可视化图。

    包含：
    - 上方: SL 空间图（s-l 坐标系）
    - 下方: XY 空间图（实际坐标）
    - 锥桶颜色: 红=GT 右侧, 蓝=GT 左侧
    - 锥桶标记: 实心=正确, 空心=错误
    - 墙壁线: 粉色虚线=左墙, 青色虚线=右墙
    """
    try:
        import matplotlib
        matplotlib.use('Agg')  # 非交互模式
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("\n  [!] matplotlib 未安装，跳过可视化。安装: pip install matplotlib")
        return

    fig, (ax_sl, ax_xy) = plt.subplots(1, 2, figsize=(18, 8))

    # ---- SL 空间 ----
    # 道路边界
    ax_sl.axhline(y=road_left, color='gray', linestyle='--', alpha=0.4, label=f'road_left={road_left:.1f}')
    ax_sl.axhline(y=road_right, color='gray', linestyle='--', alpha=0.4, label=f'road_right={road_right:.1f}')
    ax_sl.axhline(y=0, color='green', linestyle='-', alpha=0.3, linewidth=1)

    # 墙壁
    if state.left_wall:
        lw_s = [p[0] for p in state.left_wall]
        lw_l = [p[1] for p in state.left_wall]
        ax_sl.plot(lw_s, lw_l, 'm--', linewidth=2, alpha=0.7, label='left_wall')
    if state.right_wall:
        rw_s = [p[0] for p in state.right_wall]
        rw_l = [p[1] for p in state.right_wall]
        ax_sl.plot(rw_s, rw_l, 'c--', linewidth=2, alpha=0.7, label='right_wall')

    # 锥桶
    for cone in all_cones:
        gt = "L" if cone.id.startswith("L_") else "R"
        pred = cone.classified_side()
        correct = (gt == pred)

        color = 'blue' if gt == 'L' else 'red'
        marker = 'o' if correct else 's'
        size = 80 if correct else 120
        edge_width = 1.5 if correct else 2.5
        alpha = 0.8 if correct else 1.0

        ax_sl.scatter(cone.s_center, cone.l_center, c=color, marker=marker,
                      s=size, edgecolors='black' if not correct else 'none',
                      linewidths=edge_width, alpha=alpha, zorder=5)
        if not correct:
            ax_sl.annotate(cone.id, (cone.s_center, cone.l_center),
                           fontsize=7, color='darkred', fontweight='bold',
                           xytext=(5, 5), textcoords='offset points')

    ax_sl.set_xlabel('S (m)')
    ax_sl.set_ylabel('L (m)')
    ax_sl.set_title('SL Space (施工区域锥桶分类)')
    ax_sl.grid(True, alpha=0.3)

    # 图例
    legend_elements = [
        mpatches.Patch(color='red', alpha=0.7, label='GT 右侧锥桶'),
        mpatches.Patch(color='blue', alpha=0.7, label='GT 左侧锥桶'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='gray',
                   markersize=8, label='分类正确'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='gray',
                   markersize=10, markeredgecolor='black', label='分类错误'),
        plt.Line2D([0], [0], linestyle='--', color='magenta', label='left_wall'),
        plt.Line2D([0], [0], linestyle='--', color='cyan', label='right_wall'),
    ]
    ax_sl.legend(handles=legend_elements, loc='upper right', fontsize=7)

    # ---- XY 空间 ----
    # 道路边界
    lb_x = [p[0] for p in LEFT_ROAD_BOUNDARY]
    lb_y = [p[1] for p in LEFT_ROAD_BOUNDARY]
    rb_x = [p[0] for p in RIGHT_ROAD_BOUNDARY]
    rb_y = [p[1] for p in RIGHT_ROAD_BOUNDARY]
    ax_xy.plot(lb_x, lb_y, 'gray', linestyle='--', linewidth=1.5, alpha=0.5, label='left road boundary')
    ax_xy.plot(rb_x, rb_y, 'gray', linestyle='--', linewidth=1.5, alpha=0.5, label='right road boundary')

    # 参考线
    rl_x = [p[0] for p in ref_line.waypoints]
    rl_y = [p[1] for p in ref_line.waypoints]
    ax_xy.plot(rl_x, rl_y, 'green', linestyle='-', linewidth=1, alpha=0.4, label='reference line')

    # 锥桶
    for cone in all_cones:
        gt = "L" if cone.id.startswith("L_") else "R"
        pred = cone.classified_side()
        correct = (gt == pred)

        color = 'blue' if gt == 'L' else 'red'
        marker = 'o' if correct else 's'
        size = 60 if correct else 100
        edge_width = 1.5 if correct else 2.5
        alpha = 0.8 if correct else 1.0

        ax_xy.scatter(cone.x, cone.y, c=color, marker=marker,
                      s=size, edgecolors='black' if not correct else 'none',
                      linewidths=edge_width, alpha=alpha, zorder=5)
        if not correct:
            ax_xy.annotate(cone.id, (cone.x, cone.y),
                           fontsize=6, color='darkred', fontweight='bold',
                           xytext=(5, 5), textcoords='offset points')

    # 墙壁在 XY 空间 (通过 SL→XY 转换)
    if state.left_wall:
        lw_xy = [ref_line.sl_to_xy(s, l) for s, l in state.left_wall]
        lwx = [p[0] for p in lw_xy]
        lwy = [p[1] for p in lw_xy]
        ax_xy.plot(lwx, lwy, 'm--', linewidth=2, alpha=0.7, label='left_wall')
    if state.right_wall:
        rw_xy = [ref_line.sl_to_xy(s, l) for s, l in state.right_wall]
        rwx = [p[0] for p in rw_xy]
        rwy = [p[1] for p in rw_xy]
        ax_xy.plot(rwx, rwy, 'c--', linewidth=2, alpha=0.7, label='right_wall')

    ax_xy.set_xlabel('X (UTM)')
    ax_xy.set_ylabel('Y (UTM)')
    ax_xy.set_title('XY Space (施工区域锥桶位置)')
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc='upper right', fontsize=7)
    ax_xy.set_aspect('equal')

    plt.suptitle('施工区域锥桶左右分类仿真结果', fontsize=14, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n  可视化已保存至: {save_path}")
    else:
        save_path = '/home/skye/application-pnc/output/cone_classification_result.png'
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n  可视化已保存至: {save_path}")

    plt.close()


# ============================================================
# 9. 主入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="施工区域锥桶左右分类算法仿真",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 cone_classification_sim.py                    # 完整仿真
  python3 cone_classification_sim.py --step             # 逐帧步进
  python3 cone_classification_sim.py --single 10        # 只看前10个锥桶
  python3 cone_classification_sim.py --step-size 3.0    # 缩小步长（更密集的帧）
  python3 cone_classification_sim.py --look-forward 40  # 缩小感知范围
        """
    )
    parser.add_argument("--step-size", type=float, default=5.0,
                        help="每帧车辆前进距离 (m)，默认 5.0")
    parser.add_argument("--look-forward", type=float, default=60.0,
                        help="车辆前方感知距离 (m)，默认 60.0")
    parser.add_argument("--look-backward", type=float, default=10.0,
                        help="车辆后方保留距离 (m)，默认 10.0")
    parser.add_argument("--step", action="store_true",
                        help="逐帧步进模式（每帧暂停等待 Enter）")
    parser.add_argument("--single", type=int, default=0,
                        help="单帧测试模式：只分类前 N 个锥桶")
    parser.add_argument("--quiet", action="store_true",
                        help="减少输出")
    parser.add_argument("--plot", action="store_true", default=True,
                        help="生成可视化图 (默认开启)")
    parser.add_argument("--no-plot", action="store_true",
                        help="不生成可视化图")
    parser.add_argument("--save-plot", type=str, default=None,
                        help="保存可视化图到指定路径")
    parser.add_argument("--from-cpp-json", type=str, default=None,
                        help="从 C++ 诊断日志提取的 JSON 文件加载真实数据 (cpp_diag_data.json)")

    args = parser.parse_args()

    do_plot = args.plot and not args.no_plot

    if args.from_cpp_json:
        state, final_cones, ref_line = run_simulation_cpp_mode(
            json_path=args.from_cpp_json,
            step_size=args.step_size,
            look_forward=args.look_forward,
            look_backward=args.look_backward,
            step_mode=args.step,
            verbose=not args.quiet,
            do_plot=do_plot
        )
        if do_plot:
            save_path = args.save_plot
            if save_path is None:
                save_path = '/home/skye/application-pnc/output/cone_classification_result_cpp.png'
            plot_results(final_cones, state, state.max_left_bound, state.max_right_bound,
                        ref_line, save_path=save_path)
    elif args.single > 0:
        run_single_frame(args.single)
    else:
        state, final_cones, ref_line = run_simulation(
            step_size=args.step_size,
            look_forward=args.look_forward,
            look_backward=args.look_backward,
            step_mode=args.step,
            verbose=not args.quiet,
            do_plot=do_plot
        )
        if do_plot:
            save_path = args.save_plot
            if save_path is None:
                save_path = '/home/skye/application-pnc/output/cone_classification_result.png'
            plot_results(final_cones, state, state.max_left_bound, state.max_right_bound,
                        ref_line, save_path=save_path)


if __name__ == "__main__":
    main()
