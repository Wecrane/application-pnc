"""
地图抽象接口 + Apollo Map 轻量实现
对应原 C++ 项目中 Apollo HD Map 的部分接口
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from grading_system_py.common.math_utils import Vec2d, LineSegment2d, Polygon2d


@dataclass
class LaneId:
    id: str = ""


@dataclass
class LaneInfo:
    """车道信息"""
    lane_id: LaneId = field(default_factory=LaneId)
    total_length: float = 0.0
    turn_type: str = "NO_TURN"
    predecessor_ids: List[LaneId] = field(default_factory=list)
    successor_ids: List[LaneId] = field(default_factory=list)

    def get_projection(self, x: float, y: float) -> Tuple[bool, float, float]:
        return False, 0.0, 0.0

    def get_width(self, s: float) -> Tuple[float, float]:
        return 1.75, 1.75

    def get_road_width(self, s: float) -> Tuple[float, float]:
        return 3.5, 3.5


@dataclass
class SignalInfo:
    """信号灯信息"""
    signal_id: str = ""
    stop_line_points: List[List[Tuple[float, float]]] = field(default_factory=list)


@dataclass
class StopSignInfo:
    """停止标志信息"""
    stop_sign_id: str = ""
    stop_line_points: List[List[Tuple[float, float]]] = field(default_factory=list)


@dataclass
class CrosswalkInfo:
    """人行横道信息"""
    crosswalk_id: str = ""
    polygon: Polygon2d = None


@dataclass
class SpeedBumpInfo:
    """减速带信息"""
    bump_id: str = ""
    points: List[Tuple[float, float]] = field(default_factory=list)


class MapInterface(ABC):
    """地图抽象接口"""

    @abstractmethod
    def get_lanes(self, x: float, y: float, radius: float) -> List[LaneInfo]:
        ...

    @abstractmethod
    def get_lane_by_id(self, lane_id: str) -> Optional[LaneInfo]:
        ...

    @abstractmethod
    def get_forward_nearest_signals(self, x: float, y: float, distance: float) -> List[SignalInfo]:
        ...

    @abstractmethod
    def get_exact_lanes(self, x: float, y: float, radius: float) -> List[LaneInfo]:
        ...

    def find_crosswalk_for_polygon(self, x: float, y: float, radius: float,
                                   polygon: Polygon2d) -> Optional[CrosswalkInfo]:
        return None

    def get_forward_nearest_stop_signs(self, x: float, y: float, distance: float) -> List['StopSignInfo']:
        return []

    def get_forward_nearest_speed_bumps(self, x: float, y: float, distance: float) -> List['SpeedBumpInfo']:
        return []


class StubMapInterface(MapInterface):
    """Stub 实现 - 返回空结果，后续可替换为真实地图服务"""

    def get_lanes(self, x: float, y: float, radius: float) -> List[LaneInfo]:
        return []

    def get_lane_by_id(self, lane_id: str) -> Optional[LaneInfo]:
        return None

    def get_forward_nearest_signals(self, x: float, y: float, distance: float) -> List[SignalInfo]:
        return []

    def get_exact_lanes(self, x: float, y: float, radius: float) -> List[LaneInfo]:
        return []


class ApolloMapInterface(StubMapInterface):
    """从 Apollo base_map/sim_map 文件加载 crosswalk 与 signal stop_line 的轻量实现。"""

    def __init__(self, map_file: str):
        self.map_file = map_file
        self.crosswalks: List[CrosswalkInfo] = []
        self.signals: List[SignalInfo] = []
        self.stop_signs: List[StopSignInfo] = []
        self.speed_bumps: List[SpeedBumpInfo] = []
        self._load_map(map_file)

    def _load_map(self, map_file: str):
        try:
            from google.protobuf import text_format
            from modules.common_msgs.map_msgs import map_pb2
        except ImportError as exc:
            raise RuntimeError(f"Apollo map protobuf import failed: {exc}")

        pb_map = map_pb2.Map()
        if map_file.endswith('.txt'):
            with open(map_file, 'r', encoding='utf-8') as f:
                text_format.Merge(f.read(), pb_map)
        else:
            with open(map_file, 'rb') as f:
                pb_map.ParseFromString(f.read())

        for cw in getattr(pb_map, 'crosswalk', []):
            points = [Vec2d(p.x, p.y) for p in cw.polygon.point]
            if len(points) >= 3:
                self.crosswalks.append(CrosswalkInfo(cw.id.id, Polygon2d(points)))

        for sig in getattr(pb_map, 'signal', []):
            stop_lines = []
            for stop_line in sig.stop_line:
                for seg in stop_line.segment:
                    if not seg.HasField('line_segment'):
                        continue
                    pts = [(p.x, p.y) for p in seg.line_segment.point]
                    if len(pts) >= 2:
                        stop_lines.append(pts)
            if stop_lines:
                self.signals.append(SignalInfo(sig.id.id, stop_lines))

        for stop_sign in getattr(pb_map, 'stop_sign', []):
            stop_lines = []
            for stop_line in stop_sign.stop_line:
                for seg in stop_line.segment:
                    if not seg.HasField('line_segment'):
                        continue
                    pts = [(p.x, p.y) for p in seg.line_segment.point]
                    if len(pts) >= 2:
                        stop_lines.append(pts)
            if stop_lines:
                self.stop_signs.append(StopSignInfo(stop_sign.id.id, stop_lines))

        for sb in getattr(pb_map, 'speed_bump', []):
            # 减速带几何在 position(repeated Curve).segment.line_segment.point 里。
            pts = []
            for curve in sb.position:
                for seg in curve.segment:
                    if not seg.HasField('line_segment'):
                        continue
                    pts.extend((p.x, p.y) for p in seg.line_segment.point)
            if len(pts) >= 2:
                self.speed_bumps.append(SpeedBumpInfo(sb.id.id, pts))

    def get_forward_nearest_speed_bumps(self, x: float, y: float,
                                        distance: float) -> List[SpeedBumpInfo]:
        loc = Vec2d(x, y)
        result = []
        for sb in self.speed_bumps:
            pts = sb.points
            if len(pts) < 2:
                continue
            seg = LineSegment2d(Vec2d(pts[0][0], pts[0][1]),
                                Vec2d(pts[-1][0], pts[-1][1]))
            if seg.distance_to(loc) <= distance:
                result.append(sb)
        return result

    def get_forward_nearest_stop_signs(self, x: float, y: float, distance: float) -> List[StopSignInfo]:
        loc = Vec2d(x, y)
        result = []
        for ss in self.stop_signs:
            min_dist = float('inf')
            for pts in ss.stop_line_points:
                if len(pts) < 2:
                    continue
                seg = LineSegment2d(Vec2d(pts[0][0], pts[0][1]), Vec2d(pts[-1][0], pts[-1][1]))
                min_dist = min(min_dist, seg.distance_to(loc))
            if min_dist <= distance:
                result.append(ss)
        return result

    def get_forward_nearest_signals(self, x: float, y: float, distance: float) -> List[SignalInfo]:
        loc = Vec2d(x, y)
        result = []
        for sig in self.signals:
            min_dist = float('inf')
            for pts in sig.stop_line_points:
                if len(pts) < 2:
                    continue
                seg = LineSegment2d(Vec2d(pts[0][0], pts[0][1]), Vec2d(pts[-1][0], pts[-1][1]))
                min_dist = min(min_dist, seg.distance_to(loc))
            if min_dist <= distance:
                result.append(sig)
        return result

    def find_crosswalk_for_polygon(self, x: float, y: float, radius: float,
                                   polygon: Polygon2d) -> Optional[CrosswalkInfo]:
        loc = Vec2d(x, y)
        for cw in self.crosswalks:
            # 先用人位置到 crosswalk 距离做半径过滤，再检查行人多边形是否重叠
            if cw.polygon.distance_to(loc) <= radius and cw.polygon.has_overlap(polygon):
                return cw
        return None


# 全局地图实例，可在运行时替换
_map_instance: MapInterface = StubMapInterface()


def set_map_interface(map_impl: MapInterface):
    global _map_instance
    _map_instance = map_impl


def get_map_interface() -> MapInterface:
    return _map_instance
