"""
几何工具库 - 提供 Vec2d, LineSegment2d, Box2d, Polygon2d 等基本几何类型
对应原 C++ 项目中 modules/common/math/ 下的几何库
"""

import math
from typing import List, Tuple, Optional


class Vec2d:
    """二维向量"""

    def __init__(self, x: float = 0.0, y: float = 0.0):
        self._x = x
        self._y = y

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @staticmethod
    def create_unit_vec2d(angle: float) -> 'Vec2d':
        return Vec2d(math.cos(angle), math.sin(angle))

    def length(self) -> float:
        return math.hypot(self._x, self._y)

    def length_sqr(self) -> float:
        return self._x * self._x + self._y * self._y

    def angle(self) -> float:
        return math.atan2(self._y, self._x)

    def distance_to(self, other: 'Vec2d') -> float:
        return math.hypot(self._x - other._x, self._y - other._y)

    def distance_sqr_to(self, other: 'Vec2d') -> float:
        dx = self._x - other._x
        dy = self._y - other._y
        return dx * dx + dy * dy

    def cross_prod(self, other: 'Vec2d') -> float:
        return self._x * other._y - self._y * other._x

    def inner_prod(self, other: 'Vec2d') -> float:
        return self._x * other._x + self._y * other._y

    def rotate(self, angle: float) -> 'Vec2d':
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        return Vec2d(self._x * cos_a - self._y * sin_a,
                     self._x * sin_a + self._y * cos_a)

    def normalize(self) -> 'Vec2d':
        l = self.length()
        if l < 1e-10:
            return Vec2d(0, 0)
        return Vec2d(self._x / l, self._y / l)

    def __add__(self, other: 'Vec2d') -> 'Vec2d':
        return Vec2d(self._x + other._x, self._y + other._y)

    def __sub__(self, other: 'Vec2d') -> 'Vec2d':
        return Vec2d(self._x - other._x, self._y - other._y)

    def __mul__(self, scalar: float) -> 'Vec2d':
        return Vec2d(self._x * scalar, self._y * scalar)

    def __rmul__(self, scalar: float) -> 'Vec2d':
        return Vec2d(self._x * scalar, self._y * scalar)

    def __repr__(self) -> str:
        return f"Vec2d({self._x:.4f}, {self._y:.4f})"


class LineSegment2d:
    """二维线段"""

    def __init__(self, start: Vec2d, end: Vec2d):
        self._start = start
        self._end = end
        dx = end.x - start.x
        dy = end.y - start.y
        self._length = math.hypot(dx, dy)
        self._heading = math.atan2(dy, dx) if self._length > 1e-10 else 0.0
        self._unit_direction = Vec2d(dx / self._length, dy / self._length) if self._length > 1e-10 else Vec2d(1, 0)

    @property
    def start(self) -> Vec2d:
        return self._start

    @property
    def end(self) -> Vec2d:
        return self._end

    def length(self) -> float:
        return self._length

    def heading(self) -> float:
        return self._heading

    def center(self) -> Vec2d:
        return Vec2d((self._start.x + self._end.x) / 2.0,
                     (self._start.y + self._end.y) / 2.0)

    def distance_to_point(self, point: Vec2d) -> Tuple[float, Vec2d]:
        """返回 (距离, 最近点)"""
        if self._length < 1e-10:
            return self._start.distance_to(point), self._start

        dx = point.x - self._start.x
        dy = point.y - self._start.y
        proj = dx * self._unit_direction.x + dy * self._unit_direction.y
        proj = max(0.0, min(self._length, proj))

        closest = Vec2d(self._start.x + proj * self._unit_direction.x,
                        self._start.y + proj * self._unit_direction.y)
        return closest.distance_to(point), closest

    def distance_to(self, point: Vec2d) -> float:
        dist, _ = self.distance_to_point(point)
        return dist

    def has_intersect(self, other: 'LineSegment2d') -> bool:
        """检测两线段是否相交"""
        d1 = self._cross(self._start, self._end, other._start)
        d2 = self._cross(self._start, self._end, other._end)
        d3 = self._cross(other._start, other._end, self._start)
        d4 = self._cross(other._start, other._end, self._end)

        if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
           ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
            return True

        if abs(d1) < 1e-10 and self._on_segment(self._start, self._end, other._start):
            return True
        if abs(d2) < 1e-10 and self._on_segment(self._start, self._end, other._end):
            return True
        if abs(d3) < 1e-10 and self._on_segment(other._start, other._end, self._start):
            return True
        if abs(d4) < 1e-10 and self._on_segment(other._start, other._end, self._end):
            return True

        return False

    @staticmethod
    def _cross(a: Vec2d, b: Vec2d, c: Vec2d) -> float:
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)

    @staticmethod
    def _on_segment(a: Vec2d, b: Vec2d, c: Vec2d) -> bool:
        return (min(a.x, b.x) <= c.x <= max(a.x, b.x) and
                min(a.y, b.y) <= c.y <= max(a.y, b.y))


class Box2d:
    """二维矩形（带朝向）"""

    def __init__(self, center: Vec2d, heading: float, length: float, width: float):
        self._center = center
        self._heading = heading
        self._length = length
        self._width = width
        self._cos_heading = math.cos(heading)
        self._sin_heading = math.sin(heading)
        self._corners = self._compute_corners()

    def _compute_corners(self) -> List[Vec2d]:
        dx1 = self._cos_heading * self._length / 2.0
        dy1 = self._sin_heading * self._length / 2.0
        dx2 = self._sin_heading * self._width / 2.0
        dy2 = -self._cos_heading * self._width / 2.0
        cx, cy = self._center.x, self._center.y
        return [
            Vec2d(cx + dx1 + dx2, cy + dy1 + dy2),
            Vec2d(cx + dx1 - dx2, cy + dy1 - dy2),
            Vec2d(cx - dx1 - dx2, cy - dy1 - dy2),
            Vec2d(cx - dx1 + dx2, cy - dy1 + dy2),
        ]

    @classmethod
    def from_line_segment(cls, seg: LineSegment2d, width: float) -> 'Box2d':
        return cls(seg.center(), seg.heading(), seg.length(), width)

    @property
    def center(self) -> Vec2d:
        return self._center

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def length(self) -> float:
        return self._length

    @property
    def width(self) -> float:
        return self._width

    def corners(self) -> List[Vec2d]:
        return self._corners


class Polygon2d:
    """二维多边形"""

    def __init__(self, points_or_box):
        if isinstance(points_or_box, Box2d):
            self._points = points_or_box.corners()
        elif isinstance(points_or_box, list):
            self._points = list(points_or_box)
        else:
            raise ValueError("Polygon2d requires a list of Vec2d or a Box2d")

        self._num_points = len(self._points)
        self._segments = self._build_segments()
        self._min_x = min(p.x for p in self._points)
        self._max_x = max(p.x for p in self._points)
        self._min_y = min(p.y for p in self._points)
        self._max_y = max(p.y for p in self._points)

    def _build_segments(self) -> List[LineSegment2d]:
        segs = []
        n = self._num_points
        for i in range(n):
            segs.append(LineSegment2d(self._points[i], self._points[(i + 1) % n]))
        return segs

    @property
    def points(self) -> List[Vec2d]:
        return self._points

    @property
    def num_points(self) -> int:
        return self._num_points

    @property
    def line_segments(self) -> List[LineSegment2d]:
        return self._segments

    @property
    def min_x(self) -> float:
        return self._min_x

    @property
    def max_x(self) -> float:
        return self._max_x

    @property
    def min_y(self) -> float:
        return self._min_y

    @property
    def max_y(self) -> float:
        return self._max_y

    def is_point_in(self, point: Vec2d) -> bool:
        """射线法判断点是否在多边形内"""
        if point.x < self._min_x or point.x > self._max_x:
            return False
        if point.y < self._min_y or point.y > self._max_y:
            return False

        inside = False
        j = self._num_points - 1
        for i in range(self._num_points):
            pi = self._points[i]
            pj = self._points[j]
            if ((pi.y > point.y) != (pj.y > point.y)) and \
               (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x):
                inside = not inside
            j = i
        return inside

    def contains(self, other: 'Polygon2d') -> bool:
        """判断是否完全包含另一个多边形"""
        return all(self.is_point_in(p) for p in other._points)

    def has_overlap(self, other) -> bool:
        """判断与另一个多边形或线段是否有重叠"""
        if isinstance(other, Polygon2d):
            # 任一顶点在对方内部
            for p in other._points:
                if self.is_point_in(p):
                    return True
            for p in self._points:
                if other.is_point_in(p):
                    return True
            # 边相交
            for seg1 in self._segments:
                for seg2 in other._segments:
                    if seg1.has_intersect(seg2):
                        return True
            return False
        elif isinstance(other, LineSegment2d):
            if self.is_point_in(other.start) or self.is_point_in(other.end):
                return True
            for seg in self._segments:
                if seg.has_intersect(other):
                    return True
            return False
        return False

    def distance_to_point(self, point: Vec2d) -> Tuple[float, Vec2d]:
        """点到多边形的最短距离和最近点"""
        if self.is_point_in(point):
            return 0.0, point
        min_dist = float('inf')
        closest = point
        for seg in self._segments:
            dist, pt = seg.distance_to_point(point)
            if dist < min_dist:
                min_dist = dist
                closest = pt
        return min_dist, closest

    def distance_to(self, point: Vec2d) -> float:
        dist, _ = self.distance_to_point(point)
        return dist
