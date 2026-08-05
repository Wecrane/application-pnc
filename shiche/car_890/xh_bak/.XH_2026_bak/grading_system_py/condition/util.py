"""
条件评估工具类
对应原 C++ condition/util.cc
"""

import math
from typing import List, Tuple, Optional

from grading_system_py.common.math_utils import Vec2d, LineSegment2d, Box2d, Polygon2d
from grading_system_py.common.map_interface import get_map_interface, LaneInfo


class ConditionEvaluatorUtil:
    """评估工具类 - 提供几何计算和辅助判断的静态方法"""

    OBJECT_ID_PATTERN_ADC_ONLY = ""
    OBJECT_ID_PATTERN_ALL_OBSTACLES = "*"

    @staticmethod
    def get_object_polygon(obj, symmetry: bool = True) -> Polygon2d:
        """
        根据 Object 构建多边形

        Args:
            obj: Object (protobuf 或 dict-like)
            symmetry: 是否对称构建（False时考虑front/back_edge_to_center偏移）
        """
        # 如果有polygon_point直接使用
        polygon_points = getattr(obj, 'polygon_point', [])
        if polygon_points and len(polygon_points) > 0:
            points = [Vec2d(p.x, p.y) for p in polygon_points]
            return Polygon2d(points)

        position_x = obj.position_x
        position_y = obj.position_y
        heading = obj.heading
        length = obj.length
        width = obj.width

        if not symmetry:
            front_edge = getattr(obj, 'front_edge_to_center', length / 2.0)
            back_edge = getattr(obj, 'back_edge_to_center', length / 2.0)
            offset = 0.0
            if front_edge > back_edge:
                offset = front_edge - length / 2.0
            else:
                offset = back_edge - length / 2.0
            geometry_x = position_x + offset * math.cos(heading)
            geometry_y = position_y + offset * math.sin(heading)
            return Polygon2d(Box2d(Vec2d(geometry_x, geometry_y), heading, length, width))

        return Polygon2d(Box2d(Vec2d(position_x, position_y), heading, length, width))

    @staticmethod
    def get_object_polygons(world, object_ids: str) -> List[Polygon2d]:
        """获取多个对象的多边形"""
        polygons = []
        if object_ids == ConditionEvaluatorUtil.OBJECT_ID_PATTERN_ADC_ONLY:
            polygons.append(ConditionEvaluatorUtil.get_object_polygon(world.auto_driving_car, False))
            return polygons
        if object_ids == ConditionEvaluatorUtil.OBJECT_ID_PATTERN_ALL_OBSTACLES:
            for obj in world.object:
                polygons.append(ConditionEvaluatorUtil.get_object_polygon(obj))
            return polygons
        return polygons

    @staticmethod
    def is_behind(adc, obj, length: float = 50.0, width: float = 15.0) -> bool:
        """判断 obj 是否在 adc 后方区域内"""
        front_edge = getattr(adc, 'front_edge_to_center', adc.length / 2.0)
        region_center_x = adc.position_x - (length / 2.0 - front_edge) * math.cos(adc.heading)
        region_center_y = adc.position_y - (length / 2.0 - front_edge) * math.sin(adc.heading)
        region_polygon = Polygon2d(Box2d(Vec2d(region_center_x, region_center_y),
                                         adc.heading, length, width))
        obj_polygon = ConditionEvaluatorUtil.get_object_polygon(obj)
        return region_polygon.contains(obj_polygon)

    @staticmethod
    def lines_intersect(p1: Vec2d, heading1: float, p2: Vec2d, heading2: float) -> Tuple[bool, float, float]:
        """
        计算两条直线的交点参数

        Returns:
            (是否相交, s1, s2) 交点在各自线上的参数
        """
        if abs(heading1 - heading2) < 0.01:
            return False, 0.0, 0.0

        hv1 = Vec2d.create_unit_vec2d(heading1)
        hv2 = Vec2d.create_unit_vec2d(heading2)
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        det = hv2.x * hv1.y - hv2.y * hv1.x
        s1 = (dy * hv2.x - dx * hv2.y) / det
        s2 = (dy * hv1.x - dx * hv1.y) / det
        return True, s1, s2

    @staticmethod
    def rays_intersect(p1: Vec2d, heading1: float, p2: Vec2d, heading2: float,
                       tolerance: float = float('inf')) -> bool:
        """判断两条射线是否在有限范围内相交"""
        intersect, s1, s2 = ConditionEvaluatorUtil.lines_intersect(p1, heading1, p2, heading2)
        if intersect and s1 > 1.0 and s2 > 1.0 and s1 < tolerance and s2 < tolerance:
            return True
        return False

    @staticmethod
    def get_box_head_tail_position(width: float, heading: float,
                                   poly: Polygon2d) -> Tuple[bool, Vec2d, Vec2d]:
        """
        获取矩形物体的车头和车尾中心点

        Returns:
            (success, head_position, tail_position)
        """
        epsilon = 0.0001
        if poly.num_points != 4:
            return False, Vec2d(), Vec2d()

        width_lines = []
        for seg in poly.line_segments:
            if abs(seg.length() - width) < epsilon:
                width_lines.append(seg)

        if len(width_lines) != 2:
            return False, Vec2d(), Vec2d()

        center_line = LineSegment2d(width_lines[0].center(), width_lines[1].center())
        if center_line.heading() * heading > 0:
            head_position = width_lines[1].center()
            tail_position = width_lines[0].center()
        else:
            head_position = width_lines[0].center()
            tail_position = width_lines[1].center()

        return True, head_position, tail_position

    @staticmethod
    def get_object_polygon_from_points(polygon_proto) -> Polygon2d:
        """从 protobuf Polygon 消息构建多边形"""
        points = []
        for pt in polygon_proto.point:
            points.append(Vec2d(pt.x, pt.y))
        return Polygon2d(points)

    @staticmethod
    def get_lateral_distance(obj_polygon: Polygon2d, adc_x: float, adc_y: float,
                             adc_half_width: float) -> float:
        """计算主车到障碍物的横向距离"""
        adc_position = Vec2d(adc_x, adc_y)
        return obj_polygon.distance_to(adc_position) - adc_half_width

    @staticmethod
    def dist_xy(x1: float, y1: float, x2: float, y2: float) -> float:
        """计算两个二维坐标点的距离"""
        return math.hypot(x1 - x2, y1 - y2)

    @staticmethod
    def score_deducted_by_unit(delta: float, deduction_unit: float,
                               single_deduction: float) -> float:
        """按单位计算扣分"""
        if deduction_unit <= 0:
            return single_deduction
        return math.ceil(delta / deduction_unit) * single_deduction

    @staticmethod
    def ttc(master, obj) -> float:
        """
        计算 Time-To-Collision

        基于两个物体多边形最近点距离和相对速度投影
        """
        master_polygon = ConditionEvaluatorUtil.get_object_polygon(master, False)
        obj_polygon = ConditionEvaluatorUtil.get_object_polygon(obj, False)

        dist, closest_pt, master_closest_pt = ConditionEvaluatorUtil.min_dist_between_polygon(
            obj_polygon, master_polygon)

        if dist < 0.1:
            return 0.0

        dx = closest_pt.x - master_closest_pt.x
        dy = closest_pt.y - master_closest_pt.y
        ds = math.hypot(dx, dy)
        if ds < 1e-10:
            return 0.0

        dx /= ds
        dy /= ds

        master_speed = getattr(master, 'speed', 0.0)
        master_heading = master.heading
        obj_speed = getattr(obj, 'speed', 0.0)
        obj_heading = obj.heading

        master_v = master_speed * math.cos(master_heading) * dx + master_speed * math.sin(master_heading) * dy
        obj_v = obj_speed * math.cos(obj_heading) * dx + obj_speed * math.sin(obj_heading) * dy
        dv = master_v - obj_v

        if dv <= 1e-6:
            return float('inf')

        return ds / dv

    @staticmethod
    def min_dist_between_polygon(first_polygon: Polygon2d,
                                  second_polygon: Polygon2d) -> Tuple[float, Vec2d, Vec2d]:
        """
        计算两个多边形之间的最小距离

        Returns:
            (distance, first_closest_point, second_closest_point)
        """
        if first_polygon.is_point_in(second_polygon.points[0]):
            return 0.0, second_polygon.points[0], second_polygon.points[0]
        if second_polygon.is_point_in(first_polygon.points[0]):
            return 0.0, first_polygon.points[0], first_polygon.points[0]

        min_distance = float('inf')
        first_closest = Vec2d()
        second_closest = Vec2d()

        for seg in first_polygon.line_segments:
            dist, poly_pt, seg_pt = ConditionEvaluatorUtil._min_dist_polygon_segment(
                second_polygon, seg)
            if dist < min_distance:
                min_distance = dist
                first_closest = seg_pt
                second_closest = poly_pt

        return min_distance, first_closest, second_closest

    @staticmethod
    def _min_dist_polygon_segment(polygon: Polygon2d,
                                   segment: LineSegment2d) -> Tuple[float, Vec2d, Vec2d]:
        """多边形与线段的最小距离"""
        if segment.length() <= 1e-6:
            dist, closest = polygon.distance_to_point(segment.start)
            return dist, closest, segment.start

        if polygon.is_point_in(segment.center()):
            return 0.0, segment.center(), segment.center()

        # 检查是否有边相交
        for poly_seg in polygon.line_segments:
            if poly_seg.has_intersect(segment):
                return 0.0, segment.center(), segment.center()

        # 计算端点到多边形的距离
        dist_start, start_closest = polygon.distance_to_point(segment.start)
        dist_end, end_closest = polygon.distance_to_point(segment.end)

        if dist_start <= dist_end:
            min_distance = dist_start
            poly_closest = start_closest
            seg_closest = segment.start
        else:
            min_distance = dist_end
            poly_closest = end_closest
            seg_closest = segment.end

        # 计算多边形顶点到线段的距离
        for point in polygon.points:
            dist, closest = segment.distance_to_point(point)
            if dist < min_distance:
                min_distance = dist
                seg_closest = closest
                poly_closest = point

        return min_distance, poly_closest, seg_closest

    @staticmethod
    def is_adc_over_the_end(end_line: LineSegment2d, adc_x: float,
                             adc_y: float) -> Tuple[bool, float]:
        """
        判断主车是否越过终止线

        Returns:
            (is_over, distance)
        """
        start = end_line.start
        end = end_line.end
        tmp = (end.x - start.x) * (adc_y - start.y) - \
              (end.y - start.y) * (adc_x - start.x)

        if tmp == 0:
            return False, 0.0

        distance = end_line.distance_to(Vec2d(adc_x, adc_y))
        if tmp > 0:
            distance = -distance
        return tmp > 0, distance
