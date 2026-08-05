"""
物体重叠条件评估器 - 碰撞检测
对应原 C++ object_overlap_condition_handler.cc

评估逻辑:
- 获取源对象（主车）和目标对象（所有障碍物）的多边形
- 如果源和目标多边形之间距离 <= distance 阈值，则发生碰撞
- 支持方向过滤: INCLUDE_BACK（包含后方碰撞）或 EXCLUDE_BACK（排除后方碰撞）
- 碰撞时 delta_score=100

注: 原 C++ 中 Collision metric 使用 NOT(object_overlap)，
    即 object_overlap=true 表示碰撞，外层 NOT 将其翻转为 metric fail。
    在我们的实现中，evaluate 返回 true 表示"发生碰撞"。
    调用时需注意：Collision metric 配置了 logical_condition NOT 包裹，
    但简化实现中我们直接在 handler 中处理取反逻辑。
"""

from collections import deque
from typing import List

from grading_system_py.common.math_utils import Polygon2d
from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


class ObjectOverlapConditionHandler(ConditionHandlerBase):
    """物体重叠条件处理器 - 碰撞检测"""

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_object_overlap(
            condition.object_overlap_condition,
            world_list[-1], detailed_result)

    def _eval_object_overlap(self, cond, world, detailed_result) -> bool:
        """
        检测碰撞

        Returns:
            True 表示发生碰撞（metric 不通过）
            False 表示未碰撞（metric 通过）
        """
        source_object_ids = getattr(cond, 'source_object_ids', '')
        target_object_ids = getattr(cond, 'target_object_ids', '*')
        distance_threshold = getattr(cond, 'distance', 0.05)
        direction = getattr(cond, 'direction', 'INCLUDE_BACK')
        ignore_ids = getattr(cond, 'ignore_object_ids', [])
        target_types = [str(t).upper() for t in getattr(cond, 'target_types', [])]

        # 获取源对象多边形（通常是主车）
        source_polygons = ConditionEvaluatorUtil.get_object_polygons(world, source_object_ids)

        # 获取目标对象（所有障碍物）的多边形和ID
        adc = world.auto_driving_car
        target_id_polygons = []
        back_id_set = set()

        for obj in world.object:
            obj_id = obj.id
            # 检查是否需要忽略
            if obj_id in ignore_ids:
                continue
            if target_types and str(getattr(obj, 'type', '')).upper() not in target_types:
                continue

            # 检查是否在主车后方
            if direction == 'EXCLUDE_BACK':
                if ConditionEvaluatorUtil.is_behind(adc, obj):
                    back_id_set.add(obj_id)

            obj_polygon = ConditionEvaluatorUtil.get_object_polygon(obj)
            target_id_polygons.append((obj_id, obj_polygon))

        # 检测碰撞
        for src_poly in source_polygons:
            for tgt_id, tgt_poly in target_id_polygons:
                dist = self._polygon_distance(src_poly, tgt_poly)
                if dist <= distance_threshold:
                    if direction == 'INCLUDE_BACK':
                        # 包含所有方向碰撞
                        detailed_result.add_failure(
                            f"collision_{tgt_id}",
                            f"Collision with {tgt_id}, distance={dist:.3f}. ")
                        if getattr(cond, 'use_score', False):
                            detailed_result.delta_score = 100
                        return True
                    else:
                        # 排除后方碰撞
                        if tgt_id not in back_id_set:
                            detailed_result.add_failure(
                                f"collision_{tgt_id}",
                                f"Collision with {tgt_id}, distance={dist:.3f}. ")
                            if getattr(cond, 'use_score', False):
                                detailed_result.delta_score = 100
                            return True

        return False

    @staticmethod
    def _polygon_distance(poly1: Polygon2d, poly2: Polygon2d) -> float:
        """计算两个多边形的最短距离"""
        # 检查是否有重叠
        if poly1.has_overlap(poly2):
            return 0.0

        min_dist = float('inf')
        # poly1 的点到 poly2 的距离
        for point in poly1.points:
            dist = poly2.distance_to(point)
            if dist < min_dist:
                min_dist = dist
        # poly2 的点到 poly1 的距离
        for point in poly2.points:
            dist = poly1.distance_to(point)
            if dist < min_dist:
                min_dist = dist
        return min_dist
