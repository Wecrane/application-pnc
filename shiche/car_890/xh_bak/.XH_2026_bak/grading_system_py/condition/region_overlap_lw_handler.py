"""
区域重叠(LW)条件评估器 - 到达终点检测
对应原 C++ region_overlap_lw_condition_handler.cc

评估逻辑:
- 在目标点 (x, y) 处以主车朝向构建 length * width 的矩形区域
- 检测主车是否与该区域重叠（或完全包含在内）
- 一旦到达（return true），后续帧持续返回 true
- delta_score: 到达返回100，未到达返回0
"""

from collections import deque

from grading_system_py.common.math_utils import Vec2d, Box2d, Polygon2d
from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


class RegionOverlapLwConditionHandler(ConditionHandlerBase):
    """区域重叠(LW)条件处理器 - 到达终点"""

    def __init__(self):
        self._end = False

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_region_overlap_lw(
            condition.region_overlap_lw_condition,
            world_list[-1], detailed_result)

    def _eval_region_overlap_lw(self, cond, world, detailed_result) -> bool:
        if self._end:
            detailed_result.delta_score = 100
            return True

        # 获取目标对象多边形（默认为主车）
        object_ids = getattr(cond, 'object_ids', '')
        obj_polygons = ConditionEvaluatorUtil.get_object_polygons(world, object_ids)

        # 使用主车朝向
        heading = world.auto_driving_car.heading

        # 构建目标区域矩形
        x = getattr(cond, 'x', 0)
        y = getattr(cond, 'y', 0)
        length = getattr(cond, 'length', 5)
        width = getattr(cond, 'width', 12)

        region_polygon = Polygon2d(Box2d(Vec2d(x, y), heading, length, width))

        require_fully_contain = getattr(cond, 'require_fully_contain', False)

        for poly in obj_polygons:
            if require_fully_contain:
                overlap = region_polygon.contains(poly)
            else:
                overlap = region_polygon.has_overlap(poly)

            if overlap:
                self._end = True
                if getattr(cond, 'use_score', False):
                    detailed_result.delta_score = 100
                return True

        # 未到达
        detailed_result.delta_score = 0
        return False
