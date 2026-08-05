"""
在路条件评估器
对应原 C++ on_road_condition_handler.cc

评估逻辑（扣分制）:
- 检查车辆中心点是否在路上
- 检查车辆四个角点是否在路上
- 转弯车道给予额外容差
"""

from collections import deque
from typing import Any

from grading_system_py.common.math_utils import Vec2d, Polygon2d
from grading_system_py.common.map_interface import get_map_interface, LaneInfo
from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


class OnRoadConditionHandler(ConditionHandlerBase):
    """在路条件处理器"""

    MAP_SEARCH_RADIUS = 10.0

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_on_road(condition.on_road_condition,
                                  world_list, detailed_result)

    def _eval_on_road(self, on_road_cond, world_list: deque,
                      detailed_result) -> bool:
        """
        在路评估主逻辑

        Args:
            on_road_cond: OnRoadCondition 配置
            world_list: 历史世界状态
            detailed_result: 结果输出
        """
        if len(world_list) == 1:
            return True

        world = world_list[-1]
        adc = world.auto_driving_car
        use_road_boundary = getattr(on_road_cond, 'use_road_boundary', False)

        # 检查车辆中心点是否在路上
        if not self._is_on_road(adc.position_x, adc.position_y,
                                self.MAP_SEARCH_RADIUS, use_road_boundary):
            detailed_result.add_description(
                f"Off-road adc ({adc.position_x}, {adc.position_y}). ")
            if getattr(on_road_cond, 'use_score', False):
                detailed_result.delta_score = 100
            return False

        # 检查车辆四个角点是否在路上
        car_polygon = ConditionEvaluatorUtil.get_object_polygon(adc, False)
        for point in car_polygon.points:
            if not self._is_on_road(point.x, point.y,
                                    self.MAP_SEARCH_RADIUS, use_road_boundary):
                detailed_result.add_description(
                    f"Off-road corner ({point.x:.4f}, {point.y:.4f}). ")
                if getattr(on_road_cond, 'use_score', False):
                    detailed_result.delta_score = 100
                return False

        return True

    def _is_on_road(self, x: float, y: float, radius: float,
                    use_road_boundary: bool) -> bool:
        """判断点是否在路上"""
        map_intf = get_map_interface()
        lanes = map_intf.get_lanes(x, y, radius)

        if not lanes:
            # 无法获取车道时默认通过
            return True

        for lane in lanes:
            if self._is_on_lane(Vec2d(x, y), lane, use_road_boundary):
                return True

        return False

    def _is_on_lane(self, point: Vec2d, lane: LaneInfo,
                    use_road_boundary: bool) -> bool:
        """判断点是否在某车道内"""
        tolerance = 0.3

        success, accumulate_s, lateral = lane.get_projection(point.x, point.y)
        if not success:
            return False

        if accumulate_s > (lane.total_length + tolerance) or \
           (accumulate_s + tolerance) < 0.0:
            return False

        if use_road_boundary:
            left_width, right_width = lane.get_road_width(accumulate_s)
        else:
            left_width, right_width = lane.get_width(accumulate_s)

        if lateral < (left_width + tolerance) and \
           lateral > -(right_width + tolerance):
            return True

        # 转弯车道给额外容差
        if self._is_turning(lane, accumulate_s):
            return True

        return False

    def _is_turning(self, lane: LaneInfo, accumulate_s: float) -> bool:
        """判断是否在转弯区域"""
        if lane.turn_type != "NO_TURN":
            return True

        # 检查前后车道是否有转弯
        map_intf = get_map_interface()

        # 检查前驱车道
        for pred_id in lane.predecessor_ids:
            pred_lane = map_intf.get_lane_by_id(pred_id.id)
            if pred_lane and pred_lane.turn_type != "NO_TURN" and accumulate_s < 10.0:
                return True

        # 检查后继车道
        for succ_id in lane.successor_ids:
            succ_lane = map_intf.get_lane_by_id(succ_id.id)
            if succ_lane and succ_lane.turn_type != "NO_TURN" and \
               (lane.total_length - accumulate_s) < 10.0:
                return True

        return False
