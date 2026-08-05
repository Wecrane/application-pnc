"""
速度条件评估器
对应原 C++ speed_condition_handler.cc

评估逻辑（扣分制）:
- 检查速度是否在 [min_speed, max_speed] 范围内
- speed_limit 模式: 取配置限速和地图限速的最小值
- 限速区域: 车辆包围盒进入限速多边形区域时，使用该区域的限速
"""

from collections import deque
from typing import Any

from grading_system_py.common.math_utils import Vec2d, Polygon2d
from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


def _polygon_from_dict(region_dict: dict) -> Polygon2d:
    """从 JSON dict 格式的 limit_region 构建多边形"""
    points = []
    for pt in region_dict.get('point', []):
        points.append(Vec2d(pt.get('x', 0), pt.get('y', 0)))
    if len(points) < 3:
        # 返回一个不可能重叠的小多边形
        return Polygon2d([Vec2d(0, 0), Vec2d(0, 0.001), Vec2d(0.001, 0)])
    return Polygon2d(points)


class SpeedConditionHandler(ConditionHandlerBase):
    """速度条件处理器"""

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_speed(condition.speed_condition,
                                world_list[-1], detailed_result)

    def _eval_speed(self, speed_cond, world, detailed_result) -> bool:
        """
        速度评估主逻辑

        Args:
            speed_cond: SpeedCondition 配置
            world: 当前世界状态
            detailed_result: 结果输出
        """
        adc = world.auto_driving_car
        if not hasattr(adc, 'speed') or adc.speed is None:
            return True

        speed = adc.speed
        max_speed = speed_cond.max_speed
        min_speed = speed_cond.min_speed
        passed = True

        if speed_cond.name == "speed_limit":
            # 使用地图限速和配置限速中较小的值
            if hasattr(world, 'speed_limit') and world.speed_limit is not None:
                max_speed = min(world.speed_limit, max_speed)

        passed = speed >= min_speed and speed <= max_speed

        # 限速区域检测
        speed_limit_regions = getattr(speed_cond, 'speed_limit_regions', [])
        if speed_limit_regions:
            car_polygon = ConditionEvaluatorUtil.get_object_polygon(adc, False)
            for region in speed_limit_regions:
                # 兼容 dict 格式(JSON) 和对象格式
                if isinstance(region, dict):
                    region_polygon = _polygon_from_dict(region.get('limit_region', {}))
                    r_min_speed = region.get('limit_min_speed', 0)
                    r_max_speed = region.get('limit_max_speed', 1000)
                else:
                    region_polygon = ConditionEvaluatorUtil.get_object_polygon_from_points(
                        region.limit_region)
                    r_min_speed = region.limit_min_speed
                    r_max_speed = region.limit_max_speed

                is_overlap = region_polygon.has_overlap(car_polygon)
                if is_overlap:
                    passed = passed and (speed >= r_min_speed and
                                         speed <= r_max_speed)
                    max_speed = min(max_speed, r_max_speed)
                    min_speed = max(min_speed, r_min_speed)
                    break

        if not passed:
            over = speed - max_speed
            under = min_speed - speed
            if over > 0:
                detailed_result.add_failure(
                    "overspeed",
                    f"Overspeed: ego speed {speed:.2f} m/s exceeds limit "
                    f"{max_speed:.2f} m/s by {over:.2f} m/s. ")
            else:
                detailed_result.add_failure(
                    "underspeed",
                    f"Underspeed: ego speed {speed:.2f} m/s below min "
                    f"{min_speed:.2f} m/s by {under:.2f} m/s. ")
            if getattr(speed_cond, 'use_score', False):
                delta_score = ConditionEvaluatorUtil.score_deducted_by_unit(
                    max(over, under),
                    speed_cond.deduction_unit,
                    speed_cond.single_deduction)
                detailed_result.delta_score = delta_score

        return passed
