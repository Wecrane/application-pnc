"""
障碍物绕行条件评估器
对应原 C++ obstacle_bypass_condition_handler.cc

评估逻辑（扣分制）:
- 检测主车是否进入测试范围
- 检测是否绕过了指定障碍物（通过X投影）
- 绕行时检查横向距离和速度
- 最后一帧未绕行直接扣100分
"""

from collections import deque
from typing import Dict, Any

from grading_system_py.common.math_utils import Vec2d, Polygon2d
from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


class ObstacleBypassConditionHandler(ConditionHandlerBase):
    """障碍物绕行条件处理器"""

    def __init__(self):
        self._triggered = False
        self._end = False
        self._bypass = False
        self._inspection_passed: Dict[str, bool] = {
            "lateral_distance": True,
            "speed": True,
        }

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_obstacle_bypass(condition.obstacle_bypass_condition,
                                          world_list[-1], detailed_result)

    def _eval_obstacle_bypass(self, bypass_cond, world, detailed_result) -> bool:
        """
        障碍物绕行评估主逻辑

        Args:
            bypass_cond: ObstacleBypassCondition 配置
            world: 当前世界状态
            detailed_result: 结果输出
        """
        if self._end:
            # 已结束检测，避免多余扣分
            return True

        # 最后一帧判断
        if getattr(world, 'last_frame', False):
            detailed_result.add_description("last frame!!!")
            if not self._bypass:
                detailed_result.add_description(
                    "It is detected that the obstacle has never been passed, "
                    "and the direct deduction is 0 points.")
                detailed_result.delta_score = 100
                return False

        # 获取测试范围多边形
        test_range = ConditionEvaluatorUtil.get_object_polygon_from_points(
            bypass_cond.test_range)

        adc = world.auto_driving_car
        car_polygon = ConditionEvaluatorUtil.get_object_polygon(adc, False)

        # 检测是否进入测试范围
        enter_test_range = test_range.has_overlap(car_polygon)
        if not enter_test_range:
            if self._triggered:
                self._end = True  # 离开测试区域
            return True

        if not self._triggered:
            self._triggered = True

        # 搜索目标障碍物
        for obj in world.object:
            if obj.id == bypass_cond.obstacle_id:
                obj_polygon = ConditionEvaluatorUtil.get_object_polygon(obj, True)

                adc_x = adc.position_x

                # 判断主车X坐标是否在障碍物X范围内（投影判断）
                if obj_polygon.min_x <= adc_x <= obj_polygon.max_x:
                    self._bypass = True

                    # 检测横向距离
                    distance = ConditionEvaluatorUtil.get_lateral_distance(
                        obj_polygon, adc.position_x, adc.position_y, adc.width / 2)
                    is_passed = True

                    if distance < bypass_cond.min_lateral_distance:
                        detailed_result.add_description(
                            f"Expected lateral distance to this obstacle: "
                            f"{obj.id} should more than 1. but now is {distance}")
                        is_passed = False
                        self._inspection_passed["lateral_distance"] = False

                    # 检测速度
                    adc_speed = adc.speed
                    if adc_speed > bypass_cond.max_speed:
                        detailed_result.add_description(
                            f"Expected max speed: {bypass_cond.max_speed}. "
                            f"but now is {adc_speed}")
                        is_passed = False
                        self._inspection_passed["speed"] = False

                    return is_passed
                else:
                    # 主车已经通过障碍物区域
                    if self._bypass:
                        self._end = True
                        delta_score = 0.0
                        all_passed = True
                        for key, value in self._inspection_passed.items():
                            if not value:
                                all_passed = False
                                if getattr(bypass_cond, 'use_score', False):
                                    delta_score += bypass_cond.single_deduction
                        if getattr(bypass_cond, 'use_score', False):
                            detailed_result.delta_score = delta_score
                        return all_passed
                    return True

        return True
