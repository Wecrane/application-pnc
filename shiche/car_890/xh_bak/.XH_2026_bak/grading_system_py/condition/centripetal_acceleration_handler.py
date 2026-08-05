"""
向心加速度条件评估器
对应原 C++ centripetal_acceleration_condition_handler.cc

评估逻辑（扣分制）:
- 向心加速度 = speed * spin (角速度)
- 超过 max_centripetal_acceleration 则不通过
"""

from collections import deque

from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


class CentripetalAccelerationConditionHandler(ConditionHandlerBase):
    """向心加速度条件处理器"""

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_centripetal_acceleration(
            condition.centripetal_acceleration_condition,
            world_list[-1], detailed_result)

    def _eval_centripetal_acceleration(self, cond, world, detailed_result) -> bool:
        adc = world.auto_driving_car

        speed = getattr(adc, 'speed', None)
        spin = getattr(adc, 'spin', None)
        if speed is None or spin is None:
            return True

        centripetal_acceleration = abs(speed * spin)

        passed = centripetal_acceleration <= cond.max_centripetal_acceleration
        if not passed:
            detailed_result.add_description(
                f"CentripetalAcceleration is {centripetal_acceleration:.3f}. ")
            if getattr(cond, 'use_score', False):
                delta_score = ConditionEvaluatorUtil.score_deducted_by_unit(
                    centripetal_acceleration - cond.max_centripetal_acceleration,
                    cond.deduction_unit,
                    cond.single_deduction)
                detailed_result.delta_score = delta_score

        return passed
