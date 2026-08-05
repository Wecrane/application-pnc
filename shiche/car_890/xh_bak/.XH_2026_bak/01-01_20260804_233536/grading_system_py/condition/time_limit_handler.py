"""
时间限制条件评估器
对应原 C++ time_limit_condition_handler.cc

评估逻辑:
- 比较当前时间与第一帧时间的差值是否超过 timeout
- 未超时: delta_score=100 (得分)
- 超时: delta_score=0
- 注: 该 metric 的 get_deduction_score=false，delta_score 代表得到的分数而非扣分
"""

from collections import deque

from grading_system_py.condition.condition_handler_base import ConditionHandlerBase


class TimeLimitConditionHandler(ConditionHandlerBase):
    """时间限制条件处理器"""

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_time_limit(condition.time_limit_condition,
                                     world_list, detailed_result)

    def _eval_time_limit(self, cond, world_list: deque, detailed_result) -> bool:
        first_world = world_list[0]
        current_world = world_list[-1]

        timeout = cond.timeout
        duration = current_world.timestamp_sec - first_world.timestamp_sec

        passed = duration < timeout
        if not passed:
            detailed_result.delta_score = 0
            detailed_result.add_description(
                f"Time limit exceed, limit: {timeout}, current: {duration:.2f}")
        else:
            detailed_result.delta_score = 100
            detailed_result.add_description(
                f"Time limit not exceed, limit: {timeout}, current: {duration:.2f}")

        return passed
