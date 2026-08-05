"""
碰撞检测（TTC）条件评估器
对应原 C++ ttc_condition_handler.cc

评估逻辑:
- 计算主车与所有障碍物的 Time-To-Collision
- 任一障碍物 TTC < 阈值则判定为碰撞风险（不通过）
"""

from collections import deque
from typing import Any

from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


class TtcConditionHandler(ConditionHandlerBase):
    """碰撞检测条件处理器（基于TTC）"""

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_ttc(condition.ttc_condition,
                              world_list[-1], detailed_result)

    def _eval_ttc(self, ttc_cond, world, detailed_result) -> bool:
        """
        TTC评估主逻辑

        Args:
            ttc_cond: TtcCondition 配置 (包含 time 阈值)
            world: 当前世界状态
            detailed_result: 结果输出
        """
        adc = world.auto_driving_car

        for obj in world.object:
            ttc_value = ConditionEvaluatorUtil.ttc(adc, obj)
            if ttc_value < ttc_cond.time:
                detailed_result.add_description(
                    f"TTC to {obj.id} is less than {ttc_cond.time} s.")
                return False

        return True
