"""
条件评估器基类
对应原 C++ condition_handler_base.h
"""

from abc import ABC, abstractmethod
from collections import deque
from typing import Any


class ConditionHandlerBase(ABC):
    """条件处理器抽象基类"""

    @abstractmethod
    def evaluate(self, condition: Any, world_list: deque,
                 future_list: deque, detailed_result: Any) -> bool:
        """
        评估条件是否满足

        Args:
            condition: 条件配置（protobuf Condition 或等价的dict）
            world_list: 历史世界状态队列
            future_list: 未来世界状态队列
            detailed_result: 详细结果输出对象

        Returns:
            bool: 是否通过
        """
        ...
