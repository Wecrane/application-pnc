"""碰撞检测（TTC）条件评估器单元测试"""

import sys
import math
from collections import deque

sys.path.insert(0, '/apollo_workspace')

from grading_system_py.condition.ttc_handler import TtcConditionHandler
from grading_system_py.tests.mock_data import (
    MockObject, MockGradingWorld, MockDetailedResult,
    MockTtcCondition, MockCondition
)


def test_ttc_safe():
    """TTC安全 - 应该通过"""
    handler = TtcConditionHandler()

    # 主车和障碍物相距较远，相对速度小
    adc = MockObject(id="adc", position_x=0, position_y=0, heading=0,
                     speed=10, length=4.5, width=2.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    obstacle = MockObject(id="obs1", position_x=100, position_y=0, heading=0,
                          speed=10, length=4.5, width=2.0,
                          front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc, object=[obstacle])

    condition = MockCondition(ttc_condition=MockTtcCondition(time=3.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, deque([world]), deque(), result)
    assert passed, f"Expected safe TTC, got: {result.description}"


def test_ttc_danger():
    """TTC危险 - 应该不通过"""
    handler = TtcConditionHandler()

    # 主车高速接近静止障碍物
    adc = MockObject(id="adc", position_x=0, position_y=0, heading=0,
                     speed=20, length=4.5, width=2.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    obstacle = MockObject(id="obs1", position_x=15, position_y=0, heading=0,
                          speed=0, length=4.5, width=2.0,
                          front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc, object=[obstacle])

    condition = MockCondition(ttc_condition=MockTtcCondition(time=3.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, deque([world]), deque(), result)
    assert not passed, "Expected TTC danger"
    assert "TTC to obs1" in result.description


def test_ttc_no_obstacles():
    """无障碍物 - 应该通过"""
    handler = TtcConditionHandler()

    adc = MockObject(id="adc", position_x=0, position_y=0, heading=0,
                     speed=20, length=4.5, width=2.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc, object=[])

    condition = MockCondition(ttc_condition=MockTtcCondition(time=3.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, deque([world]), deque(), result)
    assert passed, "Expected pass with no obstacles"


def test_ttc_objects_moving_away():
    """障碍物远离主车 - TTC为正无穷 - 应该通过"""
    handler = TtcConditionHandler()

    adc = MockObject(id="adc", position_x=0, position_y=0, heading=0,
                     speed=5, length=4.5, width=2.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    # 障碍物在前方且速度比主车快
    obstacle = MockObject(id="obs1", position_x=20, position_y=0, heading=0,
                          speed=15, length=4.5, width=2.0,
                          front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc, object=[obstacle])

    condition = MockCondition(ttc_condition=MockTtcCondition(time=3.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, deque([world]), deque(), result)
    assert passed, f"Expected pass (diverging), got: {result.description}"


def test_ttc_collision_occurred():
    """已经发生碰撞 (TTC=0)"""
    handler = TtcConditionHandler()

    # 主车和障碍物重叠
    adc = MockObject(id="adc", position_x=0, position_y=0, heading=0,
                     speed=10, length=4.5, width=2.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    obstacle = MockObject(id="obs1", position_x=1, position_y=0, heading=0,
                          speed=0, length=4.5, width=2.0,
                          front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc, object=[obstacle])

    condition = MockCondition(ttc_condition=MockTtcCondition(time=3.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, deque([world]), deque(), result)
    assert not passed, "Expected fail for collision"


if __name__ == "__main__":
    test_ttc_safe()
    test_ttc_danger()
    test_ttc_no_obstacles()
    test_ttc_objects_moving_away()
    test_ttc_collision_occurred()
    print("All TTC tests passed!")
