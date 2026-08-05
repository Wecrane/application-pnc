"""加速度条件评估器单元测试"""

import sys
import math
from collections import deque

sys.path.insert(0, '/apollo_workspace')

from grading_system_py.condition.acceleration_handler import AccelerationConditionHandler
from grading_system_py.tests.mock_data import (
    MockObject, MockGradingWorld, MockDetailedResult,
    MockAccelerationCondition, MockCondition
)


def test_acceleration_within_range():
    """加速度在正常范围内 - 应该通过"""
    handler = AccelerationConditionHandler()

    adc = MockObject(position_x=0, position_y=0, heading=0,
                     speed=10, speed_acceleration=1.0, speed_heading=0)
    world = MockGradingWorld(timestamp_sec=1.0, auto_driving_car=adc)
    world_list = deque([world])

    condition = MockCondition(
        acceleration_condition=MockAccelerationCondition(
            name="acceleration", min_acceleration=-5.0, max_acceleration=3.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert passed, f"Expected pass, got fail: {result.description}"


def test_acceleration_exceeds_max():
    """加速度超过上限 - 应该不通过"""
    handler = AccelerationConditionHandler()

    adc = MockObject(position_x=0, position_y=0, heading=0,
                     speed=10, speed_acceleration=5.0, speed_heading=0)
    world = MockGradingWorld(timestamp_sec=1.0, auto_driving_car=adc)
    world_list = deque([world])

    condition = MockCondition(
        acceleration_condition=MockAccelerationCondition(
            name="acceleration", min_acceleration=-5.0, max_acceleration=3.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert not passed, "Expected fail for excessive acceleration"
    assert "Acceleration is" in result.description


def test_hard_brake_no_obstacle():
    """急刹车无障碍物 - 超过duration后应该不通过"""
    handler = AccelerationConditionHandler()

    # 第一帧：触发急刹
    adc1 = MockObject(position_x=0, position_y=0, heading=0,
                      speed=15, speed_acceleration=-6.0, speed_heading=0)
    world1 = MockGradingWorld(timestamp_sec=1.0, auto_driving_car=adc1)

    # 第二帧：持续急刹超过duration
    adc2 = MockObject(position_x=10, position_y=0, heading=0,
                      speed=10, speed_acceleration=-6.0, speed_heading=0)
    world2 = MockGradingWorld(timestamp_sec=3.0, auto_driving_car=adc2)

    condition = MockCondition(
        acceleration_condition=MockAccelerationCondition(
            name="HardBrake", min_acceleration=-5.0, max_acceleration=3.0,
            duration=1.0))

    # 第一帧评估
    result1 = MockDetailedResult()
    world_list = deque([world1])
    handler.evaluate(condition, world_list, deque(), result1)

    # 第二帧评估
    result2 = MockDetailedResult()
    world_list = deque([world1, world2])
    passed = handler.evaluate(condition, world_list, deque(), result2)
    assert not passed, f"Expected fail for prolonged hard brake, got: {result2.description}"


def test_hard_brake_with_low_ttc():
    """急刹车但TTC很小 - 应该通过（合理刹车）"""
    handler = AccelerationConditionHandler()

    adc = MockObject(id="adc", position_x=0, position_y=0, heading=0,
                     speed=15, speed_acceleration=-6.0, speed_heading=0,
                     length=4.5, width=2.0, front_edge_to_center=2.25,
                     back_edge_to_center=2.25)
    # 障碍物在正前方很近
    obstacle = MockObject(id="obs1", position_x=5, position_y=0, heading=0,
                          speed=2, speed_heading=0,
                          length=4.5, width=2.0, front_edge_to_center=2.25,
                          back_edge_to_center=2.25)
    world = MockGradingWorld(timestamp_sec=1.0, auto_driving_car=adc,
                            object=[obstacle])
    world_list = deque([world])

    condition = MockCondition(
        acceleration_condition=MockAccelerationCondition(
            name="HardBrake", min_acceleration=-5.0, max_acceleration=3.0,
            ttc=5.0, duration=1.0, cut_in_distance=50.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert passed, f"Expected pass (low TTC), got: {result.description}"


def test_acceleration_with_score_deduction():
    """加速度超限时扣分计算"""
    handler = AccelerationConditionHandler()

    adc = MockObject(position_x=0, position_y=0, heading=0,
                     speed=10, speed_acceleration=-7.0, speed_heading=0)
    world = MockGradingWorld(timestamp_sec=1.0, auto_driving_car=adc)
    world_list = deque([world])

    condition = MockCondition(
        acceleration_condition=MockAccelerationCondition(
            name="acceleration", min_acceleration=-5.0, max_acceleration=3.0,
            use_score=True, deduction_unit=1.0, single_deduction=5.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert not passed
    assert result.delta_score == 10.0  # delta = 2.0, ceil(2/1) * 5 = 10


if __name__ == "__main__":
    test_acceleration_within_range()
    test_acceleration_exceeds_max()
    test_hard_brake_no_obstacle()
    test_hard_brake_with_low_ttc()
    test_acceleration_with_score_deduction()
    print("All acceleration tests passed!")
