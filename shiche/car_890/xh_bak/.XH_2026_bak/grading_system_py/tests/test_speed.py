"""速度条件评估器单元测试"""

import sys
from collections import deque

sys.path.insert(0, '/apollo_workspace')

from grading_system_py.condition.speed_handler import SpeedConditionHandler
from grading_system_py.tests.mock_data import (
    MockObject, MockGradingWorld, MockDetailedResult,
    MockSpeedCondition, MockCondition, MockSpeedLimitRegion,
    MockPolygon, PolygonPoint
)


def test_speed_within_range():
    """速度在正常范围内 - 应该通过"""
    handler = SpeedConditionHandler()

    adc = MockObject(speed=10.0)
    world = MockGradingWorld(auto_driving_car=adc)
    world_list = deque([world])

    condition = MockCondition(
        speed_condition=MockSpeedCondition(min_speed=0.0, max_speed=20.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert passed, f"Expected pass, got: {result.description}"


def test_speed_exceeds_max():
    """速度超过上限 - 应该不通过"""
    handler = SpeedConditionHandler()

    adc = MockObject(speed=25.0)
    world = MockGradingWorld(auto_driving_car=adc)
    world_list = deque([world])

    condition = MockCondition(
        speed_condition=MockSpeedCondition(min_speed=0.0, max_speed=20.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert not passed, "Expected fail for excessive speed"
    assert "ego speed" in result.description
    assert "exceeds limit" in result.description
    assert result.failure_code == "overspeed"


def test_speed_below_min():
    """速度低于下限 - 应该不通过"""
    handler = SpeedConditionHandler()

    adc = MockObject(speed=1.0)
    world = MockGradingWorld(auto_driving_car=adc)
    world_list = deque([world])

    condition = MockCondition(
        speed_condition=MockSpeedCondition(min_speed=5.0, max_speed=20.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert not passed, "Expected fail for low speed"


def test_speed_limit_from_map():
    """speed_limit模式 - 使用地图限速"""
    handler = SpeedConditionHandler()

    adc = MockObject(speed=12.0)
    world = MockGradingWorld(auto_driving_car=adc, speed_limit=10.0)
    world_list = deque([world])

    condition = MockCondition(
        speed_condition=MockSpeedCondition(
            name="speed_limit", min_speed=0.0, max_speed=20.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    # speed=12 > min(map_limit=10, config_max=20) = 10, so fail
    assert not passed, "Expected fail: speed exceeds map limit"


def test_speed_with_score_deduction():
    """超速时扣分计算"""
    handler = SpeedConditionHandler()

    adc = MockObject(speed=23.0)
    world = MockGradingWorld(auto_driving_car=adc)
    world_list = deque([world])

    condition = MockCondition(
        speed_condition=MockSpeedCondition(
            min_speed=0.0, max_speed=20.0,
            use_score=True, deduction_unit=1.0, single_deduction=5.0))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert not passed
    assert result.delta_score == 15.0  # delta=3, ceil(3/1)*5 = 15


def test_speed_limit_region():
    """限速区域检测"""
    handler = SpeedConditionHandler()

    # 车在 (0,0), 尺寸 4.5x2
    adc = MockObject(position_x=0, position_y=0, heading=0,
                     length=4.5, width=2.0, speed=8.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc)
    world_list = deque([world])

    # 限速区域覆盖主车位置
    region_polygon = MockPolygon(point=[
        PolygonPoint(x=-10, y=-10),
        PolygonPoint(x=10, y=-10),
        PolygonPoint(x=10, y=10),
        PolygonPoint(x=-10, y=10),
    ])
    region = MockSpeedLimitRegion(
        limit_min_speed=0.0, limit_max_speed=5.0, limit_region=region_polygon)

    condition = MockCondition(
        speed_condition=MockSpeedCondition(
            min_speed=0.0, max_speed=20.0, speed_limit_regions=[region]))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    # speed=8 > region_max=5, should fail
    assert not passed, f"Expected fail in speed limit region, got: {result.description}"


if __name__ == "__main__":
    test_speed_within_range()
    test_speed_exceeds_max()
    test_speed_below_min()
    test_speed_limit_from_map()
    test_speed_with_score_deduction()
    test_speed_limit_region()
    print("All speed tests passed!")
