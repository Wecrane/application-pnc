"""障碍物绕行条件评估器单元测试"""

import sys
from collections import deque

sys.path.insert(0, '/apollo_workspace')

from grading_system_py.condition.obstacle_bypass_handler import ObstacleBypassConditionHandler
from grading_system_py.tests.mock_data import (
    MockObject, MockGradingWorld, MockDetailedResult,
    MockObstacleBypassCondition, MockCondition, MockPolygon, PolygonPoint
)


def _make_test_range():
    """创建一个覆盖 x:[-20,80], y:[-20,20] 的测试范围"""
    return MockPolygon(point=[
        PolygonPoint(x=-20, y=-20),
        PolygonPoint(x=80, y=-20),
        PolygonPoint(x=80, y=20),
        PolygonPoint(x=-20, y=20),
    ])


def test_bypass_not_in_test_range():
    """主车不在测试范围内 - 应该通过"""
    handler = ObstacleBypassConditionHandler()

    adc = MockObject(position_x=100, position_y=0, heading=0,
                     length=4.5, width=2.0, speed=5.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    obstacle = MockObject(id="obs1", position_x=50, position_y=3, heading=0,
                          length=4.5, width=2.0,
                          front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc, object=[obstacle])

    # 测试范围: x:[-20,80], y:[-20,20] — 主车在 x=100，不在范围内
    condition = MockCondition(
        obstacle_bypass_condition=MockObstacleBypassCondition(
            obstacle_id="obs1",
            min_lateral_distance=1.0,
            max_speed=5.0,
            test_range=_make_test_range()))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, deque([world]), deque(), result)
    assert passed, f"Expected pass (not in range), got: {result.description}"


def test_bypass_passing_obstacle():
    """主车正在绕行障碍物 - 横向距离足够"""
    handler = ObstacleBypassConditionHandler()

    # 障碍物在 x=50, y=0, 主车在 x=50, y=5 (横向距离足够)
    adc = MockObject(position_x=50, position_y=5, heading=0,
                     length=4.5, width=2.0, speed=4.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    obstacle = MockObject(id="obs1", position_x=50, position_y=0, heading=0,
                          length=4.5, width=2.0,
                          front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc, object=[obstacle])

    condition = MockCondition(
        obstacle_bypass_condition=MockObstacleBypassCondition(
            obstacle_id="obs1",
            min_lateral_distance=1.0,
            max_speed=5.0,
            test_range=_make_test_range()))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, deque([world]), deque(), result)
    assert passed, f"Expected pass (good bypass), got: {result.description}"


def test_bypass_too_close():
    """主车绕行但横向距离不足"""
    handler = ObstacleBypassConditionHandler()

    # 主车和障碍物Y方向非常近
    adc = MockObject(position_x=50, position_y=1.5, heading=0,
                     length=4.5, width=2.0, speed=4.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    obstacle = MockObject(id="obs1", position_x=50, position_y=0, heading=0,
                          length=4.5, width=2.0,
                          front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc, object=[obstacle])

    condition = MockCondition(
        obstacle_bypass_condition=MockObstacleBypassCondition(
            obstacle_id="obs1",
            min_lateral_distance=1.0,
            max_speed=5.0,
            test_range=_make_test_range()))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, deque([world]), deque(), result)
    assert not passed, "Expected fail (too close to obstacle)"
    assert "lateral distance" in result.description


def test_bypass_speed_too_high():
    """主车绕行但速度过高"""
    handler = ObstacleBypassConditionHandler()

    adc = MockObject(position_x=50, position_y=5, heading=0,
                     length=4.5, width=2.0, speed=8.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    obstacle = MockObject(id="obs1", position_x=50, position_y=0, heading=0,
                          length=4.5, width=2.0,
                          front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc, object=[obstacle])

    condition = MockCondition(
        obstacle_bypass_condition=MockObstacleBypassCondition(
            obstacle_id="obs1",
            min_lateral_distance=1.0,
            max_speed=5.0,
            test_range=_make_test_range()))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, deque([world]), deque(), result)
    assert not passed, "Expected fail (speed too high)"
    assert "max speed" in result.description


def test_bypass_last_frame_no_bypass():
    """最后一帧未绕行 - 扣100分"""
    handler = ObstacleBypassConditionHandler()

    adc = MockObject(position_x=10, position_y=0, heading=0,
                     length=4.5, width=2.0, speed=3.0,
                     front_edge_to_center=2.25, back_edge_to_center=2.25)
    obstacle = MockObject(id="obs1", position_x=50, position_y=0, heading=0,
                          length=4.5, width=2.0,
                          front_edge_to_center=2.25, back_edge_to_center=2.25)
    world = MockGradingWorld(auto_driving_car=adc, object=[obstacle], last_frame=True)

    condition = MockCondition(
        obstacle_bypass_condition=MockObstacleBypassCondition(
            obstacle_id="obs1",
            min_lateral_distance=1.0,
            max_speed=5.0,
            test_range=_make_test_range()))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, deque([world]), deque(), result)
    assert not passed, "Expected fail on last frame without bypass"
    assert result.delta_score == 100


if __name__ == "__main__":
    test_bypass_not_in_test_range()
    test_bypass_passing_obstacle()
    test_bypass_too_close()
    test_bypass_speed_too_high()
    test_bypass_last_frame_no_bypass()
    print("All obstacle_bypass tests passed!")
