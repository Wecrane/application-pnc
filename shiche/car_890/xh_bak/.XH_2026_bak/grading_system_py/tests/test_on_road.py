"""在路条件评估器单元测试"""

import sys
from collections import deque

sys.path.insert(0, '/apollo_workspace')

from grading_system_py.common.map_interface import (
    MapInterface, LaneInfo, LaneId, set_map_interface
)
from grading_system_py.condition.on_road_handler import OnRoadConditionHandler
from grading_system_py.tests.mock_data import (
    MockObject, MockGradingWorld, MockDetailedResult,
    MockOnRoadCondition, MockCondition
)


class TestMapInterface(MapInterface):
    """测试用地图实现 - 模拟一条直线车道"""

    def __init__(self, lane_width=3.5, road_width=7.0):
        self._lane_width = lane_width
        self._road_width = road_width

    def get_lanes(self, x, y, radius):
        # 模拟一条沿X轴方向的车道, Y在[-lane_width/2, lane_width/2]范围内
        lane = LaneInfo(
            lane_id=LaneId(id="lane_1"),
            total_length=1000.0,
            turn_type="NO_TURN",
        )
        # override get_projection to work with our mock
        lane.get_projection = lambda px, py: (True, px, py)
        lane.get_width = lambda s: (self._lane_width / 2, self._lane_width / 2)
        lane.get_road_width = lambda s: (self._road_width / 2, self._road_width / 2)
        return [lane]

    def get_lane_by_id(self, lane_id):
        return None

    def get_forward_nearest_signals(self, x, y, distance):
        return []

    def get_exact_lanes(self, x, y, radius):
        return self.get_lanes(x, y, radius)


class OffRoadMapInterface(MapInterface):
    """模拟车辆不在任何车道上"""

    def get_lanes(self, x, y, radius):
        # 返回一个车道，但投影表明车辆在车道外面
        lane = LaneInfo(
            lane_id=LaneId(id="lane_1"),
            total_length=1000.0,
            turn_type="NO_TURN",
        )
        # 车辆在Y=20的位置，远超车道宽度
        lane.get_projection = lambda px, py: (True, px, py)
        lane.get_width = lambda s: (1.75, 1.75)
        lane.get_road_width = lambda s: (3.5, 3.5)
        return [lane]

    def get_lane_by_id(self, lane_id):
        return None

    def get_forward_nearest_signals(self, x, y, distance):
        return []

    def get_exact_lanes(self, x, y, radius):
        return self.get_lanes(x, y, radius)


def test_on_road_pass():
    """车辆在路上 - 应该通过"""
    set_map_interface(TestMapInterface(lane_width=4.0))
    handler = OnRoadConditionHandler()

    adc = MockObject(position_x=50.0, position_y=0.5, heading=0.0,
                     length=4.5, width=2.0, front_edge_to_center=2.25,
                     back_edge_to_center=2.25)
    # 需要两帧才能触发检测
    world1 = MockGradingWorld(timestamp_sec=0.0, auto_driving_car=adc)
    world2 = MockGradingWorld(timestamp_sec=0.1, auto_driving_car=adc)
    world_list = deque([world1, world2])

    condition = MockCondition(on_road_condition=MockOnRoadCondition())
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert passed, f"Expected on-road pass, got: {result.description}"


def test_on_road_fail():
    """车辆偏出路面 - 应该不通过"""
    set_map_interface(OffRoadMapInterface())
    handler = OnRoadConditionHandler()

    # 车辆在Y=20处，远离车道
    adc = MockObject(position_x=50.0, position_y=20.0, heading=0.0,
                     length=4.5, width=2.0, front_edge_to_center=2.25,
                     back_edge_to_center=2.25)
    world1 = MockGradingWorld(timestamp_sec=0.0, auto_driving_car=adc)
    world2 = MockGradingWorld(timestamp_sec=0.1, auto_driving_car=adc)
    world_list = deque([world1, world2])

    condition = MockCondition(on_road_condition=MockOnRoadCondition())
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert not passed, "Expected off-road fail"
    assert "Off-road" in result.description


def test_on_road_single_frame_skip():
    """只有一帧时跳过检测"""
    set_map_interface(OffRoadMapInterface())
    handler = OnRoadConditionHandler()

    adc = MockObject(position_x=50.0, position_y=20.0, heading=0.0)
    world = MockGradingWorld(timestamp_sec=0.0, auto_driving_car=adc)
    world_list = deque([world])

    condition = MockCondition(on_road_condition=MockOnRoadCondition())
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert passed, "Should pass on single frame"


def test_on_road_with_score():
    """偏出路面时扣分"""
    set_map_interface(OffRoadMapInterface())
    handler = OnRoadConditionHandler()

    adc = MockObject(position_x=50.0, position_y=20.0, heading=0.0,
                     length=4.5, width=2.0, front_edge_to_center=2.25,
                     back_edge_to_center=2.25)
    world1 = MockGradingWorld(timestamp_sec=0.0, auto_driving_car=adc)
    world2 = MockGradingWorld(timestamp_sec=0.1, auto_driving_car=adc)
    world_list = deque([world1, world2])

    condition = MockCondition(
        on_road_condition=MockOnRoadCondition(use_score=True))
    result = MockDetailedResult()

    passed = handler.evaluate(condition, world_list, deque(), result)
    assert not passed
    assert result.delta_score == 100


if __name__ == "__main__":
    test_on_road_pass()
    test_on_road_fail()
    test_on_road_single_frame_skip()
    test_on_road_with_score()
    print("All on_road tests passed!")
