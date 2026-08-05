"""
测试用数据模型
模拟 Protobuf 消息的 Python 对象，用于单元测试
"""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class PolygonPoint:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class MockObject:
    """模拟 Object protobuf 消息"""
    id: str = ""
    position_x: float = 0.0
    position_y: float = 0.0
    heading: float = 0.0
    length: float = 4.5
    width: float = 2.0
    height: float = 1.8
    type: str = "UNKNOWN"
    speed: float = 0.0
    speed_acceleration: float = 0.0
    speed_heading: float = 0.0
    front_edge_to_center: float = 2.25
    back_edge_to_center: float = 2.25
    polygon_point: List[PolygonPoint] = field(default_factory=list)

    def has_speed(self) -> bool:
        return True

    def has_speed_acceleration(self) -> bool:
        return True


@dataclass
class MockSignal:
    """模拟信号灯"""
    id: str = ""
    current_signal: str = "GREEN"


@dataclass
class MockGradingWorld:
    """模拟 GradingWorld protobuf 消息"""
    timestamp_sec: float = 0.0
    auto_driving_car: MockObject = field(default_factory=MockObject)
    object: List[MockObject] = field(default_factory=list)
    speed_limit: Optional[float] = None
    last_frame: bool = False
    perceived_signal: List[MockSignal] = field(default_factory=list)


@dataclass
class MockDetailedResult:
    """模拟 DetailedResult protobuf 消息"""
    description: str = ""
    delta_score: float = 0.0
    is_pass: bool = True
    failure_code: str = ""

    def add_description(self, text: str):
        self.description += text

    def add_failure(self, code: str, text: str):
        if not self.failure_code:
            self.failure_code = code
        self.add_description(text)


# --- Condition 配置模拟 ---

@dataclass
class MockAccelerationCondition:
    name: str = ""
    min_acceleration: float = -5.0
    max_acceleration: float = 3.0
    cut_in_distance: float = 50.0
    ttc: float = 2.0
    duration: float = 1.0
    dist_stopline: float = 5.0
    light_turn_prev_time: float = 5.0
    use_score: bool = False
    deduction_unit: float = 1.0
    single_deduction: float = 5.0


@dataclass
class MockSpeedCondition:
    name: str = ""
    min_speed: float = 0.0
    max_speed: float = 20.0
    use_score: bool = False
    deduction_unit: float = 1.0
    single_deduction: float = 5.0
    speed_limit_regions: list = field(default_factory=list)


@dataclass
class MockSpeedLimitRegion:
    limit_min_speed: float = 0.0
    limit_max_speed: float = 10.0
    limit_region: "MockPolygon" = None


@dataclass
class MockPolygon:
    """模拟 hdmap.Polygon"""
    point: List[PolygonPoint] = field(default_factory=list)


@dataclass
class MockOnRoadCondition:
    use_road_boundary: bool = False
    use_score: bool = False


@dataclass
class MockObstacleBypassCondition:
    obstacle_id: str = ""
    min_lateral_distance: float = 1.0
    max_speed: float = 5.0
    test_range: "MockPolygon" = None
    use_score: bool = False
    single_deduction: float = 10.0


@dataclass
class MockTtcCondition:
    time: float = 3.0


# --- Condition wrapper (模拟 oneof) ---

@dataclass
class MockCondition:
    acceleration_condition: Optional[MockAccelerationCondition] = None
    speed_condition: Optional[MockSpeedCondition] = None
    on_road_condition: Optional[MockOnRoadCondition] = None
    obstacle_bypass_condition: Optional[MockObstacleBypassCondition] = None
    ttc_condition: Optional[MockTtcCondition] = None
