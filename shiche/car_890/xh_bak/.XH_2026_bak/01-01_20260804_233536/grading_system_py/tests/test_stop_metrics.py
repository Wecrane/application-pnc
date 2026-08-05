import unittest
from collections import deque

from grading_system_py.common.map_interface import (
    CrosswalkInfo,
    LaneInfo,
    MapInterface,
    SignalInfo,
    set_map_interface,
)
from grading_system_py.common.math_utils import Polygon2d, Vec2d
from grading_system_py.condition.crosswalk_stop_handler import (
    CrosswalkStopConditionHandler,
)
from grading_system_py.condition.object_overlap_handler import ObjectOverlapConditionHandler
from grading_system_py.condition.red_light_stop_handler import (
    RedLightStopConditionHandler,
)
from grading_system_py.main import (
    DetailedResult,
    _load_vehicle_param,
    parse_scene_manifest,
)
from grading_system_py.tests.mock_data import MockGradingWorld, MockObject, MockSignal


class StopCondition:
    min_distance = 1.0
    max_distance = 1.5
    stop_speed_eps = 0.1
    crossing_tolerance = 0.05
    use_score = False
    single_deduction = 20.0


class StopMap(MapInterface):
    def __init__(self, crosswalk=None):
        self.crosswalk = crosswalk

    def get_lanes(self, x, y, radius):
        return []

    def get_lane_by_id(self, lane_id):
        return None

    def get_forward_nearest_signals(self, x, y, distance):
        return [SignalInfo("S1", [[(3.25, -2.0), (3.25, 2.0)]])]

    def get_exact_lanes(self, x, y, radius):
        return []

    def find_crosswalk_for_polygon(self, x, y, radius, polygon):
        return self.crosswalk


def make_car(x=0.0, speed=0.0):
    car = MockObject(position_x=x, speed=speed)
    car.type = "VEHICLE"
    return car


def make_world(car, signal_color="RED", pedestrian=None):
    world = MockGradingWorld(auto_driving_car=car)
    world.perceived_signal = [MockSignal("S1", signal_color)]
    if pedestrian is not None:
        world.object.append(pedestrian)
    return world


class StopMetricTest(unittest.TestCase):
    def tearDown(self):
        set_map_interface(StopMap())

    def test_red_light_valid_stop_passes(self):
        set_map_interface(StopMap())
        handler = RedLightStopConditionHandler()
        result = DetailedResult()
        world = make_world(make_car(x=0.0, speed=0.0))

        passed = handler._eval_red_light_stop(StopCondition(), world, result)

        self.assertTrue(passed)
        self.assertEqual(result.failure_code, "")

    def test_red_light_stop_too_far_is_not_false_failed(self):
        # 修复后：车辆仍远在停止线前（signed 距离 > max_distance）时的低速帧
        # 属于接近/排队过程，不应被误判为 red_stopped_too_far。
        set_map_interface(StopMap())
        handler = RedLightStopConditionHandler()
        result = DetailedResult()
        world = make_world(make_car(x=-2.75, speed=0.0))

        passed = handler._eval_red_light_stop(StopCondition(), world, result)

        self.assertTrue(passed)
        self.assertNotEqual(result.failure_code, "red_stopped_too_far")

    def test_red_light_crossing_is_classified(self):
        set_map_interface(StopMap())
        handler = RedLightStopConditionHandler()
        first = DetailedResult()
        second = DetailedResult()
        before = make_world(make_car(x=0.0, speed=1.0))
        after = make_world(make_car(x=1.5, speed=1.0))

        self.assertTrue(
            handler._eval_red_light_stop(StopCondition(), before, first))
        self.assertFalse(
            handler._eval_red_light_stop(StopCondition(), after, second))
        self.assertIn(
            second.failure_code,
            {"red_overlap_stopline", "red_crossed_stopline"},
        )

    def test_crosswalk_moving_too_close_is_classified(self):
        crosswalk = CrosswalkInfo(
            "CW1",
            Polygon2d([
                Vec2d(4.0, -2.0),
                Vec2d(5.0, -2.0),
                Vec2d(5.0, 2.0),
                Vec2d(4.0, 2.0),
            ]),
        )
        set_map_interface(StopMap(crosswalk))
        handler = CrosswalkStopConditionHandler()
        result = DetailedResult()
        pedestrian = MockObject(position_x=4.5, position_y=0.0, speed=0.0)
        pedestrian.type = "PEDESTRIAN"
        world = make_world(
            make_car(x=1.4, speed=1.0),
            signal_color="GREEN",
            pedestrian=pedestrian,
        )

        passed = handler._eval_crosswalk_stop(StopCondition(), world, result)

        self.assertFalse(passed)
        self.assertEqual(result.failure_code, "crosswalk_moving_too_close")


class FailureEventTest(unittest.TestCase):
    def test_failure_frames_are_grouped_into_events(self):
        from grading_system_py.main import _build_failure_events

        results = []
        for timestamp in (1.0, 1.1, 1.2, 2.0):
            result = DetailedResult()
            result.timestamp = timestamp
            result.is_pass = False
            result.failure_code = "red_overlap_stopline"
            results.append(result)

        events = _build_failure_events(results)

        self.assertEqual(len(events), 2)
        self.assertEqual(events[0]["start"], 1.0)
        self.assertEqual(events[0]["end"], 1.2)
        self.assertEqual(events[0]["frames"], 3)


class ObjectOverlapCondition:
    source_object_ids = ""
    target_object_ids = "*"
    distance = 0.1
    direction = "INCLUDE_BACK"
    ignore_object_ids = []
    target_types = ["PEDESTRIAN", "BICYCLE"]
    use_score = False


class FilteredCollisionTest(unittest.TestCase):
    def test_vru_collision_filters_vehicle_obstacles(self):
        handler = ObjectOverlapConditionHandler()
        result = DetailedResult()
        car = make_car(x=0.0, speed=0.0)
        obj = MockObject(id="veh1", position_x=0.0, position_y=0.0)
        obj.type = "VEHICLE"
        world = make_world(car)
        world.object.append(obj)

        collided = handler._eval_object_overlap(
            ObjectOverlapCondition(), world, result)

        self.assertFalse(collided)

    def test_vru_collision_keeps_pedestrians(self):
        handler = ObjectOverlapConditionHandler()
        result = DetailedResult()
        car = make_car(x=0.0, speed=0.0)
        obj = MockObject(id="ped1", position_x=0.0, position_y=0.0)
        obj.type = "PEDESTRIAN"
        world = make_world(car)
        world.object.append(obj)

        collided = handler._eval_object_overlap(
            ObjectOverlapCondition(), world, result)

        self.assertTrue(collided)
        self.assertEqual(result.failure_code, "collision_ped1")


class ManifestAndVehicleParamTest(unittest.TestCase):
    def test_manifest_reads_scope_and_vehicle_param(self):
        scenes = parse_scene_manifest(
            "/apollo_workspace/auto_drive_console/metric/scenes.json")

        self.assertEqual(scenes[1].scope, "crosswalk")
        self.assertEqual(scenes[2].scope, "red_light")
        self.assertTrue(scenes[0].vehicle_param_file.endswith(
            "dkit_vehicle_param.pb.txt"))

    def test_dkit_vehicle_param_is_loaded(self):
        vehicle = _load_vehicle_param(
            "/apollo_workspace/auto_drive_console/metric/dkit_vehicle_param.pb.txt")

        self.assertAlmostEqual(vehicle["front_edge_to_center"], 3.62)
        self.assertAlmostEqual(vehicle["back_edge_to_center"], 0.87)
        self.assertAlmostEqual(vehicle["width"], 2.014)


if __name__ == "__main__":
    unittest.main()
