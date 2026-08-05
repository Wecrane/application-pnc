"""
红灯停车距离条件评估器
对应原 C++ red_light_stop_condition_handler.cc

逻辑：
- 只在检测到 RED 信号灯时检查
- 主车右转/应急灯时跳过
- 红灯期间压线或从停止线前跨到停止线后，直接失败
- 车辆停止时检查车头到 stopline 距离是否在 [min_distance, max_distance]
"""

from collections import deque
import math

from grading_system_py.common.math_utils import Vec2d, LineSegment2d
from grading_system_py.common.map_interface import get_map_interface
from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


class RedLightStopConditionHandler(ConditionHandlerBase):
    """红灯停车距离条件处理器"""

    def __init__(self):
        self._single_deducted = False
        self._qualified_stop = {}
        self._prev_signed_distance = {}
        self._hard_failed = set()

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_red_light_stop(
            condition.red_light_stop_condition, world_list[-1], detailed_result)

    def _eval_red_light_stop(self, cond, world, detailed_result) -> bool:
        perceived = getattr(world, 'perceived_signal', [])
        target_ids = set(getattr(cond, 'target_ids', []) or [])
        red_signal_ids = [s.id for s in perceived
                          if getattr(s, 'id', '') and
                          (not target_ids or getattr(s, 'id', '') in target_ids) and
                          getattr(s, 'current_signal', '') == 'RED']
        if not red_signal_ids:
            return True

        adc = world.auto_driving_car
        current_signal = getattr(adc, 'current_signal', '')
        if current_signal in ('RIGHT', 'EMERGENCY'):
            return True

        traffic_lights = get_map_interface().get_forward_nearest_signals(
            adc.position_x, adc.position_y, 50)
        if not traffic_lights:
            return True

        car_polygon = ConditionEvaluatorUtil.get_object_polygon(adc, False)
        heading_vec = Vec2d(math.cos(adc.heading), math.sin(adc.heading))
        head_position = Vec2d(
            adc.position_x + getattr(adc, 'front_edge_to_center', adc.length / 2.0) * heading_vec.x,
            adc.position_y + getattr(adc, 'front_edge_to_center', adc.length / 2.0) * heading_vec.y,
        )

        stop_speed_eps = getattr(cond, 'stop_speed_eps', 0.1)
        cross_tolerance = getattr(cond, 'crossing_tolerance', 0.05)
        adc_is_moving = getattr(adc, 'speed', 0.0) > stop_speed_eps

        for traffic_light in traffic_lights:
            signal_id = getattr(traffic_light, 'signal_id', '')
            if target_ids and signal_id not in target_ids:
                continue
            signed_dist_to_stopline = None
            has_overlap = False

            for stop_line_pts in getattr(traffic_light, 'stop_line_points', []):
                if len(stop_line_pts) < 2:
                    continue
                start = Vec2d(stop_line_pts[0][0], stop_line_pts[0][1])
                end = Vec2d(stop_line_pts[-1][0], stop_line_pts[-1][1])
                stopline_segment = LineSegment2d(start, end)

                # 压线/越线：直接失败
                if car_polygon.has_overlap(stopline_segment):
                    has_overlap = True

                _, closest = stopline_segment.distance_to_point(head_position)
                signed_dist = (closest - head_position).inner_prod(heading_vec)
                if (signed_dist_to_stopline is None or
                        abs(signed_dist) < abs(signed_dist_to_stopline)):
                    signed_dist_to_stopline = signed_dist

            if signed_dist_to_stopline is None:
                continue

            prev_signed_dist = self._prev_signed_distance.get(signal_id)

            if signal_id not in red_signal_ids:
                self._qualified_stop[signal_id] = False
                self._prev_signed_distance[signal_id] = signed_dist_to_stopline
                continue

            if has_overlap:
                self._hard_failed.add(signal_id)
                detailed_result.add_failure(
                    "red_overlap_stopline",
                    f"Ran red light: ADC overlaps stopline {signal_id}. ")
                if getattr(cond, 'use_score', False):
                    detailed_result.delta_score = 100
                self._prev_signed_distance[signal_id] = signed_dist_to_stopline
                return False

            crossed_on_red = (
                prev_signed_dist is not None and
                prev_signed_dist >= -cross_tolerance and
                signed_dist_to_stopline < -cross_tolerance
            )
            if crossed_on_red:
                self._hard_failed.add(signal_id)
                detailed_result.add_failure(
                    "red_crossed_stopline",
                    f"Ran red light: ADC crossed stopline {signal_id}. ")
                if getattr(cond, 'use_score', False):
                    detailed_result.delta_score = 100
                self._prev_signed_distance[signal_id] = signed_dist_to_stopline
                return False

            if signal_id in self._hard_failed:
                detailed_result.add_failure(
                    "red_beyond_stopline",
                    f"After red-light violation: ADC is beyond stopline {signal_id}. ")
                if getattr(cond, 'use_score', False):
                    detailed_result.delta_score = 100
                self._prev_signed_distance[signal_id] = signed_dist_to_stopline
                return False

            if adc_is_moving:
                if 0.0 <= signed_dist_to_stopline < cond.min_distance:
                    self._hard_failed.add(signal_id)
                    detailed_result.add_failure(
                        "red_moving_too_close",
                        f"No qualified red-light stop at {signal_id}: ADC is moving inside required stop distance "
                        f"{signed_dist_to_stopline:.3f}, expected at least "
                        f"{cond.min_distance}. ")
                    if getattr(cond, 'use_score', False):
                        detailed_result.delta_score = 100
                    self._prev_signed_distance[signal_id] = signed_dist_to_stopline
                    return False
                self._prev_signed_distance[signal_id] = signed_dist_to_stopline
                continue

            if 0.0 <= signed_dist_to_stopline <= cond.max_distance:
                if signed_dist_to_stopline < cond.min_distance:
                    detailed_result.add_failure(
                        "red_stopped_too_close",
                        f"Stopped too close to red-light stopline {signal_id}: "
                        f"distance={signed_dist_to_stopline:.3f}, "
                        f"expected [{cond.min_distance}, {cond.max_distance}]. ")
                    if getattr(cond, 'use_score', False) and not self._single_deducted:
                        detailed_result.delta_score = cond.single_deduction
                    self._single_deducted = True
                    self._prev_signed_distance[signal_id] = signed_dist_to_stopline
                    return False
                self._qualified_stop[signal_id] = True
            elif signed_dist_to_stopline > cond.max_distance:
                # 车辆瞬时低速但仍远在停止线前，说明还在接近/排队，不能因单帧低速
                # 就误判 stopped_too_far。保持通过，等真正进入判定区间再评。
                self._prev_signed_distance[signal_id] = signed_dist_to_stopline
                continue

            self._prev_signed_distance[signal_id] = signed_dist_to_stopline

        return True
