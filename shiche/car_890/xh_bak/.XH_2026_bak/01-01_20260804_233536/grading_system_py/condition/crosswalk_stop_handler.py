"""
人行横道停车距离条件评估器
对应原 C++ crosswalk_stop_condition_handler.cc

逻辑：
- 只在有行人/非机动车位于人行横道时检查
- 占用人行横道时压入或穿过 crosswalk，直接失败
- 车辆停止时检查车头到 crosswalk 距离是否在 [min_distance, max_distance]
"""

from collections import deque
import math

from grading_system_py.common.math_utils import Vec2d
from grading_system_py.common.map_interface import get_map_interface
from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


class CrosswalkStopConditionHandler(ConditionHandlerBase):
    """人行横道停车距离条件处理器"""

    def __init__(self):
        self._range_deducted = {
            'beyond': False,
            'less_than': False,
        }
        self._qualified_stop = {}
        self._prev_signed_distance = {}
        self._hard_failed = set()

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_crosswalk_stop(
            condition.crosswalk_stop_condition, world_list[-1], detailed_result)

    def _eval_crosswalk_stop(self, cond, world, detailed_result) -> bool:
        adc = world.auto_driving_car
        stop_speed_eps = getattr(cond, 'stop_speed_eps', 0.1)
        cross_tolerance = getattr(cond, 'crossing_tolerance', 0.05)
        adc_is_moving = getattr(adc, 'speed', 0.0) > stop_speed_eps
        car_polygon = ConditionEvaluatorUtil.get_object_polygon(adc, False)
        target_ids = set(getattr(cond, 'target_ids', []) or [])

        heading_vec = Vec2d(math.cos(adc.heading), math.sin(adc.heading))
        head_position = Vec2d(
            adc.position_x + getattr(adc, 'front_edge_to_center', adc.length / 2.0) * heading_vec.x,
            adc.position_y + getattr(adc, 'front_edge_to_center', adc.length / 2.0) * heading_vec.y,
        )

        active_crosswalks = {}

        for obj in getattr(world, 'object', []):
            obj_type = getattr(obj, 'type', '')
            is_vulnerable_road_user = obj_type in ('PEDESTRIAN', 'BICYCLE', 3, 4)
            if not is_vulnerable_road_user:
                continue

            person_polygon = ConditionEvaluatorUtil.get_object_polygon(obj, False)
            crosswalk = get_map_interface().find_crosswalk_for_polygon(
                obj.position_x, obj.position_y, 50, person_polygon)
            if crosswalk is None:
                continue
            if target_ids and crosswalk.crosswalk_id not in target_ids:
                continue
            active_crosswalks[crosswalk.crosswalk_id] = crosswalk

        if not active_crosswalks:
            for crosswalk_id in list(self._qualified_stop.keys()):
                self._qualified_stop[crosswalk_id] = False
            return True

        for crosswalk_id, crosswalk in active_crosswalks.items():
            crosswalk_polygon = crosswalk.polygon
            dist_to_crosswalk, closest = crosswalk_polygon.distance_to_point(head_position)
            signed_dist_to_crosswalk = (closest - head_position).inner_prod(heading_vec)
            prev_signed_dist = self._prev_signed_distance.get(crosswalk_id)

            # 压入人行横道：直接失败
            if car_polygon.has_overlap(crosswalk_polygon) or dist_to_crosswalk == 0:
                self._hard_failed.add(crosswalk_id)
                detailed_result.add_failure(
                    "crosswalk_entered_occupied",
                    f"Entered occupied crosswalk {crosswalk_id}. ")
                if getattr(cond, 'use_score', False):
                    detailed_result.delta_score = 100
                self._prev_signed_distance[crosswalk_id] = signed_dist_to_crosswalk
                return False

            crossed_while_occupied = (
                prev_signed_dist is not None and
                prev_signed_dist >= -cross_tolerance and
                signed_dist_to_crosswalk < -cross_tolerance
            )
            if crossed_while_occupied:
                self._hard_failed.add(crosswalk_id)
                detailed_result.add_failure(
                    "crosswalk_crossed_occupied",
                    f"Crossed occupied crosswalk {crosswalk_id}. ")
                if getattr(cond, 'use_score', False):
                    detailed_result.delta_score = 100
                self._prev_signed_distance[crosswalk_id] = signed_dist_to_crosswalk
                return False

            if crosswalk_id in self._hard_failed:
                detailed_result.add_failure(
                    "crosswalk_beyond_occupied",
                    f"After crosswalk violation: ADC is beyond occupied crosswalk {crosswalk_id}. ")
                if getattr(cond, 'use_score', False):
                    detailed_result.delta_score = 100
                self._prev_signed_distance[crosswalk_id] = signed_dist_to_crosswalk
                return False

            if adc_is_moving:
                if self._qualified_stop.get(crosswalk_id) and signed_dist_to_crosswalk <= cond.max_distance:
                    detailed_result.add_failure(
                        "crosswalk_moved_before_clear",
                        f"Moved before occupied crosswalk {crosswalk_id} was clear. ")
                    if getattr(cond, 'use_score', False):
                        detailed_result.delta_score = 100
                    self._prev_signed_distance[crosswalk_id] = signed_dist_to_crosswalk
                    return False
                if 0.0 <= signed_dist_to_crosswalk < cond.min_distance:
                    self._hard_failed.add(crosswalk_id)
                    detailed_result.add_failure(
                        "crosswalk_moving_too_close",
                        f"No qualified crosswalk stop: ADC is moving inside required stop distance "
                        f"{signed_dist_to_crosswalk:.3f}, expected at least "
                        f"{cond.min_distance}. ")
                    if getattr(cond, 'use_score', False):
                        detailed_result.delta_score = 100
                    self._prev_signed_distance[crosswalk_id] = signed_dist_to_crosswalk
                    return False
                self._prev_signed_distance[crosswalk_id] = signed_dist_to_crosswalk
                continue

            # 车辆已停稳（低速）：只有确实停在人行横道前的合理接近区间内，
            # 才把它当作"最终停车点"来判定距离；仍在远处（signed 距离 > max_distance）
            # 说明还在接近过程中，不能因为某一帧低速就误判 stopped_too_far。
            if signed_dist_to_crosswalk < 0:
                # 车头已越过人行横道参考点，属于已通过/越线，交给上面的越线逻辑，
                # 这里不再当作停车点判距。
                self._prev_signed_distance[crosswalk_id] = signed_dist_to_crosswalk
                continue

            if signed_dist_to_crosswalk > cond.max_distance:
                # 还在接近，尚未到停车判定区间，跳过（不误判 too_far）。
                self._prev_signed_distance[crosswalk_id] = signed_dist_to_crosswalk
                continue

            if signed_dist_to_crosswalk < cond.min_distance:
                detailed_result.add_failure(
                    "crosswalk_stopped_too_close",
                    f"Stopped too close to crosswalk {crosswalk_id}: "
                    f"distance={signed_dist_to_crosswalk:.3f}, expected "
                    f"[{cond.min_distance}, {cond.max_distance}]. ")
                if getattr(cond, 'use_score', False) and not self._range_deducted['less_than']:
                    detailed_result.delta_score = cond.single_deduction
                    self._range_deducted['less_than'] = True
                self._prev_signed_distance[crosswalk_id] = signed_dist_to_crosswalk
                return False

            self._qualified_stop[crosswalk_id] = True
            self._prev_signed_distance[crosswalk_id] = signed_dist_to_crosswalk

        return True
