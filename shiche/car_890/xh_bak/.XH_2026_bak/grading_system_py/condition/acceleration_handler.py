"""
加速度条件评估器
对应原 C++ acceleration_condition_handler.cc

评估逻辑（扣分制）:
- HardBrake: 检测急刹车，排除合理刹车场景
- 通用: 加速度超出 [min, max] 范围则不通过
"""

from collections import deque
from typing import List, Any

from grading_system_py.common.math_utils import Vec2d, Polygon2d, LineSegment2d
from grading_system_py.common.map_interface import get_map_interface
from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


class AccelerationConditionHandler(ConditionHandlerBase):
    """加速度条件处理器"""

    def __init__(self):
        self._trigger_timestamp = 0.0
        self._triggered = False

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_acceleration(condition.acceleration_condition,
                                       world_list, detailed_result)

    def _eval_acceleration(self, accel_cond, world_list: deque,
                           detailed_result) -> bool:
        """
        加速度评估主逻辑

        Args:
            accel_cond: AccelerationCondition 配置
            world_list: 历史世界状态
            detailed_result: 结果输出
        """
        world = world_list[-1]
        adc = world.auto_driving_car

        if not hasattr(adc, 'speed_acceleration') or adc.speed_acceleration is None:
            return True

        acceleration = adc.speed_acceleration
        passed = True

        if accel_cond.name == "HardBrake":
            self._triggered = acceleration < accel_cond.min_acceleration

            # 排除合理刹车场景
            for obj in world.object:
                # 1. 避让切入的移动障碍物
                if obj.speed > 0 and \
                   not ConditionEvaluatorUtil.is_behind(adc, obj) and \
                   ConditionEvaluatorUtil.rays_intersect(
                       Vec2d(obj.position_x, obj.position_y), obj.speed_heading,
                       Vec2d(adc.position_x, adc.position_y), adc.speed_heading,
                       accel_cond.cut_in_distance):
                    detailed_result.add_description(
                        f"Expected braking to yield to obstacle {obj.id}. ")
                    self._triggered = False
                    break

                # 2. TTC 过小
                if ConditionEvaluatorUtil.ttc(adc, obj) < accel_cond.ttc:
                    detailed_result.add_description(
                        f"TTC to {obj.id} is less than {accel_cond.ttc}s.")
                    self._triggered = False
                    break

            # 3. 信号灯变化（绿转红/黄）
            all_signals = []
            for signal in world.perceived_signal:
                if hasattr(signal, 'id') and signal.id:
                    all_signals.append(signal.id)

            map_intf = get_map_interface()
            traffic_lights = map_intf.get_forward_nearest_signals(
                adc.position_x, adc.position_y, 500)

            if traffic_lights:
                find_traffic_light_ids = self._get_traffic_light_within_distance(
                    all_signals, traffic_lights, adc, accel_cond.dist_stopline)

                cur_time = world.timestamp_sec
                prev_time = cur_time if cur_time < accel_cond.light_turn_prev_time \
                    else accel_cond.light_turn_prev_time

                if self._is_green_light_turn_red(find_traffic_light_ids,
                                                  world_list, cur_time, prev_time):
                    self._triggered = False

            if self._triggered:
                cur_time = world.timestamp_sec
                duration = cur_time - self._trigger_timestamp
                detailed_result.add_description(
                    f"Cur ego speed acceleration is {acceleration}")
                passed = duration < accel_cond.duration
                if not passed:
                    detailed_result.add_description(
                        f"The duration of deceleration less than "
                        f"{accel_cond.min_acceleration}s is greater than "
                        f"{accel_cond.duration}")
            else:
                self._triggered = True
                self._trigger_timestamp = world.timestamp_sec
        else:
            # 通用加速度范围检测
            passed = (acceleration >= accel_cond.min_acceleration and
                      acceleration <= accel_cond.max_acceleration)

        if not passed:
            detailed_result.add_description(f"Acceleration is {acceleration}. ")
            if getattr(accel_cond, 'use_score', False):
                delta_score = ConditionEvaluatorUtil.score_deducted_by_unit(
                    max(acceleration - accel_cond.max_acceleration,
                        accel_cond.min_acceleration - acceleration),
                    accel_cond.deduction_unit,
                    accel_cond.single_deduction)
                detailed_result.delta_score = delta_score

        return passed

    def _is_green_light_turn_red(self, traffic_light_ids: List[str],
                                  world_list: deque, cur_time: float,
                                  prev_time: float) -> bool:
        """判断信号灯是否从绿灯变为非绿灯"""
        if not traffic_light_ids:
            return False

        world = world_list[-1]
        for signal_id in traffic_light_ids:
            non_green = False
            is_green = False
            for prev_world in reversed(world_list):
                if cur_time - prev_world.timestamp_sec > prev_time:
                    break
                for signal in world.perceived_signal:
                    if signal_id == signal.id:
                        color = signal.current_signal
                        if color != "GREEN":
                            non_green = True
                        elif non_green and color == "GREEN":
                            is_green = True
                        break
                if non_green and is_green:
                    return True
            if non_green and is_green:
                return True
        return False

    def _get_traffic_light_within_distance(self, all_signals: List[str],
                                            traffic_lights: list,
                                            adc, dist: float) -> List[str]:
        """获取在指定距离内的信号灯ID"""
        find_ids = []
        for tl in traffic_lights:
            if tl.signal_id not in all_signals:
                continue

            car_polygon = ConditionEvaluatorUtil.get_object_polygon(adc, False)
            success, head_position, _ = ConditionEvaluatorUtil.get_box_head_tail_position(
                adc.width, adc.heading, car_polygon)
            if not success:
                continue

            for stop_line_pts in tl.stop_line_points:
                if len(stop_line_pts) < 2:
                    continue
                start_pt = Vec2d(stop_line_pts[0][0], stop_line_pts[0][1])
                end_pt = Vec2d(stop_line_pts[-1][0], stop_line_pts[-1][1])
                stopline_segment = LineSegment2d(start_pt, end_pt)

                if car_polygon.has_overlap(stopline_segment) or \
                   stopline_segment.distance_to(head_position) < dist:
                    find_ids.append(tl.signal_id)

        return find_ids
