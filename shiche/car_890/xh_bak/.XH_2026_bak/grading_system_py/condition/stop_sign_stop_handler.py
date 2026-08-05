"""
停止标志停车距离条件评估器

逻辑（参考 red_light_stop_condition_handler）:
- 在交汇路口前方存在停止标志(stop_sign)时检查
- 主车必须完全停止(speed <= stop_speed_eps)
- 车辆移动接近停止线时：若压过停止线直接失败
- 车辆停止时检查车头到停止线距离是否在 [min_distance, max_distance]
- 依赖地图中的 stop_sign stop_line
"""

from collections import deque

from grading_system_py.common.math_utils import Vec2d, LineSegment2d
from grading_system_py.common.map_interface import get_map_interface
from grading_system_py.condition.condition_handler_base import ConditionHandlerBase
from grading_system_py.condition.util import ConditionEvaluatorUtil


class StopSignStopConditionHandler(ConditionHandlerBase):
    """停止标志停车距离条件处理器"""

    def __init__(self):
        self._single_deducted = False
        # 记录每个停止标志是否已观测到主车完全停止
        self._fully_stopped = {}

    def evaluate(self, condition, world_list: deque,
                 future_list: deque, detailed_result) -> bool:
        return self._eval_stop_sign(
            condition.stop_sign_stop_condition, world_list[-1], detailed_result)

    def _eval_stop_sign(self, cond, world, detailed_result) -> bool:
        adc = world.auto_driving_car

        stop_signs = get_map_interface().get_forward_nearest_stop_signs(
            adc.position_x, adc.position_y, 50)
        if not stop_signs:
            return True

        car_polygon = ConditionEvaluatorUtil.get_object_polygon(adc, False)
        success, head_position, tail_position = \
            ConditionEvaluatorUtil.get_box_head_tail_position(
                adc.width, adc.heading, car_polygon)
        if not success:
            return True

        stop_speed_eps = getattr(cond, 'stop_speed_eps', 0.1)
        adc_is_moving = getattr(adc, 'speed', 0.0) > stop_speed_eps

        for stop_sign in stop_signs:
            ss_id = getattr(stop_sign, 'stop_sign_id', '')
            min_dist_to_stopline = float('inf')
            dist_updated = False

            for stop_line_pts in getattr(stop_sign, 'stop_line_points', []):
                if len(stop_line_pts) < 2:
                    continue
                start = Vec2d(stop_line_pts[0][0], stop_line_pts[0][1])
                end = Vec2d(stop_line_pts[-1][0], stop_line_pts[-1][1])
                stopline_segment = LineSegment2d(start, end)

                # 压线/越线：直接失败
                if car_polygon.has_overlap(stopline_segment):
                    detailed_result.add_description(
                        f"ADC overlaps stop-sign stopline {ss_id}. ")
                    if getattr(cond, 'use_score', False):
                        detailed_result.delta_score = 100
                    return False

                head_dist = stopline_segment.distance_to(head_position)
                tail_dist = stopline_segment.distance_to(tail_position)

                # 车头比车尾更远，认为已经过线，不再检查
                if head_dist > tail_dist:
                    continue

                min_dist_to_stopline = min(min_dist_to_stopline, head_dist)
                dist_updated = True

            if not dist_updated:
                continue

            # 主车尚未完全停止：还在接近停止线，允许其继续减速
            if adc_is_moving:
                continue

            # 主车已经停止，记录一次完全停止
            self._fully_stopped[ss_id] = True

            # 仍远在停止线前（超出接近判定区间）时的低速帧属于接近/排队过程，
            # 不能因单帧低速就误判 too_far。等真正进入判定区间再评。
            if min_dist_to_stopline > cond.max_distance:
                continue

            if min_dist_to_stopline < cond.min_distance:
                detailed_result.add_description(
                    f"Distance from stop-sign stopline to ADC is "
                    f"{min_dist_to_stopline:.3f}, expected "
                    f"[{cond.min_distance}, {cond.max_distance}]. ")
                if getattr(cond, 'use_score', False) and not self._single_deducted:
                    detailed_result.delta_score = cond.single_deduction
                self._single_deducted = True
                return False

        return True
