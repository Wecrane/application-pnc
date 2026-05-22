/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/scenarios/contest/stage_contest_lane_follow.h"

#include <limits>
#include <string>
#include <vector>

#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/scenarios/contest/context.h"
#include "modules/planning/scenarios/contest/contest_scenario_util.h"

namespace apollo {
namespace planning {

StageResult ContestLaneFollowStage::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    // 站点接驳：在 task 流水线执行前注入 stop fence 和限速
    InjectStationShuttleStop(frame);

    StageResult result = LaneFollowStage::Process(planning_init_point, frame);
    if (result.HasError()) {
        return result;
    }
    if (!StillInScenario(*frame)) {
        // U 型弯退出清理：重置借道标志，防止原始 LaneBorrowPath 误触发
        auto* ctx = GetContextAs<ContestScenarioContext>();
        if (ctx->kind == ContestScenarioKind::U_TURN) {
            injector_->planning_context()
                    ->mutable_planning_status()
                    ->mutable_path_decider()
                    ->set_is_in_path_lane_borrow_scenario(false);
        }
        return FinishScenario();
    }
    return result.SetStageStatus(StageStatusType::RUNNING);
}

void ContestLaneFollowStage::InjectStationShuttleStop(Frame* frame) {
    auto* ctx = GetContextAs<ContestScenarioContext>();
    if (ctx->kind != ContestScenarioKind::STATION_SHUTTLE) {
        return;
    }
    if (ctx->shuttle_departed || frame->reference_line_info().empty()) {
        return;
    }

    auto& ref_line_info = frame->mutable_reference_line_info()->front();
    const auto& nearby_path = ref_line_info.reference_line().map_path();
    const double adc_end_s = ref_line_info.AdcSlBoundary().end_s();
    const double look_forward = ctx->scenario_config.station_shuttle_look_forward_distance();

    // 查找前方最近泊车位
    double target_spot_s = -1.0;
    double best_dist = std::numeric_limits<double>::max();
    for (const auto& overlap : nearby_path.parking_space_overlaps()) {
        if (overlap.start_s <= adc_end_s)
            continue;
        const double dist = overlap.start_s - adc_end_s;
        if (dist > look_forward)
            continue;
        if (dist < best_dist) {
            best_dist = dist;
            target_spot_s = overlap.start_s;
        }
    }
    if (target_spot_s < 0.0) {
        ctx->shuttle_departed = true;
        return;
    }

    const double adc_speed = frame->vehicle_state().linear_velocity();

    if (!ctx->shuttle_arrived_at_station) {
        // 未到达：创建 stop fence
        constexpr double kStopDist = 0.5;
        const double stop_s = target_spot_s - kStopDist;
        if (stop_s > adc_end_s) {
            std::vector<std::string> empty_wait;
            apollo::planning::util::BuildStopDecision(
                    "contest_shuttle_approach",
                    stop_s,
                    kStopDist,
                    StopReasonCode::STOP_REASON_PULL_OVER,
                    empty_wait,
                    "contest_station_shuttle",
                    frame,
                    &ref_line_info);
        }
        constexpr double kStopSpeed = 0.1;
        constexpr double kArriveDist = 1.5;
        if (adc_speed < kStopSpeed && std::fabs(adc_end_s - target_spot_s) < kArriveDist) {
            ctx->shuttle_arrived_at_station = true;
            ctx->shuttle_dwell_start_time = cyber::Clock::NowInSeconds();
            AINFO << "[SS] arrived at station, dwell start";
        }
    } else {
        // 已到达：保持 stop fence + dwell 计时
        std::vector<std::string> empty_wait;
        apollo::planning::util::BuildStopDecision(
                "contest_shuttle_dwell",
                target_spot_s,
                0.5,
                StopReasonCode::STOP_REASON_PULL_OVER,
                empty_wait,
                "contest_station_shuttle",
                frame,
                &ref_line_info);

        const double elapsed = cyber::Clock::NowInSeconds() - ctx->shuttle_dwell_start_time;
        if (elapsed >= ctx->scenario_config.station_shuttle_dwell_time_sec()) {
            ctx->shuttle_departed = true;
            AINFO << "[SS] dwell complete, departing";
        }
    }

    // 站点区域限速
    const double speed_limit = ctx->scenario_config.station_shuttle_speed_limit();
    constexpr double kZoneRadius = 40.0;
    ref_line_info.mutable_reference_line()->AddSpeedLimit(
            std::max(0.0, target_spot_s - kZoneRadius), target_spot_s + kZoneRadius, speed_limit);
}

bool ContestLaneFollowStage::StillInScenario(const Frame& frame) const {
    if (frame.reference_line_info().empty()) {
        return false;
    }
    auto* context = GetContextAs<ContestScenarioContext>();
    switch (context->kind) {
    case ContestScenarioKind::LANE_CHANGE:
        return contest::IsContestLaneChange(frame);
    case ContestScenarioKind::S_CURVE:
        if (contest::IsContestUTurn(frame.reference_line_info().front(), context->scenario_config)) {
            return false;
        }
        if (contest::IsContestSCurve(frame.reference_line_info().front(), context->scenario_config)) {
            context->s_curve_exit_hold_frames = 0;
            return true;
        }
        // 防抖：连续 N 帧不在 S 弯才退出，避免锥桶检测边界振荡
        static constexpr int kSCurveExitHysteresisFrames = 10;
        context->s_curve_exit_hold_frames++;
        if (context->s_curve_exit_hold_frames < kSCurveExitHysteresisFrames) {
            AINFO << "[S_CURVE] exit hold, frame=" << context->s_curve_exit_hold_frames;
            return true;
        }
        return false;
    case ContestScenarioKind::U_TURN:
        if (!context->u_turn_active) {
            context->u_turn_active = true;
            context->u_turn_completed = false;
            context->u_turn_entry_heading = frame.vehicle_state().heading();
            AINFO << "[UTURN] enter, entry_heading=" << context->u_turn_entry_heading;
            return true;
        }
        // 完成判定用比检测更高的阈值（至少 2.7 rad ≈ 155°），确保车辆基本完成掉头
        static constexpr double kUTurnCompletionHeadingThreshold = 2.7;
        if (std::fabs(common::math::NormalizeAngle(frame.vehicle_state().heading() - context->u_turn_entry_heading))
            > kUTurnCompletionHeadingThreshold) {
            // ── 退出保护：heading 反转后暂不退出，等待车辆回到车道中心 ──
            // Contest 管道在场景退出后不再运行，所以必须在退出前完成恢复。
            // 这里检查车辆 l 是否回到 ±0.8m 以内才允许退出，最多额外保持 50 帧。
            const auto& rli = frame.reference_line_info().front();
            const auto& sl_bound = rli.AdcSlBoundary();
            const double adc_mid_l = (sl_bound.start_l() + sl_bound.end_l()) * 0.5;
            const double distance_to_destination = rli.SDistanceToDestination();
            static constexpr double kExitDestinationDistance = 8.0;
            if (distance_to_destination > kExitDestinationDistance) {
                AINFO << "[UTURN] heading reversed but destination is still ahead"
                      << " (distance=" << distance_to_destination << "m, max=" << kExitDestinationDistance
                      << "m), holding scenario";
                return true;
            }
            static constexpr double kExitMaxLateralOffset = 0.8;
            if (std::fabs(adc_mid_l) > kExitMaxLateralOffset) {
                context->u_turn_exit_hold_frames++;
                static constexpr int kMaxExitHoldFrames = 50;
                if (context->u_turn_exit_hold_frames < kMaxExitHoldFrames) {
                    AINFO << "[UTURN] heading reversed but vehicle off-center"
                          << " (mid_l=" << adc_mid_l << ", max=" << kExitMaxLateralOffset
                          << "), holding scenario, hold_frame=" << context->u_turn_exit_hold_frames;
                    return true;  // 保持场景活跃，Contest 管道继续运行
                }
                AWARN << "[UTURN] exit hold timeout after " << kMaxExitHoldFrames << " frames, forcing exit";
            }
            context->u_turn_exit_hold_frames = 0;
            context->u_turn_active = false;
            context->u_turn_completed = true;
            AINFO << "[UTURN] exit (heading reversed, mid_l=" << adc_mid_l
                  << ", distance_to_destination=" << distance_to_destination << ")";
            return false;
        }
        return true;
    case ContestScenarioKind::CONSTRUCTION_ZONE:
        if (contest::IsContestUTurn(frame.reference_line_info().front(), context->scenario_config)
            || contest::IsContestSCurve(frame.reference_line_info().front(), context->scenario_config)) {
            return false;
        }
        return contest::CountContestConstructionConesAhead(
                       frame.reference_line_info().front(),
                       context->scenario_config.construction_look_forward_distance())
                > 0;
    case ContestScenarioKind::STATION_SHUTTLE:
        return !context->shuttle_departed;
    }
    return false;
}

}  // namespace planning
}  // namespace apollo
