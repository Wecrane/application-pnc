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
    const auto* context = GetContextAs<ContestScenarioContext>();
    switch (context->kind) {
    case ContestScenarioKind::LANE_CHANGE:
        return contest::IsContestLaneChange(frame);
    case ContestScenarioKind::S_CURVE:
        return contest::IsContestSCurve(frame.reference_line_info().front(), context->scenario_config);
    case ContestScenarioKind::U_TURN:
        return contest::IsContestUTurn(frame.reference_line_info().front(), context->scenario_config);
    case ContestScenarioKind::CONSTRUCTION_ZONE:
        return contest::IsContestConstructionZone(frame.reference_line_info().front(), context->scenario_config);
    case ContestScenarioKind::STATION_SHUTTLE:
        return !context->shuttle_departed;
    }
    return false;
}

}  // namespace planning
}  // namespace apollo
