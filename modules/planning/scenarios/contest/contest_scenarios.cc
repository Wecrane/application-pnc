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

#include "modules/planning/scenarios/contest/contest_scenarios.h"

#include <cmath>
#include <string>

#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/scenarios/contest/contest_scenario_util.h"
#include "modules/planning/scenarios/contest/stage_contest_lane_follow.h"

namespace apollo {
namespace planning {

namespace {

bool IsBusBayLaneFollowExitContext(const std::shared_ptr<DependencyInjector>& injector,
                                   const Frame& frame) {
    if (injector == nullptr || injector->planning_context() == nullptr || frame.reference_line_info().empty()) {
        return false;
    }
    const auto& planning_status = injector->planning_context()->planning_status();
    if (!planning_status.destination().has_passed_destination()) {
        return false;
    }
    const auto& reference_line_info = frame.reference_line_info().front();
    if (reference_line_info.reference_line().map_path().parking_space_overlaps().empty()) {
        return false;
    }
    const double distance_to_destination = reference_line_info.SDistanceToDestination();
    return std::isfinite(distance_to_destination)
           && distance_to_destination > FLAGS_destination_check_distance;
}

}  // namespace

bool ContestScenarioBase::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    if (init_) {
        return true;
    }
    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario " << Name();
        return false;
    }
    if (!Scenario::LoadConfig<ScenarioContestConfig>(&context_.scenario_config)) {
        AERROR << "failed to load contest scenario config for " << Name();
        return false;
    }
    context_.kind = kind_;
    init_ = true;
    return true;
}

bool ContestScenarioBase::IsReferenceLineReady(const Frame& frame) const {
    return frame.local_view().planning_command != nullptr
            && frame.local_view().planning_command->has_lane_follow_command() && !frame.reference_line_info().empty();
}

bool ContestLaneChangeScenario::IsTransferable(const Scenario* other_scenario, const Frame& frame) {
    if (other_scenario == nullptr || !IsReferenceLineReady(frame)) {
        return false;
    }
    return contest::IsContestLaneChange(frame);
}

bool ContestSCurveScenario::IsTransferable(const Scenario* other_scenario, const Frame& frame) {
    if (other_scenario != nullptr && IsReferenceLineReady(frame)
        && contest::IsContestUTurn(frame.reference_line_info().front(), GetContext()->scenario_config)) {
        return false;
    }
    return other_scenario != nullptr && IsReferenceLineReady(frame)
            && contest::IsContestSCurve(frame.reference_line_info().front(), GetContext()->scenario_config);
}

bool ContestUTurnScenario::IsTransferable(const Scenario* other_scenario, const Frame& frame) {
    if (other_scenario == nullptr || !IsReferenceLineReady(frame)) {
        return false;
    }
    auto* ctx = GetContext();
    const bool u_turn_ahead = contest::IsContestUTurn(frame.reference_line_info().front(), ctx->scenario_config);
    if (!u_turn_ahead) {
        ctx->u_turn_active = false;
        ctx->u_turn_completed = false;
        return false;
    }
    return !ctx->u_turn_completed;
}

bool ContestConstructionZoneScenario::IsTransferable(const Scenario* other_scenario, const Frame& frame) {
    if (other_scenario != nullptr && IsReferenceLineReady(frame)) {
        const auto& reference_line_info = frame.reference_line_info().front();
        if (contest::IsContestUTurn(reference_line_info, GetContext()->scenario_config)
            || contest::IsContestSCurve(reference_line_info, GetContext()->scenario_config)) {
            return false;
        }
    }
    return other_scenario != nullptr && IsReferenceLineReady(frame)
            && contest::IsContestConstructionZone(
                    frame, frame.reference_line_info().front(), GetContext()->scenario_config);
}

bool ContestStationShuttleScenario::IsTransferable(const Scenario* other_scenario, const Frame& frame) {
    if (other_scenario == nullptr || !IsReferenceLineReady(frame)) {
        return false;
    }
    auto* ctx = GetContext();
    if (ctx->shuttle_departed) {
        return false;
    }
    std::string parking_spot_id;
    const bool found = contest::IsContestStationShuttle(
            frame.reference_line_info().front(), ctx->scenario_config, &parking_spot_id);
    if (found) {
        ctx->shuttle_arrived_at_station = false;
        ctx->shuttle_dwell_start_time = 0.0;
    }
    return found;
}

bool ContestRoundaboutScenario::IsTransferable(const Scenario* other_scenario, const Frame& frame) {
    if (other_scenario == nullptr || !IsReferenceLineReady(frame)) {
        return false;
    }
    auto* ctx = GetContext();

    // ── 防重入：场景已完成本次环岛，需远离出口后才允许再次切入 ──
    if (ctx->roundabout_completed) {
        const double dx = frame.vehicle_state().x() - ctx->roundabout_exit_x;
        const double dy = frame.vehicle_state().y() - ctx->roundabout_exit_y;
        const double dist_from_exit = std::sqrt(dx * dx + dy * dy);
        static constexpr double kMinReentryDistance = 100.0;  // 离出口至少 100m 才允许重入
        if (dist_from_exit < kMinReentryDistance) {
            return false;  // 还在出口附近，禁止重入
        }
        // 已远离出口，重置完成标志，允许下次正常切入
        ctx->roundabout_completed = false;
        ctx->roundabout_exit_x = 0.0;
        ctx->roundabout_exit_y = 0.0;
        AINFO << "[ROUNDABOUT][Scenario] re-entry allowed (far from exit)"
              << ", dist=" << dist_from_exit << ", adc_x=" << frame.vehicle_state().x()
              << ", adc_y=" << frame.vehicle_state().y();
    }

    // 不让 U 型弯和站点接驳场景被环岛抢走
    const std::string other_name = other_scenario->Name();
    if (other_name == "CONTEST_U_TURN" || other_name == "CONTEST_STATION_SHUTTLE"
        || other_name == "BUS_BAY_TRANSFER" || other_name == "BusBayTransferScenario") {
        return false;
    }
    if ((other_name == "LANE_FOLLOW" || other_name == "LaneFollowScenario")
        && IsBusBayLaneFollowExitContext(injector_, frame)) {
        AINFO << "[ROUNDABOUT][Scenario] skip transfer during bus-bay lane-follow exit"
              << ", dist_to_dest=" << frame.reference_line_info().front().SDistanceToDestination()
              << ", adc_x=" << frame.vehicle_state().x() << ", adc_y=" << frame.vehicle_state().y();
        return false;
    }
    const double adc_x = frame.vehicle_state().x();
    const double adc_y = frame.vehicle_state().y();
    const bool is_entry = contest::IsContestRoundaboutEntry(frame, ctx->scenario_config);
    AINFO << "[ROUNDABOUT][Scenario] IsTransferable check"
          << ", entry=" << is_entry << ", other=" << other_name << ", adc_x=" << adc_x << ", adc_y=" << adc_y;
    if (!is_entry) {
        ctx->roundabout_committed = false;
        ctx->roundabout_entry_s = 0.0;
        return false;
    }
    if (!ctx->roundabout_committed) {
        ctx->roundabout_entry_s = frame.reference_line_info().front().AdcSlBoundary().end_s();
        AINFO << "[ROUNDABOUT][Scenario] first time entry detection, entry_s=" << ctx->roundabout_entry_s
              << ", adc_x=" << adc_x << ", adc_y=" << adc_y;
    }
    AINFO << "[ROUNDABOUT][Scenario] transfer to CONTEST_ROUNDABOUT from " << other_scenario->Name()
          << ", adc_x=" << adc_x << ", adc_y=" << adc_y;
    return true;
}

}  // namespace planning
}  // namespace apollo
