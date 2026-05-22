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

#include <string>

#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/scenarios/contest/contest_scenario_util.h"
#include "modules/planning/scenarios/contest/stage_contest_lane_follow.h"

namespace apollo {
namespace planning {

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
    return other_scenario != nullptr && IsReferenceLineReady(frame) && contest::IsContestLaneChange(frame);
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

}  // namespace planning
}  // namespace apollo
