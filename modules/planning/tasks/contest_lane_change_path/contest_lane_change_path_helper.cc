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

#include "modules/planning/tasks/contest_lane_change_path/contest_lane_change_path_helper.h"

#include <algorithm>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/decision.pb.h"
#include "modules/planning/planning_base/common/contest_scenario_status.h"

namespace apollo {
namespace planning {
namespace {

constexpr double kLaneChangeWatchRearBuffer = 4.0;
constexpr double kLaneChangeWatchFrontBuffer = 0.2;
constexpr double kLaneChangeWatchLateralBuffer = 0.8;
constexpr double kLaneChangeHoldLateralHalfWidth = 0.5;
constexpr double kLaneChangeSpeedLimit = 29.0 / 3.6;
constexpr double kMinLaneChangeSpeedKmh = 10.0;

}  // namespace

bool IsContestLaneChangeScenario(const std::shared_ptr<DependencyInjector>& injector) {
    return contest::IsCurrentScenario(injector, contest::kLaneChangeScenario);
}

double ContestLaneChangeMinStartSpeed() {
    return kMinLaneChangeSpeedKmh / 3.6;
}

bool IsContestLaneChangeWindowClear(ReferenceLineInfo* reference_line_info) {
    if (reference_line_info == nullptr) {
        return false;
    }
    const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
    const double check_start_s = adc_sl_boundary.start_s() - kLaneChangeWatchRearBuffer;
    const double check_end_s = adc_sl_boundary.end_s() + kLaneChangeWatchFrontBuffer;
    const double ego_min_l = std::min(adc_sl_boundary.start_l(), adc_sl_boundary.end_l());
    const double ego_max_l = std::max(adc_sl_boundary.start_l(), adc_sl_boundary.end_l());

    for (const auto* obstacle : reference_line_info->path_decision()->obstacles().Items()) {
        if (obstacle == nullptr || obstacle->IsVirtual() || obstacle->IsStatic()) {
            continue;
        }

        double obs_start_s = std::numeric_limits<double>::max();
        double obs_end_s = -std::numeric_limits<double>::max();
        double obs_min_l = std::numeric_limits<double>::max();
        double obs_max_l = -std::numeric_limits<double>::max();

        for (const auto& point : obstacle->PerceptionPolygon().points()) {
            apollo::common::SLPoint sl_point;
            reference_line_info->reference_line().XYToSL(point, &sl_point);
            obs_start_s = std::fmin(obs_start_s, sl_point.s());
            obs_end_s = std::fmax(obs_end_s, sl_point.s());
            obs_min_l = std::fmin(obs_min_l, sl_point.l());
            obs_max_l = std::fmax(obs_max_l, sl_point.l());
        }

        if (obs_end_s < check_start_s || obs_start_s > check_end_s) {
            continue;
        }

        const double lateral_gap = std::max(ego_min_l - obs_max_l, obs_min_l - ego_max_l);
        ADEBUG << "[LC_CLEAR] obs=" << obstacle->Id() << " ego_l=[" << ego_min_l << "," << ego_max_l
               << "] obs_l=[" << obs_min_l << "," << obs_max_l << "] gap=" << lateral_gap
               << " s=[" << check_start_s << "," << check_end_s << "]";
        if (lateral_gap < kLaneChangeWatchLateralBuffer) {
            ADEBUG << "Lane change blocked: lateral_gap=" << lateral_gap << " obstacle=" << obstacle->Id();
            return false;
        }
    }
    return true;
}

bool ShouldHoldLateralInContestLaneChange(bool is_contest_lane_change, bool is_clear_to_change_lane) {
    return is_contest_lane_change && !is_clear_to_change_lane;
}

void ApplyContestLaneChangeHoldBoundary(double current_l, PathBoundary* path_bound) {
    if (path_bound == nullptr) {
        return;
    }
    ADEBUG << "[LC_BOUNDS] HOLD current_l=" << current_l;
    for (auto& point : *path_bound) {
        point.l_lower.l = std::max(point.l_lower.l, current_l - kLaneChangeHoldLateralHalfWidth);
        point.l_upper.l = std::min(point.l_upper.l, current_l + kLaneChangeHoldLateralHalfWidth);
        if (point.l_lower.l >= point.l_upper.l) {
            point.l_lower.l = current_l - kLaneChangeHoldLateralHalfWidth;
            point.l_upper.l = current_l + kLaneChangeHoldLateralHalfWidth;
        }
    }
}

void ApplyContestLaneChangeSpeedLimit(bool is_contest_lane_change, ReferenceLineInfo* reference_line_info) {
    if (!is_contest_lane_change || reference_line_info == nullptr) {
        return;
    }
    reference_line_info->mutable_reference_line()->AddSpeedLimit(
            reference_line_info->AdcSlBoundary().start_s(),
            reference_line_info->reference_line().Length(),
            kLaneChangeSpeedLimit);
}

void IgnoreObstaclesForContestLaneChange(bool is_contest_lane_change, ReferenceLineInfo* reference_line_info) {
    if (!is_contest_lane_change || reference_line_info == nullptr) {
        return;
    }
    for (auto& obstacle : reference_line_info->path_decision()->obstacles().Items()) {
        if (obstacle == nullptr || obstacle->IsVirtual()) {
            continue;
        }
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        reference_line_info->path_decision()->AddLongitudinalDecision(
                "LaneChangePath/ignore-all", obstacle->Id(), object_decision);
        reference_line_info->path_decision()->AddLateralDecision(
                "LaneChangePath/ignore-all", obstacle->Id(), object_decision);
    }
}

}  // namespace planning
}  // namespace apollo
