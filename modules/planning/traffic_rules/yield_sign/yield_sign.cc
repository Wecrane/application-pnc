/*****************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include "modules/planning/traffic_rules/yield_sign/yield_sign.h"

#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

namespace {

constexpr char kContestRoundaboutScenarioName[] = "CONTEST_ROUNDABOUT";
constexpr double kRoundaboutTargetLaneMaxCenterL = 4.5;
constexpr double kRoundaboutCommitLookForward = 6.0;
constexpr double kRoundaboutCommitLookBack = 3.0;
constexpr double kUTurnYieldMaxDistanceToRefEnd = 35.0;
constexpr double kUTurnYieldLookBackDistance = 8.0;
constexpr double kUTurnYieldLookAheadDistance = 24.0;
constexpr double kUTurnYieldSampleStep = 1.0;
constexpr double kUTurnYieldKappaThreshold = 0.12;
constexpr double kUTurnYieldHeadingChangeThreshold = 1.7;
constexpr double kUTurnYieldMinSampleLength = 8.0;

bool HasUTurnLaneTag(const ReferenceLineInfo& reference_line_info, const PathOverlap& yield_sign_overlap) {
    const double reference_line_length = reference_line_info.reference_line().Length();
    const double start_s = std::max(0.0, yield_sign_overlap.start_s - kUTurnYieldLookBackDistance);
    const double end_s = std::min(reference_line_length, yield_sign_overlap.start_s + kUTurnYieldLookAheadDistance);
    for (double s = start_s; s <= end_s; s += kUTurnYieldSampleStep) {
        if (reference_line_info.GetPathTurnType(s) == hdmap::Lane::U_TURN) {
            return true;
        }
    }
    return reference_line_info.GetPathTurnType(yield_sign_overlap.start_s) == hdmap::Lane::U_TURN;
}

bool HasTightUTurnGeometry(const ReferenceLineInfo& reference_line_info, const PathOverlap& yield_sign_overlap) {
    const auto& reference_line = reference_line_info.reference_line();
    const double reference_line_length = reference_line.Length();
    if (reference_line_length - yield_sign_overlap.start_s > kUTurnYieldMaxDistanceToRefEnd) {
        return false;
    }

    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double sample_start_s
            = std::max(0.0, std::min(adc_front_edge_s, yield_sign_overlap.start_s) - kUTurnYieldLookBackDistance);
    const double sample_end_s = std::min(
            reference_line_length,
            std::max(
                    adc_front_edge_s + kUTurnYieldLookAheadDistance,
                    yield_sign_overlap.start_s + kUTurnYieldLookAheadDistance));
    if (sample_end_s - sample_start_s < kUTurnYieldMinSampleLength) {
        return false;
    }

    double max_abs_kappa = 0.0;
    for (double s = sample_start_s; s <= sample_end_s; s += kUTurnYieldSampleStep) {
        max_abs_kappa = std::max(max_abs_kappa, std::abs(reference_line.GetReferencePoint(s).kappa()));
    }
    const double heading_change = std::abs(
            common::math::NormalizeAngle(
                    reference_line.GetReferencePoint(sample_end_s).heading()
                    - reference_line.GetReferencePoint(sample_start_s).heading()));
    return max_abs_kappa > kUTurnYieldKappaThreshold && heading_change > kUTurnYieldHeadingChangeThreshold;
}

bool IsYieldSignOnUTurnPath(const ReferenceLineInfo& reference_line_info, const PathOverlap& yield_sign_overlap) {
    return HasUTurnLaneTag(reference_line_info, yield_sign_overlap)
            || HasTightUTurnGeometry(reference_line_info, yield_sign_overlap);
}

bool IsContestRoundaboutScenario(const std::shared_ptr<DependencyInjector>& injector) {
    return injector != nullptr && injector->planning_context() != nullptr
            && injector->planning_context()->planning_status().scenario().scenario_type()
            == kContestRoundaboutScenarioName;
}

bool IsRoundaboutNonTargetLaneVehicle(const Obstacle* obstacle) {
    if (obstacle == nullptr || obstacle->IsVirtual() || obstacle->IsStatic()
        || obstacle->Perception().type() != apollo::perception::PerceptionObstacle::VEHICLE) {
        return false;
    }
    const auto& sl = obstacle->PerceptionSLBoundary();
    const double center_l = 0.5 * (sl.start_l() + sl.end_l());
    return std::fabs(center_l) > kRoundaboutTargetLaneMaxCenterL;
}

bool IsRoundaboutLaunchCommitted(
        const std::shared_ptr<DependencyInjector>& injector,
        const ReferenceLineInfo& reference_line_info) {
    // 场景活跃 = 已进入环岛区域 = 已提交
    return IsContestRoundaboutScenario(injector);
}

std::vector<std::string> FilterRoundaboutWaitForObstacles(
        const std::vector<std::string>& wait_for_obstacle_ids,
        const ReferenceLineInfo& reference_line_info) {
    std::vector<std::string> filtered;
    for (const auto& obstacle_id : wait_for_obstacle_ids) {
        const auto* obstacle = reference_line_info.path_decision().obstacles().Find(obstacle_id);
        if (IsRoundaboutNonTargetLaneVehicle(obstacle)) {
            AINFO << "[ROUNDABOUT][YieldSign] drop non-target-lane wait_for obs=" << obstacle_id;
            continue;
        }
        filtered.push_back(obstacle_id);
    }
    return filtered;
}

}  // namespace

bool YieldSign::Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) {
    if (!TrafficRule::Init(name, injector)) {
        return false;
    }
    // Load the config this task.
    return TrafficRule::LoadConfig<YieldSignConfig>(&config_);
}

Status YieldSign::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
    MakeDecisions(frame, reference_line_info);
    return Status::OK();
}

void YieldSign::MakeDecisions(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
    CHECK_NOTNULL(frame);
    CHECK_NOTNULL(reference_line_info);

    if (!config_.enabled()) {
        return;
    }

    const auto& yield_sign_status = injector_->planning_context()->planning_status().yield_sign();
    const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

    const std::vector<PathOverlap>& yield_sign_overlaps
            = reference_line_info->reference_line().map_path().yield_sign_overlaps();
    for (const auto& yield_sign_overlap : yield_sign_overlaps) {
        if (yield_sign_overlap.end_s <= adc_front_edge_s) {
            continue;
        }
        if (IsYieldSignOnUTurnPath(*reference_line_info, yield_sign_overlap)) {
            AINFO << "Skip yield_sign stop wall on U-turn-like yield_sign[" << yield_sign_overlap.object_id
                  << "] start_s[" << yield_sign_overlap.start_s << "]";
            continue;
        }

        // check if yield-sign-stop already finished, set by scenario/stage
        bool yield_sign_done = false;
        for (const auto& done_yield_sign_overlap_id : yield_sign_status.done_yield_sign_overlap_id()) {
            if (yield_sign_overlap.object_id == done_yield_sign_overlap_id) {
                yield_sign_done = true;
                break;
            }
        }
        if (yield_sign_done) {
            continue;
        }

        // build stop decision
        ADEBUG << "BuildStopDecision: yield_sign[" << yield_sign_overlap.object_id << "] start_s["
               << yield_sign_overlap.start_s << "]";
        const std::string virtual_obstacle_id = YIELD_SIGN_VO_ID_PREFIX + yield_sign_overlap.object_id;
        const std::vector<std::string> wait_for_obstacle_ids(
                yield_sign_status.wait_for_obstacle_id().begin(), yield_sign_status.wait_for_obstacle_id().end());
        const bool roundabout_entry = IsContestRoundaboutScenario(injector_);
        if (roundabout_entry && IsRoundaboutLaunchCommitted(injector_, *reference_line_info)) {
            const double adc_x = frame->vehicle_state().x();
            const double adc_y = frame->vehicle_state().y();
            AINFO << "[ROUNDABOUT][YieldSign] skip stop wall after launch commit"
                  << ", adc_x=" << adc_x << ", adc_y=" << adc_y;
            continue;
        }
        const auto filtered_wait_for_obstacle_ids = roundabout_entry
                ? FilterRoundaboutWaitForObstacles(wait_for_obstacle_ids, *reference_line_info)
                : wait_for_obstacle_ids;
        const double stop_distance = roundabout_entry ? 0.5 : config_.stop_distance();
        util::BuildStopDecision(
                virtual_obstacle_id,
                yield_sign_overlap.start_s,
                stop_distance,
                StopReasonCode::STOP_REASON_YIELD_SIGN,
                filtered_wait_for_obstacle_ids,
                Getname(),
                frame,
                reference_line_info);
    }
}

}  // namespace planning
}  // namespace apollo
