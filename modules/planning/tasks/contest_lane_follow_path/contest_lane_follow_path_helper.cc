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

#include "modules/planning/tasks/contest_lane_follow_path/contest_lane_follow_path_helper.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planning_base/common/contest_scenario_features.h"
#include "modules/planning/planning_base/common/contest_scenario_status.h"

namespace apollo {
namespace planning {
namespace {

constexpr double kDenseConeSCurveObstacleLatBuffer = 0.80;
constexpr double kLaneChangeRearBuffer = 1.0;
constexpr double kLaneChangeMinLateralGap = 0.2;
constexpr double kLaneChangeHoldHalfWidth = 0.5;
constexpr double kLaneChangeClearWindowLateralExpansion = 4.0;
constexpr double kUTurnLateralExpansion = 3.0;
constexpr double kHighKappaForJerk = 0.12;
constexpr double kJerkRelaxFactor = 2.5;
constexpr double kUTurnRefL = -0.8;
constexpr double kUTurnRefLWeightFactor = 10.0;
constexpr double kHighCurvRefLWeightBoost = 30.0;

bool HasDenseConeNudgeBound(const PathBoundPoint& point) {
    return point.l_lower.type == BoundType::OBSTACLE || point.l_upper.type == BoundType::OBSTACLE
            || point.is_nudge_bound[LEFT_INDEX] || point.is_nudge_bound[RIGHT_INDEX];
}

void UpdateDenseConeSCurvePathRef(
        const PathBoundary& path_boundary,
        double weight,
        std::vector<double>* ref_l,
        std::vector<double>* weight_ref_l) {
    if (ref_l == nullptr || weight_ref_l == nullptr || ref_l->size() != path_boundary.size()
        || weight_ref_l->size() != path_boundary.size()) {
        return;
    }

    int updated_count = 0;
    for (size_t i = 0; i < path_boundary.size(); ++i) {
        const auto& point = path_boundary[i];
        if (!HasDenseConeNudgeBound(point) || point.l_lower.l > point.l_upper.l) {
            continue;
        }
        ref_l->at(i) = 0.5 * (point.l_lower.l + point.l_upper.l);
        weight_ref_l->at(i) = std::max(weight_ref_l->at(i), weight);
        ++updated_count;
    }
    if (updated_count > 0) {
        ADEBUG << "Dense cone S-curve path ref updated with nudge bounds, count[" << updated_count << "]";
    }
}

}  // namespace

ContestLaneFollowPathContext BuildContestLaneFollowPathContext(
        const std::shared_ptr<DependencyInjector>& injector,
        const ReferenceLineInfo& reference_line_info) {
    ContestLaneFollowPathContext context;
    context.lane_change = contest::IsCurrentScenario(injector, contest::kLaneChangeScenario);
    context.dense_s_curve = contest::IsCurrentScenario(injector, contest::kSCurveScenario)
            && contest::IsDefaultDenseConeSCurve(reference_line_info);
    context.u_turn = contest::IsCurrentScenario(injector, contest::kUTurnScenario);
    return context;
}

void ApplyContestLaneChangeBoundaryOverride(
        const ReferenceLineInfo& reference_line_info,
        const SLState& init_sl_state,
        PathBoundary* path_bound) {
    if (path_bound == nullptr || path_bound->empty()) {
        return;
    }

    bool should_stay = false;
    const double ego_start_s = reference_line_info.AdcSlBoundary().start_s();
    const double ego_end_s = reference_line_info.AdcSlBoundary().end_s();
    const double ego_start_l = reference_line_info.AdcSlBoundary().start_l();
    const double ego_end_l = reference_line_info.AdcSlBoundary().end_l();

    for (const auto* obstacle : reference_line_info.path_decision().obstacles().Items()) {
        if (obstacle == nullptr || obstacle->IsVirtual() || obstacle->IsStatic()) {
            continue;
        }
        double obs_start_s = std::numeric_limits<double>::max();
        double obs_end_s = -std::numeric_limits<double>::max();
        double obs_min_l = std::numeric_limits<double>::max();
        double obs_max_l = -std::numeric_limits<double>::max();
        for (const auto& point : obstacle->PerceptionPolygon().points()) {
            apollo::common::SLPoint sl_point;
            reference_line_info.reference_line().XYToSL(point, &sl_point);
            obs_start_s = std::fmin(obs_start_s, sl_point.s());
            obs_end_s = std::fmax(obs_end_s, sl_point.s());
            obs_min_l = std::fmin(obs_min_l, sl_point.l());
            obs_max_l = std::fmax(obs_max_l, sl_point.l());
        }
        if (obs_end_s < ego_start_s - kLaneChangeRearBuffer || obs_start_s > ego_end_s) {
            continue;
        }
        const double lateral_gap = std::max(ego_start_l - obs_max_l, obs_min_l - ego_end_l);
        if (lateral_gap < kLaneChangeMinLateralGap) {
            should_stay = true;
            break;
        }
    }

    if (should_stay) {
        const double current_l = init_sl_state.second[0];
        for (auto& point : *path_bound) {
            point.l_lower.l = std::min(point.l_lower.l, current_l - kLaneChangeHoldHalfWidth);
            point.l_upper.l = std::max(point.l_upper.l, current_l + kLaneChangeHoldHalfWidth);
            if (point.l_lower.l >= point.l_upper.l) {
                point.l_lower.l = current_l - kLaneChangeHoldHalfWidth;
                point.l_upper.l = current_l + kLaneChangeHoldHalfWidth;
            }
        }
        return;
    }

    for (auto& point : *path_bound) {
        point.l_lower.l -= kLaneChangeClearWindowLateralExpansion;
        point.l_upper.l += kLaneChangeClearWindowLateralExpansion;
    }
}

void ApplyContestUTurnBoundaryExpansion(const ContestLaneFollowPathContext& context, PathBoundary* path_bound) {
    if (path_bound == nullptr || !context.u_turn) {
        return;
    }
    for (auto& point : *path_bound) {
        point.l_lower.l -= kUTurnLateralExpansion;
        point.l_upper.l += kUTurnLateralExpansion;
    }
}

void ApplyDenseConeSCurveBoundaryTuning(
        const ContestLaneFollowPathContext& context,
        double* obstacle_lat_buffer,
        bool* enable_adc_vertex_constraint) {
    if (!context.dense_s_curve || obstacle_lat_buffer == nullptr || enable_adc_vertex_constraint == nullptr) {
        return;
    }
    *obstacle_lat_buffer = std::max(*obstacle_lat_buffer, kDenseConeSCurveObstacleLatBuffer);
    *enable_adc_vertex_constraint = true;
    ADEBUG << "Dense cone S-curve detected, obstacle_lat_buffer set to " << *obstacle_lat_buffer
           << ", adc vertex constraint enabled";
}

double AdjustJerkBoundaryForContestLaneFollow(
        const ReferenceLine& reference_line,
        const PathBoundary& path_boundary,
        double jerk_bound) {
    for (size_t i = 0; i < path_boundary.size(); ++i) {
        const double s = static_cast<double>(i) * path_boundary.delta_s() + path_boundary.start_s();
        if (std::abs(reference_line.GetNearestReferencePoint(s).kappa()) > kHighKappaForJerk) {
            return jerk_bound * kJerkRelaxFactor;
        }
    }
    return jerk_bound;
}

void ApplyContestLaneFollowPathReference(
        const ContestLaneFollowPathContext& context,
        const PathBoundary& path_boundary,
        double path_reference_l_weight,
        double jerk_bound,
        double final_jerk_bound,
        std::vector<double>* ref_l,
        std::vector<double>* weight_ref_l) {
    if (ref_l == nullptr || weight_ref_l == nullptr) {
        return;
    }
    if (context.dense_s_curve) {
        UpdateDenseConeSCurvePathRef(path_boundary, path_reference_l_weight, ref_l, weight_ref_l);
    }
    if (context.u_turn) {
        // U 弯模式：不强拉向固定 l=-0.8，改用 corridor 中点 + 零参考权重
        // 让优化器在扩展后的宽边界内自由寻找平滑曲线
        for (size_t i = 0; i < ref_l->size(); ++i) {
            const double corridor_center = (path_boundary[i].l_lower.l + path_boundary[i].l_upper.l) * 0.5;
            ref_l->at(i) = corridor_center;
            weight_ref_l->at(i) = 0.0;
        }
    } else if (final_jerk_bound > jerk_bound * 1.01) {
        for (double& weight : *weight_ref_l) {
            weight *= kHighCurvRefLWeightBoost;
        }
    }
}

}  // namespace planning
}  // namespace apollo
