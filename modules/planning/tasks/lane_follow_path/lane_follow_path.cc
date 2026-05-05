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

#include "modules/planning/tasks/lane_follow_path/lane_follow_path.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/planning_msgs/decision.pb.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"
namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

namespace {

constexpr double kUTurnLookAheadS = 40.0;
constexpr double kUTurnSampleStep = 1.0;
constexpr double kUTurnKappaThreshold = 0.12;
constexpr double kUTurnHeadingChangeThreshold = 1.35;
constexpr double kUTurnGateHeadingChange = 1.52;
constexpr double kUTurnDefaultGateS = 12.0;
constexpr double kUTurnConflictLookAheadAfterGateS = 4.0;
constexpr double kUTurnConflictBeforeGateS = 0.2;
constexpr double kUTurnConflictLatRange = 5.5;
constexpr double kUTurnConflictCoreLatRange = 1.0;
constexpr double kUTurnConflictMidLatRange = 0.6;
constexpr double kUTurnConflictMinCoreOverlap = 1.6;
constexpr double kUTurnConflictMaxLatSpan = 8.0;
constexpr double kUTurnConflictMaxSSpan = 18.0;
constexpr double kUTurnVehicleLikeMinArea = 4.0;
constexpr double kUTurnStopDistance = 0.4;
constexpr double kUTurnStopWallExtraS = 8.5;
constexpr double kUTurnMinStopAheadS = 2.5;
constexpr double kDenseConeSCurveObstacleLatBuffer = 0.80;

double ClampReferenceS(const ReferenceLine& reference_line, const double s) {
    return std::max(0.0, std::min(s, std::max(0.0, reference_line.Length() - 0.1)));
}

bool IsVehicleLikeObstacle(const Obstacle& obstacle) {
    return obstacle.Perception().type() == apollo::perception::PerceptionObstacle::VEHICLE
            || obstacle.PerceptionPolygon().area() >= kUTurnVehicleLikeMinArea;
}

bool IsUTurnLikeReferenceLine(const ReferenceLineInfo& reference_line_info, double* gate_s) {
    const auto& reference_line = reference_line_info.reference_line();
    if (reference_line.Length() < 1.0) {
        return false;
    }

    const double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
    const double sample_start_s = ClampReferenceS(reference_line, adc_start_s);
    const double sample_end_s = ClampReferenceS(reference_line, adc_start_s + kUTurnLookAheadS);
    if (sample_end_s - sample_start_s < 6.0) {
        const bool has_short_uturn_tag = reference_line_info.GetPathTurnType(sample_start_s) == hdmap::Lane::U_TURN
                || reference_line_info.GetPathTurnType(
                           ClampReferenceS(reference_line, reference_line_info.AdcSlBoundary().end_s()))
                        == hdmap::Lane::U_TURN;
        if (has_short_uturn_tag && gate_s != nullptr) {
            *gate_s = sample_end_s;
        }
        if (has_short_uturn_tag) {
            return true;
        }
        return false;
    }

    const double start_heading = reference_line.GetReferencePoint(sample_start_s).heading();
    double max_abs_kappa = 0.0;
    double max_heading_change = 0.0;
    bool has_uturn_lane_tag = false;
    bool gate_found = false;
    double first_gate_s = std::min(sample_start_s + kUTurnDefaultGateS, sample_end_s);

    for (double s = sample_start_s; s <= sample_end_s; s += kUTurnSampleStep) {
        const double checked_s = ClampReferenceS(reference_line, s);
        max_abs_kappa = std::max(max_abs_kappa, std::fabs(reference_line.GetReferencePoint(checked_s).kappa()));
        const double heading_change = std::fabs(
                common::math::NormalizeAngle(reference_line.GetReferencePoint(checked_s).heading() - start_heading));
        max_heading_change = std::max(max_heading_change, heading_change);
        if (!gate_found && heading_change >= kUTurnGateHeadingChange) {
            first_gate_s = checked_s;
            gate_found = true;
        }
        if (reference_line_info.GetPathTurnType(checked_s) == hdmap::Lane::U_TURN) {
            has_uturn_lane_tag = true;
        }
    }

    const bool is_uturn_like = has_uturn_lane_tag
            || (max_abs_kappa > kUTurnKappaThreshold && max_heading_change > kUTurnHeadingChangeThreshold);
    if (is_uturn_like && gate_s != nullptr) {
        *gate_s = first_gate_s;
    }
    return is_uturn_like;
}

int GetUTurnDirection(const ReferenceLineInfo& reference_line_info) {
    const auto& reference_line = reference_line_info.reference_line();
    const double sample_start_s = ClampReferenceS(reference_line, reference_line_info.AdcSlBoundary().start_s());
    const double sample_end_s = ClampReferenceS(reference_line, sample_start_s + kUTurnLookAheadS);
    double kappa_sum = 0.0;
    for (double s = sample_start_s; s <= sample_end_s; s += kUTurnSampleStep) {
        kappa_sum += reference_line.GetReferencePoint(ClampReferenceS(reference_line, s)).kappa();
    }
    if (std::fabs(kappa_sum) > 1e-3) {
        return kappa_sum > 0.0 ? 1 : -1;
    }

    const double heading_change = common::math::NormalizeAngle(
            reference_line.GetReferencePoint(sample_end_s).heading()
            - reference_line.GetReferencePoint(sample_start_s).heading());
    return heading_change >= 0.0 ? 1 : -1;
}

bool BuildUTurnOuterExpandedBoundary(
        const ReferenceLineInfo& reference_line_info,
        const SLState& init_sl_state,
        PathBoundary* const path_bound) {
    PathBoundary self_bound;
    PathBoundary road_bound;
    if (!PathBoundsDeciderUtil::InitPathBoundary(reference_line_info, &self_bound, init_sl_state)
        || !PathBoundsDeciderUtil::GetBoundaryFromSelfLane(reference_line_info, init_sl_state, &self_bound)) {
        AERROR << "Failed to build self boundary for U-turn.";
        return false;
    }
    if (!PathBoundsDeciderUtil::InitPathBoundary(reference_line_info, &road_bound, init_sl_state)
        || !PathBoundsDeciderUtil::GetBoundaryFromRoad(reference_line_info, init_sl_state, &road_bound)) {
        AERROR << "Failed to build road boundary for U-turn.";
        return false;
    }

    const int uturn_direction = GetUTurnDirection(reference_line_info);
    const size_t bound_size = std::min(self_bound.size(), road_bound.size());
    path_bound->clear();
    path_bound->set_delta_s(self_bound.delta_s());
    for (size_t i = 0; i < bound_size; ++i) {
        PathBoundPoint point = self_bound[i];
        if (uturn_direction > 0) {
            point.l_lower = road_bound[i].l_lower;
        } else {
            point.l_upper = road_bound[i].l_upper;
        }
        if (point.l_lower.l > point.l_upper.l) {
            AINFO << "U-turn outer-expanded boundary blocked at s[" << point.s << "] lower[" << point.l_lower.l
                  << "] upper[" << point.l_upper.l << "]";
            break;
        }
        path_bound->push_back(point);
    }

    AINFO << "U-turn outer-expanded boundary direction[" << uturn_direction << "] points[" << path_bound->size() << "]";
    return path_bound->size() > 1;
}

bool IsUTurnWindowObstacle(
        const Obstacle& obstacle,
        const ReferenceLineInfo& reference_line_info,
        const double gate_s) {
    if (obstacle.IsVirtual() || !IsVehicleLikeObstacle(obstacle)) {
        return false;
    }

    const auto& adc_sl = reference_line_info.AdcSlBoundary();
    const auto& obs_sl = obstacle.PerceptionSLBoundary();
    const double conflict_start_s = adc_sl.start_s() - 6.0;
    const double conflict_end_s = std::min(
            reference_line_info.reference_line().Length(),
            std::max(gate_s + kUTurnConflictLookAheadAfterGateS, adc_sl.end_s() + 18.0));
    if (obs_sl.end_s() < conflict_start_s || obs_sl.start_s() > conflict_end_s) {
        return false;
    }
    if (obs_sl.end_l() < -kUTurnConflictLatRange || obs_sl.start_l() > kUTurnConflictLatRange) {
        return false;
    }
    return true;
}

bool IsUTurnConflictObstacle(
        const Obstacle& obstacle,
        const ReferenceLineInfo& reference_line_info,
        const double gate_s) {
    if (!IsUTurnWindowObstacle(obstacle, reference_line_info, gate_s)) {
        return false;
    }

    const auto& adc_sl = reference_line_info.AdcSlBoundary();
    const auto& obs_sl = obstacle.PerceptionSLBoundary();
    const double conflict_start_s = std::max(adc_sl.start_s() - 1.0, gate_s - kUTurnConflictBeforeGateS);
    const double conflict_end_s = std::min(
            reference_line_info.reference_line().Length(), gate_s + kUTurnConflictLookAheadAfterGateS);
    if (obs_sl.end_s() < conflict_start_s || obs_sl.start_s() > conflict_end_s) {
        return false;
    }

    const double obs_s_span = obs_sl.end_s() - obs_sl.start_s();
    const double obs_l_span = obs_sl.end_l() - obs_sl.start_l();
    if (obs_s_span > kUTurnConflictMaxSSpan || obs_l_span > kUTurnConflictMaxLatSpan) {
        AINFO << "Relax U-turn conflict for abnormal obstacle[" << obstacle.Id() << "] s_span[" << obs_s_span
              << "] l_span[" << obs_l_span << "]";
        return false;
    }

    const double obs_mid_l = 0.5 * (obs_sl.start_l() + obs_sl.end_l());
    const double core_overlap_l = std::min(obs_sl.end_l(), kUTurnConflictCoreLatRange)
            - std::max(obs_sl.start_l(), -kUTurnConflictCoreLatRange);
    const bool near_core = core_overlap_l >= kUTurnConflictMinCoreOverlap;
    const bool near_mid = std::fabs(obs_mid_l) <= kUTurnConflictMidLatRange;
    if (!near_core || !near_mid) {
        AINFO << "Relax U-turn conflict for edge obstacle[" << obstacle.Id() << "] mid_l[" << obs_mid_l
              << "] core_overlap[" << core_overlap_l << "] l[" << obs_sl.start_l() << "," << obs_sl.end_l()
              << "]";
        return false;
    }
    return true;
}

bool FindRawUTurnConflict(
        const ReferenceLineInfo& reference_line_info,
        const double gate_s,
        std::string* conflict_obstacle_id) {
    for (const auto* obstacle : reference_line_info.path_decision().obstacles().Items()) {
        if (obstacle == nullptr || !IsUTurnConflictObstacle(*obstacle, reference_line_info, gate_s)) {
            continue;
        }
        if (conflict_obstacle_id != nullptr) {
            *conflict_obstacle_id = obstacle->Id();
        }
        AINFO << "Raw U-turn conflict obstacle[" << obstacle->Id() << "] s["
              << obstacle->PerceptionSLBoundary().start_s() << "," << obstacle->PerceptionSLBoundary().end_s() << "] l["
              << obstacle->PerceptionSLBoundary().start_l() << "," << obstacle->PerceptionSLBoundary().end_l() << "]";
        return true;
    }
    return false;
}

void FilterUTurnPathObstacles(
        const ReferenceLineInfo& reference_line_info,
        const double gate_s,
        std::vector<SLPolygon>* obs_sl_polygons) {
    if (obs_sl_polygons == nullptr || obs_sl_polygons->empty()) {
        return;
    }
    const auto& obstacles = reference_line_info.path_decision().obstacles();
    std::vector<SLPolygon> filtered_polygons;
    filtered_polygons.reserve(obs_sl_polygons->size());
    for (const auto& sl_polygon : *obs_sl_polygons) {
        const auto* obstacle = obstacles.Find(sl_polygon.id());
        if (obstacle != nullptr && IsUTurnWindowObstacle(*obstacle, reference_line_info, gate_s)) {
            AINFO << "Filter U-turn vehicle obstacle from path bound, id[" << sl_polygon.id() << "]";
            continue;
        }
        filtered_polygons.push_back(sl_polygon);
    }
    *obs_sl_polygons = std::move(filtered_polygons);
}

void AddIgnoreDecisionForUTurnConflictObstacles(
        const ReferenceLineInfo& reference_line_info,
        const double gate_s,
        PathDecision* const path_decision) {
    if (path_decision == nullptr) {
        return;
    }
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();
    for (const auto* obstacle : path_decision->obstacles().Items()) {
        if (obstacle == nullptr || !IsUTurnWindowObstacle(*obstacle, reference_line_info, gate_s)) {
            continue;
        }
        path_decision->AddLongitudinalDecision("LaneFollowPath/uturn-window-ignore", obstacle->Id(), object_decision);
        path_decision->AddLateralDecision("LaneFollowPath/uturn-window-ignore", obstacle->Id(), object_decision);
    }
}

void AddUTurnSafeWindowStop(
        Frame* const frame,
        ReferenceLineInfo* const reference_line_info,
        const double gate_s,
        const std::string& conflict_obstacle_id) {
    if (frame == nullptr || reference_line_info == nullptr) {
        return;
    }
    const auto& reference_line = reference_line_info->reference_line();
    const double adc_front_s = reference_line_info->AdcSlBoundary().end_s();
    const auto& vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
    double stop_line_s = gate_s + vehicle_param.front_edge_to_center() + kUTurnStopDistance + kUTurnStopWallExtraS;
    stop_line_s = std::max(stop_line_s, adc_front_s + kUTurnMinStopAheadS);
    stop_line_s = ClampReferenceS(reference_line, stop_line_s);
    if (stop_line_s <= adc_front_s + 0.5 || stop_line_s <= kUTurnStopDistance) {
        return;
    }

    std::vector<std::string> wait_for_obstacles;
    if (!conflict_obstacle_id.empty()) {
        wait_for_obstacles.push_back(conflict_obstacle_id);
    }
    util::BuildStopDecision(
            "UTURN_SAFE_WINDOW",
            stop_line_s,
            kUTurnStopDistance,
            StopReasonCode::STOP_REASON_OBSTACLE,
            wait_for_obstacles,
            "LaneFollowPath/uturn_safe_window",
            frame,
            reference_line_info);
    AINFO << "U-turn safe window is occupied, build stop wall at s[" << stop_line_s << "] conflict["
          << conflict_obstacle_id << "]";
}

bool IsDenseConeSCurve(const ReferenceLineInfo& reference_line_info) {
    constexpr double kLookAheadS = 90.0;
    constexpr double kSmallObstacleArea = 0.5;
    constexpr double kMaxObstacleAbsL = 3.5;
    constexpr int kMinSmallObstacleCount = 4;
    constexpr double kMinAbsKappa = 0.015;

    const auto& reference_line = reference_line_info.reference_line();
    const double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

    double max_abs_kappa = 0.0;
    for (double s = adc_start_s; s < adc_start_s + kLookAheadS && s < reference_line.Length(); s += 2.0) {
        max_abs_kappa = std::max(max_abs_kappa, std::fabs(reference_line.GetReferencePoint(s).kappa()));
    }
    if (max_abs_kappa < kMinAbsKappa) {
        return false;
    }

    int small_obstacle_count = 0;
    for (const auto* obs : reference_line_info.path_decision().obstacles().Items()) {
        if (obs == nullptr || obs->IsVirtual() || obs->PerceptionPolygon().area() >= kSmallObstacleArea) {
            continue;
        }
        const auto& sl_boundary = obs->PerceptionSLBoundary();
        if (sl_boundary.end_s() < adc_start_s - 2.0 || sl_boundary.start_s() > adc_end_s + kLookAheadS) {
            continue;
        }
        const double obstacle_center_l = (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
        if (std::fabs(obstacle_center_l) > kMaxObstacleAbsL) {
            continue;
        }
        ++small_obstacle_count;
        if (small_obstacle_count >= kMinSmallObstacleCount) {
            return true;
        }
    }
    return false;
}

bool HasDenseConeNudgeBound(const PathBoundPoint& point) {
    return point.l_lower.type == BoundType::OBSTACLE || point.l_upper.type == BoundType::OBSTACLE
            || point.is_nudge_bound[LEFT_INDEX] || point.is_nudge_bound[RIGHT_INDEX];
}

void UpdateDenseConeSCurvePathRef(
        const PathBoundary& path_boundary,
        const double weight,
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
        AINFO << "Dense cone S-curve path ref updated with nudge bounds, count[" << updated_count << "]";
    }
}

}  // namespace

bool LaneFollowPath::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!Task::Init(config_dir, name, injector)) {
        return false;
    }
    // Load the config this task.
    return Task::LoadConfig<LaneFollowPathConfig>(&config_);
}

apollo::common::Status LaneFollowPath::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
    // 赛题二：变道 reference_line 上不跑 lane_follow，避免覆盖变道路径
    if (reference_line_info->IsChangeLanePath()) {
        return Status::OK();
    }
    double uturn_gate_s = 0.0;
    const bool is_uturn_like = IsUTurnLikeReferenceLine(*reference_line_info, &uturn_gate_s);
    if (!is_uturn_like) {
        ResetUTurnConflictFilter();
    }
    if (!is_uturn_like && (!reference_line_info->path_data().Empty() || reference_line_info->path_reusable())) {
        ADEBUG << "Skip this time path empty:" << reference_line_info->path_data().Empty()
               << "path reusable: " << reference_line_info->path_reusable();
        return Status::OK();
    }
    std::vector<PathBoundary> candidate_path_boundaries;
    std::vector<PathData> candidate_path_data;

    GetStartPointSLState();
    if (!DecidePathBounds(&candidate_path_boundaries)) {
        AERROR << "Decide path bound failed";
        return Status::OK();
    }
    if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
        AERROR << "Optmize path failed";
        return Status::OK();
    }
    if (!AssessPath(&candidate_path_data, reference_line_info->mutable_path_data())) {
        AERROR << "Path assessment failed";
        return Status::OK();
    }

    if (is_uturn_like) {
        std::string raw_conflict_obstacle_id;
        const bool raw_conflict = FindRawUTurnConflict(*reference_line_info, uturn_gate_s, &raw_conflict_obstacle_id);
        const bool filtered_conflict = UpdateUTurnConflictFilter(raw_conflict, raw_conflict_obstacle_id);
        AddIgnoreDecisionForUTurnConflictObstacles(
                *reference_line_info, uturn_gate_s, reference_line_info->path_decision());
        if (filtered_conflict) {
            AddUTurnSafeWindowStop(frame, reference_line_info, uturn_gate_s, uturn_conflict_obstacle_id_);
        } else {
            AINFO << "U-turn safe window clear, gate_s[" << uturn_gate_s << "]";
        }
    }

    return Status::OK();
}

bool LaneFollowPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
    boundary->emplace_back();
    auto& path_bound = boundary->back();
    std::string blocking_obstacle_id = "";
    std::string lane_type = "";
    double path_narrowest_width = 0;
    // 1. Initialize the path boundaries to be an indefinitely large area.
    if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_)) {
        const std::string msg = "Failed to initialize path boundaries.";
        AERROR << msg;
        return false;
    }
    std::string borrow_lane_type;
    bool is_include_adc = config_.is_extend_lane_bounds_to_include_adc()
            && !injector_->planning_context()->planning_status().path_decider().is_in_path_lane_borrow_scenario();
    double uturn_gate_s = 0.0;
    const bool is_uturn_like = IsUTurnLikeReferenceLine(*reference_line_info_, &uturn_gate_s);
    // 2. Decide a rough boundary based on lane info and ADC's position
    if (is_uturn_like) {
        if (!BuildUTurnOuterExpandedBoundary(*reference_line_info_, init_sl_state_, &path_bound)) {
            AERROR << "Failed to decide U-turn outer-expanded boundary.";
            return false;
        }
        AINFO << "U-turn-like lane follow path uses outer-expanded boundary, gate_s[" << uturn_gate_s << "]";
    } else {
        if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(*reference_line_info_, init_sl_state_, &path_bound)) {
            AERROR << "Failed to decide a rough boundary based on self lane.";
            return false;
        }
    }
    // 赛题二：变道场景下判断是否清晰，不清时锁横位，清晰时正常变道
    if (!is_uturn_like && frame_->reference_line_info().size() > 1) {
        auto indexed_obstacles = reference_line_info_->path_decision()->obstacles();
        bool should_stay = false;
        double es = reference_line_info_->AdcSlBoundary().start_s();
        double ee = reference_line_info_->AdcSlBoundary().end_s();
        double elo = reference_line_info_->AdcSlBoundary().start_l();
        double ehi = reference_line_info_->AdcSlBoundary().end_l();
        constexpr double kRearBuf = 1.0, kMinGap = 0.2;
        for (const auto* obs : indexed_obstacles.Items()) {
            if (obs->IsVirtual() || obs->IsStatic())
                continue;
            double os = 1e9, oe = -1e9, ol = 1e9, oh = -1e9;
            for (const auto& p : obs->PerceptionPolygon().points()) {
                apollo::common::SLPoint sl;
                reference_line_info_->reference_line().XYToSL(p, &sl);
                os = std::fmin(os, sl.s());
                oe = std::fmax(oe, sl.s());
                ol = std::fmin(ol, sl.l());
                oh = std::fmax(oh, sl.l());
            }
            if (oe < es - kRearBuf || os > ee)
                continue;
            double gap = std::max(elo - oh, ol - ehi);
            if (gap < kMinGap) {
                should_stay = true;
                break;
            }
        }
        if (should_stay) {
            double current_l = init_sl_state_.second[0];
            for (size_t i = 0; i < path_bound.size(); ++i) {
                double& lo = path_bound[i].l_lower.l;
                double& hi = path_bound[i].l_upper.l;
                if (lo > current_l - 0.5)
                    lo = current_l - 0.5;
                if (hi < current_l + 0.5)
                    hi = current_l + 0.5;
                if (lo >= hi) {
                    lo = current_l - 0.5;
                    hi = current_l + 0.5;
                }
            }
        } else {
            // 清晰时扩宽边界覆盖邻车道，横移速度放开
            for (size_t i = 0; i < path_bound.size(); ++i) {
                path_bound[i].l_lower.l -= 4.0;
                path_bound[i].l_upper.l += 4.0;
            }
        }
    }
    if (is_include_adc) {
        PathBoundsDeciderUtil::ExtendBoundaryByADC(
                *reference_line_info_, init_sl_state_, config_.extend_buffer(), &path_bound);
    }
    PrintCurves print_curve;
    auto indexed_obstacles = reference_line_info_->path_decision()->obstacles();
    for (const auto* obs : indexed_obstacles.Items()) {
        const auto& sl_bound = obs->PerceptionSLBoundary();
        for (int i = 0; i < sl_bound.boundary_point_size(); i++) {
            std::string name = obs->Id() + "_obs_sl_boundary";
            print_curve.AddPoint(name, sl_bound.boundary_point(i).s(), sl_bound.boundary_point(i).l());
        }
    }
    print_curve.PrintToLog();
    path_bound.set_label(absl::StrCat("regular/", "self"));
    // 3. Fine-tune the boundary based on static obstacles
    PathBound temp_path_bound = path_bound;
    std::vector<SLPolygon> obs_sl_polygons;
    const double saved_obstacle_lat_buffer = FLAGS_obstacle_lat_buffer;
    const bool saved_enable_adc_vertex_constraint = FLAGS_enable_adc_vertex_constraint;
    const bool dense_cone_s_curve = IsDenseConeSCurve(*reference_line_info_);
    if (dense_cone_s_curve) {
        FLAGS_obstacle_lat_buffer
                = std::max(FLAGS_obstacle_lat_buffer, kDenseConeSCurveObstacleLatBuffer);
        FLAGS_enable_adc_vertex_constraint = true;
        AINFO << "Dense cone S-curve detected, obstacle_lat_buffer set to " << FLAGS_obstacle_lat_buffer
              << ", adc vertex constraint enabled";
    }
    PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons, init_sl_state_, dense_cone_s_curve);
    if (is_uturn_like) {
        FilterUTurnPathObstacles(*reference_line_info_, uturn_gate_s, &obs_sl_polygons);
    }
    const bool static_obstacle_boundary_success = PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
            *reference_line_info_,
            &obs_sl_polygons,
            init_sl_state_,
            &path_bound,
            &blocking_obstacle_id,
            &path_narrowest_width);
    FLAGS_obstacle_lat_buffer = saved_obstacle_lat_buffer;
    FLAGS_enable_adc_vertex_constraint = saved_enable_adc_vertex_constraint;
    if (!static_obstacle_boundary_success) {
        const std::string msg
                = "Failed to decide fine tune the boundaries after "
                  "taking into consideration all static obstacles.";
        AERROR << msg;
        return false;
    }
    // 4. Append some extra path bound points to avoid zero-length path data.
    int counter = 0;
    while (!blocking_obstacle_id.empty() && path_bound.size() < temp_path_bound.size()
           && counter < FLAGS_num_extra_tail_bound_point) {
        path_bound.push_back(temp_path_bound[path_bound.size()]);
        counter++;
    }

    // lane_follow_status update
    auto* lane_follow_status = injector_->planning_context()->mutable_planning_status()->mutable_lane_follow();
    if (!blocking_obstacle_id.empty()) {
        double current_time = ::apollo::cyber::Clock::NowInSeconds();
        lane_follow_status->set_block_obstacle_id(blocking_obstacle_id);
        if (lane_follow_status->lane_follow_block()) {
            lane_follow_status->set_block_duration(
                    lane_follow_status->block_duration() + current_time - lane_follow_status->last_block_timestamp());
        } else {
            lane_follow_status->set_block_duration(0);
            lane_follow_status->set_lane_follow_block(true);
        }
        lane_follow_status->set_last_block_timestamp(current_time);
    } else {
        if (lane_follow_status->lane_follow_block()) {
            lane_follow_status->set_block_duration(0);
            lane_follow_status->set_lane_follow_block(false);
            lane_follow_status->set_last_block_timestamp(0);
        }
    }

    ADEBUG << "Completed generating path boundaries.";
    if (is_uturn_like) {
        constexpr double kUTurnInitBoundaryTolerance = 0.08;
        const double init_l = init_sl_state_.second[0];
        if (init_l > path_bound[0].l_upper.l && init_l - path_bound[0].l_upper.l <= kUTurnInitBoundaryTolerance) {
            path_bound[0].l_upper.l = init_l;
        }
        if (init_l < path_bound[0].l_lower.l && path_bound[0].l_lower.l - init_l <= kUTurnInitBoundaryTolerance) {
            path_bound[0].l_lower.l = init_l;
        }
    }
    if (init_sl_state_.second[0] > path_bound[0].l_upper.l || init_sl_state_.second[0] < path_bound[0].l_lower.l) {
        AINFO << "not in self lane maybe lane borrow or out of road. init l : " << init_sl_state_.second[0]
              << ", path_bound l: [ " << path_bound[0].l_lower.l << "," << path_bound[0].l_upper.l << " ]";
        return false;
    }
    // std::vector<std::pair<double, double>> regular_path_bound_pair;
    // for (size_t i = 0; i < path_bound.size(); ++i) {
    //   regular_path_bound_pair.emplace_back(std::get<1>(path_bound[i]),
    //                                        std::get<2>(path_bound[i]));
    // }
    path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
    RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
    return true;
}

bool LaneFollowPath::OptimizePath(
        const std::vector<PathBoundary>& path_boundaries,
        std::vector<PathData>* candidate_path_data) {
    const auto& config = config_.path_optimizer_config();
    const ReferenceLine& reference_line = reference_line_info_->reference_line();
    std::array<double, 3> end_state = {0.0, 0.0, 0.0};
    const bool dense_cone_s_curve = IsDenseConeSCurve(*reference_line_info_);
    for (const auto& path_boundary : path_boundaries) {
        size_t path_boundary_size = path_boundary.boundary().size();
        if (path_boundary_size <= 1U) {
            AERROR << "Get invalid path boundary with size: " << path_boundary_size;
            return false;
        }
        std::vector<double> opt_l, opt_dl, opt_ddl;
        std::vector<std::pair<double, double>> ddl_bounds;
        PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line, &ddl_bounds);
        PrintCurves print_debug;
        for (size_t i = 0; i < path_boundary_size; ++i) {
            double s = static_cast<double>(i) * path_boundary.delta_s() + path_boundary.start_s();
            double kappa = reference_line.GetNearestReferencePoint(s).kappa();
            print_debug.AddPoint("ref_kappa", s, kappa);
        }
        print_debug.PrintToLog();
        const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(std::fmax(init_sl_state_.first[1], 1e-12));
        std::vector<double> ref_l(path_boundary_size, 0);
        std::vector<double> weight_ref_l(path_boundary_size, 0);

        PathOptimizerUtil::UpdatePathRefWithBound(
                path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);
        if (dense_cone_s_curve) {
            UpdateDenseConeSCurvePathRef(path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);
        }
        bool res_opt = PathOptimizerUtil::OptimizePath(
                init_sl_state_,
                end_state,
                ref_l,
                weight_ref_l,
                path_boundary,
                ddl_bounds,
                jerk_bound,
                config,
                &opt_l,
                &opt_dl,
                &opt_ddl);
        if (res_opt) {
            auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
                    opt_l, opt_dl, opt_ddl, path_boundary.delta_s(), path_boundary.start_s());
            PathData path_data;
            path_data.SetReferenceLine(&reference_line);
            path_data.SetFrenetPath(std::move(frenet_frame_path));
            if (FLAGS_use_front_axe_center_in_path_planning) {
                auto discretized_path
                        = DiscretizedPath(PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(path_data));
                path_data.SetDiscretizedPath(discretized_path);
            }
            path_data.set_path_label(path_boundary.label());
            path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
            candidate_path_data->push_back(std::move(path_data));
            PrintCurves print_path_kappa;
            for (const auto& p : candidate_path_data->back().discretized_path()) {
                print_path_kappa.AddPoint(
                        path_boundary.label() + "_path_kappa", p.s() + init_sl_state_.first[0], p.kappa());
            }
            print_path_kappa.PrintToLog();
        }
    }
    if (candidate_path_data->empty()) {
        return false;
    }
    return true;
}

bool LaneFollowPath::AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path) {
    PathData& curr_path_data = candidate_path_data->back();
    RecordDebugInfo(curr_path_data, curr_path_data.path_label(), reference_line_info_);
    if (!PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_, curr_path_data)) {
        AINFO << "Lane follow path is invalid";
        return false;
    }

    std::vector<PathPointDecision> path_decision;
    PathAssessmentDeciderUtil::InitPathPointDecision(curr_path_data, PathData::PathPointType::IN_LANE, &path_decision);
    curr_path_data.SetPathPointDecisionGuide(std::move(path_decision));

    if (curr_path_data.Empty()) {
        AINFO << "Lane follow path is empty after trimed";
        return false;
    }
    *final_path = curr_path_data;
    AINFO << final_path->path_label() << final_path->blocking_obstacle_id();
    reference_line_info_->MutableCandidatePathData()->push_back(*final_path);
    reference_line_info_->SetBlockingObstacle(curr_path_data.blocking_obstacle_id());
    return true;
}

bool LaneFollowPath::UpdateUTurnConflictFilter(const bool raw_conflict, const std::string& conflict_obstacle_id) {
    constexpr int kSeenCyclesToLatch = 8;
    constexpr int kClearCyclesToRelease = 3;
    constexpr int kMaxFilterCycles = 20;

    if (raw_conflict) {
        uturn_conflict_seen_count_ = std::min(uturn_conflict_seen_count_ + 1, kMaxFilterCycles);
        uturn_conflict_clear_count_ = 0;
        if (!conflict_obstacle_id.empty()) {
            uturn_conflict_obstacle_id_ = conflict_obstacle_id;
        }
        if (uturn_conflict_seen_count_ >= kSeenCyclesToLatch) {
            uturn_conflict_latched_ = true;
        }
    } else {
        uturn_conflict_seen_count_ = 0;
        uturn_conflict_clear_count_ = std::min(uturn_conflict_clear_count_ + 1, kMaxFilterCycles);
        if (uturn_conflict_clear_count_ >= kClearCyclesToRelease) {
            uturn_conflict_latched_ = false;
            uturn_conflict_obstacle_id_.clear();
        }
    }

    AINFO << "U-turn conflict filter raw[" << raw_conflict << "] latched[" << uturn_conflict_latched_ << "] seen["
          << uturn_conflict_seen_count_ << "] clear[" << uturn_conflict_clear_count_ << "] obs["
          << uturn_conflict_obstacle_id_ << "]";
    return uturn_conflict_latched_;
}

void LaneFollowPath::ResetUTurnConflictFilter() {
    uturn_conflict_seen_count_ = 0;
    uturn_conflict_clear_count_ = 0;
    uturn_conflict_latched_ = false;
    uturn_conflict_obstacle_id_.clear();
}

}  // namespace planning
}  // namespace apollo
