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

#include "modules/planning/tasks/lane_borrow_path/lane_borrow_path.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

bool LaneBorrowPath::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!Task::Init(config_dir, name, injector)) {
        return false;
    }
    // Load the config this task.
    zone_left_base = zone_right_base = 0.0;
    return Task::LoadConfig<LaneBorrowPathConfig>(&config_);
}

apollo::common::Status LaneBorrowPath::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
    // 赛题二：变道 reference_line 上不跑 lane_borrow，避免干扰变道
    if (reference_line_info->IsChangeLanePath()) {
        return Status::OK();
    }
    config_.mutable_path_optimizer_config()->set_l_weight(3.0);
    config_.mutable_path_optimizer_config()->set_path_reference_l_weight(10000.0);
    if (!config_.is_allow_lane_borrowing() || reference_line_info->path_reusable()) {
        ADEBUG << "path reusable" << reference_line_info->path_reusable() << ",skip";
        return Status::OK();
    }
    if (!IsNecessaryToBorrowLane()) {
        ADEBUG << "No need to borrow lane";
        return Status::OK();
    }
    std::vector<PathBoundary> candidate_path_boundaries;
    std::vector<PathData> candidate_path_data;

    GetStartPointSLState();
    if (!DecidePathBounds(&candidate_path_boundaries)) {
        return Status::OK();
    }
    if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
        return Status::OK();
    }
    if (HasDynamicVehicleConflictOnBorrowPath(candidate_path_data)) {
        if (GenerateBorrowHoldPath(reference_line_info->mutable_path_data())) {
            AINFO << "Borrow path is temporarily blocked by a dynamic vehicle, hold current lateral position.";
            return Status::OK();
        }
        AWARN << "Failed to generate borrow hold path, continue assessing borrow candidates.";
    }
    if (AssessPath(&candidate_path_data, reference_line_info->mutable_path_data())) {
        ADEBUG << "lane borrow path success";
    }

    // 施工区检测：识别锥桶等小障碍物，设置限速并忽略障碍物
    constexpr double kMinPassengerArea = 0.1;
    constexpr double kSpeedLimit = 8.33;  // 30 km/h
    constexpr double kBuffer = 10.0;

    std::unordered_set<std::string> small_obstacle_ids;

    for (const auto& obs : reference_line_info->path_decision()->obstacles().Items()) {
        if (!obs->IsVirtual() && obs->PerceptionPolygon().area() < kMinPassengerArea) {
            small_obstacle_ids.insert(obs->Id());
        }
    }

    if (construct_zone && !small_obstacle_ids.empty()) {
        double lower_bound = std::numeric_limits<double>::infinity();
        double upper_bound = -std::numeric_limits<double>::infinity();

        for (const auto& obs : reference_line_info->path_decision()->obstacles().Items()) {
            if (small_obstacle_ids.count(obs->Id()) == 0)
                continue;

            const auto& sl = obs->PerceptionSLBoundary();
            lower_bound = std::min(lower_bound, sl.start_s());
            upper_bound = std::max(upper_bound, sl.end_s());
        }

        reference_line_info->mutable_reference_line()->AddSpeedLimit(
                lower_bound - kBuffer, upper_bound + kBuffer, kSpeedLimit);

        reference_line_info->mutable_path_data()->set_path_label("regular/construct_zone");
    }
    if (construct_zone || U_turn_construct) {
        for (auto& obs : reference_line_info->path_decision()->obstacles().Items()) {
            if (obs->IsVirtual())
                continue;
            ObjectDecisionType object_decision;
            object_decision.mutable_ignore();
            reference_line_info->path_decision()->AddLongitudinalDecision(
                    "PathDecider/ignore-backward-obstacle", obs->Id(), object_decision);
        }
    }

    return Status::OK();
}

bool LaneBorrowPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
    U_turn_construct = false;
    double max_kappa = 0.0;
    double curr_s = reference_line_info_->AdcSlBoundary().start_s() - 20.0;
    while (curr_s - reference_line_info_->AdcSlBoundary().start_s() < 100.0) {
        max_kappa = std::max(
                max_kappa, std::fabs(reference_line_info_->reference_line().GetReferencePoint(curr_s).kappa()));
        curr_s += 2.0;
    }
    int cone_cnts = 0;
    for (auto& obs : reference_line_info_->path_decision()->obstacles().Items()) {
        if (!obs->IsVirtual() && obs->PerceptionPolygon().area() < 0.5) {
            if ((obs->PerceptionSLBoundary().end_l() - reference_line_info_->AdcSlBoundary().end_l()) < 7.0)
                cone_cnts++;
        }
    }

    U_turn_construct = max_kappa > 0.001 && cone_cnts > 9;

    // ── 施工区域检测：锥桶位置记忆 ──
    // 锥桶是静态施工设施，闪烁仅因感知不稳定。记录所有见过的锥桶坐标，
    // 只要该坐标仍在主车前方，就持续认为该位置有锥桶。
    constexpr double kConeHistoryCleanupDist = 50.0;  // 清理主车后方 50m 的历史记录

    // 1. 清理主车后方远距离的历史记录，避免无限增长
    double adc_start_s = reference_line_info_->AdcSlBoundary().start_s();
    cone_history_.erase(
            std::remove_if(
                    cone_history_.begin(),
                    cone_history_.end(),
                    [adc_start_s](const std::pair<double, double>& p) {
                        return p.first < adc_start_s - kConeHistoryCleanupDist;
                    }),
            cone_history_.end());

    // 2. 统计当前可见锥桶 + 记录到历史
    int small_obs_count = 0;
    for (const auto* obs : reference_line_info_->path_decision()->obstacles().Items()) {
        if (obs && !obs->IsVirtual() && obs->PerceptionPolygon().area() < 0.5) {
            const auto& sl = obs->PerceptionSLBoundary();
            double obs_s = sl.end_s();
            double obs_l = (sl.start_l() + sl.end_l()) * 0.5;
            if (obs_s > adc_start_s && sl.start_s() - reference_line_info_->AdcSlBoundary().end_s() < 100.0) {
                small_obs_count++;
                // 记录位置（去重：相邻1m内的不重复记录）
                bool already_recorded = false;
                for (const auto& h : cone_history_) {
                    if (std::fabs(h.first - obs_s) < 1.0 && std::fabs(h.second - obs_l) < 1.0) {
                        already_recorded = true;
                        break;
                    }
                }
                if (!already_recorded) {
                    cone_history_.emplace_back(obs_s, obs_l);
                }
            }
        }
    }

    // 3. 统计历史锥桶位置中仍在主车前方的
    int history_cone_ahead = 0;
    double adc_end_s = reference_line_info_->AdcSlBoundary().end_s();
    for (const auto& h : cone_history_) {
        if (h.first > adc_end_s && h.first - adc_end_s < 100.0) {
            history_cone_ahead++;
        }
    }

    // 4. 综合判断：当前可见 + 历史记忆 ≥ 3 即视为施工区域
    //    滞回：一旦激活，需要锥桶完全消失（total<1）才退出，避免末尾锥桶
    //    数量波动导致 construct_zone 反复切换致规划模式跳变。
    int total_cone_estimate = small_obs_count + history_cone_ahead;
    if (total_cone_estimate >= 3) {
        construct_zone = true;
    } else if (total_cone_estimate < 1) {
        construct_zone = false;
        zone_left_base = zone_right_base = 0.0;
        construct_decision.clear();
    }
    // 中间态（1~2个锥桶）：保持上一帧状态不变

    // ── construct_zone 模式：生成跨全部可用车道的双向边界 ──
    if (construct_zone) {
        return DecideConstructZoneBoundary(boundary);
    }

    // ── 原始借道逻辑：逐方向（左/右）生成边界 ──
    for (size_t i = 0; i < decided_side_pass_direction_.size(); i++) {
        boundary->emplace_back();
        auto& path_bound = boundary->back();
        std::string blocking_obstacle_id = "";
        std::string borrow_lane_type = "";
        double path_narrowest_width = 0;
        // 1. Initialize the path boundaries to be an indefinitely large area.
        if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_)) {
            const std::string msg = "Failed to initialize path boundaries.";
            AERROR << msg;
            boundary->pop_back();
            continue;
        }
        // 2. Decide a rough boundary based on lane info and ADC's position
        if (!GetBoundaryFromNeighborLane(decided_side_pass_direction_[i], &path_bound, &borrow_lane_type)) {
            AERROR << "Failed to decide a rough boundary based on lane and adc.";
            boundary->pop_back();
            continue;
        }

        std::string label;
        if (decided_side_pass_direction_[i] == SidePassDirection::LEFT_BORROW) {
            label = "regular/left" + borrow_lane_type;
        } else {
            label = "regular/right" + borrow_lane_type;
        }
        path_bound.set_label(label);

        // 3. Fine-tune the boundary based on static obstacles
        PathBound temp_path_bound = path_bound;
        std::vector<SLPolygon> obs_sl_polygons;
        PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons, init_sl_state_);
        zone_left_base = zone_right_base = 0.0;
        construct_decision.clear();
        if (U_turn_construct) {
            for (auto& sl_polygon : obs_sl_polygons) {
                if (std::fabs(sl_polygon.MaxL() - reference_line_info_->AdcSlBoundary().end_l()) > 7.0) {
                    sl_polygon.SetNudgeInfo(SLPolygon::UNDEFINED);
                } else {
                    sl_polygon.SetNudgeInfo(SLPolygon::RIGHT_NUDGE);
                }
            }
        }
        double temp = FLAGS_obstacle_lat_buffer;
        if (obs_sl_polygons.size() >= 4)
            FLAGS_obstacle_lat_buffer = 0.2;
        FLAGS_obstacle_lon_end_buffer_park = 5.0;
        if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
                    *reference_line_info_,
                    &obs_sl_polygons,
                    init_sl_state_,
                    &path_bound,
                    &blocking_obstacle_id,
                    &path_narrowest_width)) {
            const std::string msg
                    = "Failed to decide fine tune the boundaries after "
                      "taking into consideration all static obstacles.";
            AERROR << msg;
            FLAGS_obstacle_lat_buffer = temp;
            boundary->pop_back();
            continue;
        }
        FLAGS_obstacle_lat_buffer = temp;
        // 4. Append some extra path bound points to avoid zero-length path data.
        int counter = 0;
        while (!blocking_obstacle_id.empty() && path_bound.size() < temp_path_bound.size()
               && counter < FLAGS_num_extra_tail_bound_point) {
            path_bound.push_back(temp_path_bound[path_bound.size()]);
            counter++;
        }
        ADEBUG << "Completed generating path boundaries.";

        path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
        RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
    }
    return !boundary->empty();
}

bool LaneBorrowPath::HasDynamicVehicleConflictOnBorrowPath(const std::vector<PathData>& candidate_path_data) const {
    constexpr double kBorrowConflictLookAheadS = 35.0;
    constexpr double kBorrowConflictRearBuffer = 2.0;
    constexpr double kBorrowConflictFrontBuffer = 3.0;
    constexpr double kBorrowConflictLatBuffer = 0.5;

    if (candidate_path_data.empty() || decided_side_pass_direction_.empty()) {
        return false;
    }

    const auto& adc_sl = reference_line_info_->AdcSlBoundary();
    const double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
    const auto& vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
    const double adc_half_width = vehicle_param.width() * 0.5;
    const bool borrow_left = std::any_of(
            decided_side_pass_direction_.begin(), decided_side_pass_direction_.end(), [](SidePassDirection direction) {
                return direction == SidePassDirection::LEFT_BORROW;
            });
    const bool borrow_right = std::any_of(
            decided_side_pass_direction_.begin(), decided_side_pass_direction_.end(), [](SidePassDirection direction) {
                return direction == SidePassDirection::RIGHT_BORROW;
            });

    for (const auto* obs : reference_line_info_->path_decision()->obstacles().Items()) {
        if (obs == nullptr || obs->IsVirtual() || obs->IsStatic()
            || obs->Perception().type() != apollo::perception::PerceptionObstacle::VEHICLE) {
            continue;
        }
        const auto& obs_sl = obs->PerceptionSLBoundary();
        if (obs_sl.end_s() < adc_sl.start_s() - kBorrowConflictRearBuffer
            || obs_sl.start_s() > adc_sl.end_s() + kBorrowConflictLookAheadS) {
            continue;
        }
        const double obs_center_l = (obs_sl.start_l() + obs_sl.end_l()) * 0.5;
        if ((obs_center_l >= adc_center_l && !borrow_left) || (obs_center_l <= adc_center_l && !borrow_right)) {
            continue;
        }

        for (const auto& path_data : candidate_path_data) {
            for (const auto& point : path_data.frenet_frame_path()) {
                if (point.s() < adc_sl.start_s() || point.s() > adc_sl.end_s() + kBorrowConflictLookAheadS) {
                    continue;
                }
                const double ego_start_s = point.s() - vehicle_param.back_edge_to_center() - kBorrowConflictRearBuffer;
                const double ego_end_s = point.s() + vehicle_param.front_edge_to_center() + kBorrowConflictFrontBuffer;
                if (obs_sl.end_s() < ego_start_s || obs_sl.start_s() > ego_end_s) {
                    continue;
                }
                const double ego_left_l = point.l() + adc_half_width + kBorrowConflictLatBuffer;
                const double ego_right_l = point.l() - adc_half_width - kBorrowConflictLatBuffer;
                if (obs_sl.start_l() <= ego_left_l && obs_sl.end_l() >= ego_right_l) {
                    AINFO << "Borrow dynamic conflict obs[" << obs->Id() << "] obs_s[" << obs_sl.start_s() << ","
                          << obs_sl.end_s() << "] obs_l[" << obs_sl.start_l() << "," << obs_sl.end_l() << "] path_l["
                          << point.l() << "]";
                    return true;
                }
            }
        }
    }

    return false;
}

bool LaneBorrowPath::GenerateBorrowHoldPath(PathData* final_path) {
    if (final_path == nullptr) {
        return false;
    }

    PathBoundary path_bound;
    std::string blocking_obstacle_id;
    double path_narrowest_width = 0.0;
    const double current_l = init_sl_state_.second[0];
    constexpr double kHoldHalfWidth = 0.35;
    constexpr double kHoldSpeedLimit = 2.0;
    constexpr double kHoldSpeedLimitDistance = 45.0;

    if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_)) {
        return false;
    }
    if (!PathBoundsDeciderUtil::GetBoundaryFromRoad(*reference_line_info_, init_sl_state_, &path_bound)) {
        return false;
    }
    for (auto& point : path_bound) {
        point.l_lower.l = std::max(point.l_lower.l, current_l - kHoldHalfWidth);
        point.l_upper.l = std::min(point.l_upper.l, current_l + kHoldHalfWidth);
        if (point.l_lower.l > point.l_upper.l) {
            AINFO << "Borrow hold path blocked by road boundary at s[" << point.s << "] current_l[" << current_l
                  << "] bound[" << point.l_lower.l << "," << point.l_upper.l << "]";
            return false;
        }
    }
    path_bound.set_label("regular/borrow_hold");

    std::vector<SLPolygon> obs_sl_polygons;
    PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons, init_sl_state_);
    if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
                *reference_line_info_,
                &obs_sl_polygons,
                init_sl_state_,
                &path_bound,
                &blocking_obstacle_id,
                &path_narrowest_width)) {
        return false;
    }
    path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
    if (path_bound.size() <= 1) {
        return false;
    }

    const auto& config = config_.path_optimizer_config();
    const ReferenceLine& reference_line = reference_line_info_->reference_line();
    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_bound, reference_line, &ddl_bounds);
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(std::fmax(init_sl_state_.first[1], 1e-12));
    std::vector<double> ref_l(path_bound.size(), current_l);
    std::vector<double> weight_ref_l(path_bound.size(), config.path_reference_l_weight());
    std::array<double, 3> end_state = {current_l, 0.0, 0.0};

    if (!PathOptimizerUtil::OptimizePath(
                init_sl_state_,
                end_state,
                ref_l,
                weight_ref_l,
                path_bound,
                ddl_bounds,
                jerk_bound,
                config,
                &opt_l,
                &opt_dl,
                &opt_ddl)) {
        return false;
    }

    auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
            opt_l, opt_dl, opt_ddl, path_bound.delta_s(), path_bound.start_s());
    final_path->SetReferenceLine(&reference_line);
    final_path->SetFrenetPath(std::move(frenet_frame_path));
    if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path
                = DiscretizedPath(PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(*final_path));
        final_path->SetDiscretizedPath(discretized_path);
    }
    final_path->set_path_label(path_bound.label());
    final_path->set_blocking_obstacle_id(blocking_obstacle_id);
    reference_line_info_->mutable_reference_line()->AddSpeedLimit(
            reference_line_info_->AdcSlBoundary().start_s(),
            reference_line_info_->AdcSlBoundary().end_s() + kHoldSpeedLimitDistance,
            kHoldSpeedLimit);
    RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
    return true;
}

bool LaneBorrowPath::OptimizePath(
        const std::vector<PathBoundary>& path_boundaries,
        std::vector<PathData>* candidate_path_data) {
    const auto& config = config_.path_optimizer_config();
    const ReferenceLine& reference_line = reference_line_info_->reference_line();

    for (const auto& path_boundary : path_boundaries) {
        std::vector<double> opt_l, opt_dl, opt_ddl;
        std::vector<std::pair<double, double>> ddl_bounds;
        PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line, &ddl_bounds);
        const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(std::fmax(init_sl_state_.first[1], 1e-12));
        std::vector<double> ref_l;
        std::vector<double> weight_ref_l;
        double ref_weight = config.path_reference_l_weight();
        if (!U_turn_construct)
            ref_weight = 50;
        // 施工区模式：参考目标用可行通道中心，而非 l=0 的中心线。
        // 避免"回中心线趋势"把车拉向左侧锥桶导致卡在道路边界上。
        if (path_boundary.label().find("construct_zone") != std::string::npos) {
            ref_l.resize(path_boundary.size());
            weight_ref_l.resize(path_boundary.size());
            for (size_t i = 0; i < path_boundary.size(); ++i) {
                ref_l[i] = (path_boundary[i].l_lower.l + path_boundary[i].l_upper.l) * 0.5;
                weight_ref_l[i] = ref_weight;
            }
        } else {
            PathOptimizerUtil::UpdatePathRefWithBound(path_boundary, ref_weight, &ref_l, &weight_ref_l);
        }

        // 施工区域：路径终点不应强制归零，使用边界中点作为目标终点
        std::array<double, 3> end_state = {0.0, 0.0, 0.0};
        if (path_boundary.label().find("construct_zone") != std::string::npos) {
            const auto& last_pt = path_boundary.back();
            double end_l = (last_pt.l_lower.l + last_pt.l_upper.l) * 0.5;
            end_state = {end_l, 0.0, 0.0};
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
            lastframe_ = std::make_unique<PathData>();
            lastframe_->SetReferenceLine(&reference_line);
            lastframe_->SetFrenetPath(std::move(frenet_frame_path));
            if (FLAGS_use_front_axe_center_in_path_planning) {
                auto discretized_path
                        = DiscretizedPath(PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(*lastframe_));
                lastframe_->SetDiscretizedPath(discretized_path);
            }
            lastframe_->set_path_label(path_boundary.label());
            lastframe_->set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
            candidate_path_data->push_back(*lastframe_);
        } else if (lastframe_ != nullptr) {
            candidate_path_data->push_back(*lastframe_);
        }
    }
    if (candidate_path_data->empty()) {
        return false;
    }
    return true;
}

bool LaneBorrowPath::AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path) {
    std::vector<PathData> valid_path_data;
    for (auto& curr_path_data : *candidate_path_data) {
        if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_, curr_path_data)) {
            SetPathInfo(&curr_path_data);
            if (reference_line_info_->SDistanceToDestination() < FLAGS_path_trim_destination_threshold) {
                PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data);
            }
            if (curr_path_data.Empty()) {
                AINFO << "lane borrow path is empty after trimed";
                continue;
            }
            valid_path_data.push_back(curr_path_data);
        }
    }
    if (valid_path_data.empty()) {
        AINFO << "All lane borrow path are not valid";
        return false;
    }
    auto* mutable_path_decider_status
            = injector_->planning_context()->mutable_planning_status()->mutable_path_decider();
    const std::string blocking_obstacle_id = mutable_path_decider_status->front_static_obstacle_id();
    const Obstacle* blocking_obstacle = reference_line_info_->path_decision()->obstacles().Find(blocking_obstacle_id);
    if (valid_path_data.size() > 1) {
        if (ComparePathData(valid_path_data[0], valid_path_data[1], blocking_obstacle)) {
            *final_path = valid_path_data[0];
        } else {
            *final_path = valid_path_data[1];
        }
    } else {
        *final_path = valid_path_data[0];
    }
    RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
    return true;
}

bool LaneBorrowPath::GetBoundaryFromNeighborLane(
        const SidePassDirection pass_direction,
        PathBoundary* const path_bound,
        std::string* borrow_lane_type) {
    // Sanity checks.
    CHECK_NOTNULL(path_bound);
    ACHECK(!path_bound->empty());
    const ReferenceLine& reference_line = reference_line_info_->reference_line();
    double adc_lane_width = PathBoundsDeciderUtil::GetADCLaneWidth(reference_line, init_sl_state_.first[0]);
    double offset_to_map = 0;
    bool borrowing_reverse_lane = false;
    reference_line.GetOffsetToMap(init_sl_state_.first[0], &offset_to_map);
    // Go through every point, update the boundary based on lane info and
    // ADC's position.
    double past_lane_left_width = adc_lane_width / 2.0;
    double past_lane_right_width = adc_lane_width / 2.0;
    int path_blocked_idx = -1;
    mx_left_bound = 0.0;
    mx_right_bound = 0.0;
    for (size_t i = 0; i < path_bound->size(); ++i) {
        double curr_s = (*path_bound)[i].s;
        // 1. Get the current lane width at current point.
        double curr_lane_left_width = 0.0;
        double curr_lane_right_width = 0.0;
        double offset_to_lane_center = 0.0;
        if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width, &curr_lane_right_width)) {
            AWARN << "Failed to get lane width at s = " << curr_s;
            curr_lane_left_width = past_lane_left_width;
            curr_lane_right_width = past_lane_right_width;
        } else {
            reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
            curr_lane_left_width += offset_to_lane_center;
            curr_lane_right_width -= offset_to_lane_center;
            past_lane_left_width = curr_lane_left_width;
            past_lane_right_width = curr_lane_right_width;
        }
        // 2. Get the neighbor lane widths at the current point.
        double curr_neighbor_lane_width = 0.0;
        if (CheckLaneBoundaryType(*reference_line_info_, curr_s, pass_direction)) {
            hdmap::Id neighbor_lane_id;
            if (pass_direction == SidePassDirection::LEFT_BORROW) {
                // Borrowing left neighbor lane.
                if (reference_line_info_->GetNeighborLaneInfo(
                            ReferenceLineInfo::LaneType::LeftForward,
                            curr_s,
                            &neighbor_lane_id,
                            &curr_neighbor_lane_width)) {
                    apollo::hdmap::LaneInfoConstPtr lane
                            = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_lane_id);
                    for (auto id : lane->lane().left_neighbor_forward_lane_id()) {
                        curr_neighbor_lane_width += hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(id)->GetWidth(curr_s);
                    }
                    for (auto id : lane->lane().left_neighbor_reverse_lane_id()) {
                        double lane_width = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(id)->GetWidth(curr_s);
                        curr_neighbor_lane_width += lane_width;
                    }
                    ADEBUG << "Borrow left forward neighbor lane." << neighbor_lane_id.id();
                } else if (
                        reference_line_info_->GetNeighborLaneInfo(
                                ReferenceLineInfo::LaneType::LeftReverse,
                                curr_s,
                                &neighbor_lane_id,
                                &curr_neighbor_lane_width)) {
                    borrowing_reverse_lane = true;
                    ADEBUG << "Borrow left reverse neighbor lane." << neighbor_lane_id.id();
                } else {
                    ADEBUG << "There is no left neighbor lane.";
                }
            } else if (pass_direction == SidePassDirection::RIGHT_BORROW) {
                // Borrowing right neighbor lane.
                if (reference_line_info_->GetNeighborLaneInfo(
                            ReferenceLineInfo::LaneType::RightForward,
                            curr_s,
                            &neighbor_lane_id,
                            &curr_neighbor_lane_width)) {
                    apollo::hdmap::LaneInfoConstPtr lane
                            = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_lane_id);
                    for (auto id : lane->lane().right_neighbor_forward_lane_id()) {
                        curr_neighbor_lane_width += hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(id)->GetWidth(curr_s);
                    }
                    ADEBUG << "Borrow right forward neighbor lane." << neighbor_lane_id.id();
                } else if (
                        reference_line_info_->GetNeighborLaneInfo(
                                ReferenceLineInfo::LaneType::RightReverse,
                                curr_s,
                                &neighbor_lane_id,
                                &curr_neighbor_lane_width)) {
                    borrowing_reverse_lane = true;
                    ADEBUG << "Borrow right reverse neighbor lane." << neighbor_lane_id.id();
                } else {
                    ADEBUG << "There is no right neighbor lane.";
                }
            }
        }
        // 3. Calculate the proper boundary based on lane-width, ADC's position,
        //    and ADC's velocity.
        double offset_to_map = 0.0;
        reference_line.GetOffsetToMap(curr_s, &offset_to_map);

        double curr_left_bound_lane = curr_lane_left_width
                + (pass_direction == SidePassDirection::LEFT_BORROW ? curr_neighbor_lane_width : 0.0);

        double curr_right_bound_lane = -curr_lane_right_width
                - (pass_direction == SidePassDirection::RIGHT_BORROW ? curr_neighbor_lane_width : 0.0);
        double curr_left_bound = 0.0;
        double curr_right_bound = 0.0;
        curr_left_bound = curr_left_bound_lane - offset_to_map;
        curr_right_bound = curr_right_bound_lane - offset_to_map;
        mx_left_bound = std::fmax(mx_left_bound, curr_left_bound);
        mx_right_bound = std::fmin(mx_right_bound, curr_right_bound);

        // 4. Update the boundary.
        if (!PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
                    curr_left_bound, curr_right_bound, BoundType::LANE, BoundType::LANE, "", "", &path_bound->at(i))) {
            path_blocked_idx = static_cast<int>(i);
        }
        if (path_blocked_idx != -1) {
            break;
        }
    }
    PathBoundsDeciderUtil::TrimPathBounds(path_blocked_idx, path_bound);
    *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
    return true;
}
void LaneBorrowPath::UpdateSelfPathInfo() {
    auto cur_path = reference_line_info_->path_data();
    if (!cur_path.Empty() && cur_path.path_label().find("self") != std::string::npos
        && cur_path.blocking_obstacle_id().empty()) {
        use_self_lane_ = std::min(use_self_lane_ + 1, 10);
    } else {
        use_self_lane_ = 0;
    }
    blocking_obstacle_id_ = cur_path.blocking_obstacle_id();
}
bool LaneBorrowPath::IsNecessaryToBorrowLane() {
    auto* mutable_path_decider_status
            = injector_->planning_context()->mutable_planning_status()->mutable_path_decider();
    if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
        UpdateSelfPathInfo();
        // If originally borrowing neighbor lane:
        if (use_self_lane_ >= 6) {
            // If have been able to use self-lane for some time, then switch to
            // non-lane-borrowing.
            mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
            decided_side_pass_direction_.clear();
            AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
        }
    } else {
        // If originally not borrowing neighbor lane:
        AINFO << "Blocking obstacle ID[" << mutable_path_decider_status->front_static_obstacle_id() << "]";
        // ADC requirements check for lane-borrowing:
        if (!HasSingleReferenceLine(*frame_)) {
            return false;
        }
        if (!IsWithinSidePassingSpeedADC(*frame_)) {
            return false;
        }

        // Obstacle condition check for lane-borrowing:
        if (!IsBlockingObstacleFarFromIntersection(*reference_line_info_)) {
            return false;
        }
        if (!IsLongTermBlockingObstacle()) {
            return false;
        }
        if (!IsBlockingObstacleWithinDestination(*reference_line_info_)) {
            return false;
        }
        if (!IsSidePassableObstacle(*reference_line_info_)) {
            return false;
        }

        // switch to lane-borrowing
        if (decided_side_pass_direction_.empty()) {
            // first time init decided_side_pass_direction
            bool left_borrowable;
            bool right_borrowable;
            CheckLaneBorrow(*reference_line_info_, &left_borrowable, &right_borrowable);
            if (!left_borrowable && !right_borrowable) {
                mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
                AINFO << "LEFT AND RIGHT LANE CAN NOT BORROW";
                return false;
            } else {
                mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
                if (left_borrowable) {
                    decided_side_pass_direction_.push_back(SidePassDirection::LEFT_BORROW);
                }
                if (right_borrowable) {
                    decided_side_pass_direction_.push_back(SidePassDirection::RIGHT_BORROW);
                }
            }
        }
        use_self_lane_ = 0;
        AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
    }
    return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

bool LaneBorrowPath::HasSingleReferenceLine(const Frame& frame) {
    return frame.reference_line_info().size() == 1;
}

bool LaneBorrowPath::IsWithinSidePassingSpeedADC(const Frame& frame) {
    return frame.PlanningStartPoint().v() < config_.lane_borrow_max_speed();
}

bool LaneBorrowPath::IsLongTermBlockingObstacle() {
    if (injector_->planning_context()->planning_status().path_decider().front_static_obstacle_cycle_counter()
        >= config_.long_term_blocking_obstacle_cycle_threshold()) {
        ADEBUG << "The blocking obstacle is long-term existing.";
        return true;
    } else {
        ADEBUG << "The blocking obstacle is not long-term existing.";
        return false;
    }
}

bool LaneBorrowPath::IsBlockingObstacleWithinDestination(const ReferenceLineInfo& reference_line_info) {
    const auto& path_decider_status = injector_->planning_context()->planning_status().path_decider();
    const std::string blocking_obstacle_id = path_decider_status.front_static_obstacle_id();
    if (blocking_obstacle_id.empty()) {
        ADEBUG << "There is no blocking obstacle.";
        return true;
    }
    const Obstacle* blocking_obstacle = reference_line_info.path_decision().obstacles().Find(blocking_obstacle_id);
    if (blocking_obstacle == nullptr) {
        ADEBUG << "Blocking obstacle is no longer there.";
        return true;
    }

    double blocking_obstacle_s = blocking_obstacle->PerceptionSLBoundary().start_s();
    double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
    ADEBUG << "ADC is at s = " << adc_end_s;
    ADEBUG << "Destination is at s = " << reference_line_info.SDistanceToDestination() + adc_end_s;
    if (blocking_obstacle_s - adc_end_s > reference_line_info.SDistanceToDestination()) {
        return false;
    }
    return true;
}

bool LaneBorrowPath::IsBlockingObstacleFarFromIntersection(const ReferenceLineInfo& reference_line_info) {
    const auto& path_decider_status = injector_->planning_context()->planning_status().path_decider();
    const std::string blocking_obstacle_id = path_decider_status.front_static_obstacle_id();
    if (blocking_obstacle_id.empty()) {
        ADEBUG << "There is no blocking obstacle.";
        return true;
    }
    const Obstacle* blocking_obstacle = reference_line_info.path_decision().obstacles().Find(blocking_obstacle_id);
    if (blocking_obstacle == nullptr) {
        ADEBUG << "Blocking obstacle is no longer there.";
        return true;
    }

    // Get blocking obstacle's s.
    double blocking_obstacle_s = blocking_obstacle->PerceptionSLBoundary().end_s();
    ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
    // Get intersection's s and compare with threshold.
    const auto& first_encountered_overlaps = reference_line_info.FirstEncounteredOverlaps();
    for (const auto& overlap : first_encountered_overlaps) {
        ADEBUG << overlap.first << ", " << overlap.second.DebugString();
        if (overlap.first != ReferenceLineInfo::SIGNAL && overlap.first != ReferenceLineInfo::STOP_SIGN) {
            continue;
        }

        auto distance = overlap.second.start_s - blocking_obstacle_s;
        if (overlap.first == ReferenceLineInfo::SIGNAL || overlap.first == ReferenceLineInfo::STOP_SIGN) {
            if (distance < kIntersectionClearanceDist) {
                ADEBUG << "Too close to signal intersection (" << distance << "m); don't SIDE_PASS.";
                return false;
            }
        } else {
            if (distance < kJunctionClearanceDist) {
                ADEBUG << "Too close to overlap_type[" << overlap.first << "] (" << distance << "m); don't SIDE_PASS";
                return false;
            }
        }
    }

    return true;
}

bool LaneBorrowPath::IsSidePassableObstacle(const ReferenceLineInfo& reference_line_info) {
    const auto& path_decider_status = injector_->planning_context()->planning_status().path_decider();
    const std::string blocking_obstacle_id = path_decider_status.front_static_obstacle_id();
    if (blocking_obstacle_id.empty()) {
        ADEBUG << "There is no blocking obstacle.";
        return false;
    }
    const Obstacle* blocking_obstacle = reference_line_info.path_decision().obstacles().Find(blocking_obstacle_id);
    if (blocking_obstacle == nullptr) {
        ADEBUG << "Blocking obstacle is no longer there.";
        return false;
    }

    return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);
}

void LaneBorrowPath::CheckLaneBorrow(
        const ReferenceLineInfo& reference_line_info,
        bool* left_neighbor_lane_borrowable,
        bool* right_neighbor_lane_borrowable) {
    const ReferenceLine& reference_line = reference_line_info.reference_line();

    *left_neighbor_lane_borrowable = true;
    *right_neighbor_lane_borrowable = true;

    static constexpr double kLookforwardDistance = 100.0;
    double check_s = reference_line_info.AdcSlBoundary().end_s();
    const double lookforward_distance = std::min(check_s + kLookforwardDistance, reference_line.Length());
    while (check_s < lookforward_distance) {
        auto ref_point = reference_line.GetNearestReferencePoint(check_s);
        if (ref_point.lane_waypoints().empty()) {
            *left_neighbor_lane_borrowable = false;
            *right_neighbor_lane_borrowable = false;
            return;
        }
        auto ptr_lane_info = reference_line_info.LocateLaneInfo(check_s);
        if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()
            && ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
            *left_neighbor_lane_borrowable = false;
        }
        if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()
            && ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
            *right_neighbor_lane_borrowable = false;
        }
        const auto waypoint = ref_point.lane_waypoints().front();
        hdmap::LaneBoundaryType::Type lane_boundary_type = hdmap::LaneBoundaryType::UNKNOWN;

        if (*left_neighbor_lane_borrowable) {
            lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
            if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW
                || lane_boundary_type == hdmap::LaneBoundaryType::DOUBLE_YELLOW
                || lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
                *left_neighbor_lane_borrowable = false;
            }
            ADEBUG << "s[" << check_s << "] left_lane_boundary_type[" << LaneBoundaryType_Type_Name(lane_boundary_type)
                   << "]";
        }
        if (*right_neighbor_lane_borrowable) {
            lane_boundary_type = hdmap::RightBoundaryType(waypoint);
            if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW
                || lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
                *right_neighbor_lane_borrowable = false;
            }
            ADEBUG << "s[" << check_s << "] right_neighbor_lane_borrowable["
                   << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
        }
        check_s += 2.0;
    }
}

bool LaneBorrowPath::CheckLaneBoundaryType(
        const ReferenceLineInfo& reference_line_info,
        const double check_s,
        const SidePassDirection& lane_borrow_info) {
    const ReferenceLine& reference_line = reference_line_info.reference_line();
    auto ref_point = reference_line.GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
        return false;
    }

    const auto waypoint = ref_point.lane_waypoints().front();
    hdmap::LaneBoundaryType::Type lane_boundary_type = hdmap::LaneBoundaryType::UNKNOWN;
    if (lane_borrow_info == SidePassDirection::LEFT_BORROW) {
        lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
    } else if (lane_borrow_info == SidePassDirection::RIGHT_BORROW) {
        lane_boundary_type = hdmap::RightBoundaryType(waypoint);
    }
    if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW
        || lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        return false;
    }
    return true;
}

void LaneBorrowPath::SetPathInfo(PathData* const path_data) {
    std::vector<PathPointDecision> path_decision;
    PathAssessmentDeciderUtil::InitPathPointDecision(*path_data, PathData::PathPointType::IN_LANE, &path_decision);
    // Go through every path_point, and add in-lane/out-of-lane info.
    const auto& discrete_path = path_data->discretized_path();
    bool is_prev_point_out_lane = false;
    SLBoundary ego_sl_boundary;
    for (size_t i = 0; i < discrete_path.size(); ++i) {
        if (!GetSLBoundary(*path_data, i, reference_line_info_, &ego_sl_boundary)) {
            ADEBUG << "Unable to get SL-boundary of ego-vehicle.";
            continue;
        }
        double lane_left_width = 0.0;
        double lane_right_width = 0.0;
        double middle_s = (ego_sl_boundary.start_s() + ego_sl_boundary.end_s()) / 2.0;
        if (reference_line_info_->reference_line().GetLaneWidth(middle_s, &lane_left_width, &lane_right_width)) {
            // Rough sl boundary estimate using single point lane width
            double back_to_inlane_extra_buffer = 0.2;
            double in_and_out_lane_hysteresis_buffer = is_prev_point_out_lane ? back_to_inlane_extra_buffer : 0.0;
            // For lane-borrow path, as long as ADC is not on the lane of
            // reference-line, it is out on other lanes. It might even be
            // on reverse lane!
            if (ego_sl_boundary.end_l() > lane_left_width + in_and_out_lane_hysteresis_buffer
                || ego_sl_boundary.start_l() < -lane_right_width - in_and_out_lane_hysteresis_buffer) {
                if (path_data->path_label().find("reverse") != std::string::npos) {
                    std::get<1>((path_decision)[i]) = PathData::PathPointType::OUT_ON_REVERSE_LANE;
                } else if (path_data->path_label().find("forward") != std::string::npos) {
                    std::get<1>((path_decision)[i]) = PathData::PathPointType::OUT_ON_FORWARD_LANE;
                } else {
                    std::get<1>((path_decision)[i]) = PathData::PathPointType::UNKNOWN;
                }
                if (!is_prev_point_out_lane) {
                    if (ego_sl_boundary.end_l() > lane_left_width + back_to_inlane_extra_buffer
                        || ego_sl_boundary.start_l() < -lane_right_width - back_to_inlane_extra_buffer) {
                        is_prev_point_out_lane = true;
                    }
                }
            } else {
                // The path point is within the reference_line's lane.
                std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
                if (is_prev_point_out_lane) {
                    is_prev_point_out_lane = false;
                }
            }

        } else {
            AERROR << "reference line not ready when setting path point guide, middle_s" << middle_s << ",index" << i
                   << "path point" << discrete_path[i].DebugString();
            break;
        }
    }
    path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

bool ComparePathData(const PathData& lhs, const PathData& rhs, const Obstacle* blocking_obstacle) {
    ADEBUG << "Comparing " << lhs.path_label() << " and " << rhs.path_label();
    static constexpr double kNeighborPathLengthComparisonTolerance = 25.0;
    double lhs_path_length = lhs.frenet_frame_path().back().s();
    double rhs_path_length = rhs.frenet_frame_path().back().s();
    // Select longer path.
    // If roughly same length, then select self-lane path.
    if (std::fabs(lhs_path_length - rhs_path_length) > kNeighborPathLengthComparisonTolerance) {
        return lhs_path_length > rhs_path_length;
    }
    // If roughly same length, and must borrow neighbor lane,
    // then prefer to borrow forward lane rather than reverse lane.
    int lhs_on_reverse = ContainsOutOnReverseLane(lhs.path_point_decision_guide());
    int rhs_on_reverse = ContainsOutOnReverseLane(rhs.path_point_decision_guide());
    // TODO(jiacheng): make this a flag.
    if (std::abs(lhs_on_reverse - rhs_on_reverse) > 6) {
        return lhs_on_reverse < rhs_on_reverse;
    }
    // For two lane-borrow directions, based on ADC's position,
    // select the more convenient one.
    if (blocking_obstacle) {
        // select left/right path based on blocking_obstacle's position
        const double obstacle_l = (blocking_obstacle->PerceptionSLBoundary().start_l()
                                   + blocking_obstacle->PerceptionSLBoundary().end_l())
                / 2;
        ADEBUG << "obstacle[" << blocking_obstacle->Id() << "] l[" << obstacle_l << "]";
        return (obstacle_l > 0.0 ? (lhs.path_label().find("right") != std::string::npos)
                                 : (lhs.path_label().find("left") != std::string::npos));
    } else {
        // select left/right path based on ADC's position
        double adc_l = lhs.frenet_frame_path().front().l();
        if (adc_l < -1.0) {
            return lhs.path_label().find("right") != std::string::npos;
        } else if (adc_l > 1.0) {
            return lhs.path_label().find("left") != std::string::npos;
        }
    }
    // If same length, both neighbor lane are forward,
    // then select the one that returns to in-lane earlier.
    static constexpr double kBackToSelfLaneComparisonTolerance = 20.0;
    int lhs_back_idx = GetBackToInLaneIndex(lhs.path_point_decision_guide());
    int rhs_back_idx = GetBackToInLaneIndex(rhs.path_point_decision_guide());
    double lhs_back_s = lhs.frenet_frame_path()[lhs_back_idx].s();
    double rhs_back_s = rhs.frenet_frame_path()[rhs_back_idx].s();
    if (std::fabs(lhs_back_s - rhs_back_s) > kBackToSelfLaneComparisonTolerance) {
        return lhs_back_idx < rhs_back_idx;
    }
    // If same length, both forward, back to inlane at same time,
    // select the left one to side-pass.
    bool lhs_on_leftlane = lhs.path_label().find("left") != std::string::npos;
    return lhs_on_leftlane;
}

int ContainsOutOnReverseLane(const std::vector<PathPointDecision>& path_point_decision) {
    int ret = 0;
    for (const auto& curr_decision : path_point_decision) {
        if (std::get<1>(curr_decision) == PathData::PathPointType::OUT_ON_REVERSE_LANE) {
            ++ret;
        }
    }
    return ret;
}

int GetBackToInLaneIndex(const std::vector<PathPointDecision>& path_point_decision) {
    // ACHECK(!path_point_decision.empty());
    // ACHECK(std::get<1>(path_point_decision.back()) ==
    //       PathData::PathPointType::IN_LANE);

    for (int i = static_cast<int>(path_point_decision.size()) - 1; i >= 0; --i) {
        if (std::get<1>(path_point_decision[i]) != PathData::PathPointType::IN_LANE) {
            return i;
        }
    }
    return 0;
}

bool LaneBorrowPath::DecideConstructZoneBoundary(std::vector<PathBoundary>* boundary) {
    boundary->emplace_back();
    auto& path_bound = boundary->back();
    std::string blocking_obstacle_id = "";
    double path_narrowest_width = 0;

    // 1. Initialize the path boundary.
    if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_)) {
        AERROR << "Failed to init path boundary for construct zone.";
        boundary->pop_back();
        return false;
    }

    // 2. Build a bidirectional boundary spanning all available lanes.
    GetConstructZoneBoundary(&path_bound);
    path_bound.set_label("regular/construct_zone");

    // 3. Save a temp copy for tail padding.
    PathBound temp_path_bound = path_bound;

    // 4. 绕过 GetSLPolygons 的默认过滤逻辑，直接收集所有小障碍物。
    //    默认的 GetSLPolygons 只返回"阻塞"障碍物 + |l| ≤ 3.5m 的小障碍物。
    //    施工区 S 弯锥桶横跨 3 车道，|l| 可达 5~8m，必须全部纳入。
    std::vector<SLPolygon> obs_sl_polygons;
    std::unordered_map<std::string, std::pair<double, double>> cone_xy;  // id -> (x, y)
    {
        const double adc_back_s = reference_line_info_->AdcSlBoundary().start_s();
        const double adc_front_s = reference_line_info_->AdcSlBoundary().end_s();
        for (const auto* obs : reference_line_info_->path_decision()->obstacles().Items()) {
            if (!obs || obs->IsVirtual())
                continue;
            if (obs->PerceptionPolygon().area() >= 0.5)
                continue;
            const auto& sl = obs->PerceptionSLBoundary();
            if (sl.end_s() < adc_back_s)
                continue;
            if (sl.start_s() - adc_front_s > 100.0)
                continue;
            obs_sl_polygons.emplace_back(sl, obs->Id());
            // 计算XY中心
            double cx = 0, cy = 0;
            const auto& pts = obs->PerceptionPolygon().points();
            for (const auto& p : pts) {
                cx += p.x();
                cy += p.y();
            }
            if (!pts.empty()) {
                cx /= pts.size();
                cy /= pts.size();
            }
            cone_xy[obs->Id()] = {cx, cy};
        }
        std::sort(obs_sl_polygons.begin(), obs_sl_polygons.end(), [](const SLPolygon& a, const SLPolygon& b) {
            return a.MinS() < b.MinS();
        });
    }
    AINFO << "[CONSTRUCT_ZONE] collected " << obs_sl_polygons.size() << " small obstacles, mx_left=" << mx_left_bound
          << " mx_right=" << mx_right_bound;

    // 5. 墙追踪分类锥桶 + 分配nudge方向(在函数内部完成)
    ComputeConstructZoneBoundary(obs_sl_polygons, cone_xy, &path_bound);

    // 6. 标准nudge系统计算边界
    {
        double temp_lat = FLAGS_obstacle_lat_buffer;
        if (obs_sl_polygons.size() >= 4)
            FLAGS_obstacle_lat_buffer = 0.2;
        FLAGS_obstacle_lon_end_buffer_park = 0.1;
        PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
                *reference_line_info_,
                &obs_sl_polygons,
                init_sl_state_,
                &path_bound,
                &blocking_obstacle_id,
                &path_narrowest_width);
        FLAGS_obstacle_lat_buffer = temp_lat;
    }

    // 7. 尾部补齐 + 日志。
    int counter = 0;
    while (!blocking_obstacle_id.empty() && path_bound.size() < temp_path_bound.size()
           && counter < FLAGS_num_extra_tail_bound_point) {
        path_bound.push_back(temp_path_bound[path_bound.size()]);
        counter++;
    }
    path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
    RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
    AINFO << "Construct zone boundary generated: left=" << mx_left_bound << " right=" << mx_right_bound;
    return true;
}

void LaneBorrowPath::GetConstructZoneBoundary(PathBoundary* const path_bound) {
    CHECK_NOTNULL(path_bound);
    ACHECK(!path_bound->empty());
    const ReferenceLine& reference_line = reference_line_info_->reference_line();
    double adc_lane_width = PathBoundsDeciderUtil::GetADCLaneWidth(reference_line, init_sl_state_.first[0]);
    double offset_to_map = 0;
    reference_line.GetOffsetToMap(init_sl_state_.first[0], &offset_to_map);

    double past_lane_left_width = adc_lane_width / 2.0;
    double past_lane_right_width = adc_lane_width / 2.0;
    int path_blocked_idx = -1;
    mx_left_bound = 0.0;
    mx_right_bound = 0.0;

    for (size_t i = 0; i < path_bound->size(); ++i) {
        double curr_s = (*path_bound)[i].s;

        // 1. Get self lane width at current s.
        double curr_lane_left_width = 0.0;
        double curr_lane_right_width = 0.0;
        double offset_to_lane_center = 0.0;
        if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width, &curr_lane_right_width)) {
            AWARN << "Failed to get lane width at s = " << curr_s;
            curr_lane_left_width = past_lane_left_width;
            curr_lane_right_width = past_lane_right_width;
        } else {
            reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
            curr_lane_left_width += offset_to_lane_center;
            curr_lane_right_width -= offset_to_lane_center;
            past_lane_left_width = curr_lane_left_width;
            past_lane_right_width = curr_lane_right_width;
        }

        // 2. Get ALL left-side neighbor lanes (recursive: neighbor + neighbor-of-neighbor).
        double left_neighbor_width = 0.0;
        if (CheckLaneBoundaryType(*reference_line_info_, curr_s, SidePassDirection::LEFT_BORROW)) {
            hdmap::Id neighbor_lane_id;
            double nb_width = 0.0;
            if (reference_line_info_->GetNeighborLaneInfo(
                        ReferenceLineInfo::LaneType::LeftForward, curr_s, &neighbor_lane_id, &nb_width)) {
                left_neighbor_width += nb_width;
                apollo::hdmap::LaneInfoConstPtr lane = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_lane_id);
                // Neighbor's left forward lanes (3rd+ lanes)
                for (const auto& id : lane->lane().left_neighbor_forward_lane_id()) {
                    left_neighbor_width += hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(id)->GetWidth(curr_s);
                }
                for (const auto& id : lane->lane().left_neighbor_reverse_lane_id()) {
                    left_neighbor_width += hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(id)->GetWidth(curr_s);
                }
            }
        }

        // 3. Get ALL right-side neighbor lanes (recursive: neighbor + neighbor-of-neighbor).
        double right_neighbor_width = 0.0;
        if (CheckLaneBoundaryType(*reference_line_info_, curr_s, SidePassDirection::RIGHT_BORROW)) {
            hdmap::Id neighbor_lane_id;
            double nb_width = 0.0;
            if (reference_line_info_->GetNeighborLaneInfo(
                        ReferenceLineInfo::LaneType::RightForward, curr_s, &neighbor_lane_id, &nb_width)) {
                right_neighbor_width += nb_width;
                apollo::hdmap::LaneInfoConstPtr lane = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_lane_id);
                for (const auto& id : lane->lane().right_neighbor_forward_lane_id()) {
                    right_neighbor_width += hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(id)->GetWidth(curr_s);
                }
            }
        }

        // 4. Calculate the combined boundary.
        double offset_to_map_s = 0.0;
        reference_line.GetOffsetToMap(curr_s, &offset_to_map_s);
        double curr_left_bound = curr_lane_left_width + left_neighbor_width - offset_to_map_s;
        double curr_right_bound = -curr_lane_right_width - right_neighbor_width - offset_to_map_s;

        // 5. Track the overall boundary extents.
        mx_left_bound = std::fmax(mx_left_bound, curr_left_bound);
        mx_right_bound = std::fmin(mx_right_bound, curr_right_bound);

        // 6. Update the path boundary point.
        if (!PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
                    curr_left_bound, curr_right_bound, BoundType::LANE, BoundType::LANE, "", "", &path_bound->at(i))) {
            path_blocked_idx = static_cast<int>(i);
        }
        if (path_blocked_idx != -1) {
            break;
        }
    }
    PathBoundsDeciderUtil::TrimPathBounds(path_blocked_idx, path_bound);
}

void LaneBorrowPath::ComputeConstructZoneBoundary(
        std::vector<SLPolygon>& cones,
        const std::unordered_map<std::string, std::pair<double, double>>& cone_xy,
        PathBoundary* const path_bound) {
    if (path_bound->empty())
        return;

    constexpr double kWallMergeDist = 0.8;
    constexpr double kCleanupBehindDist = 30.0;
    const double adc_back_s = reference_line_info_->AdcSlBoundary().start_s();
    double road_left = mx_left_bound;
    double road_right = mx_right_bound;

    // ── 日志工具：统一前缀 [WALL] 方便 grep 提取 ──
    auto fmt = [](double v) {
        char b[32];
        snprintf(b, sizeof(b), "%.2f", v);
        return std::string(b);
    };
    auto dbg = [](const std::string& msg) { AINFO << "[WALL] " << msg; };

    dbg("FRAME|cones=" + std::to_string(cones.size()) + "|lw=" + std::to_string(left_wall_.size())
        + "|rw=" + std::to_string(right_wall_.size()) + "|road=[" + fmt(road_right) + "," + fmt(road_left) + "]");

    // ── 1. 清理 ──
    auto prune = [adc_back_s](std::vector<std::pair<double, double>>& w) {
        w.erase(std::remove_if(
                        w.begin(),
                        w.end(),
                        [adc_back_s](const auto& p) { return p.first < adc_back_s - kCleanupBehindDist; }),
                w.end());
    };
    prune(left_wall_);
    prune(right_wall_);

    // ── 2. 墙插值 ──
    // wall_l_classify: 锥桶分类用，>20m间隙才视为无墙，容忍更大闪烁
    auto wall_l_classify = [](const std::vector<std::pair<double, double>>& w, double s) -> double {
        if (w.empty())
            return std::numeric_limits<double>::quiet_NaN();
        auto it = std::lower_bound(w.begin(), w.end(), s, [](const auto& p, double v) { return p.first < v; });
        if (it == w.end()) {
            if (s - w.back().first > 20.0)
                return std::numeric_limits<double>::quiet_NaN();
            return w.back().second;
        }
        if (it == w.begin()) {
            if (it->first - s > 20.0)
                return std::numeric_limits<double>::quiet_NaN();
            return it->second;
        }
        auto pr = std::prev(it);
        if (it->first - pr->first > 20.0)
            return std::numeric_limits<double>::quiet_NaN();
        double r = (s - pr->first) / (it->first - pr->first);
        return pr->second + r * (it->second - pr->second);
    };

    // ── 3. 初始化：滑动窗口冷启动 + 逐S追踪 ──
    if (left_wall_.empty() && right_wall_.empty() && !cones.empty()) {
        std::vector<std::pair<double, double>> sorted;
        for (const auto& c : cones)
            sorted.emplace_back((c.MinS() + c.MaxS()) * 0.5, (c.MinL() + c.MaxL()) * 0.5);
        std::sort(sorted.begin(), sorted.end());

        double lw_l = std::numeric_limits<double>::quiet_NaN();
        double rw_l = std::numeric_limits<double>::quiet_NaN();
        int nl = 0, nr = 0;

        for (size_t i = 0; i < sorted.size(); ++i) {
            double s = sorted[i].first, l = sorted[i].second;

            // 冷启动用道路边界决定第一个锥桶归哪面墙
            if (std::isnan(lw_l) && std::isnan(rw_l)) {
                if (std::fabs(l - road_left) < std::fabs(l - road_right)) {
                    left_wall_.emplace_back(s, l);
                    nl++;
                    lw_l = l;
                } else {
                    right_wall_.emplace_back(s, l);
                    nr++;
                    rw_l = l;
                }
                continue;
            }

            // 用墙线插值（而非EMA）判断归属，避免S弯漂移
            bool to_left;
            double pl = wall_l_classify(left_wall_, s);
            double pr = wall_l_classify(right_wall_, s);
            if (std::isnan(pl) && std::isnan(pr)) {
                // 双墙均无预测（如锥桶出现在墙建立前），用道路边界兜底
                if (std::fabs(l - road_left) < std::fabs(l - road_right)) {
                    left_wall_.emplace_back(s, l);
                    nl++;
                } else {
                    right_wall_.emplace_back(s, l);
                    nr++;
                }
                continue;
            } else if (std::isnan(pl)) {
                // 右墙有预测，左墙无。锥桶远离右墙 → 可能左墙新段
                // 增加道路边界兜底：若锥桶更靠近左边界而非右墙，即使距离>5m也归左墙
                double d_to_rw = std::fabs(l - pr);
                double d_to_road_l = std::fabs(l - road_left);
                if (d_to_rw > 5.0 && d_to_rw > d_to_road_l) {
                    left_wall_.emplace_back(s, l);
                    nl++;
                } else if (d_to_rw > 5.0) {
                    // 远离右墙但也不靠左边界 → 保守归入右墙
                    right_wall_.emplace_back(s, l);
                    nr++;
                } else {
                    right_wall_.emplace_back(s, l);
                    nr++;
                }
                continue;
            } else if (std::isnan(pr)) {
                // 左墙有预测，右墙无。锥桶远离左墙 → 可能右墙新段
                double d_to_lw = std::fabs(l - pl);
                double d_to_road_r = std::fabs(l - road_right);
                if (d_to_lw > 5.0 && d_to_lw > d_to_road_r) {
                    right_wall_.emplace_back(s, l);
                    nr++;
                } else if (d_to_lw > 5.0) {
                    // 远离左墙但也不靠右边界 → 保守归入左墙
                    left_wall_.emplace_back(s, l);
                    nl++;
                } else {
                    left_wall_.emplace_back(s, l);
                    nl++;
                }
                continue;
            } else {
                to_left = std::fabs(l - pl) < std::fabs(l - pr);
            }

            if (to_left) {
                left_wall_.emplace_back(s, l);
                nl++;
            } else {
                right_wall_.emplace_back(s, l);
                nr++;
            }
        }
        dbg("INIT|sliding|nL=" + std::to_string(nl) + "|nR=" + std::to_string(nr));
    }

    // ── 4. 逐锥桶分类 ──
    // 关键修复：同一物理锥桶在不同帧中 SL 投影会随车辆位置变化，
    // 仅依赖 wall_l_classify(s) 会导致分类闪烁。用 XY 坐标（1m网格）
    // 建立分类记忆，已记住的锥桶直接用记忆分类，消除 SL 投影依赖。
    auto hash_xy = [](double x, double y) -> uint64_t {
        int64_t ix = static_cast<int64_t>(std::round(x));
        int64_t iy = static_cast<int64_t>(std::round(y));
        return (static_cast<uint64_t>(ix) << 32) | (static_cast<uint64_t>(static_cast<uint32_t>(iy)));
    };

    for (const auto& c : cones) {
        double s = (c.MinS() + c.MaxS()) * 0.5;
        double l = (c.MinL() + c.MaxL()) * 0.5;

        bool to_left;
        const char* why;

        // 1) 先查 XY 位置记忆（1m 网格）
        auto xy_it = cone_xy.find(c.id());
        bool from_memory = false;
        double pl = std::numeric_limits<double>::quiet_NaN();
        double pr = std::numeric_limits<double>::quiet_NaN();
        if (xy_it != cone_xy.end()) {
            uint64_t h = hash_xy(xy_it->second.first, xy_it->second.second);
            auto mem_it = cone_wall_memory_.find(h);
            if (mem_it != cone_wall_memory_.end()) {
                to_left = mem_it->second;
                why = "mem";
                from_memory = true;
            }
        }

        if (!from_memory) {
            // 2) 记忆未命中 → 用墙插值分类
            pl = wall_l_classify(left_wall_, s);
            pr = wall_l_classify(right_wall_, s);

            if (std::isnan(pl) && std::isnan(pr))
                continue;
            else if (std::isnan(pl)) {
                // 右墙有预测，左墙无 → 增加道路边界兜底
                double d_to_rw = std::fabs(l - pr);
                double d_to_road_l = std::fabs(l - road_left);
                if (d_to_rw > 5.0 && d_to_rw > d_to_road_l) {
                    to_left = true;
                    why = "farL";
                } else if (d_to_rw > 5.0) {
                    to_left = false;
                    why = "newL";
                } else {
                    to_left = false;
                    why = "noL";
                }
            } else if (std::isnan(pr)) {
                // 左墙有预测，右墙无 → 增加道路边界兜底
                double d_to_lw = std::fabs(l - pl);
                double d_to_road_r = std::fabs(l - road_right);
                if (d_to_lw > 5.0 && d_to_lw > d_to_road_r) {
                    to_left = false;
                    why = "farR";
                } else if (d_to_lw > 5.0) {
                    to_left = true;
                    why = "newR";
                } else {
                    to_left = true;
                    why = "noR";
                }
            } else {
                double dl = std::fabs(l - pl), dr = std::fabs(l - pr);
                to_left = dl < dr;
                static char buf[32];
                snprintf(buf, sizeof(buf), "dL=%.2f dR=%.2f", dl, dr);
                why = buf;
            }

            // 3) 记入 XY 位置记忆（仅对新分类的锥桶）
            if (xy_it != cone_xy.end()) {
                cone_wall_memory_[hash_xy(xy_it->second.first, xy_it->second.second)] = to_left;
            }
        }

        auto& tgt = to_left ? left_wall_ : right_wall_;
        bool merged = false;
        for (auto& pt : tgt) {
            if (std::fabs(pt.first - s) < kWallMergeDist) {
                pt.second = pt.second * 0.7 + l * 0.3;
                merged = true;
                break;
            }
        }
        if (!merged) {
            tgt.emplace_back(s, l);
            std::sort(tgt.begin(), tgt.end());
        }

        dbg(std::string("CONE|id=") + c.id()
            + "|xy=" + (xy_it != cone_xy.end() ? fmt(xy_it->second.first) + "," + fmt(xy_it->second.second) : "nan,nan")
            + "|s=" + fmt(s) + "|l=" + fmt(l) + "|pL=" + (std::isnan(pl) ? "nan" : fmt(pl)) + "|pR="
            + (std::isnan(pr) ? "nan" : fmt(pr)) + "|->" + (to_left ? "L" : "R") + "|" + why + (merged ? "|m" : "|n"));
    }
    // 分配nudge：左墙→RIGHT_NUDGE，右墙→LEFT_NUDGE
    for (auto& c : cones) {
        double s = (c.MinS() + c.MaxS()) * 0.5;
        double l = (c.MinL() + c.MaxL()) * 0.5;
        double pl = wall_l_classify(left_wall_, s);
        double pr = wall_l_classify(right_wall_, s);
        if (std::isnan(pl) && std::isnan(pr))
            continue;
        if (std::isnan(pl))
            c.SetNudgeInfo(std::fabs(l - pr) > 5.0 ? SLPolygon::RIGHT_NUDGE : SLPolygon::LEFT_NUDGE);
        else if (std::isnan(pr))
            c.SetNudgeInfo(std::fabs(l - pl) > 5.0 ? SLPolygon::LEFT_NUDGE : SLPolygon::RIGHT_NUDGE);
        else
            c.SetNudgeInfo(std::fabs(l - pl) < std::fabs(l - pr) ? SLPolygon::RIGHT_NUDGE : SLPolygon::LEFT_NUDGE);
    }
}

void LaneBorrowPath::ConstructDecision(
        const ReferenceLineInfo& reference_line_info,
        std::vector<SLPolygon>* const sl_polygon,
        PathBoundary* const path_boundary) {
    // 施工区域 nudge 决策：比较每个锥桶到左右边界的距离。
    //   锥桶靠近左边界 → RIGHT_NUDGE（从锥桶右侧/下方绕）
    //   锥桶靠近右边界 → LEFT_NUDGE （从锥桶左侧/上方绕）
    //
    // 这替代了原有的贪心 zone 收缩算法（S弯场景会将所有锥桶错分为同一方向）。
    for (size_t j = 0; j < sl_polygon->size(); j++) {
        if (construct_decision.find(sl_polygon->at(j).id()) != construct_decision.end()) {
            continue;
        }
        if (sl_polygon->at(j).MinS() - reference_line_info.AdcSlBoundary().start_s() > 100) {
            AINFO << "[CONSTRUCT_ZONE] breaking at cone " << sl_polygon->at(j).id()
                  << " MinS=" << sl_polygon->at(j).MinS() << " adc_s=" << reference_line_info.AdcSlBoundary().start_s();
            break;
        }
        double mid_l = (sl_polygon->at(j).MaxL() + sl_polygon->at(j).MinL()) * 0.5;
        // 过滤完全超出借道边界的障碍物
        if (mid_l > mx_left_bound || mid_l < mx_right_bound) {
            AINFO << "[CONSTRUCT_ZONE] skip out-of-bounds cone " << sl_polygon->at(j).id() << " l=" << mid_l
                  << " bounds=[" << mx_right_bound << "," << mx_left_bound << "]";
            continue;
        }
        if (std::fabs(mid_l) > 1e3) {
            AERROR << "real l is "
                   << reference_line_info.path_decision()
                              .obstacles()
                              .Find(sl_polygon->at(j).id())
                              ->PerceptionSLBoundary()
                              .end_l();
            break;
        }

        // 核心决策：锥桶更靠近左边界还是右边界？
        double dist_to_left = std::fabs(mid_l - mx_left_bound);
        double dist_to_right = std::fabs(mid_l - mx_right_bound);
        if (dist_to_left < dist_to_right) {
            // 锥桶靠近左边界 → 从右侧绕行
            sl_polygon->at(j).SetNudgeInfo(SLPolygon::RIGHT_NUDGE);
            construct_decision[sl_polygon->at(j).id()] = true;
            AINFO << "[CONSTRUCT_ZONE] cone " << sl_polygon->at(j).id()
                  << " s=" << (sl_polygon->at(j).MinS() + sl_polygon->at(j).MaxS()) * 0.5 << " l=" << mid_l
                  << " distL=" << dist_to_left << " distR=" << dist_to_right << " → RIGHT_NUDGE";
        } else {
            // 锥桶靠近右边界 → 从左侧绕行
            sl_polygon->at(j).SetNudgeInfo(SLPolygon::LEFT_NUDGE);
            construct_decision[sl_polygon->at(j).id()] = false;
            AINFO << "[CONSTRUCT_ZONE] cone " << sl_polygon->at(j).id()
                  << " s=" << (sl_polygon->at(j).MinS() + sl_polygon->at(j).MaxS()) * 0.5 << " l=" << mid_l
                  << " distL=" << dist_to_left << " distR=" << dist_to_right << " → LEFT_NUDGE";
        }
    }
}

}  // namespace planning
}  // namespace apollo
