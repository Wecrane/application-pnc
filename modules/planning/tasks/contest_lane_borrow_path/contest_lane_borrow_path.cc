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

#include "modules/planning/tasks/contest_lane_borrow_path/contest_lane_borrow_path.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/contest_scenario_features.h"
#include "modules/planning/planning_base/common/contest_scenario_status.h"
#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"
#include "modules/planning/tasks/contest_lane_borrow_path/u_turn_cone_nudge_helper.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

bool ContestLaneBorrowPath::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!Task::Init(config_dir, name, injector)) {
        return false;
    }
    return Task::LoadConfig<ContestLaneBorrowPathConfig>(&config_);
}

apollo::common::Status ContestLaneBorrowPath::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
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
    const bool is_contest_construction = IsContestConstructionScenario();
    const bool is_contest_u_turn = IsContestUTurnScenario();
    if (!is_contest_construction && construction_zone_.active) {
        ResetConstructZoneState("leave construction scenario");
    }

    // ── 独立锥桶检测：不依赖 path_decider 的 blocking_obstacle_id ──
    // 赛题五第二个场景中，车辆起始位置前方紧贴锥桶，path_decider 可能
    // 不将其识别为阻塞障碍物（障碍物太小/太近），导致 IsNecessaryToBorrowLane
    // 因 front_static_obstacle_id 为空而返回 false。
    // 这里做一次独立计数，≥3 个锥桶即强制进入借道模式。
    const int early_cone_count =
            is_contest_construction ? contest::CountDefaultConstructionConesAhead(*reference_line_info) : 0;

    if (is_contest_construction && early_cone_count >= 3) {
        ForceConstructionLaneBorrow(early_cone_count);
    }

    // 施工区锥桶检测优先：跳过 IsNecessaryToBorrowLane() 中 use_self_lane_ 的退出逻辑，
    // 防止刚被强制打开的借道模式又被 UpdateSelfPathInfo → use_self_lane_≥6 关掉，
    // 导致 Force lane borrow → Switch to SELF-LANE → Force lane borrow 的死循环振荡。
    if (!is_contest_construction || early_cone_count < 3) {
        if (!IsNecessaryToBorrowLane()) {
            ADEBUG << "No need to borrow lane";
            return Status::OK();
        }
    }
    std::vector<PathBoundary> candidate_path_boundaries;
    std::vector<PathData> candidate_path_data;

    GetStartPointSLState();
    if (!DecidePathBounds(&candidate_path_boundaries)) {
        return Status::OK();
    }
    // 施工区模式：降低对中心线的拉力，让路径自由跟随可行通道中心
    if (is_contest_construction && construction_zone_.active) {
        config_.mutable_path_optimizer_config()->set_l_weight(1.0);
    }
    if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
        return Status::OK();
    }
    // 倒车模式下跳过动态车辆冲突检测：
    // 倒车是为了远离前方锥桶，不需要检测后方来车。
    if (!reverse_recovery_.active && HasDynamicVehicleConflictOnBorrowPath(candidate_path_data)) {
        if (GenerateBorrowHoldPath(reference_line_info->mutable_path_data())) {
            AINFO << "Borrow path is temporarily blocked by a dynamic vehicle, hold current lateral position.";
            return Status::OK();
        }
        AWARN << "Failed to generate borrow hold path, continue assessing borrow candidates.";
    }
    // 倒车模式：直接使用生成的倒车路径，不走 AssessPath
    // （倒车路径 s 递减，IsValidRegularPath 可能因方向异常而拒绝）
    if (reverse_recovery_.active && !candidate_path_data.empty()) {
        *reference_line_info->mutable_path_data() = candidate_path_data.front();
        AINFO << "[REVERSE] Path set directly (bypass AssessPath), label=" << candidate_path_data.front().path_label();
    } else if (AssessPath(&candidate_path_data, reference_line_info->mutable_path_data())) {
        ADEBUG << "contest lane borrow path success";
    }

    ApplyConstructionZoneSpeedLimitAndLabel(reference_line_info);
    if ((is_contest_construction && construction_zone_.active) || (is_contest_u_turn && u_turn_construct_)) {
        IgnoreAllObstacles(reference_line_info);
    }

    return Status::OK();
}

bool ContestLaneBorrowPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
    const bool is_contest_construction = IsContestConstructionScenario();
    u_turn_construct_ = ShouldUseUTurnConeNudge();
    if (!is_contest_construction && construction_zone_.active) {
        ResetConstructZoneState("skip construction boundary outside construction scenario");
    }

    if (is_contest_construction) {
        UpdateConstructionZoneState();
    }

    // ── construct_zone 模式：生成跨全部可用车道的双向边界 ──
    if (is_contest_construction && construction_zone_.active) {
        if (MaybeGenerateReverseRecoveryBoundary(boundary)) {
            return !boundary->empty();
        }

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
        if (u_turn_construct_) {
            ApplyUTurnConeNudge(&obs_sl_polygons);
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

bool ContestLaneBorrowPath::HasDynamicVehicleConflictOnBorrowPath(const std::vector<PathData>& candidate_path_data) const {
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

bool ContestLaneBorrowPath::GenerateBorrowHoldPath(PathData* final_path) {
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

bool ContestLaneBorrowPath::OptimizePath(
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
        if (!u_turn_construct_)
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
        bool is_reverse_path = (path_boundary.label().find("reverse_path") != std::string::npos);
        if (path_boundary.label().find("construct_zone") != std::string::npos) {
            const auto& last_pt = path_boundary.back();
            double end_l = (last_pt.l_lower.l + last_pt.l_upper.l) * 0.5;
            end_state = {end_l, 0.0, 0.0};
        }
        if (is_reverse_path) {
            // 倒车路径：终点横向保持当前 l，不强制归零
            end_state = {init_sl_state_.second[0], 0.0, 0.0};
        }

        // 倒车路径：反转初始运动方向（dl, ddl 取负），让优化器沿参考线反向规划
        SLState opt_init_state = init_sl_state_;
        if (is_reverse_path) {
            opt_init_state.second[1] = -opt_init_state.second[1];
            opt_init_state.second[2] = -opt_init_state.second[2];
        }

        bool res_opt = PathOptimizerUtil::OptimizePath(
                opt_init_state,
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

            // 倒车路径：反转 s 方向 + 恢复 dl/ddl 符号
            // 保持 s 递减（与官方 reverse_path 行为一致），不排序
            if (is_reverse_path) {
                double start_s = path_boundary.start_s();
                for (auto& point : frenet_frame_path) {
                    double frenet_delta_s = point.s() - start_s;
                    point.set_s(start_s - frenet_delta_s);
                    point.set_dl(-point.dl());
                    point.set_ddl(-point.ddl());
                }
            }

            last_frame_ = std::make_unique<PathData>();
            last_frame_->SetReferenceLine(&reference_line);
            last_frame_->SetFrenetPath(std::move(frenet_frame_path));
            if (FLAGS_use_front_axe_center_in_path_planning) {
                auto discretized_path
                        = DiscretizedPath(PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(*last_frame_));
                last_frame_->SetDiscretizedPath(discretized_path);
            }
            last_frame_->set_path_label(path_boundary.label());
            last_frame_->set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
            if (is_reverse_path) {
                last_frame_->set_is_reverse_path(true);
            }
            candidate_path_data->push_back(*last_frame_);
        } else if (is_reverse_path) {
            // 倒车路径：OSQP 总是 primal infeasible（边界太窄/方向冲突），
            // 直接生成直线后退路径（s 递减：start_s → end_s）。
            // 与官方 reverse_path 任务一致：路径 s 递减 + is_reverse_path=true，
            // 速度规划/Control 据此以负速度沿路径后退。
            double start_s = path_boundary.start_s();          // ADC 当前位置 s（路径起点，高 s）
            double end_s = path_boundary.back().s;             // 后方目标 s（路径终点，低 s）
            double step = std::fabs(path_boundary.delta_s());  // 采样步长
            double lat = init_sl_state_.second[0];             // 保持当前横向位置不变

            FrenetFramePath fallback_frenet;
            for (double s = start_s; s >= end_s - 1e-6; s -= step) {
                common::FrenetFramePoint pt;
                pt.set_s(s);
                pt.set_l(lat);
                pt.set_dl(0.0);
                pt.set_ddl(0.0);
                fallback_frenet.push_back(pt);
            }

            last_frame_ = std::make_unique<PathData>();
            last_frame_->SetReferenceLine(&reference_line);
            last_frame_->SetFrenetPath(std::move(fallback_frenet));
            last_frame_->set_path_label(path_boundary.label());
            last_frame_->set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
            last_frame_->set_is_reverse_path(true);
            candidate_path_data->push_back(*last_frame_);
            AINFO << "[REVERSE] Direct backward path (skip OSQP), s=[" << start_s << "->" << end_s << "]";
        } else if (last_frame_ != nullptr && last_frame_->path_label() == path_boundary.label()) {
            candidate_path_data->push_back(*last_frame_);
        }
    }
    if (candidate_path_data->empty()) {
        return false;
    }
    return true;
}

bool ContestLaneBorrowPath::AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path) {
    std::vector<PathData> valid_path_data;
    for (auto& curr_path_data : *candidate_path_data) {
        if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_, curr_path_data)) {
            SetPathInfo(&curr_path_data);
            if (reference_line_info_->SDistanceToDestination() < FLAGS_path_trim_destination_threshold) {
                PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data);
            }
            if (curr_path_data.Empty()) {
                AINFO << "contest lane borrow path is empty after trimed";
                continue;
            }
            valid_path_data.push_back(curr_path_data);
        }
    }
    if (valid_path_data.empty()) {
        AINFO << "All contest lane borrow path are not valid";
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

bool ContestLaneBorrowPath::GetBoundaryFromNeighborLane(
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
    construction_zone_.max_left_bound = 0.0;
    construction_zone_.max_right_bound = 0.0;
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
        construction_zone_.max_left_bound = std::fmax(construction_zone_.max_left_bound, curr_left_bound);
        construction_zone_.max_right_bound = std::fmin(construction_zone_.max_right_bound, curr_right_bound);

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
void ContestLaneBorrowPath::UpdateSelfPathInfo() {
    auto cur_path = reference_line_info_->path_data();
    if (!cur_path.Empty() && cur_path.path_label().find("self") != std::string::npos
        && cur_path.blocking_obstacle_id().empty()) {
        use_self_lane_ = std::min(use_self_lane_ + 1, 10);
    } else {
        use_self_lane_ = 0;
    }
    blocking_obstacle_id_ = cur_path.blocking_obstacle_id();
}

bool ContestLaneBorrowPath::IsContestConstructionScenario() const {
    return contest::IsCurrentScenario(injector_, contest::kConstructionZoneScenario);
}

bool ContestLaneBorrowPath::IsContestUTurnScenario() const {
    return contest::IsCurrentScenario(injector_, contest::kUTurnScenario);
}

bool ContestLaneBorrowPath::ShouldUseUTurnConeNudge() const {
    return IsContestUTurnScenario() && planning::ShouldUseUTurnConeNudge(*reference_line_info_);
}

void ContestLaneBorrowPath::ApplyUTurnConeNudge(std::vector<SLPolygon>* obs_sl_polygons) const {
    planning::ApplyUTurnConeNudge(*reference_line_info_, obs_sl_polygons);
}

void ContestLaneBorrowPath::UpdateConstructionZoneState() {
    UpdateConstructionZoneTrackingState(frame_, reference_line_info_, kLowConeExitThreshold, &construction_zone_);
}

bool ContestLaneBorrowPath::MaybeGenerateReverseRecoveryBoundary(std::vector<PathBoundary>* boundary) {
    const double adc_speed = frame_->PlanningStartPoint().v();
    const double adc_s = reference_line_info_->AdcSlBoundary().start_s();
    const double adc_x = frame_->vehicle_state().x();
    const double adc_y = frame_->vehicle_state().y();

    if (reverse_recovery_.active) {
        reverse_recovery_.frame_count++;
        const double backed_dist_s = reverse_recovery_.start_s - adc_s;
        const double backed_dist_xy = std::hypot(adc_x - reverse_recovery_.start_x, adc_y - reverse_recovery_.start_y);
        const bool backed_enough =
                reverse_recovery_.frame_count >= kReverseMinFrames && backed_dist_xy > kReverseDistance;
        const bool reverse_timeout = reverse_recovery_.frame_count > kReverseMaxFrames;
        if (backed_enough || reverse_timeout) {
            reverse_recovery_.active = false;
            AINFO << "[REVERSE] Complete: backed_xy=" << backed_dist_xy << "m, backed_s=" << backed_dist_s
                  << "m, frames=" << reverse_recovery_.frame_count << (reverse_timeout ? ", timeout" : "");
            reverse_recovery_.frame_count = 0;
            return false;
        }

        AINFO << "[REVERSE] Active: frame=" << reverse_recovery_.frame_count << ", backed_xy=" << backed_dist_xy
              << "m, backed_s=" << backed_dist_s << "m";
        PathBoundary reverse_bound;
        if (!GenerateReversePathBoundary(&reverse_bound)) {
            return true;
        }
        boundary->push_back(reverse_bound);
        return true;
    }

    const double adc_move_xy = std::hypot(adc_x - reverse_recovery_.last_adc_x, adc_y - reverse_recovery_.last_adc_y);
    if (adc_speed < kStuckSpeedThreshold && adc_move_xy < 0.3) {
        reverse_recovery_.frame_count++;
    } else {
        reverse_recovery_.frame_count = 0;
    }
    reverse_recovery_.last_adc_s = adc_s;
    reverse_recovery_.last_adc_x = adc_x;
    reverse_recovery_.last_adc_y = adc_y;

    if (reverse_recovery_.frame_count < kStuckFrameThreshold) {
        return false;
    }

    reverse_recovery_.active = true;
    reverse_recovery_.start_s = adc_s;
    reverse_recovery_.start_x = adc_x;
    reverse_recovery_.start_y = adc_y;
    reverse_recovery_.frame_count = 0;
    AINFO << "[REVERSE] STUCK detected (speed=" << adc_speed << "), starting reverse from s=" << adc_s << ", xy=("
          << adc_x << "," << adc_y << ")";

    PathBoundary reverse_bound;
    if (!GenerateReversePathBoundary(&reverse_bound)) {
        return true;
    }
    boundary->push_back(reverse_bound);
    return true;
}

void ContestLaneBorrowPath::ForceConstructionLaneBorrow(int cone_count) {
    auto* mutable_path_decider_status
            = injector_->planning_context()->mutable_planning_status()->mutable_path_decider();
    if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
        return;
    }
    mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
    if (decided_side_pass_direction_.empty()) {
        decided_side_pass_direction_.push_back(SidePassDirection::LEFT_BORROW);
        decided_side_pass_direction_.push_back(SidePassDirection::RIGHT_BORROW);
    }
    AINFO << "Force lane borrow by early cone detection, count=" << cone_count;
}

void ContestLaneBorrowPath::ApplyConstructionZoneSpeedLimitAndLabel(ReferenceLineInfo* reference_line_info) const {
    if (!IsContestConstructionScenario() || !construction_zone_.active || reference_line_info == nullptr) {
        return;
    }
    constexpr double kMinPassengerArea = 0.1;
    constexpr double kSpeedLimit = 8.33;  // 30 km/h
    constexpr double kBuffer = 10.0;

    double lower_bound = std::numeric_limits<double>::infinity();
    double upper_bound = -std::numeric_limits<double>::infinity();
    for (const auto& obs : reference_line_info->path_decision()->obstacles().Items()) {
        if (obs == nullptr || obs->IsVirtual() || obs->PerceptionPolygon().area() >= kMinPassengerArea) {
            continue;
        }
        const auto& sl = obs->PerceptionSLBoundary();
        lower_bound = std::min(lower_bound, sl.start_s());
        upper_bound = std::max(upper_bound, sl.end_s());
    }
    if (!std::isfinite(lower_bound) || !std::isfinite(upper_bound)) {
        return;
    }
    reference_line_info->mutable_reference_line()->AddSpeedLimit(
            lower_bound - kBuffer, upper_bound + kBuffer, kSpeedLimit);
    reference_line_info->mutable_path_data()->set_path_label("regular/construct_zone");
}

void ContestLaneBorrowPath::IgnoreAllObstacles(ReferenceLineInfo* reference_line_info) const {
    if (reference_line_info == nullptr) {
        return;
    }
    for (auto& obs : reference_line_info->path_decision()->obstacles().Items()) {
        if (obs == nullptr || obs->IsVirtual()) {
            continue;
        }
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        reference_line_info->path_decision()->AddLongitudinalDecision(
                "PathDecider/ignore-backward-obstacle", obs->Id(), object_decision);
    }
}

void ContestLaneBorrowPath::ResetConstructZoneState(const std::string& reason) {
    construction_zone_.Reset();
    reverse_recovery_.Reset();
    AINFO << "[WALL] EXIT construct_zone by " << reason;
}

bool ContestLaneBorrowPath::IsNecessaryToBorrowLane() {
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

bool ContestLaneBorrowPath::HasSingleReferenceLine(const Frame& frame) {
    return frame.reference_line_info().size() == 1;
}

bool ContestLaneBorrowPath::IsWithinSidePassingSpeedADC(const Frame& frame) {
    return frame.PlanningStartPoint().v() < config_.lane_borrow_max_speed();
}

bool ContestLaneBorrowPath::IsLongTermBlockingObstacle() {
    if (injector_->planning_context()->planning_status().path_decider().front_static_obstacle_cycle_counter()
        >= config_.long_term_blocking_obstacle_cycle_threshold()) {
        ADEBUG << "The blocking obstacle is long-term existing.";
        return true;
    } else {
        ADEBUG << "The blocking obstacle is not long-term existing.";
        return false;
    }
}

bool ContestLaneBorrowPath::IsBlockingObstacleWithinDestination(const ReferenceLineInfo& reference_line_info) {
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

bool ContestLaneBorrowPath::IsBlockingObstacleFarFromIntersection(const ReferenceLineInfo& reference_line_info) {
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

bool ContestLaneBorrowPath::IsSidePassableObstacle(const ReferenceLineInfo& reference_line_info) {
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

void ContestLaneBorrowPath::CheckLaneBorrow(
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

bool ContestLaneBorrowPath::CheckLaneBoundaryType(
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

void ContestLaneBorrowPath::SetPathInfo(PathData* const path_data) {
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

bool ContestLaneBorrowPath::DecideConstructZoneBoundary(std::vector<PathBoundary>* boundary) {
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
            if (!contest::IsSmallRealObstacle(obs)) {
                continue;
            }
            const auto& sl = obs->PerceptionSLBoundary();
            if (sl.end_s() < adc_back_s) {
                continue;
            }
            if (sl.start_s() - adc_front_s > contest::kDefaultConstructionLookForwardDistance) {
                continue;
            }
            double cx = 0.0;
            double cy = 0.0;
            if (!contest::GetObstacleCenterXY(obs, &cx, &cy) || !contest::IsDefaultConstructionZoneXY(cx, cy)) {
                continue;
            }
            obs_sl_polygons.emplace_back(sl, obs->Id());
            cone_xy[obs->Id()] = {cx, cy};
        }
        std::sort(obs_sl_polygons.begin(), obs_sl_polygons.end(), [](const SLPolygon& a, const SLPolygon& b) {
            return a.MinS() < b.MinS();
        });
    }
    ADEBUG << "[CONSTRUCT_ZONE] collected " << obs_sl_polygons.size()
           << " small obstacles, mx_left=" << construction_zone_.max_left_bound
           << " mx_right=" << construction_zone_.max_right_bound;

    // 5. 墙追踪分类锥桶 + 分配nudge方向(在函数内部完成)
    ComputeConstructZoneBoundary(obs_sl_polygons, cone_xy, &path_bound);

    // 6. 标准nudge系统计算边界
    {
        double temp_lat = FLAGS_obstacle_lat_buffer;
        if (obs_sl_polygons.size() >= 4)
            FLAGS_obstacle_lat_buffer = 0.5;
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
    AINFO << "Construct zone boundary generated: left=" << construction_zone_.max_left_bound
          << " right=" << construction_zone_.max_right_bound;
    return true;
}

void ContestLaneBorrowPath::GetConstructZoneBoundary(PathBoundary* const path_bound) {
    CHECK_NOTNULL(path_bound);
    ACHECK(!path_bound->empty());
    const ReferenceLine& reference_line = reference_line_info_->reference_line();
    double adc_lane_width = PathBoundsDeciderUtil::GetADCLaneWidth(reference_line, init_sl_state_.first[0]);
    double offset_to_map = 0;
    reference_line.GetOffsetToMap(init_sl_state_.first[0], &offset_to_map);

    double past_lane_left_width = adc_lane_width / 2.0;
    double past_lane_right_width = adc_lane_width / 2.0;
    int path_blocked_idx = -1;
    construction_zone_.max_left_bound = 0.0;
    construction_zone_.max_right_bound = 0.0;

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
        construction_zone_.max_left_bound = std::fmax(construction_zone_.max_left_bound, curr_left_bound);
        construction_zone_.max_right_bound = std::fmin(construction_zone_.max_right_bound, curr_right_bound);

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

void ContestLaneBorrowPath::ComputeConstructZoneBoundary(
        std::vector<SLPolygon>& cones,
        const ConstructionConeXYMap& cone_xy,
        PathBoundary* const path_bound) {
    if (path_bound->empty()) {
        return;
    }
    const double adc_back_s = reference_line_info_->AdcSlBoundary().start_s();
    ComputeConstructionZoneBoundary(&cones, cone_xy, adc_back_s, &construction_zone_);
}

bool ContestLaneBorrowPath::GenerateReversePathBoundary(PathBoundary* boundary) {
    const double adc_s = reference_line_info_->AdcSlBoundary().start_s();
    const double adc_l = init_sl_state_.second[0];
    return GenerateReverseRecoveryPathBoundary(
            reference_line_info_->reference_line(), adc_s, adc_l, kReverseDistance, boundary);
}

}  // namespace planning
}  // namespace apollo
