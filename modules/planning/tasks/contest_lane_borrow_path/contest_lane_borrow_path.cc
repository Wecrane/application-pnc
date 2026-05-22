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

namespace {

int CountConstructionConesAhead(const ReferenceLineInfo& reference_line_info, double look_forward_distance) {
    const double adc_back_s = reference_line_info.AdcSlBoundary().start_s();
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    int cone_count = 0;
    for (const auto* obstacle : reference_line_info.path_decision().obstacles().Items()) {
        if (!contest::IsSmallRealObstacle(obstacle)) {
            continue;
        }
        const auto& sl = obstacle->PerceptionSLBoundary();
        if (sl.end_s() > adc_back_s - 6.0 &&
            sl.start_s() < adc_end_s + look_forward_distance) {
            ++cone_count;
        }
    }
    return cone_count;
}

bool HasCloseConstructionConeAhead(const ReferenceLineInfo& reference_line_info) {
    constexpr double kCloseFrontS = 6.0;
    constexpr double kCloseRearS = 1.5;
    constexpr double kCloseLateral = 2.4;
    const auto& adc_sl = reference_line_info.AdcSlBoundary();
    for (const auto* obstacle : reference_line_info.path_decision().obstacles().Items()) {
        if (!contest::IsSmallRealObstacle(obstacle)) {
            continue;
        }
        const auto& sl = obstacle->PerceptionSLBoundary();
        const double cone_l = (sl.start_l() + sl.end_l()) * 0.5;
        const bool near_longitudinal =
                sl.end_s() > adc_sl.start_s() - kCloseRearS &&
                sl.start_s() < adc_sl.end_s() + kCloseFrontS;
        if (near_longitudinal && std::fabs(cone_l) < kCloseLateral) {
            return true;
        }
    }
    return false;
}

double ClampLToPathBoundary(double target_l, const PathBoundPoint& point) {
    constexpr double kBoundaryMargin = 0.25;
    double lower = point.l_lower.l + kBoundaryMargin;
    double upper = point.l_upper.l - kBoundaryMargin;
    if (lower > upper) {
        lower = point.l_lower.l;
        upper = point.l_upper.l;
    }
    if (lower > upper) {
        return 0.5 * (point.l_lower.l + point.l_upper.l);
    }
    return std::min(std::max(target_l, lower), upper);
}

void BuildUTurnLargeRadiusReference(
        const ReferenceLine& reference_line,
        const PathBoundary& path_boundary,
        double ref_weight,
        std::vector<double>* ref_l,
        std::vector<double>* weight_ref_l) {
    if (ref_l == nullptr || weight_ref_l == nullptr || path_boundary.empty()) {
        return;
    }

    constexpr double kCurveKappaThreshold = 0.015;
    constexpr double kPrepareDistance = 18.0;
    constexpr double kReturnDistance = 28.0;
    constexpr double kOuterLaneOffset = 2.8;

    int curve_start_idx = -1;
    int curve_end_idx = -1;
    double dominant_kappa = 0.0;
    for (size_t i = 0; i < path_boundary.size(); ++i) {
        const double kappa = reference_line.GetNearestReferencePoint(path_boundary[i].s).kappa();
        if (std::fabs(kappa) < kCurveKappaThreshold) {
            continue;
        }
        if (curve_start_idx < 0) {
            curve_start_idx = static_cast<int>(i);
        }
        curve_end_idx = static_cast<int>(i);
        if (std::fabs(kappa) > std::fabs(dominant_kappa)) {
            dominant_kappa = kappa;
        }
    }

    ref_l->resize(path_boundary.size());
    weight_ref_l->resize(path_boundary.size());
    if (curve_start_idx < 0 || std::fabs(dominant_kappa) < 1e-6) {
        for (size_t i = 0; i < path_boundary.size(); ++i) {
            ref_l->at(i) = ClampLToPathBoundary(
                    0.5 * (path_boundary[i].l_lower.l + path_boundary[i].l_upper.l),
                    path_boundary[i]);
            weight_ref_l->at(i) = ref_weight;
        }
        return;
    }

    // Positive kappa turns left; a larger turning radius is on the right side
    // of the reference line (negative l). Negative kappa is mirrored.
    const double outer_l = dominant_kappa > 0.0 ? -kOuterLaneOffset : kOuterLaneOffset;
    const double curve_start_s = path_boundary[curve_start_idx].s;
    const double curve_end_s = path_boundary[curve_end_idx].s;
    for (size_t i = 0; i < path_boundary.size(); ++i) {
        const double s = path_boundary[i].s;
        double target_l = outer_l;
        if (s < curve_start_s - kPrepareDistance) {
            target_l = 0.0;
        } else if (s < curve_start_s) {
            const double ratio = (s - (curve_start_s - kPrepareDistance)) / kPrepareDistance;
            target_l = outer_l * std::min(std::max(ratio, 0.0), 1.0);
        } else if (s > curve_end_s) {
            const double ratio = (s - curve_end_s) / kReturnDistance;
            target_l = outer_l * (1.0 - std::min(std::max(ratio, 0.0), 1.0));
        }
        ref_l->at(i) = ClampLToPathBoundary(target_l, path_boundary[i]);
        weight_ref_l->at(i) = ref_weight;
    }
    AINFO << "[UTURN] large-radius ref generated, outer_l=" << outer_l
          << ", curve_s=[" << curve_start_s << "," << curve_end_s
          << "], prepare=" << kPrepareDistance << ", return=" << kReturnDistance;
}

}  // namespace

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
    if (contest::IsCurrentScenario(injector_, contest::kUTurnScenario) || u_turn_construct_) {
        // U 型弯场景：生成宽走廊路径。
        // 温和拉力 (l_weight=0.5, ref=5) 防止漂移，紧曲率段失败时回退零权重。
        // u_turn_construct_ 在场景退出后作为 latch 提供一帧过渡保护。
        if (!u_turn_construct_) {
            AINFO << "[UTURN] generating wide-corridor path";
        } else {
            AINFO << "[UTURN] latch transition path (scenario already exited)";
        }
        u_turn_construct_ = true;

        GetStartPointSLState();

        // 温和拉力：不用零权重，防止车辆在弯道中持续漂移。
        // l_weight=0.5 让优化器保持在 corridor 中点附近（不是 lane 中心），
        // ref_weight=5 提供轻微参考，相比原值(3.0/10000)大幅降低。
        // 这个值在 Lane_1955 的紧曲率半径(2.5m)下仍可行。
        config_.mutable_path_optimizer_config()->set_path_reference_l_weight(5.0);
        config_.mutable_path_optimizer_config()->set_l_weight(0.5);

        std::vector<PathBoundary> candidate_path_boundaries;
        std::vector<PathData> candidate_path_data;

        if (!DecideUTurnPathBoundary(&candidate_path_boundaries)) {
            AERROR << "[UTURN] failed to decide path boundary";
            return Status::OK();
        }
        if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
            // 温和拉力在极紧曲率下仍可能失败，回退到零权重重试
            if (config_.path_optimizer_config().l_weight() > 0.0) {
                AINFO << "[UTURN] gentle weight failed, retrying with zero weight";
                config_.mutable_path_optimizer_config()->set_path_reference_l_weight(0.0);
                config_.mutable_path_optimizer_config()->set_l_weight(0.0);
                if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
                    AERROR << "[UTURN] failed to optimize path (zero weight also failed)";
                    return Status::OK();
                }
                AINFO << "[UTURN] path generated with zero weight fallback";
            } else {
                AERROR << "[UTURN] failed to optimize path";
                return Status::OK();
            }
        }
        // 直接使用优化后的路径，不走 AssessPath（可能因偏离参考线被拒绝）
        if (!candidate_path_data.empty()) {
            *reference_line_info->mutable_path_data() = candidate_path_data.front();
            // 清除 blocking_obstacle_id，防止 RuleBasedStopDecider 因锥桶
            // 障碍物注入 PATH_END stop 墙（"PATH END regular/uturn_wide STOP"）
            reference_line_info->mutable_path_data()->set_blocking_obstacle_id("");
            AINFO << "[UTURN] path generated, label=" << candidate_path_data.front().path_label();
        }

        AddUTurnSpeedLimit(reference_line_info);

        // 标记为借道场景，防止 PathDecider 对 U 型弯内的障碍物
        // 注入 STOP 决策（红色 stop 墙），导致路径被切断。
        auto* mutable_path_decider_status = injector_->planning_context()
                ->mutable_planning_status()->mutable_path_decider();
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);

        IgnoreAllObstacles(reference_line_info);

        // 场景退出后 u_turn_construct_ 提供一帧过渡：下一帧自动进入正常流程。
        if (!contest::IsCurrentScenario(injector_, contest::kUTurnScenario)) {
            u_turn_construct_ = false;
            mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
            AINFO << "[UTURN] latch consumed, returning to normal path generation";
        }

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
    // 这里做一次独立计数；进入施工区后只要前方仍有锥桶，就保持借道模式。
    const int early_cone_count = is_contest_construction
            ? CountConstructionConesAhead(*reference_line_info, contest::kDefaultConstructionLookForwardDistance)
            : 0;

    if (is_contest_construction && early_cone_count > 0) {
        ForceConstructionLaneBorrow(early_cone_count);
    }

    // 施工区锥桶检测优先：跳过 IsNecessaryToBorrowLane() 中 use_self_lane_ 的退出逻辑，
    // 防止刚被强制打开的借道模式又被 UpdateSelfPathInfo → use_self_lane_≥6 关掉，
    // 导致 Force lane borrow → Switch to SELF-LANE → Force lane borrow 的死循环振荡。
    if (!is_contest_construction || early_cone_count <= 0) {
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
    // 施工区模式：降中心线拉力到 0，让路径自由跟随可行通道中心
    // 避免"回中心线趋势"把车拉向锥桶导致碰撞
    if (is_contest_construction && construction_zone_.active) {
        config_.mutable_path_optimizer_config()->set_l_weight(0.0);
        config_.mutable_path_optimizer_config()->set_path_reference_l_weight(0.0);
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
        // 倒车加速：限速 4.5 m/s，比默认更快
        constexpr double kReverseSpeedLimit = 4.5;
        const double adc_s = reference_line_info->AdcSlBoundary().start_s();
        reference_line_info->mutable_reference_line()->AddSpeedLimit(
                adc_s - kReverseDistance, adc_s, kReverseSpeedLimit);
        reference_line_info->SetCruiseSpeed(kReverseSpeedLimit);
        AINFO << "[REVERSE] speed limit " << kReverseSpeedLimit << " m/s for reverse";
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
    // 注意：u_turn_construct_ 不再在此处重置。
    // 其生命周期由 U-turn fast path 管理：
    // - 进入时设为 true
    // - 退出时由 HasUTurnGeometryAhead() 判断后设为 false
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
        const bool is_reverse_path = (path_boundary.label().find("reverse_path") != std::string::npos);
        if (is_reverse_path) {
            if (!reverse_recovery_.path_initialized || path_boundary.empty()
                || reverse_recovery_.reference_line_cache == nullptr) {
                AERROR << "[CZ][REVERSE] skip invalid fixed reverse path, initialized="
                       << reverse_recovery_.path_initialized << ", boundary_empty=" << path_boundary.empty()
                       << ", has_straight_ref=" << (reverse_recovery_.reference_line_cache != nullptr);
                continue;
            }
            const double step = std::fabs(path_boundary.delta_s());
            if (step < 1e-6 || reverse_recovery_.fixed_start_s <= reverse_recovery_.fixed_end_s) {
                AERROR << "[CZ][REVERSE] invalid fixed reverse path range, start_s="
                       << reverse_recovery_.fixed_start_s << ", end_s=" << reverse_recovery_.fixed_end_s
                       << ", step=" << step;
                continue;
            }

            double current_reverse_s = reverse_recovery_.fixed_start_s;
            common::SLPoint current_reverse_sl;
            const common::math::Vec2d adc_xy(frame_->vehicle_state().x(), frame_->vehicle_state().y());
            if (reverse_recovery_.reference_line_cache->XYToSL(adc_xy, &current_reverse_sl)) {
                current_reverse_s = current_reverse_sl.s();
            } else {
                const double dx = frame_->vehicle_state().x() - reverse_recovery_.fixed_start_x;
                const double dy = frame_->vehicle_state().y() - reverse_recovery_.fixed_start_y;
                current_reverse_s = reverse_recovery_.fixed_start_s
                        + dx * std::cos(reverse_recovery_.fixed_heading)
                        + dy * std::sin(reverse_recovery_.fixed_heading);
                AWARN << "[CZ][REVERSE] failed to project ADC to fixed reverse reference, fallback_s="
                      << current_reverse_s;
            }
            current_reverse_s = std::min(
                    reverse_recovery_.fixed_start_s,
                    std::max(reverse_recovery_.fixed_end_s, current_reverse_s));
            const double total_reverse_length = current_reverse_s - reverse_recovery_.fixed_end_s;
            if (total_reverse_length <= 1e-3) {
                AERROR << "[CZ][REVERSE] skip reverse path because current_s already reaches target, current_s="
                       << current_reverse_s << ", end_s=" << reverse_recovery_.fixed_end_s;
                continue;
            }

            FrenetFramePath reverse_frenet;
            for (double traveled = 0.0; traveled <= total_reverse_length + 1e-6; traveled += step) {
                common::FrenetFramePoint pt;
                pt.set_s(current_reverse_s - traveled);
                pt.set_l(reverse_recovery_.fixed_l);
                pt.set_dl(0.0);
                pt.set_ddl(0.0);
                reverse_frenet.push_back(pt);
            }

            last_frame_ = std::make_unique<PathData>();
            last_frame_->SetReferenceLine(reverse_recovery_.reference_line_cache.get());
            if (!last_frame_->SetFrenetPath(std::move(reverse_frenet))) {
                AERROR << "[CZ][REVERSE] failed to set fixed straight reverse path.";
                continue;
            }
            last_frame_->set_path_label(path_boundary.label());
            last_frame_->set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
            last_frame_->set_is_reverse_path(true);
            candidate_path_data->push_back(*last_frame_);
            AINFO << "[CZ][REVERSE] fixed straight reference reverse path, length="
                  << total_reverse_length << ", path_s=[" << current_reverse_s
                  << "->" << reverse_recovery_.fixed_end_s << "], heading="
                  << reverse_recovery_.fixed_heading << ", l=" << reverse_recovery_.fixed_l
                  << ", adc_xy=(" << frame_->vehicle_state().x() << ","
                  << frame_->vehicle_state().y() << ")";
            continue;
        }

        std::vector<double> opt_l, opt_dl, opt_ddl;
        std::vector<std::pair<double, double>> ddl_bounds;
        PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line, &ddl_bounds);
        const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(std::fmax(init_sl_state_.first[1], 1e-12));
        std::vector<double> ref_l;
        std::vector<double> weight_ref_l;
        double ref_weight = config.path_reference_l_weight();
        // U 弯模式用外侧大半径参考；施工区继续用 corridor 中点。
        const bool use_corridor_center =
                (path_boundary.label().find("construct_zone") != std::string::npos)
                || (path_boundary.label().find("uturn_wide") != std::string::npos);
        if (!u_turn_construct_ && !use_corridor_center)
            ref_weight = 50;
        if (use_corridor_center) {
            if (path_boundary.label().find("uturn_wide") != std::string::npos) {
                BuildUTurnLargeRadiusReference(
                        reference_line, path_boundary, ref_weight, &ref_l, &weight_ref_l);
            } else {
                ref_l.resize(path_boundary.size());
                weight_ref_l.resize(path_boundary.size());
                for (size_t i = 0; i < path_boundary.size(); ++i) {
                    ref_l[i] = (path_boundary[i].l_lower.l + path_boundary[i].l_upper.l) * 0.5;
                    weight_ref_l[i] = ref_weight;
                }
            }
        } else {
            PathOptimizerUtil::UpdatePathRefWithBound(path_boundary, ref_weight, &ref_l, &weight_ref_l);
        }

        std::array<double, 3> end_state = {0.0, 0.0, 0.0};
        if (use_corridor_center) {
            double end_l = ref_l.empty() ? 0.0 : ref_l.back();
            end_state = {end_l, 0.0, 0.0};
        }

        SLState opt_init_state = init_sl_state_;

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
            candidate_path_data->push_back(*last_frame_);
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
    const double adc_heading = frame_->vehicle_state().heading();
    const bool close_cone_ahead = HasCloseConstructionConeAhead(*reference_line_info_);
    if (reverse_retrigger_hold_frames_ > 0) {
        --reverse_retrigger_hold_frames_;
    }

    // ── 倒车执行中 ──
    if (reverse_recovery_.active) {
        reverse_recovery_.frame_count++;
        const double backed_dist_s = reverse_recovery_.start_s - adc_s;
        const double backed_dist_xy = std::hypot(adc_x - reverse_recovery_.start_x, adc_y - reverse_recovery_.start_y);

        // 计算沿固定参考线的剩余距离（heading 投影法，避免 XYToSL 依赖）
        double reverse_target_remain = std::numeric_limits<double>::infinity();
        if (reverse_recovery_.path_initialized && reverse_recovery_.reference_line_cache != nullptr) {
            const double dx = adc_x - reverse_recovery_.fixed_start_x;
            const double dy = adc_y - reverse_recovery_.fixed_start_y;
            const double current_reverse_s = reverse_recovery_.fixed_start_s
                    + dx * std::cos(reverse_recovery_.fixed_heading)
                    + dy * std::sin(reverse_recovery_.fixed_heading);
            reverse_target_remain = std::max(0.0, current_reverse_s - reverse_recovery_.fixed_end_s);
        }

        const double reverse_distance = std::max(kMinReverseDistance, kReverseDistance);
        const double reverse_target_remain_threshold =
                std::max(kMinReverseTargetRemainThreshold, kReverseTargetRemainThreshold);
        const bool backed_enough_by_xy = backed_dist_xy > reverse_distance;
        const bool backed_enough_by_ref = reverse_target_remain <= reverse_target_remain_threshold;
        const bool backed_enough = reverse_recovery_.frame_count >= kReverseMinFrames
                && (backed_enough_by_xy || backed_enough_by_ref);
        const bool reverse_timeout = reverse_recovery_.frame_count > kReverseMaxFrames;

        if (backed_enough || reverse_timeout) {
            const std::string reason = reverse_timeout
                    ? "timeout"
                    : (backed_enough_by_xy ? "distance" : "ref_target");
            AINFO << "[CZ][REVERSE] complete: reason=" << reason
                  << ", backed_xy=" << backed_dist_xy << "m, backed_s=" << backed_dist_s
                  << "m, target_remain=" << reverse_target_remain
                  << "m, target_threshold=" << reverse_target_remain_threshold
                  << "m, reverse_distance=" << reverse_distance
                  << "m, frames=" << reverse_recovery_.frame_count;
            reverse_recovery_.Reset();
            construction_reverse_completed_ = true;
            reverse_retrigger_hold_frames_ = kReverseRetriggerHoldFrames;
            reverse_finish_x_ = adc_x;
            reverse_finish_y_ = adc_y;
            return false;
        }

        // 缓存有效性检查
        if (!reverse_recovery_.path_initialized || reverse_recovery_.boundary_cache.empty()) {
            AERROR << "[CZ][REVERSE] fixed reverse path cache is empty, abort reverse session.";
            reverse_recovery_.Reset();
            return false;
        }

        if (reverse_recovery_.frame_count % 10 == 1) {
            AINFO << "[CZ][REVERSE] reuse fixed reverse path: frame=" << reverse_recovery_.frame_count
                  << ", backed_xy=" << backed_dist_xy << "m, target_remain=" << reverse_target_remain
                  << "m, points=" << reverse_recovery_.boundary_cache.size();
        }
        boundary->push_back(reverse_recovery_.boundary_cache);
        return true;
    }

    // ── 卡死检测（未进入倒车）──
    const double adc_move_xy = std::hypot(adc_x - reverse_recovery_.last_adc_x, adc_y - reverse_recovery_.last_adc_y);
    // 发车点前方有锥桶紧贴：降低触发门槛，加快倒车响应
    // stuck_frame_threshold: 15→5 (0.5s @ 10Hz 即可触发)
    // stuck_speed_threshold: 0.3→0.5 (稍微放宽速度判定)
    static constexpr double kStuckSpeedThresholdFast = 0.5;
    static constexpr int kStuckFrameThresholdFast = 5;
    if (adc_speed < kStuckSpeedThresholdFast && adc_move_xy < 0.3) {
        reverse_recovery_.frame_count++;
    } else {
        reverse_recovery_.frame_count = 0;
    }
    reverse_recovery_.last_adc_s = adc_s;
    reverse_recovery_.last_adc_x = adc_x;
    reverse_recovery_.last_adc_y = adc_y;

    const double move_after_reverse =
            std::hypot(adc_x - reverse_finish_x_, adc_y - reverse_finish_y_);
    const bool first_reverse_allowed = !construction_reverse_completed_ && close_cone_ahead;
    const bool repeat_reverse_allowed =
            construction_reverse_completed_ &&
            reverse_retrigger_hold_frames_ <= 0 &&
            move_after_reverse > kReverseRetriggerMinMove &&
            reverse_recovery_.frame_count >= kRepeatReverseStuckFrameThreshold;
    if (!first_reverse_allowed && !repeat_reverse_allowed &&
        reverse_recovery_.frame_count < kStuckFrameThresholdFast) {
        return false;
    }
    if (construction_reverse_completed_ && !repeat_reverse_allowed) {
        if (close_cone_ahead && reverse_retrigger_hold_frames_ % 20 == 0) {
            AINFO << "[CZ][REVERSE] skip retrigger after completed reverse, hold_frames="
                  << reverse_retrigger_hold_frames_ << ", moved_after_reverse="
                  << move_after_reverse << ", stuck_frames="
                  << reverse_recovery_.frame_count;
        }
        return false;
    }

    const int trigger_frame_count = reverse_recovery_.frame_count;
    // ── 进入倒车模式：构建固定直线参考线 + 缓存边界 ──
    reverse_recovery_.active = true;
    reverse_recovery_.start_s = adc_s;
    reverse_recovery_.start_x = adc_x;
    reverse_recovery_.start_y = adc_y;
    reverse_recovery_.frame_count = 0;
    reverse_recovery_.path_initialized = false;
    reverse_recovery_.boundary_cache.clear();
    reverse_recovery_.reference_line_cache.reset();

    AINFO << "[CZ][REVERSE] trigger, close_cone=" << close_cone_ahead
          << ", stuck_frames=" << trigger_frame_count
          << ", speed=" << adc_speed << ", start_xy=("
          << adc_x << "," << adc_y << "), heading=" << adc_heading;

    const double reverse_distance = std::max(kMinReverseDistance, kReverseDistance);
    if (!BuildReverseStraightReferenceLine(&reverse_recovery_, adc_x, adc_y, adc_heading, reverse_distance)) {
        AERROR << "[CZ][REVERSE] failed to build reverse straight reference line.";
        reverse_recovery_.Reset();
        return true;
    }

    if (!GenerateCachedReversePathBoundary(reverse_recovery_, &reverse_recovery_.boundary_cache)) {
        AERROR << "[CZ][REVERSE] failed to generate cached reverse boundary.";
        reverse_recovery_.Reset();
        return true;
    }

    reverse_recovery_.path_initialized = true;
    boundary->push_back(reverse_recovery_.boundary_cache);
    return true;
}

void ContestLaneBorrowPath::ForceConstructionLaneBorrow(int cone_count) {
    auto* mutable_path_decider_status
            = injector_->planning_context()->mutable_planning_status()->mutable_path_decider();
    if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
        construction_zone_.active = true;
        construction_zone_.low_cone_counter = 0;
        construction_zone_.no_cone_counter = 0;
        return;
    }
    mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
    construction_zone_.active = true;
    construction_zone_.low_cone_counter = 0;
    construction_zone_.no_cone_counter = 0;
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
                "ContestLaneBorrowPath/ignore-uturn-obstacle", obs->Id(), object_decision);
        reference_line_info->path_decision()->AddLateralDecision(
                "ContestLaneBorrowPath/ignore-uturn-obstacle", obs->Id(), object_decision);
    }
}

void ContestLaneBorrowPath::ResetConstructZoneState(const std::string& reason) {
    construction_zone_.Reset();
    reverse_recovery_.Reset();
    construction_reverse_completed_ = false;
    reverse_retrigger_hold_frames_ = 0;
    reverse_finish_x_ = 0.0;
    reverse_finish_y_ = 0.0;
    AINFO << "[WALL] EXIT construct_zone by " << reason;
}

bool ContestLaneBorrowPath::DecideUTurnPathBoundary(std::vector<PathBoundary>* boundary) {
    // U 弯宽走廊策略:
    // 参考线半径小 → 优化器需要更大的横向空间来找平滑曲线。
    // 先用 road edge 扩展，若宽度不足则强制扩展到至少 ±3m (总宽 6m)。
    if (boundary == nullptr) {
        return false;
    }
    boundary->clear();

    PathBoundary path_bound;
    std::string blocking_obstacle_id = "";
    double path_narrowest_width = 0;

    // 1. 初始化基础边界
    if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_)) {
        AERROR << "[UTURN] Failed to initialize path boundary";
        return false;
    }

    // 2. 使用 road boundary 扩展（比 lane boundary 更宽，包含路肩等）
    //    再叠加左右借道，确保覆盖到相邻车道
    if (!PathBoundsDeciderUtil::GetBoundaryFromRoad(*reference_line_info_, init_sl_state_, &path_bound)) {
        AWARN << "[UTURN] GetBoundaryFromRoad failed, trying lane-based expansion";
    }
    std::string left_type;
    GetBoundaryFromNeighborLane(SidePassDirection::LEFT_BORROW, &path_bound, &left_type);
    std::string right_type;
    GetBoundaryFromNeighborLane(SidePassDirection::RIGHT_BORROW, &path_bound, &right_type);

    // 3. U 弯专用：强制走廊接近两车道宽度。
    //    Lane_1955 的中心线半径很小，必须允许路径提前外抛到第二车道。
    constexpr double kMinUTurnHalfWidth = 4.5;  // 单侧最小半宽
    int narrow_count = 0;
    for (size_t i = 0; i < path_bound.size(); ++i) {
        const double half_width = std::min(
                std::fabs(path_bound[i].l_lower.l),
                std::fabs(path_bound[i].l_upper.l));
        if (half_width < kMinUTurnHalfWidth) {
            ++narrow_count;
        }
        path_bound[i].l_lower.l = std::min(path_bound[i].l_lower.l, -kMinUTurnHalfWidth);
        path_bound[i].l_upper.l = std::max(path_bound[i].l_upper.l, kMinUTurnHalfWidth);
    }
    AINFO << "[UTURN] enforced min half-width=" << kMinUTurnHalfWidth
          << "m, narrow points=" << narrow_count << "/" << path_bound.size();

    // 4. 障碍物避让（轻量：只避让大障碍物，小锥桶用 nudge）
    PathBoundary temp_path_bound = path_bound;
    std::vector<SLPolygon> obs_sl_polygons;
    PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons, init_sl_state_);
    ApplyUTurnConeNudge(&obs_sl_polygons);

    double saved_lat_buffer = FLAGS_obstacle_lat_buffer;
    FLAGS_obstacle_lat_buffer = 0.1;  // 放宽障碍物横向缓冲
    FLAGS_obstacle_lon_end_buffer_park = 5.0;
    if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
                *reference_line_info_,
                &obs_sl_polygons,
                init_sl_state_,
                &path_bound,
                &blocking_obstacle_id,
                &path_narrowest_width)) {
        FLAGS_obstacle_lat_buffer = saved_lat_buffer;
        AWARN << "[UTURN] obstacle boundary refinement failed, using expanded boundary";
        // 不 fatal：用扩展后的边界继续
        path_bound = temp_path_bound;
        // 重新强制最小宽度
        for (size_t i = 0; i < path_bound.size(); ++i) {
            path_bound[i].l_lower.l = std::min(path_bound[i].l_lower.l, -kMinUTurnHalfWidth);
            path_bound[i].l_upper.l = std::max(path_bound[i].l_upper.l, kMinUTurnHalfWidth);
        }
    }
    FLAGS_obstacle_lat_buffer = saved_lat_buffer;

    // 5. 补尾点防止零长度路径
    int counter = 0;
    while (!blocking_obstacle_id.empty() && path_bound.size() < temp_path_bound.size()
           && counter < FLAGS_num_extra_tail_bound_point) {
        path_bound.push_back(temp_path_bound[path_bound.size()]);
        counter++;
    }

    path_bound.set_label("regular/self/uturn_wide");
    path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
    RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
    boundary->push_back(path_bound);

    AINFO << "[UTURN] wide boundary generated, size=" << path_bound.size()
          << ", label=" << path_bound.label();
    return true;
}

void ContestLaneBorrowPath::AddUTurnSpeedLimit(ReferenceLineInfo* reference_line_info) const {
    if (reference_line_info == nullptr) {
        return;
    }
    // U 弯区域限速：从当前 ADC 位置起，覆盖整个 U 弯区域
    // 限速 5.0 m/s (18 km/h)，适合低速调头
    constexpr double kUTurnSpeedLimit = 5.0;     // m/s
    constexpr double kUTurnSpeedBuffer = 20.0;   // 提前 20m 开始减速
    const double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
    const double ref_length = reference_line_info->reference_line().Length();
    const double limit_end_s = std::min(ref_length, adc_end_s + 80.0);
    reference_line_info->mutable_reference_line()->AddSpeedLimit(
            adc_end_s - kUTurnSpeedBuffer, limit_end_s, kUTurnSpeedLimit);
    AINFO << "[UTURN] speed limit " << kUTurnSpeedLimit << " m/s from s="
          << (adc_end_s - kUTurnSpeedBuffer) << " to " << limit_end_s;
}

bool ContestLaneBorrowPath::HasUTurnGeometryAhead(const ReferenceLineInfo& reference_line_info) const {
    // 检查前方车道是否仍为 U_TURN 类型（用于 U 型弯退出 latch 判断）
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    const double ref_length = reference_line_info.reference_line().Length();
    static constexpr double kCheckForwardDistance = 60.0;
    for (double s = adc_end_s;
         s < adc_end_s + kCheckForwardDistance && s < ref_length;
         s += 5.0) {
        if (reference_line_info.GetPathTurnType(s) == hdmap::Lane::U_TURN) {
            return true;
        }
    }
    // 也检查当前位置（车辆可能刚好在 U_TURN 段上）
    if (reference_line_info.GetPathTurnType(adc_end_s) == hdmap::Lane::U_TURN) {
        return true;
    }
    return false;
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
