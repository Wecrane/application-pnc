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

#include "modules/planning/tasks/contest_lane_change_path/contest_lane_change_path.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"
#include "modules/planning/planning_base/common/contest_scenario_status.h"
#include "modules/planning/tasks/contest_lane_change_path/contest_lane_change_path_helper.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::cyber::Clock;

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;
constexpr double kContestLaneChangeSuccessCooldown = 3.0;

bool ContestLaneChangePath::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!Task::Init(config_dir, name, injector)) {
        return false;
    }
    // Load the config this task.
    return Task::LoadConfig<ContestLaneChangePathConfig>(&config_);
}

apollo::common::Status ContestLaneChangePath::Process(Frame* frame, ReferenceLineInfo* reference_line_info) {
    ADEBUG << "[LC_PROCESS] called, is_change_lane=" << reference_line_info->IsChangeLanePath()
           << " path_reusable=" << reference_line_info->path_reusable()
           << " ref_line_count=" << frame->reference_line_info().size();
    const bool is_roundabout_scenario = contest::IsCurrentScenario(injector_, contest::kRoundaboutScenario);
    if (contest::IsCurrentScenario(injector_, contest::kUTurnScenario) || is_roundabout_scenario) {
        const double now = Clock::NowInSeconds();
        const auto& change_lane_status = injector_->planning_context()->planning_status().change_lane();
        if (is_roundabout_scenario) {
            // Roundabout suppresses change-lane reference lines only for the
            // current frame. Keep the persistent lane-change state immediately
            // reusable so the post-roundabout contest lane-change can start as
            // soon as the next scenario is selected.
            if (!change_lane_status.has_status()
                || change_lane_status.status() == ChangeLaneStatus::IN_CHANGE_LANE
                || now - change_lane_status.timestamp() < kContestLaneChangeSuccessCooldown
                || !change_lane_status.path_id().empty()) {
                UpdateStatus(
                        now - kContestLaneChangeSuccessCooldown, ChangeLaneStatus::CHANGE_LANE_FINISHED, "");
            }
            lane_change_window_armed_ = true;
            consecutive_clear_count_ = 0;
            consecutive_occupied_count_ = 0;
            lane_change_window_open_count_ = 0;
            consecutive_empty_frames_ = 0;
        } else if (
                !change_lane_status.has_status()
                || change_lane_status.status() == ChangeLaneStatus::IN_CHANGE_LANE) {
            UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, "");
        }
        if (is_roundabout_scenario && reference_line_info->IsChangeLanePath()) {
            reference_line_info->SetDrivable(false);
            AINFO << "[ROUNDABOUT][LaneChangePath] skip change-lane path, lane_id="
                  << reference_line_info->Lanes().Id();
        }
        return Status::OK();
    }
    UpdateLaneChangeStatus();

    const bool is_contest_lane_change = IsContestLaneChangeScenario(injector_);
    ApplyContestLaneChangeSpeedLimit(is_contest_lane_change, reference_line_info);

    const auto& status = injector_->planning_context()->mutable_planning_status()->mutable_change_lane()->status();
    if (is_contest_lane_change && reference_line_info->IsChangeLanePath()
        && status != ChangeLaneStatus::IN_CHANGE_LANE) {
        reference_line_info->set_path_reusable(false);
        reference_line_info->SetDrivable(false);
        return Status(ErrorCode::PLANNING_ERROR, "Contest lane change is cooling down or not triggered");
    }
    if (!reference_line_info->IsChangeLanePath() || reference_line_info->path_reusable()) {
        ADEBUG << "[LC_PROCESS] SKIP: not change_lane or path reusable, returning OK";
        return Status::OK();
    }
    if (!is_contest_lane_change && status != ChangeLaneStatus::IN_CHANGE_LANE) {
        AINFO << injector_->planning_context()->mutable_planning_status()->mutable_change_lane()->DebugString();
        return Status(ErrorCode::PLANNING_ERROR, "Not satisfy lane change conditions");
    }

    ADEBUG << "[LC_PROCESS] GENERATING lane change path, status=" << status << " clear=" << is_clear_to_change_lane_;
    // 始终规划路径：无车时继续往目标车道挪，有车时锁住当前横向位置直行
    std::vector<PathBoundary> candidate_path_boundaries;
    std::vector<PathData> candidate_path_data;

    GetStartPointSLState();
    if (!DecidePathBounds(&candidate_path_boundaries)) {
        ADEBUG << "[LC_PROCESS] FAIL: DecidePathBounds failed";
        return Status(ErrorCode::PLANNING_ERROR, "lane change path bounds failed");
    }
    if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
        ADEBUG << "[LC_PROCESS] FAIL: OptimizePath failed";
        return Status(ErrorCode::PLANNING_ERROR, "lane change path optimize failed");
    }
    if (!AssessPath(&candidate_path_data, reference_line_info->mutable_path_data())) {
        ADEBUG << "[LC_PROCESS] FAIL: AssessPath failed, no valid path";
        return Status(ErrorCode::PLANNING_ERROR, "No valid lane change path");
    }
    ADEBUG << "[LC_PROCESS] SUCCESS: lane change path generated";

    // 赛题二：变道路径一旦生成，就不让后续 stop/follow 逻辑把它截断
    if (is_contest_lane_change) {
        IgnoreObstaclesForContestLaneChange(is_contest_lane_change, reference_line_info);
    }

    return Status::OK();
}

bool ContestLaneChangePath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
    boundary->emplace_back();
    auto& path_bound = boundary->back();
    double path_narrowest_width = 0;
    // 1. Initialize the path boundaries to be an indefinitely large area.
    if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_)) {
        const std::string msg = "Failed to initialize path boundaries.";
        AERROR << msg;
        return false;
    }
    // 2. Decide a rough boundary based on lane info and ADC's position
    if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(*reference_line_info_, init_sl_state_, &path_bound)) {
        AERROR << "Failed to decide a rough boundary based on self lane.";
        return false;
    }
    if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(
                *reference_line_info_, init_sl_state_, config_.extend_adc_buffer(), &path_bound)) {
        AERROR << "Failed to decide a rough boundary based on adc.";
        return false;
    }

    // 3. Remove the S-length of target lane out of the path-bound.
    GetBoundaryFromLaneChangeForbiddenZone(&path_bound);

    path_bound.set_label("regular/lane_change");

    PathBound temp_path_bound = path_bound;
    std::string blocking_obstacle_id;
    std::vector<SLPolygon> obs_sl_polygons;
    const bool should_hold_laterally
            = ShouldHoldLateralInContestLaneChange(IsContestLaneChangeScenario(injector_), is_clear_to_change_lane_);
    if (should_hold_laterally) {
        ApplyContestLaneChangeHoldBoundary(init_sl_state_.second[0], &path_bound);
    } else {
        PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons, init_sl_state_);
        if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
                    *reference_line_info_,
                    &obs_sl_polygons,
                    init_sl_state_,
                    &path_bound,
                    &blocking_obstacle_id,
                    &path_narrowest_width)) {
            AERROR << "Failed to decide fine tune the boundaries after "
                      "taking into consideration all static obstacles.";
            return false;
        }
    }

    // Append some extra path bound points to avoid zero-length path data.
    int counter = 0;
    while (!blocking_obstacle_id.empty() && path_bound.size() < temp_path_bound.size()
           && counter < FLAGS_num_extra_tail_bound_point) {
        path_bound.push_back(temp_path_bound[path_bound.size()]);
        counter++;
    }
    path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
    RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
    return true;
}
bool ContestLaneChangePath::OptimizePath(
        const std::vector<PathBoundary>& path_boundaries,
        std::vector<PathData>* candidate_path_data) {
    const auto& config = config_.path_optimizer_config();
    const ReferenceLine& reference_line = reference_line_info_->reference_line();
    const bool should_hold_laterally
            = ShouldHoldLateralInContestLaneChange(IsContestLaneChangeScenario(injector_), is_clear_to_change_lane_);
    ADEBUG << "[LC_OPT] hold_laterally=" << should_hold_laterally << " current_l=" << init_sl_state_.second[0];
    std::array<double, 3> end_state = {should_hold_laterally ? init_sl_state_.second[0] : 0.0, 0.0, 0.0};
    for (const auto& path_boundary : path_boundaries) {
        size_t path_boundary_size = path_boundary.boundary().size();
        if (path_boundary_size <= 1U) {
            AERROR << "Get invalid path boundary with size: " << path_boundary_size;
            return false;
        }
        std::vector<double> opt_l, opt_dl, opt_ddl;
        std::vector<std::pair<double, double>> ddl_bounds;
        PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line, &ddl_bounds);
        const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(std::fmax(init_sl_state_.first[1], 1e-12));
        std::vector<double> ref_l(path_boundary_size, should_hold_laterally ? init_sl_state_.second[0] : 0.0);
        std::vector<double> weight_ref_l(
                path_boundary_size, should_hold_laterally ? config.path_reference_l_weight() : 0.0);

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
        }
    }
    if (candidate_path_data->empty()) {
        return false;
    }
    return true;
}

bool ContestLaneChangePath::AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path) {
    std::vector<PathData> valid_path_data;
    for (auto& curr_path_data : *candidate_path_data) {
        if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_, curr_path_data)) {
            SetPathInfo(&curr_path_data);
            if (reference_line_info_->SDistanceToDestination() < FLAGS_path_trim_destination_threshold) {
                PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data);
            }
            if (curr_path_data.Empty()) {
                AINFO << "lane change path is empty after trimed";
                continue;
            }
            valid_path_data.push_back(curr_path_data);
        }
    }
    if (valid_path_data.empty()) {
        AINFO << "All lane change path are not valid";
        return false;
    }

    *final_path = valid_path_data[0];
    RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
    return true;
}

void ContestLaneChangePath::UpdateLaneChangeStatus() {
    std::string change_lane_id;
    auto* prev_status = injector_->planning_context()->mutable_planning_status()->mutable_change_lane();
    double now = Clock::NowInSeconds();
    const bool is_contest_lane_change = IsContestLaneChangeScenario(injector_);
    // Init lane change status
    if (!prev_status->has_status()) {
        AINFO << "[LC_STATUS] INIT: no prev status, setting FINISHED";
        UpdateStatus(now - kContestLaneChangeSuccessCooldown, ChangeLaneStatus::CHANGE_LANE_FINISHED, "");
        return;
    }
    bool has_change_lane = frame_->reference_line_info().size() > 1;
    if (!has_change_lane) {
        if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
            AINFO << "[LC_STATUS] EXIT: no change lane (size=" << frame_->reference_line_info().size()
                  << "), was IN_CHANGE_LANE -> FINISHED";
            UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, prev_status->path_id());
        }
        consecutive_clear_count_ = 0;
        consecutive_occupied_count_ = 0;
        lane_change_window_armed_ = false;
        lane_change_window_open_count_ = 0;
        consecutive_empty_frames_ = 0;
        return;
    }
    // has change lane
    if (reference_line_info_->IsChangeLanePath()) {
        const bool raw_window_clear = IsContestLaneChangeWindowClear(reference_line_info_);
        is_clear_to_change_lane_ = raw_window_clear;
        if (is_contest_lane_change && prev_status->status() != ChangeLaneStatus::IN_CHANGE_LANE) {
            if (raw_window_clear) {
                consecutive_occupied_count_ = 0;
                if (lane_change_window_armed_) {
                    ++consecutive_clear_count_;
                    if (consecutive_clear_count_ >= kRequiredConsecutiveClearFrames
                        && lane_change_window_open_count_ < kLaneChangeWindowHoldFrames) {
                        is_clear_to_change_lane_ = true;
                        ++lane_change_window_open_count_;
                    } else {
                        is_clear_to_change_lane_ = false;
                        if (lane_change_window_open_count_ >= kLaneChangeWindowHoldFrames) {
                            lane_change_window_armed_ = false;
                        }
                    }
                } else {
                    // 无车通过但窗口持续为空：累计空帧数，超时自动触发
                    consecutive_clear_count_ = 0;
                    ++consecutive_empty_frames_;
                    if (consecutive_empty_frames_ >= kEmptyLaneAutoArmFrames) {
                        lane_change_window_armed_ = true;
                        consecutive_empty_frames_ = 0;
                        AINFO << "[LC_STATUS] lane empty for " << kEmptyLaneAutoArmFrames
                              << " frames, auto-arming window";
                    }
                    is_clear_to_change_lane_ = false;
                }
            } else {
                consecutive_clear_count_ = 0;
                lane_change_window_open_count_ = 0;
                consecutive_empty_frames_ = 0;  // 有车进入，重置空帧计数
                ++consecutive_occupied_count_;
                if (consecutive_occupied_count_ >= kRequiredConsecutiveOccupiedFrames) {
                    lane_change_window_armed_ = true;
                }
                is_clear_to_change_lane_ = false;
            }
            AINFO << "[LC_STATUS] window raw_clear=" << raw_window_clear
                  << " occupied_frames=" << consecutive_occupied_count_ << " clear_frames=" << consecutive_clear_count_
                  << " armed=" << lane_change_window_armed_ << " open_frames=" << lane_change_window_open_count_
                  << " trigger=" << is_clear_to_change_lane_;
        }
        change_lane_id = reference_line_info_->Lanes().Id();
        double ego_speed = frame_->vehicle_state().linear_velocity();
        const double min_lane_change_speed = ContestLaneChangeMinStartSpeed();

        if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
            double elapsed = now - prev_status->timestamp();
            if (elapsed > config_.change_lane_fail_freeze_time() && ego_speed >= min_lane_change_speed
                && is_clear_to_change_lane_
                && (!is_contest_lane_change || consecutive_clear_count_ >= kRequiredConsecutiveClearFrames)) {
                AINFO << "[LC_STATUS] RETRY: FAILED -> IN_CHANGE_LANE, elapsed=" << elapsed
                      << " freeze=" << config_.change_lane_fail_freeze_time() << " speed=" << ego_speed * 3.6
                      << " clear_frames=" << consecutive_clear_count_;
                UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
                return;
            }
            is_clear_to_change_lane_ = false;
            return;
        } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FINISHED) {
            const double elapsed = now - prev_status->timestamp();
            const double success_freeze_time = is_contest_lane_change
                    ? std::max(config_.change_lane_success_freeze_time(), kContestLaneChangeSuccessCooldown)
                    : config_.change_lane_success_freeze_time();
            if (elapsed <= success_freeze_time) {
                AINFO << "[LC_STATUS] WAIT: cooldown elapsed=" << elapsed << " freeze=" << success_freeze_time;
                is_clear_to_change_lane_ = false;
                if (raw_window_clear) {
                    consecutive_clear_count_ = 0;
                    lane_change_window_armed_ = false;
                    lane_change_window_open_count_ = 0;
                    consecutive_empty_frames_ = 0;
                }
                return;
            }
            if (is_contest_lane_change && !is_clear_to_change_lane_) {
                AINFO << "[LC_STATUS] WAIT: no debounced pass-by window";
                return;
            }
            // 赛题二：先提速到最低变道速度，再开始找变道窗口。
            if (is_contest_lane_change && ego_speed < min_lane_change_speed) {
                AINFO << "[LC_STATUS] WAIT: speed=" << ego_speed * 3.6 << " km/h < " << min_lane_change_speed * 3.6
                      << ", waiting to accelerate";
                return;
            }
            // 连续安全帧确认：需连续 kRequiredConsecutiveClearFrames 帧安全
            if (is_contest_lane_change && consecutive_clear_count_ < kRequiredConsecutiveClearFrames) {
                AINFO << "[LC_STATUS] WAIT: clear_frames=" << consecutive_clear_count_ << "/"
                      << kRequiredConsecutiveClearFrames << ", waiting for stable safety";
                return;
            }
            AINFO << "[LC_STATUS] START: FINISHED -> IN_CHANGE_LANE, elapsed=" << elapsed
                  << " speed=" << ego_speed * 3.6 << " km/h id=" << change_lane_id
                  << " clear_frames=" << consecutive_clear_count_;
            UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
        } else if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
            if (prev_status->path_id() != change_lane_id) {
                AINFO << "[LC_STATUS] SWITCH: IN_CHANGE_LANE but id changed (prev=" << prev_status->path_id()
                      << " now=" << change_lane_id << ") -> FINISHED";
                UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, prev_status->path_id());
                is_clear_to_change_lane_ = false;
                consecutive_clear_count_ = 0;
                consecutive_occupied_count_ = 0;
                lane_change_window_armed_ = false;
                lane_change_window_open_count_ = 0;
                consecutive_empty_frames_ = 0;
            } else {
                is_clear_to_change_lane_ = true;
                AINFO << "[LC_STATUS] CONTINUE: IN_CHANGE_LANE, clear=" << is_clear_to_change_lane_
                      << " id=" << change_lane_id;
            }
        }
    }
}

void ContestLaneChangePath::GetLaneChangeStartPoint(
        const ReferenceLine& reference_line,
        double adc_frenet_s,
        common::math::Vec2d* start_xy) {
    double lane_change_start_s = config_.lane_change_prepare_length() + adc_frenet_s;
    common::SLPoint lane_change_start_sl;
    lane_change_start_sl.set_s(lane_change_start_s);
    lane_change_start_sl.set_l(0.0);
    reference_line.SLToXY(lane_change_start_sl, start_xy);
}

void ContestLaneChangePath::GetBoundaryFromLaneChangeForbiddenZone(PathBoundary* const path_bound) {
    // Sanity checks.
    CHECK_NOTNULL(path_bound);

    if (is_clear_to_change_lane_) {
        is_exist_lane_change_start_position_ = false;
        return;
    }
    double lane_change_start_s = 0.0;
    const ReferenceLine& reference_line = reference_line_info_->reference_line();
    // If there is a pre-determined lane-change starting position, then use it;
    // otherwise, decide one.
    if (is_exist_lane_change_start_position_) {
        common::SLPoint point_sl;
        reference_line.XYToSL(lane_change_start_xy_, &point_sl);
        lane_change_start_s = point_sl.s();
    } else {
        // TODO(jiacheng): train ML model to learn this.
        lane_change_start_s = config_.lane_change_prepare_length() + init_sl_state_.first[0];

        // Update the lane_change_start_xy_ decided by lane_change_start_s
        GetLaneChangeStartPoint(reference_line, init_sl_state_.first[0], &lane_change_start_xy_);
    }

    // Remove the target lane out of the path-boundary, up to the decided S.
    if (lane_change_start_s < init_sl_state_.first[0]) {
        // If already passed the decided S, then return.
        return;
    }
    double adc_half_width = VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
    for (size_t i = 0; i < path_bound->size(); ++i) {
        double curr_s = (*path_bound)[i].s;
        if (curr_s > lane_change_start_s) {
            break;
        }
        double curr_lane_left_width = 0.0;
        double curr_lane_right_width = 0.0;
        double offset_to_map = 0.0;
        reference_line.GetOffsetToMap(curr_s, &offset_to_map);
        if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width, &curr_lane_right_width)) {
            double offset_to_lane_center = 0.0;
            reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
            curr_lane_left_width += offset_to_lane_center;
            curr_lane_right_width -= offset_to_lane_center;
        }
        curr_lane_left_width -= offset_to_map;
        curr_lane_right_width += offset_to_map;

        (*path_bound)[i].l_lower.l = init_sl_state_.second[0] > curr_lane_left_width
                ? curr_lane_left_width + adc_half_width
                : (*path_bound)[i].l_lower.l;
        (*path_bound)[i].l_lower.l = std::fmin((*path_bound)[i].l_lower.l, init_sl_state_.second[0] - 0.1);
        (*path_bound)[i].l_upper.l = init_sl_state_.second[0] < -curr_lane_right_width
                ? -curr_lane_right_width - adc_half_width
                : (*path_bound)[i].l_upper.l;
        (*path_bound)[i].l_upper.l = std::fmax((*path_bound)[i].l_upper.l, init_sl_state_.second[0] + 0.1);
    }
}

void ContestLaneChangePath::UpdateStatus(
        double timestamp,
        ChangeLaneStatus::Status status_code,
        const std::string& path_id) {
    auto* lane_change_status = injector_->planning_context()->mutable_planning_status()->mutable_change_lane();
    AINFO << "lane change update from" << lane_change_status->DebugString() << "to";
    lane_change_status->set_timestamp(timestamp);
    lane_change_status->set_path_id(path_id);
    lane_change_status->set_status(status_code);
    AINFO << lane_change_status->DebugString();
}

bool ContestLaneChangePath::HysteresisFilter(
        const double obstacle_distance,
        const double safe_distance,
        const double distance_buffer,
        const bool is_obstacle_blocking) {
    if (is_obstacle_blocking) {
        return obstacle_distance < safe_distance + distance_buffer;
    } else {
        return obstacle_distance < safe_distance - distance_buffer;
    }
}

void ContestLaneChangePath::SetPathInfo(PathData* const path_data) {
    std::vector<PathPointDecision> path_decision;
    PathAssessmentDeciderUtil::InitPathPointDecision(*path_data, PathData::PathPointType::IN_LANE, &path_decision);
    // Go through every path_point, and add in-lane/out-of-lane info.
    const auto& discrete_path = path_data->discretized_path();
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
            // For lane-change path, only transitioning part is labeled as
            // out-of-lane.
            if (ego_sl_boundary.start_l() > lane_left_width || ego_sl_boundary.end_l() < -lane_right_width) {
                // This means that ADC hasn't started lane-change yet.
                std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
            } else if (
                    ego_sl_boundary.start_l() > -lane_right_width + back_to_inlane_extra_buffer
                    && ego_sl_boundary.end_l() < lane_left_width - back_to_inlane_extra_buffer) {
                // This means that ADC has safely completed lane-change with margin.
                std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
            } else {
                // ADC is right across two lanes.
                std::get<1>((path_decision)[i]) = PathData::PathPointType::OUT_ON_FORWARD_LANE;
            }
        } else {
            AERROR << "reference line not ready when setting path point guide";
            return;
        }
    }
    path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

bool ContestLaneChangePath::CheckLastFrameSucceed(const apollo::planning::Frame* const last_frame) {
    if (IsContestLaneChangeScenario(injector_)) {
        // 赛题二：不做失败检测，始终直线保持等机会
        return true;
    }
    if (last_frame) {
        for (const auto& reference_line_info : last_frame->reference_line_info()) {
            if (!reference_line_info.IsChangeLanePath()) {
                continue;
            }
            const auto history_trajectory_type = reference_line_info.trajectory_type();
            if (history_trajectory_type == ADCTrajectory::SPEED_FALLBACK) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace planning
}  // namespace apollo
