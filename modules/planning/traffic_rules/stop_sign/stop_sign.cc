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

#include "modules/planning/traffic_rules/stop_sign/stop_sign.h"

#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include "cyber/time/clock.h"
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

bool IsContestRoundaboutScenario(
    const std::shared_ptr<DependencyInjector>& injector) {
  if (injector == nullptr || injector->planning_context() == nullptr) {
    return false;
  }
  // 方式 1：当前场景即为 CONTEST_ROUNDABOUT
  if (injector->planning_context()->planning_status().scenario().scenario_type()
      == kContestRoundaboutScenarioName) {
    return true;
  }
  // 方式 2：场景已退出但提交标志仍有效（one-shot 机制）
  if (injector->planning_context()->planning_status()
          .path_decider().is_in_path_lane_borrow_scenario()) {
    return true;
  }
  return false;
}

bool IsRoundaboutNonTargetLaneVehicle(const Obstacle* obstacle) {
  if (obstacle == nullptr || obstacle->IsVirtual() || obstacle->IsStatic() ||
      obstacle->Perception().type() != apollo::perception::PerceptionObstacle::VEHICLE) {
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
    const auto* obstacle =
        reference_line_info.path_decision().obstacles().Find(obstacle_id);
    if (IsRoundaboutNonTargetLaneVehicle(obstacle)) {
      AINFO << "[ROUNDABOUT][StopSign] drop non-target-lane wait_for obs="
            << obstacle_id;
      continue;
    }
    filtered.push_back(obstacle_id);
  }
  return filtered;
}

}  // namespace

bool StopSign::Init(const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<StopSignConfig>(&config_);
}

Status StopSign::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void StopSign::MakeDecisions(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.enabled()) {
    return;
  }

  const auto& stop_sign_status =
      injector_->planning_context()->planning_status().stop_sign();
  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();

  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info->reference_line().map_path().stop_sign_overlaps();
  for (const auto& stop_sign_overlap : stop_sign_overlaps) {
    if (stop_sign_overlap.end_s <= adc_back_edge_s) {
      continue;
    }

    if (stop_sign_overlap.object_id ==
        stop_sign_status.done_stop_sign_overlap_id()) {
      continue;
    }

    // build stop decision
    ADEBUG << "BuildStopDecision: stop_sign[" << stop_sign_overlap.object_id
           << "] start_s[" << stop_sign_overlap.start_s << "]";
    const std::string virtual_obstacle_id =
        STOP_SIGN_VO_ID_PREFIX + stop_sign_overlap.object_id;
    const std::vector<std::string> wait_for_obstacle_ids(
        stop_sign_status.wait_for_obstacle_id().begin(),
        stop_sign_status.wait_for_obstacle_id().end());
    const bool roundabout_entry = IsContestRoundaboutScenario(injector_);
    if (roundabout_entry &&
        IsRoundaboutLaunchCommitted(injector_, *reference_line_info)) {
      const double adc_x = frame->vehicle_state().x();
      const double adc_y = frame->vehicle_state().y();
      AINFO << "[ROUNDABOUT][StopSign] skip stop wall after launch commit"
            << ", adc_x=" << adc_x << ", adc_y=" << adc_y;
      continue;
    }
    const auto filtered_wait_for_obstacle_ids = roundabout_entry
        ? FilterRoundaboutWaitForObstacles(wait_for_obstacle_ids,
                                          *reference_line_info)
        : wait_for_obstacle_ids;
    const double stop_distance = roundabout_entry ? 0.5 : config_.stop_distance();
    util::BuildStopDecision(
        virtual_obstacle_id, stop_sign_overlap.start_s, stop_distance,
        StopReasonCode::STOP_REASON_STOP_SIGN, filtered_wait_for_obstacle_ids, Getname(),
        frame, reference_line_info);
  }
}

}  // namespace planning
}  // namespace apollo
