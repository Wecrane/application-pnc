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
constexpr double kRoundaboutInnerLaneHoldSec = 1.0;
constexpr double kRoundaboutCommitMinSpeed = 0.8;
constexpr double kRoundaboutTargetLaneMaxCenterL = 1.8;
constexpr double kRoundaboutCommitLookForward = 6.0;
constexpr double kRoundaboutCommitLookBack = 3.0;

std::unordered_map<std::string, double> roundabout_inner_lane_memory;
bool roundabout_launch_committed = false;

bool IsContestRoundaboutScenario(
    const std::shared_ptr<DependencyInjector>& injector) {
  return injector != nullptr && injector->planning_context() != nullptr &&
         injector->planning_context()->planning_status().scenario().scenario_type() ==
             kContestRoundaboutScenarioName;
}

bool IsRoundaboutNonTargetLaneVehicle(const Obstacle* obstacle) {
  if (obstacle == nullptr || obstacle->IsVirtual() || obstacle->IsStatic() ||
      obstacle->Perception().type() != apollo::perception::PerceptionObstacle::VEHICLE) {
    return false;
  }
  const double now = cyber::Clock::NowInSeconds();
  const auto& sl = obstacle->PerceptionSLBoundary();
  const double center_l = 0.5 * (sl.start_l() + sl.end_l());
  if (std::fabs(center_l) > kRoundaboutTargetLaneMaxCenterL) {
    roundabout_inner_lane_memory[obstacle->Id()] = now;
    return true;
  }
  const auto iter = roundabout_inner_lane_memory.find(obstacle->Id());
  return iter != roundabout_inner_lane_memory.end() &&
         now - iter->second < kRoundaboutInnerLaneHoldSec;
}

bool IsRoundaboutCommitArea(const ReferenceLineInfo& reference_line_info) {
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  for (const auto& overlap :
       reference_line_info.reference_line().map_path().pnc_junction_overlaps()) {
    if (overlap.start_s <= adc_end_s + kRoundaboutCommitLookForward &&
        overlap.end_s >= adc_end_s - kRoundaboutCommitLookBack) {
      return true;
    }
  }
  return false;
}

bool IsRoundaboutLaunchCommitted(
    const std::shared_ptr<DependencyInjector>& injector,
    const ReferenceLineInfo& reference_line_info) {
  if (!IsContestRoundaboutScenario(injector)) {
    roundabout_launch_committed = false;
    roundabout_inner_lane_memory.clear();
    return false;
  }
  const auto& vehicle_state = injector->vehicle_state()->vehicle_state();
  if (!roundabout_launch_committed &&
      IsRoundaboutCommitArea(reference_line_info) &&
      vehicle_state.linear_velocity() > kRoundaboutCommitMinSpeed) {
    roundabout_launch_committed = true;
    AINFO << "[ROUNDABOUT][Launch] committed in StopSign by junction feature"
          << ", v=" << vehicle_state.linear_velocity();
  }
  return roundabout_launch_committed;
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
      AINFO << "[ROUNDABOUT][StopSign] skip stop wall after launch commit";
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
