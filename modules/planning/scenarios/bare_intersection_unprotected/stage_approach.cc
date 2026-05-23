/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/bare_intersection_unprotected/stage_approach.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common_msgs/map_msgs/map_lane.pb.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::hdmap::PathOverlap;

namespace {
bool IsRoundaboutEntryLike(const ReferenceLineInfo& reference_line_info) {
  constexpr double kEntryLookForward = 35.0;
  constexpr double kInsideJunctionBuffer = 8.0;
  constexpr double kCurveLookForward = 55.0;
  constexpr double kMinMaxKappa = 0.025;
  constexpr double kMinHeadingChange = 0.65;
  constexpr double kUTurnHeadingChange = 1.8;
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  bool near_pnc_junction = false;
  for (const auto& overlap :
       reference_line_info.reference_line().map_path().pnc_junction_overlaps()) {
    if (overlap.end_s < adc_end_s - kInsideJunctionBuffer ||
        overlap.start_s > adc_end_s + kEntryLookForward) {
      continue;
    }
    near_pnc_junction = true;
    break;
  }
  if (!near_pnc_junction) {
    return false;
  }
  const auto& reference_line = reference_line_info.reference_line();
  const double end_s =
      std::min(reference_line.Length() - 1.0, adc_end_s + kCurveLookForward);
  if (end_s - adc_end_s < 8.0) {
    return false;
  }
  double max_abs_kappa = 0.0;
  for (double s = adc_end_s; s <= end_s; s += 2.0) {
    if (reference_line_info.GetPathTurnType(s) == hdmap::Lane::U_TURN) {
      return false;
    }
    max_abs_kappa =
        std::max(max_abs_kappa, std::fabs(reference_line.GetReferencePoint(s).kappa()));
  }
  const double heading_change = std::fabs(common::math::NormalizeAngle(
      reference_line.GetReferencePoint(end_s).heading() -
      reference_line.GetReferencePoint(adc_end_s).heading()));
  return max_abs_kappa > kMinMaxKappa && heading_change > kMinHeadingChange &&
         heading_change < kUTurnHeadingChange;
}

bool CanCommitRoundaboutLaunch(const Obstacle& obstacle) {
  if (obstacle.reference_line_st_boundary().IsEmpty()) {
    return true;
  }
  const auto& boundary = obstacle.reference_line_st_boundary();
  const double obstacle_traveled_s =
      boundary.bottom_left_point().s() - boundary.bottom_right_point().s();
  constexpr double kEpsilon = 1e-6;
  constexpr double kMinLaunchMinT = 0.35;
  constexpr double kMinLaunchGapS = 4.5;
  if (obstacle_traveled_s >= kEpsilon) {
    return false;
  }
  const bool passable =
      boundary.min_t() > kMinLaunchMinT && boundary.min_s() > kMinLaunchGapS;
  if (passable) {
    const double required_avg_v =
        boundary.min_s() / std::max(boundary.min_t(), 0.1);
    AINFO << "[ROUNDABOUT][CLEAR] commit launch ahead of obstacle, obs="
          << obstacle.Id() << ", min_t=" << boundary.min_t()
          << ", min_s=" << boundary.min_s()
          << ", obs_speed=" << obstacle.speed()
          << ", required_avg_v=" << required_avg_v;
  }
  return passable;
}

bool IsRoundaboutNonTargetLaneVehicle(const Obstacle& obstacle) {
  if (obstacle.IsVirtual() || obstacle.IsStatic() ||
      obstacle.Perception().type() != apollo::perception::PerceptionObstacle::VEHICLE) {
    return false;
  }
  constexpr double kTargetLaneLateralRange = 2.2;
  const auto& sl_boundary = obstacle.PerceptionSLBoundary();
  const double nearest_l =
      std::min(std::fabs(sl_boundary.start_l()), std::fabs(sl_boundary.end_l()));
  const bool outside_target_lane = nearest_l > kTargetLaneLateralRange;
  if (outside_target_lane) {
    AINFO << "[ROUNDABOUT][CLEAR] ignore non-target-lane vehicle, obs="
          << obstacle.Id() << ", start_l=" << sl_boundary.start_l()
          << ", end_l=" << sl_boundary.end_l();
  }
  return outside_target_lane;
}
}  // namespace

StageResult BareIntersectionUnprotectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(
      GetContextAs<BareIntersectionUnprotectedContext>()->scenario_config);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  const std::string pnc_junction_overlap_id =
      GetContextAs<BareIntersectionUnprotectedContext>()
          ->current_pnc_junction_overlap_id;
  if (pnc_junction_overlap_id.empty()) {
    return FinishScenario();
  }

  // get overlap along reference line
  PathOverlap* current_pnc_junction =
      reference_line_info.GetOverlapOnReferenceLine(
          pnc_junction_overlap_id, ReferenceLineInfo::PNC_JUNCTION);
  if (!current_pnc_junction) {
    return FinishScenario();
  }

  static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_to_pnc_junction =
      current_pnc_junction->start_s - adc_front_edge_s;
  ADEBUG << "pnc_junction_overlap_id[" << pnc_junction_overlap_id
         << "] start_s[" << current_pnc_junction->start_s
         << "] distance_adc_to_pnc_junction[" << distance_adc_to_pnc_junction
         << "]";
  if (distance_adc_to_pnc_junction < -kPassStopLineBuffer) {
    // passed stop line
    return FinishStage(frame);
  }

  // set cruise_speed to slow down
  frame->mutable_reference_line_info()->front().LimitCruiseSpeed(
      scenario_config_.approach_cruise_speed());

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(current_pnc_junction->start_s,
                                            false);

  result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  std::vector<std::string> wait_for_obstacle_ids;
  bool clear = CheckClear(reference_line_info, &wait_for_obstacle_ids);

  if (scenario_config_.enable_explicit_stop()) {
    bool stop = false;
    static constexpr double kCheckClearDistance = 5.0;  // meter
    static constexpr double kStartWatchDistance = 2.0;  // meter
    if (distance_adc_to_pnc_junction <= kCheckClearDistance &&
        distance_adc_to_pnc_junction >= kStartWatchDistance && !clear) {
      stop = true;
    } else if (distance_adc_to_pnc_junction < kStartWatchDistance) {
      // creeping area
      counter_ = clear ? counter_ + 1 : 0;

      if (counter_ >= 5) {
        counter_ = 0;  // reset
      } else {
        stop = true;
      }
    }

    if (stop) {
      // build stop decision
      ADEBUG << "BuildStopDecision: bare pnc_junction["
             << pnc_junction_overlap_id << "] start_s["
             << current_pnc_junction->start_s << "]";
      const std::string virtual_obstacle_id =
          "PNC_JUNCTION_" + current_pnc_junction->object_id;
      planning::util::BuildStopDecision(
          virtual_obstacle_id, current_pnc_junction->start_s,
          scenario_config_.stop_distance(),
          StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids,
          "bare intersection", frame,
          &(frame->mutable_reference_line_info()->front()));
    }
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

bool BareIntersectionUnprotectedStageApproach::CheckClear(
    const ReferenceLineInfo& reference_line_info,
    std::vector<std::string>* wait_for_obstacle_ids) {
  // TODO(all): move to conf
  static constexpr double kConf_min_boundary_t = 6.0;        // second
  static constexpr double kConf_ignore_max_st_min_t = 0.1;   // second
  static constexpr double kConf_ignore_min_st_min_s = 15.0;  // meter

  bool all_far_away = true;
  const bool roundabout_entry = IsRoundaboutEntryLike(reference_line_info);
  for (auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->reference_line_st_boundary().min_t() < kConf_min_boundary_t) {
      const double kepsilon = 1e-6;
      double obstacle_traveled_s =
          obstacle->reference_line_st_boundary().bottom_left_point().s() -
          obstacle->reference_line_st_boundary().bottom_right_point().s();
      ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
             << obstacle->reference_line_st_boundary().min_t()
             << "] obstacle_st_min_s["
             << obstacle->reference_line_st_boundary().min_s()
             << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

      // ignore the obstacle which is already on reference line and moving
      // along the direction of ADC
      if (obstacle_traveled_s < kepsilon &&
          obstacle->reference_line_st_boundary().min_t() <
              kConf_ignore_max_st_min_t &&
          obstacle->reference_line_st_boundary().min_s() >
              kConf_ignore_min_st_min_s) {
        continue;
      }

      if (roundabout_entry && IsRoundaboutNonTargetLaneVehicle(*obstacle)) {
        continue;
      }

      if (roundabout_entry && CanCommitRoundaboutLaunch(*obstacle)) {
        continue;
      }

      wait_for_obstacle_ids->push_back(obstacle->Id());
      all_far_away = false;
    }
  }
  return all_far_away;
}

StageResult BareIntersectionUnprotectedStageApproach::FinishStage(
    Frame* frame) {
  next_stage_ = "BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE";

  // reset cruise_speed
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.LimitCruiseSpeed(FLAGS_default_cruise_speed);

  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
