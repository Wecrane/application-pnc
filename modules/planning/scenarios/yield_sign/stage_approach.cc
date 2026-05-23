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

#include "modules/planning/scenarios/yield_sign/stage_approach.h"

#include <algorithm>
#include <cmath>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/common_msgs/map_msgs/map_lane.pb.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
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
    AINFO << "[ROUNDABOUT][YIELD] ignore non-target-lane vehicle, obs="
          << obstacle.Id() << ", start_l=" << sl_boundary.start_l()
          << ", end_l=" << sl_boundary.end_l();
  }
  return outside_target_lane;
}
}  // namespace

StageResult YieldSignStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  auto scenario_context = GetContextAs<YieldSignContext>();
  scenario_config_.CopyFrom(scenario_context->scenario_config);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "YieldSignStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  if (scenario_context->current_yield_sign_overlap_ids.empty()) {
    return FinishScenario();
  }
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_yield_sign()
      ->clear_wait_for_obstacle_id();

  for (const auto& yield_sign_overlap_id :
       scenario_context->current_yield_sign_overlap_ids) {
    // get overlap along reference line
    PathOverlap* current_yield_sign_overlap =
        reference_line_info.GetOverlapOnReferenceLine(
            yield_sign_overlap_id, ReferenceLineInfo::YIELD_SIGN);
    if (!current_yield_sign_overlap) {
      continue;
    }

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_yield_sign_overlap->start_s, false);

    static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double distance_adc_pass_stop_sign =
        adc_front_edge_s - current_yield_sign_overlap->start_s;
    if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
      // passed stop line
      return FinishStage();
    }

    const double distance_adc_to_stop_line =
        current_yield_sign_overlap->start_s - adc_front_edge_s;
    ADEBUG << "yield_sign_overlap_id[" << yield_sign_overlap_id << "] start_s["
           << current_yield_sign_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "]";
    bool yield_sign_done = false;
    if (distance_adc_to_stop_line <
        scenario_config_.max_valid_stop_distance()) {
      // close enough, check yield_sign clear
      yield_sign_done = true;
      const bool roundabout_entry = IsRoundaboutEntryLike(reference_line_info);
      const auto& path_decision = reference_line_info.path_decision();
      for (const auto* obstacle : path_decision.obstacles().Items()) {
        const std::string& obstacle_id = obstacle->Id();
        std::string obstacle_type_name =
            PerceptionObstacle_Type_Name(obstacle->Perception().type());
        ADEBUG << "yield_sign[" << yield_sign_overlap_id << "] obstacle_id["
               << obstacle_id << "] type[" << obstacle_type_name << "]";
        if (obstacle->IsVirtual()) {
          continue;
        }

        if (obstacle->reference_line_st_boundary().IsEmpty()) {
          continue;
        }

        static constexpr double kMinSTBoundaryT = 6.0;  // sec
        if (obstacle->reference_line_st_boundary().min_t() > kMinSTBoundaryT) {
          continue;
        }
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
        // max st_min_t(sec) to ignore
        static constexpr double kIgnoreMaxSTMinT = 0.1;
        // min st_min_s(m) to ignore
        static constexpr double kIgnoreMinSTMinS = 15.0;
        if (obstacle_traveled_s < kepsilon &&
            obstacle->reference_line_st_boundary().min_t() < kIgnoreMaxSTMinT &&
            obstacle->reference_line_st_boundary().min_s() > kIgnoreMinSTMinS) {
          continue;
        }

        if (roundabout_entry && IsRoundaboutNonTargetLaneVehicle(*obstacle)) {
          continue;
        }

        if (roundabout_entry && CanCommitRoundaboutLaunch(*obstacle)) {
          continue;
        }

        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_yield_sign()
            ->add_wait_for_obstacle_id(obstacle->Id());

        yield_sign_done = false;
      }
    }

    if (yield_sign_done) {
      return FinishStage();
    }
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult YieldSignStageApproach::FinishStage() {
  // update PlanningContext
  auto* yield_sign_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_yield_sign();
  yield_sign_status->mutable_done_yield_sign_overlap_id()->Clear();
  auto scenario_context = GetContextAs<YieldSignContext>();
  for (const auto& yield_sign_overlap_id :
       scenario_context->current_yield_sign_overlap_ids) {
    yield_sign_status->add_done_yield_sign_overlap_id(yield_sign_overlap_id);
  }
  yield_sign_status->clear_wait_for_obstacle_id();

  scenario_context->creep_start_time = Clock::NowInSeconds();

  next_stage_ = "YIELD_SIGN_CREEP";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
