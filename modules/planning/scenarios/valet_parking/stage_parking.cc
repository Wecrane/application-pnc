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
#include "modules/planning/scenarios/valet_parking/stage_parking.h"

#include <cmath>
#include <limits>

#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

namespace {

constexpr double kParkingFallbackDistanceThreshold = 1.6;
constexpr double kParkingFallbackHeadingThreshold = 0.45;

}  // namespace

StageResult StageParking::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  auto scenario_context = GetContextAs<ValetParkingContext>();
  scenario_context->RememberStaticBarriers(*frame, "parking");
  scenario_context->RestoreRememberedBarriers(frame);
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      scenario_context->target_parking_spot_id;
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "StageParking planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  constexpr double kBayDwellSeconds = 5.0;
  const bool open_space_finished =
      frame->open_space_info().openspace_planning_finish();
  const double adc_speed = std::fabs(frame->vehicle_state().linear_velocity());
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  bool fallback_finished = false;
  double distance_to_end = std::numeric_limits<double>::infinity();
  double heading_diff = std::numeric_limits<double>::infinity();
  double opposite_heading_diff = std::numeric_limits<double>::infinity();
  common::math::Vec2d end_xy_world;
  double end_theta_world = 0.0;
  const auto& open_space_info = frame->open_space_info();
  const auto& end_pose = open_space_info.open_space_end_pose();
  if (end_pose.size() >= 4) {
    end_xy_world = common::math::Vec2d(end_pose[0], end_pose[1]);
    end_xy_world.SelfRotate(open_space_info.origin_heading());
    end_xy_world += open_space_info.origin_point();
    end_theta_world =
        common::math::NormalizeAngle(end_pose[2] +
                                     open_space_info.origin_heading());
    distance_to_end =
        std::hypot(frame->vehicle_state().x() - end_xy_world.x(),
                   frame->vehicle_state().y() - end_xy_world.y());
    heading_diff = std::fabs(common::math::AngleDiff(
        frame->vehicle_state().heading(), end_theta_world));
    opposite_heading_diff = std::fabs(common::math::AngleDiff(
        frame->vehicle_state().heading(), end_theta_world + M_PI));
    fallback_finished =
        !open_space_finished && adc_speed <= max_adc_stop_speed &&
        distance_to_end <= kParkingFallbackDistanceThreshold &&
        (heading_diff <= kParkingFallbackHeadingThreshold ||
         opposite_heading_diff <= kParkingFallbackHeadingThreshold);
    AINFO << "Bay parking finish check, open_space_finished="
          << open_space_finished
          << ", fallback_finished=" << fallback_finished
          << ", distance_to_end=" << distance_to_end
          << ", heading_diff=" << heading_diff
          << ", opposite_heading_diff=" << opposite_heading_diff
          << ", speed=" << adc_speed
          << ", max_stop_speed=" << max_adc_stop_speed << ", vehicle=("
          << frame->vehicle_state().x() << ", " << frame->vehicle_state().y()
          << ", " << frame->vehicle_state().heading() << "), end=("
          << end_xy_world.x() << ", " << end_xy_world.y() << ", "
          << end_theta_world << ")";
    if (fallback_finished) {
      AINFO << "Bay parking fallback finish accepted, distance_to_end="
            << distance_to_end << ", heading_diff=" << heading_diff
            << ", opposite_heading_diff=" << opposite_heading_diff;
    }
  } else {
    AINFO << "Bay parking fallback skipped, invalid open_space_end_pose "
          << "size=" << end_pose.size();
  }

  if (open_space_finished || fallback_finished) {
    frame->mutable_open_space_info()->set_openspace_planning_finish(false);
    if (adc_speed > max_adc_stop_speed) {
      dwell_timer_active_ = false;
      AINFO << "Bay parking reached open-space end but ADC not static, "
            << "speed=" << adc_speed
            << ", max_stop_speed=" << max_adc_stop_speed;
      return result.SetStageStatus(StageStatusType::RUNNING);
    }

    const double now = cyber::Clock::NowInSeconds();
    if (!dwell_timer_active_) {
      dwell_timer_active_ = true;
      dwell_start_sec_ = now;
      AINFO << "Parking reached, start bay dwell timer, target_wait_s="
            << kBayDwellSeconds;
    }
    const double elapsed = now - dwell_start_sec_;
    AINFO << "Bay dwell waiting, elapsed=" << elapsed
          << ", target_wait_s=" << kBayDwellSeconds;
    if (elapsed >= kBayDwellSeconds) {
      AINFO << "Bay dwell complete, switch to bay departure stage";
      next_stage_ = "VALET_PARKING_DEPARTING";
      return result.SetStageStatus(StageStatusType::FINISHED);
    }
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StageParking::FinishStage() {
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
