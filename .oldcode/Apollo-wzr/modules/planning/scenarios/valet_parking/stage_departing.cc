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

#include "modules/planning/scenarios/valet_parking/stage_departing.h"

#include <cmath>
#include <limits>

#include "modules/common/math/math_utils.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

StageResult StageDeparting::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  (void)planning_init_point;
  CHECK_NOTNULL(frame);
  StageResult result;

  if (!departing_status_initialized_) {
    InitDepartingStatus(frame);
    departing_status_initialized_ = true;
  }

  auto scenario_context = GetContextAs<ValetParkingContext>();
  scenario_context->LatchStaticObstacles(*frame, "departing");
  scenario_context->InjectLatchedStaticObstacles(frame);
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  result = ExecuteTaskOnOpenSpace(frame);
  LogDepartingRoiDiagnostics(*frame);
  if (result.HasError()) {
    AERROR << "StageDeparting planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  const bool open_space_finished =
      frame->open_space_info().openspace_planning_finish();
  const bool ready_to_return = CheckReadyToReturnLaneFollow(*frame);
  constexpr double kMinEarlyReturnSpeed = 0.4;
  constexpr double kMaxEarlyReturnDistanceToEnd = 6.0;
  const double adc_speed = frame->vehicle_state().linear_velocity();
  const auto& open_space_info = frame->open_space_info();
  const auto& end_pose = open_space_info.open_space_end_pose();
  const auto& origin_point = open_space_info.origin_point();
  const double origin_heading = open_space_info.origin_heading();
  common::math::Vec2d vehicle_xy(frame->vehicle_state().x(),
                                 frame->vehicle_state().y());
  vehicle_xy -= origin_point;
  vehicle_xy.SelfRotate(-origin_heading);
  double distance_to_end = std::numeric_limits<double>::infinity();
  double x_to_end = std::numeric_limits<double>::infinity();
  if (end_pose.size() >= 2) {
    const common::math::Vec2d end_xy(end_pose[0], end_pose[1]);
    distance_to_end = vehicle_xy.DistanceTo(end_xy);
    x_to_end = end_pose[0] - vehicle_xy.x();
  }
  const bool close_to_end = distance_to_end < kMaxEarlyReturnDistanceToEnd;
  const bool early_return = !open_space_finished && ready_to_return &&
                            adc_speed > kMinEarlyReturnSpeed && close_to_end;
  AINFO << "Valet departing finish check, open_space_finished="
        << open_space_finished << ", ready_to_return=" << ready_to_return
        << ", early_return=" << early_return << ", speed=" << adc_speed
        << ", min_early_return_speed=" << kMinEarlyReturnSpeed
        << ", close_to_end=" << close_to_end
        << ", distance_to_end=" << distance_to_end
        << ", max_early_return_distance_to_end="
        << kMaxEarlyReturnDistanceToEnd << ", x_to_end=" << x_to_end
        << ", vehicle_xy=(" << vehicle_xy.x() << ", " << vehicle_xy.y()
        << "), end_pose_size=" << end_pose.size();

  if (open_space_finished || early_return) {
    frame->mutable_open_space_info()->set_openspace_planning_finish(false);
    if (ready_to_return) {
      GetContextAs<ValetParkingContext>()->station_shuttle_completed = true;
      AINFO << "Finish valet departing, return to lane follow, early_return="
            << early_return << ", speed=" << adc_speed;
      return FinishScenario();
    }
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

void StageDeparting::InitDepartingStatus(Frame* frame) {
  auto* park_and_go_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_park_and_go();
  park_and_go_status->Clear();
  const auto& vehicle_state = frame->vehicle_state();
  park_and_go_status->mutable_adc_init_position()->set_x(vehicle_state.x());
  park_and_go_status->mutable_adc_init_position()->set_y(vehicle_state.y());
  park_and_go_status->mutable_adc_init_position()->set_z(0.0);
  park_and_go_status->set_adc_init_heading(vehicle_state.heading());
  park_and_go_status->set_in_check_stage(false);
  AINFO << "Init valet departing status, adc_init_x=" << vehicle_state.x()
        << ", adc_init_y=" << vehicle_state.y()
        << ", adc_init_heading=" << vehicle_state.heading();
}

void StageDeparting::LogDepartingRoiDiagnostics(const Frame& frame) const {
  const auto& open_space_info = frame.open_space_info();
  const auto& xy_boundary = open_space_info.ROI_xy_boundary();
  const auto& end_pose = open_space_info.open_space_end_pose();
  const auto& origin_point = open_space_info.origin_point();
  const double origin_heading = open_space_info.origin_heading();

  common::math::Vec2d vehicle_xy(frame.vehicle_state().x(),
                                 frame.vehicle_state().y());
  vehicle_xy -= origin_point;
  vehicle_xy.SelfRotate(-origin_heading);

  bool vehicle_inside = false;
  bool end_pose_inside = false;
  if (xy_boundary.size() >= 4) {
    vehicle_inside = vehicle_xy.x() >= xy_boundary[0] &&
                     vehicle_xy.x() <= xy_boundary[1] &&
                     vehicle_xy.y() >= xy_boundary[2] &&
                     vehicle_xy.y() <= xy_boundary[3];
    if (end_pose.size() >= 2) {
      end_pose_inside = end_pose[0] >= xy_boundary[0] &&
                        end_pose[0] <= xy_boundary[1] &&
                        end_pose[1] >= xy_boundary[2] &&
                        end_pose[1] <= xy_boundary[3];
    }
  }

  AINFO << "Valet departing ROI diagnostics, origin=(" << origin_point.x()
        << ", " << origin_point.y() << "), origin_heading=" << origin_heading
        << ", vehicle_xy=(" << vehicle_xy.x() << ", " << vehicle_xy.y()
        << "), xy_boundary_size=" << xy_boundary.size()
        << ", vehicle_inside_roi=" << vehicle_inside
        << ", end_pose_size=" << end_pose.size()
        << ", end_pose_inside_roi=" << end_pose_inside;
  if (xy_boundary.size() >= 4) {
    AINFO << "Valet departing ROI xy_boundary, x_min=" << xy_boundary[0]
          << ", x_max=" << xy_boundary[1] << ", y_min=" << xy_boundary[2]
          << ", y_max=" << xy_boundary[3];
  }
  if (end_pose.size() >= 4) {
    AINFO << "Valet departing end_pose, x=" << end_pose[0]
          << ", y=" << end_pose[1] << ", theta=" << end_pose[2]
          << ", v=" << end_pose[3];
  }
}

bool StageDeparting::CheckReadyToReturnLaneFollow(const Frame& frame) const {
  if (frame.reference_line_info().empty()) {
    AINFO << "Valet departing not ready: no reference line";
    return false;
  }

  constexpr double kMaxReturnL = 0.6;
  constexpr double kMaxHeadingDiff = 0.25;
  const auto& vehicle_state = frame.vehicle_state();
  const common::math::Vec2d adc_position(vehicle_state.x(), vehicle_state.y());
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  common::SLPoint adc_sl;
  reference_line.XYToSL(adc_position, &adc_sl);
  const auto reference_point = reference_line.GetReferencePoint(adc_sl.s());
  const double heading_diff = std::fabs(common::math::NormalizeAngle(
      vehicle_state.heading() - reference_point.heading()));
  const bool ready =
      std::fabs(adc_sl.l()) < kMaxReturnL && heading_diff < kMaxHeadingDiff;
  AINFO << "Valet departing return check, adc_s=" << adc_sl.s()
        << ", adc_l=" << adc_sl.l()
        << ", vehicle_heading=" << vehicle_state.heading()
        << ", ref_heading=" << reference_point.heading()
        << ", heading_diff=" << heading_diff << ", ready=" << ready;
  return ready;
}

}  // namespace planning
}  // namespace apollo
