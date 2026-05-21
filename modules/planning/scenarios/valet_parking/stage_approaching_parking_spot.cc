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

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"

namespace apollo {
namespace planning {
namespace {

constexpr double kPreviewPathLength = 200.0;
constexpr double kPreviewSampleStep = 0.5;
constexpr double kHeadingSampleGap = 0.5;
constexpr double kHalfPi = 1.5707963267948966;
constexpr double kPi = 3.1415926535897932;

double SampledLaneHeading(const hdmap::LaneInfoConstPtr& lane,
                          const double lane_s,
                          const double fallback_heading) {
  const double forward_s =
      std::min(lane_s + kHeadingSampleGap, lane->total_length());
  const double backward_s = std::max(lane_s - kHeadingSampleGap, 0.0);
  if (forward_s <= backward_s) {
    return fallback_heading;
  }
  const auto forward_point = lane->GetSmoothPoint(forward_s);
  const auto backward_point = lane->GetSmoothPoint(backward_s);
  return std::atan2(forward_point.y() - backward_point.y(),
                    forward_point.x() - backward_point.x());
}

std::vector<ReferencePoint> BuildPreviewReferencePoints(
    const hdmap::LaneInfoConstPtr& lane, const double projection_s,
    const double anchor_x, const double anchor_y, const double anchor_heading,
    const double adc_anchor_s) {
  const double cos_heading = std::cos(anchor_heading);
  const double sin_heading = std::sin(anchor_heading);
  std::vector<ReferencePoint> points;
  points.reserve(static_cast<size_t>(kPreviewPathLength /
                                     kPreviewSampleStep + 1.0));
  for (double offset = 0.0; offset <= kPreviewPathLength;
       offset += kPreviewSampleStep) {
    const double preview_s = adc_anchor_s + offset;
    hdmap::MapPathPoint map_point(
        {anchor_x + preview_s * cos_heading,
         anchor_y + preview_s * sin_heading},
        anchor_heading);
    hdmap::LaneWaypoint waypoint;
    waypoint.lane = lane;
    waypoint.s = projection_s + offset;
    map_point.add_lane_waypoint(waypoint);
    points.emplace_back(map_point, 0.0, 0.0);
  }
  return points;
}

}  // namespace

bool StageApproachingParkingSpot::Init(
    const StagePipeline& config,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_dir, void* context) {
  if (!Stage::Init(config, injector, config_dir, context)) {
    return false;
  }
  scenario_config_.CopyFrom(
      GetContextAs<ValetParkingContext>()->scenario_config);
  main_road_seed_ready_ = false;
  return true;
}
StageResult StageApproachingParkingSpot::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: StageApproachingParkingSpot";
  CHECK_NOTNULL(frame);
  StageResult result;
  auto scenario_context = GetContextAs<ValetParkingContext>();
  scenario_context->RememberStaticBarriers(*frame, "approach");

  if (scenario_context->target_parking_spot_id.empty()) {
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      scenario_context->target_parking_spot_id;
  frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(
      scenario_context->pre_stop_rightaway_flag);
  *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
      scenario_context->pre_stop_rightaway_point;

  InstallMainRoadPreview(frame);

  auto* reference_lines = frame->mutable_reference_line_info();
  for (auto& reference_line : *reference_lines) {
    auto* path_decision = reference_line.path_decision();
    if (nullptr == path_decision) {
      continue;
    }
    auto* dest_obstacle = path_decision->Find(FLAGS_destination_obstacle_id);
    if (nullptr == dest_obstacle) {
      continue;
    }
    ObjectDecisionType decision;
    decision.mutable_ignore();
    dest_obstacle->EraseDecision();
    dest_obstacle->AddLongitudinalDecision("ignore-dest-in-valet-parking",
                                           decision);
  }

  result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

  scenario_context->pre_stop_rightaway_flag =
      frame->open_space_info().pre_stop_rightaway_flag();
  scenario_context->pre_stop_rightaway_point =
      frame->open_space_info().pre_stop_rightaway_point();

  if (CheckADCStop(*frame)) {
    next_stage_ = "VALET_PARKING_PARKING";
    return StageResult(StageStatusType::FINISHED);
  }
  if (result.HasError()) {
    AERROR << "StopSignUnprotectedStagePreStop planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StageApproachingParkingSpot::InstallMainRoadPreview(
    Frame* frame) {
  auto* ref_lines = frame->mutable_reference_line_info();
  if (ref_lines->empty()) {
    AINFO << "Bus-bay main-road preview skipped: no reference line";
    return false;
  }

  const auto& vehicle_state = frame->vehicle_state();
  hdmap::LaneInfoConstPtr lane;
  double lane_s = 0.0;
  double lane_l = 0.0;
  const auto adc_point = common::util::PointFactory::ToPointENU(vehicle_state);
  hdmap::HDMapUtil::BaseMap().GetNearestLaneWithDistance(
      adc_point, 5.0, &lane, &lane_s, &lane_l);
  if (lane == nullptr) {
    AINFO << "Bus-bay main-road preview skipped: lane lookup failed";
    return false;
  }

  double projection_s = 0.0;
  double projection_l = 0.0;
  if (!lane->GetProjection({vehicle_state.x(), vehicle_state.y()},
                           &projection_s, &projection_l)) {
    AINFO << "Bus-bay main-road preview skipped: projection failed, near_s="
          << lane_s << ", near_l=" << lane_l;
    return false;
  }

  if (!main_road_seed_ready_) {
    const auto anchor_point = lane->GetSmoothPoint(projection_s);
    double anchor_heading =
        SampledLaneHeading(lane, projection_s, vehicle_state.heading());
    if (std::fabs(common::math::NormalizeAngle(anchor_heading -
                                               vehicle_state.heading())) >
        kHalfPi) {
      anchor_heading = common::math::NormalizeAngle(anchor_heading + kPi);
    }
    main_road_seed_x_ = anchor_point.x();
    main_road_seed_y_ = anchor_point.y();
    main_road_seed_heading_ = anchor_heading;
    main_road_seed_ready_ = true;
    AINFO << "Bus-bay main-road preview anchor fixed, x="
          << main_road_seed_x_
          << ", y=" << main_road_seed_y_
          << ", heading=" << main_road_seed_heading_
          << ", adc_heading=" << vehicle_state.heading()
          << ", lane_s=" << projection_s << ", lane_l=" << projection_l;
  }

  const double heading = main_road_seed_heading_;
  const double cos_heading = std::cos(heading);
  const double sin_heading = std::sin(heading);
  const double adc_anchor_s =
      (vehicle_state.x() - main_road_seed_x_) * cos_heading +
      (vehicle_state.y() - main_road_seed_y_) * sin_heading;
  const double adc_anchor_l =
      -(vehicle_state.x() - main_road_seed_x_) * sin_heading +
      (vehicle_state.y() - main_road_seed_y_) * cos_heading;
  auto points = BuildPreviewReferencePoints(
      lane, projection_s, main_road_seed_x_,
      main_road_seed_y_, heading, adc_anchor_s);
  if (points.size() < 2) {
    AINFO << "Bus-bay main-road preview skipped: too few points";
    return false;
  }

  ReferenceLine preview_line(points);
  auto source_iter = ref_lines->begin();
  const auto source_index = source_iter->index();
  const auto cruise_speed = source_iter->GetBaseCruiseSpeed();
  auto preview_iter = ref_lines->emplace(
      source_iter, frame->vehicle_state(), frame->PlanningStartPoint(),
      preview_line, source_iter->Lanes());
  preview_iter->set_index(source_index);
  if (!preview_iter->Init(frame->obstacles(), cruise_speed)) {
    ref_lines->erase(preview_iter);
    AINFO << "Bus-bay main-road preview rebuild failed, point_count="
          << points.size() << ", heading=" << heading
          << ", adc_s=" << adc_anchor_s << ", adc_l=" << adc_anchor_l
          << ", obstacle_count=" << frame->obstacles().size();
    return false;
  }

  ref_lines->erase(source_iter);
  AINFO << "Bus-bay main-road preview reference ready, point_count="
        << points.size() << ", heading=" << heading
        << ", heading_delta="
        << common::math::NormalizeAngle(vehicle_state.heading() - heading)
        << ", adc_s=" << adc_anchor_s << ", adc_l=" << adc_anchor_l
        << ", lane_s=" << projection_s << ", lane_l=" << projection_l
        << ", obstacle_count=" << frame->obstacles().size();
  return true;
}

bool StageApproachingParkingSpot::CheckADCStop(const Frame& frame) {
  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();

  // check stop close enough to stop line of the stop_sign
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double stop_fence_start_s =
      frame.open_space_info().open_space_pre_stop_fence_s();
  const double distance_stop_line_to_adc_front_edge =
      stop_fence_start_s - adc_front_edge_s;
  constexpr double kRollingHandoffMaxSpeed = 0.8;
  constexpr double kRollingHandoffDistance = 2.0;
  constexpr double kRollingHandoffPastFenceBuffer = 1.0;
  constexpr double kNearStopFenceLogDistance = 5.0;

  if (stop_fence_start_s <= 1.0e-6) {
    ADEBUG << "Bus-bay approach stop check skipped: no pre-stop fence, speed="
           << adc_speed << ", adc_front_edge_s=" << adc_front_edge_s;
    return false;
  }

  const bool stopped_close_enough =
      adc_speed <= max_adc_stop_speed &&
      distance_stop_line_to_adc_front_edge <=
          scenario_config_.max_valid_stop_distance();
  const bool rolling_handoff =
      adc_speed <= kRollingHandoffMaxSpeed &&
      distance_stop_line_to_adc_front_edge <= kRollingHandoffDistance &&
      distance_stop_line_to_adc_front_edge >= -kRollingHandoffPastFenceBuffer;
  if (distance_stop_line_to_adc_front_edge < kNearStopFenceLogDistance ||
      rolling_handoff || stopped_close_enough) {
    AINFO << "Bus-bay approach pre-stop handoff check, speed=" << adc_speed
          << ", max_stop_speed=" << max_adc_stop_speed
          << ", stop_fence_s=" << stop_fence_start_s
          << ", adc_front_edge_s=" << adc_front_edge_s
          << ", distance_to_stop_fence="
          << distance_stop_line_to_adc_front_edge
          << ", max_valid_stop_distance="
          << scenario_config_.max_valid_stop_distance()
          << ", rolling_max_speed=" << kRollingHandoffMaxSpeed
          << ", rolling_distance=" << kRollingHandoffDistance
          << ", rolling_past_buffer=" << kRollingHandoffPastFenceBuffer
          << ", stopped_close_enough=" << stopped_close_enough
          << ", rolling_handoff=" << rolling_handoff;
  }

  if (!stopped_close_enough && !rolling_handoff) {
    return false;
  }
  AINFO << "Finish bus-bay approach pre-stop, switch to parking, "
        << "stopped_close_enough=" << stopped_close_enough
        << ", rolling_handoff=" << rolling_handoff
        << ", speed=" << adc_speed
        << ", distance_to_stop_fence=" << distance_stop_line_to_adc_front_edge;
  return true;
}

}  // namespace planning
}  // namespace apollo
