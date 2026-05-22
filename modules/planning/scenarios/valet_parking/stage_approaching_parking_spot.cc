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
bool StageApproachingParkingSpot::Init(
    const StagePipeline& config,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_dir, void* context) {
  if (!Stage::Init(config, injector, config_dir, context)) {
    return false;
  }
  scenario_config_.CopyFrom(
      GetContextAs<ValetParkingContext>()->scenario_config);
  has_straight_reference_anchor_ = false;
  return true;
}
StageResult StageApproachingParkingSpot::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: StageApproachingParkingSpot";
  CHECK_NOTNULL(frame);
  StageResult result;
  auto scenario_context = GetContextAs<ValetParkingContext>();
  scenario_context->LatchStaticObstacles(*frame, "approach");

  if (scenario_context->target_parking_spot_id.empty()) {
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      scenario_context->target_parking_spot_id;
  frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(
      scenario_context->pre_stop_rightaway_flag);
  *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
      scenario_context->pre_stop_rightaway_point;

  // Rebuild the reference line info from a straight reference line. Replacing
  // only ReferenceLine after ReferenceLineInfo::Init leaves obstacle SL/ST
  // caches in the old reference frame.
  {
    const auto& vehicle_state = frame->vehicle_state();
    const double vehicle_x = vehicle_state.x();
    const double vehicle_y = vehicle_state.y();
    auto* ref_lines = frame->mutable_reference_line_info();

    hdmap::LaneInfoConstPtr main_lane;
    double lane_s = 0.0;
    double lane_l = 0.0;
    auto adc_point = common::util::PointFactory::ToPointENU(vehicle_state);
    hdmap::HDMapUtil::BaseMap().GetNearestLaneWithDistance(
        adc_point, 5.0, &main_lane, &lane_s, &lane_l);

    if (ref_lines->empty()) {
      AINFO << "Skip straight ReferenceLineInfo rebuild: no reference line";
    } else if (main_lane == nullptr) {
      AINFO << "Skip straight ReferenceLineInfo rebuild: no nearby main lane";
    } else {
      double proj_s = 0.0;
      double proj_l = 0.0;
      if (!main_lane->GetProjection({vehicle_x, vehicle_y}, &proj_s,
                                    &proj_l)) {
        AINFO << "Skip straight ReferenceLineInfo rebuild: lane projection "
                 "failed, lane_s="
              << lane_s << ", lane_l=" << lane_l;
      } else {
        constexpr double kStraightReferenceLineLength = 200.0;
        constexpr double kStraightReferenceLineStep = 0.5;

        if (!has_straight_reference_anchor_) {
          constexpr double kHalfPi = 1.5707963267948966;
          constexpr double kPi = 3.1415926535897932;
          constexpr double kLaneHeadingSampleDistance = 0.5;
          const auto anchor_point = main_lane->GetSmoothPoint(proj_s);
          const double heading_forward_s = std::min(
              proj_s + kLaneHeadingSampleDistance, main_lane->total_length());
          const double heading_backward_s =
              std::max(proj_s - kLaneHeadingSampleDistance, 0.0);
          const auto heading_forward_point =
              main_lane->GetSmoothPoint(heading_forward_s);
          const auto heading_backward_point =
              main_lane->GetSmoothPoint(heading_backward_s);
          double anchor_heading = vehicle_state.heading();
          if (heading_forward_s > heading_backward_s) {
            anchor_heading = std::atan2(
                heading_forward_point.y() - heading_backward_point.y(),
                heading_forward_point.x() - heading_backward_point.x());
          }
          if (std::fabs(common::math::NormalizeAngle(
                  anchor_heading - vehicle_state.heading())) > kHalfPi) {
            anchor_heading =
                common::math::NormalizeAngle(anchor_heading + kPi);
          }
          straight_reference_anchor_x_ = anchor_point.x();
          straight_reference_anchor_y_ = anchor_point.y();
          straight_reference_anchor_heading_ = anchor_heading;
          has_straight_reference_anchor_ = true;
          AINFO << "Locked straight reference anchor for valet approach, "
                << "anchor_x=" << straight_reference_anchor_x_
                << ", anchor_y=" << straight_reference_anchor_y_
                << ", anchor_heading=" << straight_reference_anchor_heading_
                << ", vehicle_heading=" << vehicle_state.heading()
                << ", proj_s=" << proj_s << ", proj_l=" << proj_l;
        }

        const double anchor_heading = straight_reference_anchor_heading_;
        const double anchor_cos = std::cos(anchor_heading);
        const double anchor_sin = std::sin(anchor_heading);
        const double vehicle_anchor_s =
            (vehicle_x - straight_reference_anchor_x_) * anchor_cos +
            (vehicle_y - straight_reference_anchor_y_) * anchor_sin;
        const double vehicle_anchor_l =
            -(vehicle_x - straight_reference_anchor_x_) * anchor_sin +
            (vehicle_y - straight_reference_anchor_y_) * anchor_cos;

        std::vector<ReferencePoint> ref_points;
        ref_points.reserve(static_cast<size_t>(
            kStraightReferenceLineLength / kStraightReferenceLineStep + 1.0));

        for (double dist = 0.0; dist <= kStraightReferenceLineLength;
             dist += kStraightReferenceLineStep) {
          const double ref_s = vehicle_anchor_s + dist;
          const double px = straight_reference_anchor_x_ + ref_s * anchor_cos;
          const double py = straight_reference_anchor_y_ + ref_s * anchor_sin;
          hdmap::MapPathPoint map_point({px, py}, anchor_heading);
          hdmap::LaneWaypoint lane_waypoint;
          lane_waypoint.lane = main_lane;
          lane_waypoint.s = proj_s + dist;
          map_point.add_lane_waypoint(lane_waypoint);
          ref_points.emplace_back(map_point, 0.0, 0.0);
        }

        if (ref_points.size() < 2) {
          AINFO << "Skip straight ReferenceLineInfo rebuild: insufficient "
                   "reference points";
        } else {
          ReferenceLine straight_ref_line(ref_points);
          auto old_ref_iter = ref_lines->begin();
          const auto old_index = old_ref_iter->index();
          const auto old_base_cruise_speed =
              old_ref_iter->GetBaseCruiseSpeed();
          auto straight_ref_iter = ref_lines->emplace(
              old_ref_iter, frame->vehicle_state(), frame->PlanningStartPoint(),
              straight_ref_line, old_ref_iter->Lanes());
          straight_ref_iter->set_index(old_index);
          if (straight_ref_iter->Init(frame->obstacles(),
                                      old_base_cruise_speed)) {
            ref_lines->erase(old_ref_iter);
            AINFO << "Rebuilt straight ReferenceLineInfo for valet approach, "
                  << "points=" << ref_points.size()
                  << ", anchor_heading=" << anchor_heading
                  << ", vehicle_heading=" << vehicle_state.heading()
                  << ", heading_delta="
                  << common::math::NormalizeAngle(vehicle_state.heading() -
                                                  anchor_heading)
                  << ", anchor_s=" << vehicle_anchor_s
                  << ", anchor_l=" << vehicle_anchor_l
                  << ", proj_s=" << proj_s << ", proj_l=" << proj_l
                  << ", obstacle_count=" << frame->obstacles().size();
          } else {
            ref_lines->erase(straight_ref_iter);
            AINFO << "Failed to rebuild straight ReferenceLineInfo for valet "
                     "approach, keep original reference line, points="
                  << ref_points.size()
                  << ", anchor_heading=" << anchor_heading
                  << ", vehicle_heading=" << vehicle_state.heading()
                  << ", heading_delta="
                  << common::math::NormalizeAngle(vehicle_state.heading() -
                                                  anchor_heading)
                  << ", anchor_s=" << vehicle_anchor_s
                  << ", anchor_l=" << vehicle_anchor_l
                  << ", proj_s=" << proj_s << ", proj_l=" << proj_l
                  << ", obstacle_count=" << frame->obstacles().size();
          }
        }
      }
    }
  }

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
    ADEBUG << "Valet approach stop check skipped: no pre-stop fence, speed="
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
    AINFO << "Valet approach pre-stop handoff check, speed=" << adc_speed
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
  AINFO << "Finish valet approach pre-stop, switch to parking, "
        << "stopped_close_enough=" << stopped_close_enough
        << ", rolling_handoff=" << rolling_handoff
        << ", speed=" << adc_speed
        << ", distance_to_stop_fence=" << distance_stop_line_to_adc_front_edge;
  return true;
}

}  // namespace planning
}  // namespace apollo
