/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/open_space_pre_stop_decider/open_space_pre_stop_decider.h"

#include <memory>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

namespace {
constexpr double kParkingPreStopFenceBuildRange = 60.0;
}  // namespace

bool OpenSpacePreStopDecider::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!Decider::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  bool res = Decider::LoadConfig<OpenSpacePreStopDeciderConfig>(&config_);
  AINFO << "Load config:" << config_.DebugString();
  return res;
}

Status OpenSpacePreStopDecider::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  double target_s = 0.0;
  const auto& stop_type = config_.stop_type();
  if (stop_type == OpenSpacePreStopDeciderConfig::PARKING) {
    if (!CheckParkingSpotPreStop(frame, reference_line_info, &target_s)) {
      const std::string msg = "Checking parking spot pre stop fails";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    // Build the pre-stop fence early enough for high-speed bus-bay approach,
    // while still avoiding very distant parking spots.  Use distance from the
    // ADC instead of absolute reference-line s so straight-reference rebuilds
    // do not change when the wall appears.
    const double adc_front_s = reference_line_info->AdcSlBoundary().end_s();
    const double distance_to_target = target_s - adc_front_s;
    if (distance_to_target > kParkingPreStopFenceBuildRange) {
      AINFO << "Parking spot too far (target_s=" << target_s
            << ", adc_front_s=" << adc_front_s
            << ", distance_to_target=" << distance_to_target
            << "), skip stop fence, build_range="
            << kParkingPreStopFenceBuildRange;
      return Status::OK();
    }
    SetParkingSpotStopFence(target_s, frame, reference_line_info);
    return Status::OK();
  }
  if (stop_type == OpenSpacePreStopDeciderConfig::PULL_OVER) {
    if (!CheckPullOverPreStop(frame, reference_line_info, &target_s)) {
      const std::string msg = "Checking pull over pre stop fails";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    SetPullOverStopFence(target_s, frame, reference_line_info);
    return Status::OK();
  }
  const std::string msg = "This stop type not implemented";
  AERROR << msg;
  return Status(ErrorCode::PLANNING_ERROR, msg);
}

bool OpenSpacePreStopDecider::CheckPullOverPreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    double* target_s) {
  *target_s = 0.0;
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    common::SLPoint pull_over_sl;
    const auto& reference_line = reference_line_info->reference_line();
    reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
    *target_s = pull_over_sl.s();
  }
  return true;
}

bool OpenSpacePreStopDecider::CheckParkingSpotPreStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    double* target_s) {
  const auto& target_parking_spot_id =
      frame->open_space_info().target_parking_spot_id();
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  if (target_parking_spot_id.empty()) {
    AERROR << "no target parking spot id found when setting pre stop fence";
    return false;
  }

  double target_area_center_s = 0.0;
  bool target_area_found = false;
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
  ParkingSpaceInfoConstPtr target_parking_spot_ptr;
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  for (const auto& parking_overlap : parking_space_overlaps) {
    if (parking_overlap.object_id == target_parking_spot_id) {
      // TODO(Jinyun) parking overlap s are wrong on map, not usable
      // target_area_center_s =
      //     (parking_overlap.start_s + parking_overlap.end_s) / 2.0;
      hdmap::Id id;
      id.set_id(parking_overlap.object_id);
      target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
      Vec2d left_bottom_point =
          target_parking_spot_ptr->polygon().points().at(0);
      Vec2d right_bottom_point =
          target_parking_spot_ptr->polygon().points().at(1);
      Vec2d right_up_point = target_parking_spot_ptr->polygon().points().at(2);
      Vec2d left_up_point = target_parking_spot_ptr->polygon().points().at(3);
      Vec2d center_point = (left_bottom_point + right_bottom_point +
                            right_up_point + left_up_point) /
                           4.0;
      double center_l;
      nearby_path.GetNearestPoint(center_point, &target_area_center_s,
                                  &center_l);
      target_area_found = true;
    }
  }

  if (!target_area_found) {
    // 直参考线没有 parking_space_overlaps，直接从 hdmap 获取停车位坐标
    hdmap::Id id;
    id.set_id(target_parking_spot_id);
    auto spot_ptr = hdmap->GetParkingSpaceById(id);
    if (spot_ptr) {
      const auto& pts = spot_ptr->polygon().points();
      Vec2d center_point = (pts[0] + pts[1] + pts[2] + pts[3]) / 4.0;
      // 直参考线从车辆位置开始，GetNearestPoint 会返回 s=0
      // 改用投影计算：车辆在参考线上的 s + 车辆到停车位的纵向距离
      double vehicle_s = 0.0;
      double vehicle_l = 0.0;
      Vec2d vehicle_pos(frame->vehicle_state().x(), frame->vehicle_state().y());
      nearby_path.GetNearestPoint(vehicle_pos, &vehicle_s, &vehicle_l);
      double center_l;
      double center_s;
      nearby_path.GetNearestPoint(center_point, &center_s, &center_l);
      // 用停车位在参考线上的投影 s 作为 target_s
      // 如果停车位在车辆侧面（l 很大），center_s 可能不准
      // 此时用车辆 s + 纵向偏移
      if (std::fabs(center_l) > 3.0) {
        // 停车位在车辆侧面，计算纵向距离
        double dx = center_point.x() - vehicle_pos.x();
        double dy = center_point.y() - vehicle_pos.y();
        double heading = frame->vehicle_state().heading();
        double longitudinal_dist = dx * std::cos(heading) + dy * std::sin(heading);
        target_area_center_s = vehicle_s + longitudinal_dist;
      } else {
        target_area_center_s = center_s;
      }
      target_area_found = true;
      AINFO << "Found parking spot from hdmap, s=" << target_area_center_s << " l=" << center_l
            << " vehicle_s=" << vehicle_s;
    }
  }

  if (!target_area_found) {
    AERROR << "no target parking spot found on reference line";
    return false;
  }
  *target_s = target_area_center_s;
  return true;
}

void OpenSpacePreStopDecider::SetParkingSpotStopFence(
    const double target_s, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const double front_edge_to_center = common::VehicleConfigHelper::Instance()
                                          ->GetConfig()
                                          .vehicle_param()
                                          .front_edge_to_center();
  double stop_line_s = 0.0;
  double stop_distance_to_target = config_.stop_distance_to_target();
  CHECK_GE(stop_distance_to_target, 1.0e-8);
  const double parking_spot_pre_stop_distance =
      config_.parking_spot_pre_stop_distance();
  CHECK_GE(parking_spot_pre_stop_distance, 0.0);
  // 在停车位中心前方一定距离处停车，给车辆留出倒车空间。
  stop_line_s =
      target_s + front_edge_to_center + parking_spot_pre_stop_distance;
  AINFO << "Set parking spot pre-stop fence, target_s=" << target_s
        << ", front_edge_to_center=" << front_edge_to_center
        << ", parking_spot_pre_stop_distance="
        << parking_spot_pre_stop_distance << ", stop_line_s=" << stop_line_s
        << ", adc_front_edge_s=" << adc_front_edge_s;
  const std::string stop_wall_id = OPEN_SPACE_STOP_ID;
  std::vector<std::string> wait_for_obstacles;
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);
  util::BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          reference_line_info);
}

void OpenSpacePreStopDecider::SetPullOverStopFence(
    const double target_s, Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  const auto& nearby_path = reference_line_info->reference_line().map_path();
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const VehicleState& vehicle_state = frame->vehicle_state();
  double stop_line_s = 0.0;
  double stop_distance_to_target = config_.stop_distance_to_target();
  double static_linear_velocity_epsilon = 1.0e-2;
  CHECK_GE(stop_distance_to_target, 1.0e-8);
  double target_vehicle_offset = target_s - adc_front_edge_s;
  if (target_vehicle_offset > stop_distance_to_target) {
    stop_line_s = target_s - stop_distance_to_target;
  } else {
    if (!frame->open_space_info().pre_stop_rightaway_flag()) {
      // TODO(Jinyun) Use constant comfortable deacceleration rather than
      // distance by config to set stop fence
      stop_line_s = adc_front_edge_s + config_.rightaway_stop_distance();
      if (std::abs(vehicle_state.linear_velocity()) <
          static_linear_velocity_epsilon) {
        stop_line_s = adc_front_edge_s;
      }
      *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) =
          nearby_path.GetSmoothPoint(stop_line_s);
      frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(true);
    } else {
      double stop_point_s = 0.0;
      double stop_point_l = 0.0;
      nearby_path.GetNearestPoint(
          frame->open_space_info().pre_stop_rightaway_point(), &stop_point_s,
          &stop_point_l);
      stop_line_s = stop_point_s;
    }
  }

  const std::string stop_wall_id = OPEN_SPACE_STOP_ID;
  std::vector<std::string> wait_for_obstacles;
  frame->mutable_open_space_info()->set_open_space_pre_stop_fence_s(
      stop_line_s);
  util::BuildStopDecision(stop_wall_id, stop_line_s, 0.0,
                          StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP,
                          wait_for_obstacles, "OpenSpacePreStopDecider", frame,
                          reference_line_info);
}
}  // namespace planning
}  // namespace apollo
