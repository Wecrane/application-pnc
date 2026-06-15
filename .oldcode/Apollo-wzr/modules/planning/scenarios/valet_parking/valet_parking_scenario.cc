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

#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"

#include <cmath>
#include <limits>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"
#include "modules/planning/scenarios/valet_parking/stage_departing.h"
#include "modules/planning/scenarios/valet_parking/stage_parking.h"

namespace apollo {
namespace planning {
namespace {
constexpr char kLatchedStaticObstaclePrefix[] = "valet_latched_static_";

bool IsSameStaticObstacleBox(const common::math::Box2d& lhs,
                             const common::math::Box2d& rhs) {
  constexpr double kCenterMatchDistance = 0.8;
  constexpr double kHeadingMatchTolerance = 0.35;
  return lhs.center().DistanceTo(rhs.center()) < kCenterMatchDistance &&
         std::fabs(std::remainder(lhs.heading() - rhs.heading(),
                                  2.0 * M_PI)) < kHeadingMatchTolerance;
}
}  // namespace

using apollo::common::VehicleState;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

void ValetParkingContext::LatchStaticObstacles(const Frame& frame,
                                               const std::string& source) {
  constexpr double kMaxLatchDistanceToAdc = 40.0;
  constexpr size_t kMaxLatchedStaticObstacles = 16;
  const Vec2d adc_xy(frame.vehicle_state().x(), frame.vehicle_state().y());
  for (const auto* obstacle : frame.obstacles()) {
    if (obstacle == nullptr || obstacle->IsVirtual() || !obstacle->IsStatic()) {
      continue;
    }
    const Box2d box = obstacle->PerceptionBoundingBox();
    if (box.DistanceTo(adc_xy) > kMaxLatchDistanceToAdc) {
      continue;
    }

    bool merged = false;
    for (auto& latched_box : latched_static_obstacle_boxes) {
      if (IsSameStaticObstacleBox(latched_box, box)) {
        latched_box = box;
        merged = true;
        AINFO << "Merge latched valet static obstacle, source=" << source
              << ", obstacle=" << obstacle->Id()
              << ", perception_id=" << obstacle->PerceptionId()
              << ", center=(" << box.center().x() << ", "
              << box.center().y() << ")"
              << ", latched_count=" << latched_static_obstacle_boxes.size();
        break;
      }
    }
    if (merged) {
      continue;
    }
    if (latched_static_obstacle_boxes.size() >= kMaxLatchedStaticObstacles) {
      AINFO << "Skip latch valet static obstacle because memory is full, "
            << "source=" << source << ", obstacle=" << obstacle->Id()
            << ", perception_id=" << obstacle->PerceptionId()
            << ", max_latched=" << kMaxLatchedStaticObstacles;
      continue;
    }
    latched_static_obstacle_boxes.push_back(box);
    AINFO << "Latch valet static obstacle, source=" << source
          << ", obstacle=" << obstacle->Id()
          << ", perception_id=" << obstacle->PerceptionId()
          << ", center=(" << box.center().x() << ", " << box.center().y()
          << "), heading=" << box.heading() << ", length=" << box.length()
          << ", width=" << box.width()
          << ", latched_count=" << latched_static_obstacle_boxes.size();
  }
}

void ValetParkingContext::InjectLatchedStaticObstacles(Frame* frame) const {
  if (frame == nullptr || latched_static_obstacle_boxes.empty()) {
    return;
  }
  size_t injected_count = 0;
  for (size_t i = 0; i < latched_static_obstacle_boxes.size(); ++i) {
    bool already_visible = false;
    for (const auto* obstacle : frame->obstacles()) {
      if (obstacle == nullptr || obstacle->IsVirtual() ||
          !obstacle->IsStatic()) {
        continue;
      }
      if (IsSameStaticObstacleBox(latched_static_obstacle_boxes[i],
                                  obstacle->PerceptionBoundingBox())) {
        already_visible = true;
        AINFO << "Skip injecting latched valet static obstacle because current "
              << "frame already has it, obstacle=" << obstacle->Id()
              << ", perception_id=" << obstacle->PerceptionId();
        break;
      }
    }
    if (already_visible) {
      continue;
    }
    const std::string id = std::string(kLatchedStaticObstaclePrefix) +
                           std::to_string(i);
    const auto* obstacle =
        frame->CreateStaticVirtualObstacle(id, latched_static_obstacle_boxes[i]);
    if (obstacle == nullptr) {
      continue;
    }
    ++injected_count;
    const auto& box = latched_static_obstacle_boxes[i];
    AINFO << "Inject latched valet static obstacle, id=" << id
          << ", center=(" << box.center().x() << ", " << box.center().y()
          << "), heading=" << box.heading() << ", length=" << box.length()
          << ", width=" << box.width();
  }
  AINFO << "Valet static obstacle latch summary, latched="
        << latched_static_obstacle_boxes.size()
        << ", injected=" << injected_count;
}

bool ValetParkingScenario::Init(std::shared_ptr<DependencyInjector> injector,
                                const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioValetParkingConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  init_ = true;
  return true;
}

bool ValetParkingScenario::IsTransferable(const Scenario* const other_scenario,
                                          const Frame& frame) {
  context_.LatchStaticObstacles(frame, "transfer");
  if (context_.station_shuttle_completed) {
    AINFO << "Skip valet parking transfer: station shuttle already completed";
    return false;
  }

  if (injector_->planning_context()->planning_status().path_decider().is_in_path_lane_borrow_scenario()) {
    return false;
  }

  const auto& nearby_path = frame.reference_line_info().front().reference_line().map_path();
  hdmap::LaneInfoConstPtr lane;
  auto adc_point = common::util::PointFactory::ToPointENU(frame.vehicle_state());
  double s = 0.0;
  double l = 0.0;
  HDMapUtil::BaseMap().GetNearestLaneWithDistance(adc_point, 5.0, &lane, &s, &l);

  if (lane == nullptr || !lane->IsOnLane({adc_point.x(), adc_point.y()})) {
    if (frame.vehicle_state().linear_velocity() < 0.2) {
      for (auto& parkingspace : nearby_path.parking_space_overlaps()) {
        forbiden.insert(parkingspace.object_id);
      }
    }
    return false;
  }

  // 车道宽度检测：港湾车道很窄（~2m），主路车道正常宽度（>3m）
  // 如果车辆在窄车道上，说明在港湾内，激活 forbiden 机制
  {
    double left_width = 0.0;
    double right_width = 0.0;
    lane->GetWidth(s, &left_width, &right_width);
    double total_width = left_width + right_width;
    AINFO << "lane width: " << total_width << " lane_id: " << lane->id().id();
    if (total_width < 2.5) {
      AINFO << "Vehicle on narrow lane (width=" << total_width << "), likely in bay. Adding to forbiden.";
      if (frame.vehicle_state().linear_velocity() < 0.5) {
        for (auto& parkingspace : nearby_path.parking_space_overlaps()) {
          forbiden.insert(parkingspace.object_id);
        }
      }
      return false;
    }
  }

  if (injector_->planning_context()->mutable_planning_status()->mutable_change_lane()->status()
      == ChangeLaneStatus::IN_CHANGE_LANE) {
    return false;
  }

  if (!frame.local_view().planning_command->has_parking_command()) {
    PathOverlap parking_space_overlap;
    const auto& vehicle_state = frame.vehicle_state();
    if (!SearchForNearbyCandidate(frame, nearby_path, &parking_space_overlap)) {
      AINFO << "No parking spot found in surrounding area";
      return false;
    }
    double parking_spot_range_to_start = context_.scenario_config.parking_spot_range_to_start();
    if (!CheckDistanceToParkingSpot(
                frame, vehicle_state, nearby_path, parking_spot_range_to_start, parking_space_overlap)) {
      AINFO << "target parking spot found, but too far, distance larger than "
               "pre-defined distance"
            << parking_space_overlap.object_id;
      return false;
    }
    // 方向检测：车辆到停车点方向与参考线航向的夹角
    // 车辆在主路上时，停车点在侧方港湾里，方向夹角大
    // 车辆在港湾里时，方向夹角小
    {
      const auto& ref_line = frame.reference_line_info().front().reference_line();
      const hdmap::HDMap* hdmap_ptr = hdmap::HDMapUtil::BaseMapPtr();
      hdmap::Id spot_id;
      spot_id.set_id(parking_space_overlap.object_id);
      auto spot_ptr = hdmap_ptr->GetParkingSpaceById(spot_id);
      if (spot_ptr) {
        const auto& pts = spot_ptr->polygon().points();
        Vec2d spot_center = (pts[0] + pts[1] + pts[2] + pts[3]) / 4.0;
        Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
        // 方向：车辆到停车点
        double dir_to_spot = std::atan2(spot_center.y() - vehicle_vec.y(),
                                        spot_center.x() - vehicle_vec.x());
        // 参考线航向：在车辆位置处
        common::SLPoint sl_point;
        ref_line.XYToSL(vehicle_vec, &sl_point);
        auto ref_pt = ref_line.GetReferencePoint(sl_point.s());
        double ref_heading = ref_pt.heading();
        AINFO << "direction check: dir_to_spot=" << dir_to_spot
              << " ref_heading=" << ref_heading
              << " spot: " << parking_space_overlap.object_id
              << " vehicle_s=" << sl_point.s();
      }
    }
    context_.target_parking_spot_id = parking_space_overlap.object_id;
    AINFO << "parking nearby";
    return true;
  }

  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }
  std::string target_parking_spot_id;
  if (frame.local_view().planning_command->has_parking_command() &&
      frame.local_view()
          .planning_command->parking_command()
          .has_parking_spot_id()) {
    target_parking_spot_id = frame.local_view()
                                 .planning_command->parking_command()
                                 .parking_spot_id();
  }

  if (target_parking_spot_id.empty()) {
    return false;
  }

  PathOverlap parking_space_overlap;
  const auto& vehicle_state = frame.vehicle_state();

  if (!SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot_id,
                                     &parking_space_overlap)) {
    ADEBUG << "No such parking spot found after searching all path forward "
              "possible"
           << target_parking_spot_id;
    return false;
  }

  // 方向检测：车辆到停车点方向与参考线航向的夹角
  {
    const auto& ref_line = frame.reference_line_info().front().reference_line();
    const hdmap::HDMap* hdmap_ptr = hdmap::HDMapUtil::BaseMapPtr();
    hdmap::Id spot_id;
    spot_id.set_id(parking_space_overlap.object_id);
    auto spot_ptr = hdmap_ptr->GetParkingSpaceById(spot_id);
    if (spot_ptr) {
      const auto& pts = spot_ptr->polygon().points();
      Vec2d spot_center = (pts[0] + pts[1] + pts[2] + pts[3]) / 4.0;
      Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
      double dir_to_spot = std::atan2(spot_center.y() - vehicle_vec.y(),
                                      spot_center.x() - vehicle_vec.x());
      common::SLPoint sl_point;
      ref_line.XYToSL(vehicle_vec, &sl_point);
      auto ref_pt = ref_line.GetReferencePoint(sl_point.s());
      double ref_heading = ref_pt.heading();
      double angle_diff = dir_to_spot - ref_heading;
      while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
      angle_diff = std::abs(angle_diff);
      AINFO << "direction check: angle_diff=" << angle_diff
            << " dir_to_spot=" << dir_to_spot << " ref_heading=" << ref_heading
            << " spot: " << parking_space_overlap.object_id
            << " vehicle_s=" << sl_point.s();
      if (angle_diff > 0.15) {
        AINFO << "Reject parking: direction to spot (" << angle_diff
              << " rad) differs significantly from ref line heading, vehicle likely on main road";
        return false;
      }
    }
  }

  double parking_spot_range_to_start =
      context_.scenario_config.parking_spot_range_to_start();
  if (!CheckDistanceToParkingSpot(frame, vehicle_state, nearby_path,
                                  parking_spot_range_to_start,
                                  parking_space_overlap)) {
    ADEBUG << "target parking spot found, but too far, distance larger than "
              "pre-defined distance"
           << target_parking_spot_id;
    return false;
  }
  context_.target_parking_spot_id = target_parking_spot_id;
  return true;
}

bool ValetParkingScenario::SearchTargetParkingSpotOnPath(
    const Path& nearby_path, const std::string& target_parking_id,
    PathOverlap* parking_space_overlap) {
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
  for (const auto& parking_overlap : parking_space_overlaps) {
    if (parking_overlap.object_id == target_parking_id) {
      *parking_space_overlap = parking_overlap;
      return true;
    }
  }
  return false;
}

bool ValetParkingScenario::SearchForNearbyCandidate(
        const Frame& frame,
        const Path& nearby_path,
        PathOverlap* parking_space_overlap) {
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
  hdmap::Id id;
  bool found = false;
  double dist = std::numeric_limits<double>::max();
  for (const auto& parking_overlap : parking_space_overlaps) {
    id.set_id(parking_overlap.object_id);
    const auto parking_space = hdmap->GetParkingSpaceById(id);
    if (!parking_space) {
      AINFO << "Skip parking spot because hdmap lookup failed, spot="
            << parking_overlap.object_id;
      continue;
    }
    const auto parking_box = parking_space->polygon().MinAreaBoundingBox();
    for (auto& obs :
         frame.reference_line_info().front().path_decision().obstacles().Items()) {
      if (!obs->IsVirtual() && obs->IsStatic() &&
          obs->PerceptionBoundingBox().HasOverlap(parking_box)) {
        const bool first_latch =
            occupied_parking_spots_.insert(parking_overlap.object_id).second;
        AINFO << "Latch occupied parking spot, spot="
              << parking_overlap.object_id << ", obstacle=" << obs->Id()
              << ", perception_id=" << obs->PerceptionId()
              << ", first_latch=" << first_latch;
        break;
      }
    }

    if (forbiden.find(parking_overlap.object_id) != forbiden.end()) {
      AINFO << "Skip parking spot by forbidden set, spot="
            << parking_overlap.object_id;
      continue;
    }
    if (occupied_parking_spots_.find(parking_overlap.object_id) !=
        occupied_parking_spots_.end()) {
      AINFO << "Skip parking spot by occupied latch, spot="
            << parking_overlap.object_id;
      continue;
    }

    const double candidate_dist = std::fabs(
        frame.reference_line_info().front().AdcSlBoundary().end_s() -
        parking_overlap.start_s);
    AINFO << "Parking candidate available, spot="
          << parking_overlap.object_id << ", distance=" << candidate_dist;
    if (candidate_dist < dist) {
      dist = candidate_dist;
      *parking_space_overlap = parking_overlap;
    }
    found = true;
  }
  if (found) {
    AINFO << "Selected parking candidate, spot="
          << parking_space_overlap->object_id << ", distance=" << dist
          << ", occupied_latched_count=" << occupied_parking_spots_.size();
  } else {
    AINFO << "No available parking candidate after occupied latch filtering";
  }
  return found;
}

bool ValetParkingScenario::CheckDistanceToParkingSpot(
    const Frame& frame, const VehicleState& vehicle_state,
    const Path& nearby_path, const double parking_start_range,
    const PathOverlap& parking_space_overlap) {
  // TODO(Jinyun) parking overlap s are wrong on map, not usable
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  double center_point_s, center_point_l;
  id.set_id(parking_space_overlap.object_id);
  ParkingSpaceInfoConstPtr target_parking_spot_ptr =
      hdmap->GetParkingSpaceById(id);
  Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
  Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
  Vec2d right_top_point = target_parking_spot_ptr->polygon().points().at(2);
  Vec2d left_top_point = target_parking_spot_ptr->polygon().points().at(3);
  Vec2d center_point = (left_bottom_point + right_bottom_point +
                        right_top_point + left_top_point) /
                       4.0;
  nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l);
  double vehicle_point_s = 0.0;
  double vehicle_point_l = 0.0;
  Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
  if (std::abs(center_point_s - vehicle_point_s) < parking_start_range) {
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
