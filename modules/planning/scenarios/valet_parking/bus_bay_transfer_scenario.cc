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

#include "modules/planning/scenarios/valet_parking/bus_bay_transfer_scenario.h"

#include <cmath>
#include <limits>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_bus_bay.h"
#include "modules/planning/scenarios/valet_parking/stage_departing_from_bus_bay.h"
#include "modules/planning/scenarios/valet_parking/stage_dwelling_at_bus_bay.h"

namespace apollo {
namespace planning {
namespace {

// ---------- anonymous helpers ----------

constexpr double kBoxCenterMatchDist = 0.8;
constexpr double kBoxHeadingMatchTol = 0.35;

bool BoxesAreSimilar(const common::math::Box2d& a,
                     const common::math::Box2d& b) {
  return a.center().DistanceTo(b.center()) < kBoxCenterMatchDist &&
         std::fabs(std::remainder(a.heading() - b.heading(),
                                  2.0 * M_PI)) < kBoxHeadingMatchTol;
}

constexpr double kMaxObstacleLatchRange = 40.0;
constexpr size_t kMaxLatchedBoxCount = 16;

bool TryMergeIntoLatched(const common::math::Box2d& candidate,
                         std::vector<common::math::Box2d>* latched,
                         const std::string& source,
                         const std::string& obs_id,
                         int32_t perception_id) {
  for (auto& box : *latched) {
    if (BoxesAreSimilar(box, candidate)) {
      box = candidate;
      AINFO << "Bus-bay: merged latched obstacle, src=" << source
            << ", obs_id=" << obs_id << ", perc_id=" << perception_id
            << ", xy=(" << candidate.center().x() << ","
            << candidate.center().y() << "), count=" << latched->size();
      return true;
    }
  }
  return false;
}

void AddNewLatchedBox(const common::math::Box2d& box,
                      std::vector<common::math::Box2d>* latched,
                      const std::string& source,
                      const std::string& obs_id,
                      int32_t perception_id) {
  if (latched->size() >= kMaxLatchedBoxCount) {
    AINFO << "Bus-bay: latch memory full, src=" << source
          << ", obs_id=" << obs_id << ", perc_id=" << perception_id
          << ", max=" << kMaxLatchedBoxCount;
    return;
  }
  latched->push_back(box);
  AINFO << "Bus-bay: latched new obstacle, src=" << source
        << ", obs_id=" << obs_id << ", perc_id=" << perception_id
        << ", xy=(" << box.center().x() << "," << box.center().y()
        << "), heading=" << box.heading() << ", l=" << box.length()
        << ", w=" << box.width() << ", count=" << latched->size();
}

// ---------- direction helpers ----------

void LogDirectionCheck(const Frame& frame,
                       const hdmap::PathOverlap& overlap,
                       const std::string& tag) {
  const auto& vstate = frame.vehicle_state();
  const auto& ref = frame.reference_line_info().front().reference_line();
  const auto* hdmap_ptr = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id spot_id;
  spot_id.set_id(overlap.object_id);
  auto spot_ptr = hdmap_ptr->GetParkingSpaceById(spot_id);
  if (!spot_ptr) return;
  const auto& pts = spot_ptr->polygon().points();
  common::math::Vec2d center =
      (pts[0] + pts[1] + pts[2] + pts[3]) / 4.0;
  double dir_to = std::atan2(center.y() - vstate.y(),
                             center.x() - vstate.x());
  common::SLPoint sl;
  ref.XYToSL({vstate.x(), vstate.y()}, &sl);
  double ref_h = ref.GetReferencePoint(sl.s()).heading();
  AINFO << "Bus-bay: " << tag << " direction, dir_to=" << dir_to
        << ", ref_h=" << ref_h << ", spot=" << overlap.object_id
        << ", adc_s=" << sl.s();
}

double ComputeAngleBetweenVehicleAndSpot(const Frame& frame,
                                         const hdmap::PathOverlap& overlap) {
  const auto& vstate = frame.vehicle_state();
  const auto& ref = frame.reference_line_info().front().reference_line();
  const auto* hdmap_ptr = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id spot_id;
  spot_id.set_id(overlap.object_id);
  auto spot_ptr = hdmap_ptr->GetParkingSpaceById(spot_id);
  if (!spot_ptr) return -1.0;
  const auto& pts = spot_ptr->polygon().points();
  common::math::Vec2d center =
      (pts[0] + pts[1] + pts[2] + pts[3]) / 4.0;
  double dir_to = std::atan2(center.y() - vstate.y(),
                             center.x() - vstate.x());
  common::SLPoint sl;
  ref.XYToSL({vstate.x(), vstate.y()}, &sl);
  double ref_h = ref.GetReferencePoint(sl.s()).heading();
  double diff = dir_to - ref_h;
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  return std::abs(diff);
}

}  // namespace

using apollo::common::VehicleState;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

// ========== BusBayTransferContext ==========

void BusBayTransferContext::LatchStaticObstacles(const Frame& frame,
                                                 const std::string& source) {
  const Vec2d adc_xy(frame.vehicle_state().x(),
                     frame.vehicle_state().y());
  for (const auto* obs : frame.obstacles()) {
    if (obs == nullptr || obs->IsVirtual() || !obs->IsStatic()) continue;
    const Box2d box = obs->PerceptionBoundingBox();
    if (box.DistanceTo(adc_xy) > kMaxObstacleLatchRange) continue;

    if (TryMergeIntoLatched(box, &latched_static_obstacle_boxes,
                            source, obs->Id(), obs->PerceptionId())) {
      continue;
    }
    AddNewLatchedBox(box, &latched_static_obstacle_boxes,
                     source, obs->Id(), obs->PerceptionId());
  }
}

void BusBayTransferContext::InjectLatchedStaticObstacles(Frame* frame) const {
  if (frame == nullptr || latched_static_obstacle_boxes.empty()) return;
  AINFO << "Bus-bay: skip replaying latched obstacles, count="
        << latched_static_obstacle_boxes.size()
        << ", visible=" << frame->obstacles().size();
}

// ========== BusBayTransferScenario ==========

bool BusBayTransferScenario::Init(
    std::shared_ptr<DependencyInjector> injector,
    const std::string& name) {
  if (init_) return true;

  if (!Scenario::Init(injector, name)) {
    AERROR << "Bus-bay: scenario init failed: " << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioBusBayTransferConfig>(
          &context_.scenario_config)) {
    AERROR << "Bus-bay: config load failed: " << Name();
    return false;
  }
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  init_ = true;
  return true;
}

bool BusBayTransferScenario::Enter(Frame* frame) {
  auto saved_config = context_.scenario_config;
  const std::string saved_spot_id = context_.target_parking_spot_id;
  context_ = BusBayTransferContext();
  context_.scenario_config.CopyFrom(saved_config);
  context_.target_parking_spot_id = saved_spot_id;
  forbiden.clear();
  occupied_parking_spots_.clear();
  return Scenario::Enter(frame);
}

bool BusBayTransferScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  // --- early return checks ---
  if (frame.reference_line_info().empty()) {
    AINFO << "Bus-bay: skip transfer (empty ref line)";
    return false;
  }
  context_.LatchStaticObstacles(frame, "transfer");
  if (context_.shuttle_mission_completed) {
    AINFO << "Bus-bay: skip transfer (shuttle already done)";
    return false;
  }

  if (injector_->planning_context()
          ->planning_status().path_decider()
          .is_in_path_lane_borrow_scenario()) {
    return false;
  }

  // --- lane lookup ---
  const auto& nearby_path =
      frame.reference_line_info().front().reference_line().map_path();
  hdmap::LaneInfoConstPtr lane;
  auto adc_pt =
      common::util::PointFactory::ToPointENU(frame.vehicle_state());
  double s = 0.0, l = 0.0;
  HDMapUtil::BaseMap().GetNearestLaneWithDistance(adc_pt, 5.0, &lane, &s, &l);

  if (lane == nullptr || !lane->IsOnLane({adc_pt.x(), adc_pt.y()})) {
    if (frame.vehicle_state().linear_velocity() < 0.2) {
      for (auto& ps : nearby_path.parking_space_overlaps()) {
        forbiden.insert(ps.object_id);
      }
    }
    return false;
  }

  // --- narrow lane (bus bay) detection ---
  {
    double left_w = 0.0, right_w = 0.0;
    lane->GetWidth(s, &left_w, &right_w);
    double total_w = left_w + right_w;
    AINFO << "Bus-bay: lane width=" << total_w
          << ", lane_id=" << lane->id().id();
    if (total_w < 2.5) {
      AINFO << "Bus-bay: on narrow lane (width=" << total_w
            << "), likely inside bay, adding to forbidden set";
      if (frame.vehicle_state().linear_velocity() < 0.5) {
        for (auto& ps : nearby_path.parking_space_overlaps()) {
          forbiden.insert(ps.object_id);
        }
      }
      return false;
    }
  }

  if (injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_change_lane()
          ->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
    return false;
  }

  // --- autonomous search (no parking command) ---
  if (!frame.local_view().planning_command->has_parking_command()) {
    PathOverlap candidate;
    const auto& vstate = frame.vehicle_state();
    if (!SearchForNearbyCandidate(frame, nearby_path, &candidate)) {
      AINFO << "Bus-bay: no available bay spot nearby";
      return false;
    }
    double range =
        context_.scenario_config.parking_spot_range_to_start();
    if (!CheckDistanceToParkingSpot(frame, vstate, nearby_path,
                                    range, candidate)) {
      AINFO << "Bus-bay: candidate spot too far: " << candidate.object_id;
      return false;
    }
    LogDirectionCheck(frame, candidate, "auto-search");
    context_.target_parking_spot_id = candidate.object_id;
    AINFO << "Bus-bay: auto-detected bay spot nearby";
    return true;
  }

  // --- command-driven transfer ---
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }
  std::string target_id;
  if (frame.local_view().planning_command->has_parking_command() &&
      frame.local_view().planning_command->parking_command()
          .has_parking_spot_id()) {
    target_id = frame.local_view().planning_command->parking_command()
                    .parking_spot_id();
  }
  if (target_id.empty()) return false;

  PathOverlap target_overlap;
  const auto& vstate = frame.vehicle_state();
  if (!SearchTargetParkingSpotOnPath(nearby_path, target_id,
                                     &target_overlap)) {
    ADEBUG << "Bus-bay: commanded spot not on path: " << target_id;
    return false;
  }

  // direction gate: vehicle must be aligned with spot direction
  double angle_diff = ComputeAngleBetweenVehicleAndSpot(frame, target_overlap);
  AINFO << "Bus-bay: commanded direction check, angle_diff=" << angle_diff
        << ", spot=" << target_overlap.object_id;
  if (angle_diff > 0.15) {
    AINFO << "Bus-bay: reject (vehicle likely on main road, "
          << "angle_diff=" << angle_diff << " rad)";
    return false;
  }

  double range = context_.scenario_config.parking_spot_range_to_start();
  if (!CheckDistanceToParkingSpot(frame, vstate, nearby_path,
                                  range, target_overlap)) {
    ADEBUG << "Bus-bay: commanded spot too far: " << target_id;
    return false;
  }
  context_.target_parking_spot_id = target_id;
  return true;
}

// ---------- static helpers ----------

bool BusBayTransferScenario::SearchTargetParkingSpotOnPath(
    const Path& nearby_path, const std::string& target_parking_id,
    PathOverlap* parking_space_overlap) {
  for (const auto& overlap : nearby_path.parking_space_overlaps()) {
    if (overlap.object_id == target_parking_id) {
      *parking_space_overlap = overlap;
      return true;
    }
  }
  return false;
}

bool BusBayTransferScenario::SearchForNearbyCandidate(
    const Frame& frame, const Path& nearby_path,
    PathOverlap* parking_space_overlap) {
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  const auto& overlaps = nearby_path.parking_space_overlaps();
  hdmap::Id id;
  bool found = false;
  double best_dist = std::numeric_limits<double>::max();

  for (const auto& overlap : overlaps) {
    id.set_id(overlap.object_id);
    const auto space = hdmap->GetParkingSpaceById(id);
    if (!space) {
      AINFO << "Bus-bay: skip spot (hdmap lookup failed): "
            << overlap.object_id;
      continue;
    }
    const auto space_box = space->polygon().MinAreaBoundingBox();

    // obstacle occupancy check
    for (auto& obs : frame.reference_line_info()
                         .front().path_decision().obstacles().Items()) {
      if (!obs->IsVirtual() && obs->IsStatic() &&
          obs->PerceptionBoundingBox().HasOverlap(space_box)) {
        bool first = occupied_parking_spots_.insert(overlap.object_id).second;
        AINFO << "Bus-bay: occupied spot latched, spot="
              << overlap.object_id << ", obs=" << obs->Id()
              << ", perc_id=" << obs->PerceptionId()
              << ", first_latch=" << first;
        break;
      }
    }

    if (forbiden.find(overlap.object_id) != forbiden.end()) {
      AINFO << "Bus-bay: skip spot (forbidden): " << overlap.object_id;
      continue;
    }
    if (occupied_parking_spots_.find(overlap.object_id) !=
        occupied_parking_spots_.end()) {
      AINFO << "Bus-bay: skip spot (occupied): " << overlap.object_id;
      continue;
    }

    double dist = std::fabs(
        frame.reference_line_info().front().AdcSlBoundary().end_s() -
        overlap.start_s);
    AINFO << "Bus-bay: candidate spot=" << overlap.object_id
          << ", dist=" << dist;
    if (dist < best_dist) {
      best_dist = dist;
      *parking_space_overlap = overlap;
    }
    found = true;
  }

  if (found) {
    AINFO << "Bus-bay: selected candidate spot="
          << parking_space_overlap->object_id << ", dist=" << best_dist
          << ", occupied_count=" << occupied_parking_spots_.size();
  } else {
    AINFO << "Bus-bay: no candidate after filtering";
  }
  return found;
}

bool BusBayTransferScenario::CheckDistanceToParkingSpot(
    const Frame& frame, const VehicleState& vehicle_state,
    const Path& nearby_path, const double parking_start_range,
    const PathOverlap& parking_space_overlap) {
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  id.set_id(parking_space_overlap.object_id);
  ParkingSpaceInfoConstPtr spot = hdmap->GetParkingSpaceById(id);
  const auto& pts = spot->polygon().points();
  Vec2d center = (pts[0] + pts[1] + pts[2] + pts[3]) / 4.0;

  double spot_s = 0.0, spot_l = 0.0;
  nearby_path.GetNearestPoint(center, &spot_s, &spot_l);
  double adc_s = 0.0, adc_l = 0.0;
  Vec2d adc_vec(vehicle_state.x(), vehicle_state.y());
  nearby_path.GetNearestPoint(adc_vec, &adc_s, &adc_l);

  return std::abs(spot_s - adc_s) < parking_start_range;
}

}  // namespace planning
}  // namespace apollo
