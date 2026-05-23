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

#include "modules/planning/planning_base/common/util/common.h"

#include <algorithm>
#include <cmath>

#include "modules/common/math/math_utils.h"
#include "modules/common_msgs/map_msgs/map_lane.pb.h"

namespace apollo {
namespace planning {
namespace util {

using apollo::common::util::WithinBound;

namespace {

constexpr double kRoundaboutStopWallDistance = 0.6;

bool HasPrefix(const std::string& text, const std::string& prefix) {
  return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

bool IsRoundaboutStopWallId(const std::string& stop_wall_id) {
  return HasPrefix(stop_wall_id, "PNC_JUNCTION_") ||
         HasPrefix(stop_wall_id, "YS_") || HasPrefix(stop_wall_id, "SS_") ||
         HasPrefix(stop_wall_id, "KC_JC_") ||
         HasPrefix(stop_wall_id, "PATH_END_");
}

bool IsRoundaboutEntryLike(const ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr) {
    return false;
  }
  constexpr double kEntryLookForward = 35.0;
  constexpr double kInsideJunctionBuffer = 8.0;
  constexpr double kCurveLookForward = 55.0;
  constexpr double kMinMaxKappa = 0.025;
  constexpr double kMinHeadingChange = 0.65;
  constexpr double kUTurnHeadingChange = 1.8;

  const double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  bool near_pnc_junction = false;
  for (const auto& overlap :
       reference_line_info->reference_line().map_path().pnc_junction_overlaps()) {
    if (overlap.end_s < adc_end_s - kInsideJunctionBuffer) {
      continue;
    }
    if (overlap.start_s > adc_end_s + kEntryLookForward) {
      continue;
    }
    near_pnc_junction = true;
    break;
  }
  if (!near_pnc_junction) {
    return false;
  }

  const auto& reference_line = reference_line_info->reference_line();
  const double ref_length = reference_line.Length();
  const double start_s = std::max(0.0, adc_end_s);
  const double end_s = std::min(ref_length - 1.0, adc_end_s + kCurveLookForward);
  if (end_s - start_s < 8.0) {
    return false;
  }

  double max_abs_kappa = 0.0;
  for (double s = start_s; s <= end_s; s += 2.0) {
    if (reference_line_info->GetPathTurnType(s) == hdmap::Lane::U_TURN) {
      return false;
    }
    max_abs_kappa =
        std::max(max_abs_kappa, std::fabs(reference_line.GetReferencePoint(s).kappa()));
  }
  const double heading_change = std::fabs(common::math::NormalizeAngle(
      reference_line.GetReferencePoint(end_s).heading() -
      reference_line.GetReferencePoint(start_s).heading()));
  return max_abs_kappa > kMinMaxKappa &&
         heading_change > kMinHeadingChange &&
         heading_change < kUTurnHeadingChange;
}

double ClampRoundaboutStopDistance(const std::string& stop_wall_id,
                                   const ReferenceLineInfo* reference_line_info,
                                   const double stop_distance) {
  if (!IsRoundaboutStopWallId(stop_wall_id) ||
      !IsRoundaboutEntryLike(reference_line_info)) {
    return stop_distance;
  }
  const double clamped_stop_distance =
      std::min(stop_distance, kRoundaboutStopWallDistance);
  if (clamped_stop_distance < stop_distance) {
    AINFO << "[ROUNDABOUT][StopWall] clamp stop_distance, wall="
          << stop_wall_id << ", from=" << stop_distance
          << ", to=" << clamped_stop_distance;
  }
  return clamped_stop_distance;
}

bool KeepRoundaboutWaitForObstacle(const std::string& stop_wall_id,
                                   const std::string& obstacle_id,
                                   const ReferenceLineInfo* reference_line_info) {
  if (!IsRoundaboutStopWallId(stop_wall_id) ||
      !IsRoundaboutEntryLike(reference_line_info)) {
    return true;
  }
  AINFO << "[ROUNDABOUT][StopWall] drop wait_for visualization, wall="
        << stop_wall_id << ", obs=" << obstacle_id;
  return false;
}

}  // namespace

/*
 * @brief: build virtual obstacle of stop wall, and add STOP decision
 */
int BuildStopDecision(const std::string& stop_wall_id, const double stop_line_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info,
                      double stop_wall_width) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), stop_line_s)) {
    AERROR << "stop_line_s[" << stop_line_s << "] is not on reference line";
    return 0;
  }

  // create virtual stop wall
  const auto* obstacle =
      frame->CreateStopObstacle(reference_line_info, stop_wall_id,
                                stop_line_s, stop_wall_width);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }
  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to add obstacle[" << stop_wall_id << "]";
    return -1;
  }

  const double clamped_stop_distance = ClampRoundaboutStopDistance(
      stop_wall_id, reference_line_info, stop_distance);

  // build stop decision
  const double stop_s = stop_line_s - clamped_stop_distance;
  const auto& stop_point = reference_line.GetReferencePoint(stop_s);
  const double stop_heading =
      reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-clamped_stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  for (size_t i = 0; i < wait_for_obstacles.size(); ++i) {
    if (KeepRoundaboutWaitForObstacle(stop_wall_id, wait_for_obstacles[i],
                                      reference_line_info)) {
      stop_decision->add_wait_for_obstacle(wait_for_obstacles[i]);
    }
  }

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return 0;
}

int BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id, const double lane_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  const auto& reference_line = reference_line_info->reference_line();

  // create virtual stop wall
  const auto* obstacle =
      frame->CreateStopObstacle(stop_wall_id, lane_id, lane_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }

  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create obstacle for: " << stop_wall_id;
    return -1;
  }

  const auto& stop_wall_box = stop_wall->PerceptionBoundingBox();
  if (!reference_line.IsOnLane(stop_wall_box.center())) {
    ADEBUG << "stop point is not on lane. SKIP STOP decision";
    return 0;
  }

  const double clamped_stop_distance = ClampRoundaboutStopDistance(
      stop_wall_id, reference_line_info, stop_distance);

  // build stop decision
  auto stop_point = reference_line.GetReferencePoint(
      stop_wall->PerceptionSLBoundary().start_s() - clamped_stop_distance);

  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-clamped_stop_distance);
  stop_decision->set_stop_heading(stop_point.heading());
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  for (size_t i = 0; i < wait_for_obstacles.size(); ++i) {
    if (KeepRoundaboutWaitForObstacle(stop_wall_id, wait_for_obstacles[i],
                                      reference_line_info)) {
      stop_decision->add_wait_for_obstacle(wait_for_obstacles[i]);
    }
  }

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return 0;
}

}  // namespace util
}  // namespace planning
}  // namespace apollo
