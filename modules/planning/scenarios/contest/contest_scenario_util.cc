/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/contest/contest_scenario_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "modules/common/math/math_utils.h"
#include "modules/planning/planning_base/common/contest_scenario_features.h"

namespace apollo {
namespace planning {
namespace contest {

bool IsContestLaneChange(const Frame& frame) {
  return frame.local_view().planning_command != nullptr &&
         frame.local_view().planning_command->has_lane_follow_command() &&
         frame.reference_line_info().size() > 1;
}

bool IsContestSCurve(const ReferenceLineInfo& reference_line_info,
                     const ScenarioContestConfig& config) {
  const auto& reference_line = reference_line_info.reference_line();
  const double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

  double max_abs_kappa = 0.0;
  for (double s = adc_start_s;
       s < adc_start_s + config.s_curve_look_forward_distance() &&
       s < reference_line.Length();
       s += kFeatureSampleStep) {
    max_abs_kappa =
        std::max(max_abs_kappa,
                 std::fabs(reference_line.GetReferencePoint(s).kappa()));
  }
  if (max_abs_kappa < config.s_curve_min_abs_kappa()) {
    return false;
  }

  int small_obstacle_count = 0;
  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (!IsSmallRealObstacle(obstacle)) {
      continue;
    }
    const auto& sl_boundary = obstacle->PerceptionSLBoundary();
    if (sl_boundary.end_s() < adc_start_s - 2.0 ||
        sl_boundary.start_s() >
            adc_end_s + config.s_curve_look_forward_distance()) {
      continue;
    }
    const double obstacle_center_l =
        (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
    if (std::fabs(obstacle_center_l) > config.s_curve_max_obstacle_abs_l()) {
      continue;
    }
    if (++small_obstacle_count >= config.s_curve_min_cone_count()) {
      return true;
    }
  }
  return false;
}

bool IsContestUTurn(const ReferenceLineInfo& reference_line_info,
                    const ScenarioContestConfig& config) {
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  const auto& reference_line = reference_line_info.reference_line();
  const double ref_length = reference_line.Length();

  for (double s = adc_end_s;
       s < adc_end_s + config.u_turn_look_forward_distance() &&
       s < ref_length;
       s += kFeatureSampleStep) {
    if (reference_line_info.GetPathTurnType(s) == hdmap::Lane::U_TURN) {
      return true;
    }
  }
  if (reference_line_info.GetPathTurnType(adc_end_s) == hdmap::Lane::U_TURN) {
    return true;
  }

  double high_kappa_s = -1.0;
  for (double s = adc_end_s + 5.0; s + kFeatureSampleStep < ref_length;
       s += kFeatureSampleStep) {
    if (std::fabs(reference_line.GetReferencePoint(s).kappa()) >
        config.u_turn_kappa_threshold()) {
      high_kappa_s = s;
      break;
    }
  }
  if (high_kappa_s < 0.0) {
    return false;
  }

  const double window_start =
      std::max(0.0, high_kappa_s - kDefaultUTurnHeadingWindowRadius);
  const double window_end =
      std::min(ref_length - 1.0,
               high_kappa_s + kDefaultUTurnHeadingWindowRadius);
  if (window_end - window_start < 10.0) {
    return false;
  }
  const double heading_change = std::fabs(common::math::NormalizeAngle(
      reference_line.GetReferencePoint(window_end).heading() -
      reference_line.GetReferencePoint(window_start).heading()));
  return heading_change > config.u_turn_heading_change_threshold();
}

bool IsContestConstructionZone(const ReferenceLineInfo& reference_line_info,
                               const ScenarioContestConfig& config) {
  const double adc_back_s = reference_line_info.AdcSlBoundary().start_s();
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  int cone_count = 0;
  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (!IsSmallRealObstacle(obstacle)) {
      continue;
    }
    const auto& sl = obstacle->PerceptionSLBoundary();
    // 只统计主车前方范围内的锥桶，不依赖XY硬编码区域
    if (sl.start_s() > adc_back_s - 3.0 &&
        sl.start_s() - adc_end_s < config.construction_look_forward_distance()) {
      ++cone_count;
    }
  }
  return cone_count >= config.construction_min_cone_count();
}

bool IsContestStationShuttle(const ReferenceLineInfo& reference_line_info,
                             const ScenarioContestConfig& config,
                             std::string* out_parking_spot_id) {
  if (out_parking_spot_id == nullptr) {
    return false;
  }
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  const auto& nearby_path =
      reference_line_info.reference_line().map_path();
  const double look_forward = config.station_shuttle_look_forward_distance();

  // 遍历 reference line 上的 parking space overlap，找前方最近的泊车位
  double best_distance = std::numeric_limits<double>::max();
  std::string best_spot_id;

  for (const auto& overlap : nearby_path.parking_space_overlaps()) {
    if (overlap.start_s <= adc_end_s) {
      continue;  // 已驶过的泊车位
    }
    const double dist = overlap.start_s - adc_end_s;
    if (dist > look_forward) {
      continue;  // 超出探测范围
    }
    if (dist < best_distance) {
      best_distance = dist;
      best_spot_id = overlap.object_id;
    }
  }

  if (best_spot_id.empty()) {
    return false;
  }
  *out_parking_spot_id = best_spot_id;
  return true;
}

}  // namespace contest
}  // namespace planning
}  // namespace apollo
