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

#pragma once

#include <algorithm>
#include <cmath>

#include "modules/common/math/math_utils.h"
#include "modules/common_msgs/map_msgs/map_lane.pb.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/common/reference_line_info.h"

namespace apollo {
namespace planning {
namespace contest {

constexpr double kSmallConeObstacleArea = 0.5;
constexpr double kFeatureSampleStep = 2.0;
constexpr double kDefaultConstructionMinX = 423990.0;
constexpr double kDefaultConstructionMaxX = 424210.0;
constexpr double kDefaultConstructionMinY = 4437580.0;
constexpr double kDefaultConstructionMaxY = 4437650.0;
constexpr double kDefaultConstructionLookForwardDistance = 100.0;
constexpr double kDefaultSCurveLookForwardDistance = 90.0;
constexpr double kDefaultSCurveMinAbsKappa = 0.015;
constexpr double kDefaultSCurveMaxObstacleAbsL = 3.5;
constexpr int kDefaultSCurveMinConeCount = 4;
constexpr double kDefaultUTurnLookForwardDistance = 35.0;
constexpr double kDefaultUTurnKappaThreshold = 0.12;
constexpr double kDefaultUTurnHeadingChangeThreshold = 2.0;
constexpr double kDefaultUTurnHeadingWindowRadius = 17.0;

inline bool IsSmallRealObstacle(const Obstacle* obstacle) {
    return obstacle != nullptr && !obstacle->IsVirtual()
            && obstacle->PerceptionPolygon().area() < kSmallConeObstacleArea;
}

inline bool GetObstacleCenterXY(const Obstacle* obstacle, double* cx, double* cy) {
    if (obstacle == nullptr || cx == nullptr || cy == nullptr) {
        return false;
    }
    const auto& points = obstacle->PerceptionPolygon().points();
    if (points.empty()) {
        return false;
    }
    *cx = 0.0;
    *cy = 0.0;
    for (const auto& point : points) {
        *cx += point.x();
        *cy += point.y();
    }
    *cx /= points.size();
    *cy /= points.size();
    return true;
}

inline bool IsDefaultConstructionZoneXY(double x, double y) {
    return x > kDefaultConstructionMinX && x < kDefaultConstructionMaxX && y > kDefaultConstructionMinY
            && y < kDefaultConstructionMaxY;
}

inline int CountDefaultConstructionConesAhead(const ReferenceLineInfo& reference_line_info) {
    const double adc_back_s = reference_line_info.AdcSlBoundary().start_s();
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    int cone_count = 0;
    for (const auto* obstacle : reference_line_info.path_decision().obstacles().Items()) {
        if (!IsSmallRealObstacle(obstacle)) {
            continue;
        }
        double cx = 0.0;
        double cy = 0.0;
        if (!GetObstacleCenterXY(obstacle, &cx, &cy) || !IsDefaultConstructionZoneXY(cx, cy)) {
            continue;
        }
        const auto& sl = obstacle->PerceptionSLBoundary();
        if (sl.end_s() > adc_back_s - 6.0 &&
            sl.start_s() < adc_end_s + kDefaultConstructionLookForwardDistance) {
            ++cone_count;
        }
    }
    return cone_count;
}

inline bool IsDefaultDenseConeSCurve(const ReferenceLineInfo& reference_line_info) {
    const auto& reference_line = reference_line_info.reference_line();
    const double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

    double max_abs_kappa = 0.0;
    for (double s = adc_start_s; s < adc_start_s + kDefaultSCurveLookForwardDistance && s < reference_line.Length();
         s += kFeatureSampleStep) {
        max_abs_kappa = std::max(max_abs_kappa, std::fabs(reference_line.GetReferencePoint(s).kappa()));
    }
    if (max_abs_kappa < kDefaultSCurveMinAbsKappa) {
        return false;
    }

    int small_obstacle_count = 0;
    for (const auto* obstacle : reference_line_info.path_decision().obstacles().Items()) {
        if (!IsSmallRealObstacle(obstacle)) {
            continue;
        }
        const auto& sl_boundary = obstacle->PerceptionSLBoundary();
        if (sl_boundary.end_s() < adc_start_s - 2.0
            || sl_boundary.start_s() > adc_end_s + kDefaultSCurveLookForwardDistance) {
            continue;
        }
        const double obstacle_center_l = (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
        if (std::fabs(obstacle_center_l) > kDefaultSCurveMaxObstacleAbsL) {
            continue;
        }
        if (++small_obstacle_count >= kDefaultSCurveMinConeCount) {
            return true;
        }
    }
    return false;
}

inline bool HasDefaultUTurnLaneInPath(const ReferenceLineInfo& reference_line_info) {
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    const auto& reference_line = reference_line_info.reference_line();
    const double ref_length = reference_line.Length();

    for (double s = adc_end_s; s < adc_end_s + kDefaultUTurnLookForwardDistance && s < ref_length;
         s += kFeatureSampleStep) {
        if (reference_line_info.GetPathTurnType(s) == hdmap::Lane::U_TURN) {
            return true;
        }
    }
    if (reference_line_info.GetPathTurnType(adc_end_s) == hdmap::Lane::U_TURN) {
        return true;
    }

    double high_kappa_s = -1.0;
    for (double s = adc_end_s + 5.0; s + kFeatureSampleStep < ref_length; s += kFeatureSampleStep) {
        if (std::fabs(reference_line.GetReferencePoint(s).kappa()) > kDefaultUTurnKappaThreshold) {
            high_kappa_s = s;
            break;
        }
    }
    if (high_kappa_s < 0.0) {
        return false;
    }

    const double window_start = std::max(0.0, high_kappa_s - kDefaultUTurnHeadingWindowRadius);
    const double window_end = std::min(ref_length - 1.0, high_kappa_s + kDefaultUTurnHeadingWindowRadius);
    if (window_end - window_start < 10.0) {
        return false;
    }
    const double heading_change = std::fabs(
            common::math::NormalizeAngle(
                    reference_line.GetReferencePoint(window_end).heading()
                    - reference_line.GetReferencePoint(window_start).heading()));
    return heading_change > kDefaultUTurnHeadingChangeThreshold;
}

}  // namespace contest
}  // namespace planning
}  // namespace apollo
