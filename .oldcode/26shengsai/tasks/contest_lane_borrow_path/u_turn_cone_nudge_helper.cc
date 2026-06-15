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

#include "modules/planning/tasks/contest_lane_borrow_path/u_turn_cone_nudge_helper.h"

#include <algorithm>
#include <cmath>

#include "modules/planning/planning_base/common/contest_scenario_features.h"

namespace apollo {
namespace planning {
namespace {

constexpr double kLookBackDistance = 20.0;
constexpr double kLookForwardDistance = 100.0;
constexpr double kMinKappa = 0.001;
constexpr double kMaxConeLateralGap = 7.0;
constexpr int kMinConeCount = 10;

}  // namespace

bool ShouldUseUTurnConeNudge(const ReferenceLineInfo& reference_line_info) {
    double max_kappa = 0.0;
    const double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
    for (double s = adc_start_s - kLookBackDistance;
         s - adc_start_s < kLookForwardDistance;
         s += contest::kFeatureSampleStep) {
        max_kappa = std::max(max_kappa, std::fabs(reference_line_info.reference_line().GetReferencePoint(s).kappa()));
    }

    int cone_count = 0;
    for (const auto* obstacle : reference_line_info.path_decision().obstacles().Items()) {
        if (!contest::IsSmallRealObstacle(obstacle)) {
            continue;
        }
        if (obstacle->PerceptionSLBoundary().end_l() - reference_line_info.AdcSlBoundary().end_l()
            < kMaxConeLateralGap) {
            ++cone_count;
        }
    }
    return max_kappa > kMinKappa && cone_count >= kMinConeCount;
}

void ApplyUTurnConeNudge(
        const ReferenceLineInfo& reference_line_info,
        std::vector<SLPolygon>* obs_sl_polygons) {
    if (obs_sl_polygons == nullptr) {
        return;
    }
    const double adc_end_l = reference_line_info.AdcSlBoundary().end_l();
    for (auto& sl_polygon : *obs_sl_polygons) {
        if (std::fabs(sl_polygon.MaxL() - adc_end_l) > kMaxConeLateralGap) {
            sl_polygon.SetNudgeInfo(SLPolygon::UNDEFINED);
        } else {
            sl_polygon.SetNudgeInfo(SLPolygon::RIGHT_NUDGE);
        }
    }
}

}  // namespace planning
}  // namespace apollo
