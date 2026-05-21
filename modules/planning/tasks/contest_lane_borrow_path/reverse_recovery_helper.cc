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

#include "modules/planning/tasks/contest_lane_borrow_path/reverse_recovery_helper.h"

#include <algorithm>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

bool GenerateReverseRecoveryPathBoundary(
        const ReferenceLine& reference_line,
        double adc_s,
        double adc_l,
        double reverse_distance,
        PathBoundary* boundary) {
    if (boundary == nullptr) {
        return false;
    }
    boundary->clear();

    constexpr double kReverseSampleStep = -0.1;
    constexpr double kReferenceLineStartPadding = 0.5;
    constexpr double kLateralMargin = 0.5;
    constexpr double kLateralFallbackMargin = 0.2;

    const double target_s = adc_s - reverse_distance;
    const double ref_min_s = reference_line.GetMapPath().accumulated_s().front();
    const double actual_target = std::max(target_s, ref_min_s + kReferenceLineStartPadding);

    boundary->set_delta_s(kReverseSampleStep);
    boundary->set_label("regular/reverse_path");

    for (double s = adc_s; s > actual_target; s += kReverseSampleStep) {
        double lane_left = 0.0;
        double lane_right = 0.0;
        if (!reference_line.GetLaneWidth(s, &lane_left, &lane_right)) {
            break;
        }

        double offset = 0.0;
        reference_line.GetOffsetToMap(s, &offset);
        const double left_bound = lane_left - offset;
        const double right_bound = -lane_right - offset;

        double l_lower = std::max(right_bound, adc_l - kLateralMargin);
        double l_upper = std::min(left_bound, adc_l + kLateralMargin);

        if (adc_l > l_upper) {
            l_upper = std::min(left_bound, adc_l + kLateralFallbackMargin);
        }
        if (adc_l < l_lower) {
            l_lower = std::max(right_bound, adc_l - kLateralFallbackMargin);
        }

        if (l_lower >= l_upper) {
            l_lower = right_bound;
            l_upper = left_bound;
        }
        boundary->emplace_back(s, l_lower, l_upper);
    }

    if (boundary->empty()) {
        AERROR << "[REVERSE] Failed to generate reverse boundary";
        return false;
    }

    AINFO << "[REVERSE] Boundary: s=[" << boundary->back().s << "," << boundary->front().s
          << "], points=" << boundary->size();
    return true;
}

}  // namespace planning
}  // namespace apollo
