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

#include "modules/planning/planning_base/common/path_boundary.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
namespace planning {

struct ReverseRecoveryState {
    bool active = false;
    int frame_count = 0;
    double start_s = 0.0;
    double start_x = 0.0;
    double start_y = 0.0;
    double last_adc_s = 0.0;
    double last_adc_x = 0.0;
    double last_adc_y = 0.0;

    void Reset() {
        active = false;
        frame_count = 0;
        start_s = 0.0;
        start_x = 0.0;
        start_y = 0.0;
        last_adc_s = 0.0;
        last_adc_x = 0.0;
        last_adc_y = 0.0;
    }
};

bool GenerateReverseRecoveryPathBoundary(
        const ReferenceLine& reference_line,
        double adc_s,
        double adc_l,
        double reverse_distance,
        PathBoundary* boundary);

}  // namespace planning
}  // namespace apollo
