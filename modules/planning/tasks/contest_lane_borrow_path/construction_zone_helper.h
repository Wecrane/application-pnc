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

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/planning_base/common/sl_polygon.h"

namespace apollo {
namespace planning {

class Frame;
class ReferenceLineInfo;

using ConstructionConeXYMap = std::unordered_map<std::string, std::pair<double, double>>;

struct ConstructionZoneState {
    bool active = false;
    std::vector<std::pair<double, double>> cone_history;
    std::vector<std::pair<double, double>> left_wall;
    std::vector<std::pair<double, double>> right_wall;
    std::unordered_map<uint64_t, bool> cone_wall_memory;
    std::vector<std::pair<double, double>> classified_left_xy;
    std::vector<std::pair<double, double>> classified_right_xy;
    int no_cone_counter = 0;
    int low_cone_counter = 0;
    double farthest_cone_s = -1.0;
    double farthest_cone_x = -1.0;
    double farthest_cone_y = 0.0;
    double max_left_bound = 0.0;
    double max_right_bound = 0.0;

    void Reset() {
        active = false;
        cone_history.clear();
        left_wall.clear();
        right_wall.clear();
        cone_wall_memory.clear();
        classified_left_xy.clear();
        classified_right_xy.clear();
        no_cone_counter = 0;
        low_cone_counter = 0;
        farthest_cone_s = -1.0;
        farthest_cone_x = -1.0;
        farthest_cone_y = 0.0;
        max_left_bound = 0.0;
        max_right_bound = 0.0;
    }
};

void ComputeConstructionZoneBoundary(
        std::vector<SLPolygon>* cones,
        const ConstructionConeXYMap& cone_xy,
        double adc_back_s,
        ConstructionZoneState* construction_zone);

void UpdateConstructionZoneTrackingState(
        Frame* frame,
        ReferenceLineInfo* reference_line_info,
        int low_cone_exit_threshold,
        ConstructionZoneState* construction_zone);

}  // namespace planning
}  // namespace apollo
