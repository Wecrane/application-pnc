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

/******************************************************************************
 * @file scene_speed_limit.cc
 *****************************************************************************/

#include <memory>
#include <vector>

#include "modules/planning/traffic_rules/scene_speed_limit/scene_speed_limit.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool SceneSpeedLimit::Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) {
    if (!TrafficRule::Init(name, injector)) {
        return false;
    }
    return TrafficRule::LoadConfig<SceneSpeedLimitConfig>(&config_);
}

Status SceneSpeedLimit::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
    ReferenceLine* reference_line = reference_line_info->mutable_reference_line();
    const auto& map_path = reference_line_info->reference_line().map_path();

    if (config_.crosswalk_enabled()) {
        const std::vector<PathOverlap>& crosswalk_overlaps = map_path.crosswalk_overlaps();
        for (const auto& crosswalk_overlap : crosswalk_overlaps) {
            reference_line->AddSpeedLimit(
                    crosswalk_overlap.start_s - config_.crosswalk_forward_buffer(),
                    crosswalk_overlap.end_s + config_.crosswalk_backward_buffer(),
                    config_.crosswalk_limit_speed());
        }
    }

    if (config_.traffic_light_enabled()) {
        const std::vector<PathOverlap>& signal_overlaps = map_path.signal_overlaps();
        for (const auto& signal_overlap : signal_overlaps) {
            reference_line->AddSpeedLimit(
                    signal_overlap.start_s - config_.traffic_light_forward_buffer(),
                    signal_overlap.end_s + config_.traffic_light_backward_buffer(),
                    config_.traffic_light_limit_speed());
        }
    }

    if (config_.speed_bump_enabled()) {
        const std::vector<PathOverlap>& speed_bump_overlaps = map_path.speed_bump_overlaps();
        for (const auto& speed_bump_overlap : speed_bump_overlaps) {
            reference_line->AddSpeedLimit(
                    speed_bump_overlap.start_s - config_.speed_bump_forward_buffer(),
                    speed_bump_overlap.end_s + config_.speed_bump_backward_buffer(),
                    config_.speed_bump_limit_speed());
        }
    }

    return Status::OK();
}

}  // namespace planning
}  // namespace apollo