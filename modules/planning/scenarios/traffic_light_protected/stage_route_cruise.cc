/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/traffic_light_protected/stage_route_cruise.h"

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/traffic_light_protected/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::hdmap::PathOverlap;
using apollo::perception::TrafficLight;

StageResult TrafficLightProtectedStageRouteCruise::Process(const TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: RouteCruise";
    CHECK_NOTNULL(frame);
    CHECK_NOTNULL(context_);

    auto context = GetContextAs<TrafficLightProtectedContext>();
    const ScenarioTrafficLightProtectedConfig& scenario_config = context->scenario_config;

    // Set cruise speed to route_cruise_speed (default 60 km/h).
    // Traffic light zones have no speed limit.
    auto& reference_line_info = frame->mutable_reference_line_info()->front();
    reference_line_info.SetCruiseSpeed(scenario_config.route_cruise_speed());
    AINFO << "RouteCruise: setting cruise speed to " << scenario_config.route_cruise_speed() << " m/s ("
          << scenario_config.route_cruise_speed() * 3.6 << " km/h)";

    // Execute normal driving tasks (path planning, speed optimization, etc.)
    StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
    if (result.HasError()) {
        AERROR << "TrafficLightProtectedStageRouteCruise planning error";
    }

    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const auto& signal_overlaps = reference_line_info.reference_line().map_path().signal_overlaps();

    // Find the nearest upcoming traffic light group
    PathOverlap* next_traffic_light = nullptr;
    for (const auto& overlap : signal_overlaps) {
        if (overlap.start_s > adc_front_edge_s) {
            if (next_traffic_light == nullptr || overlap.start_s < next_traffic_light->start_s) {
                next_traffic_light = const_cast<PathOverlap*>(&overlap);
            }
        }
    }

    if (next_traffic_light != nullptr) {
        const double distance_to_traffic_light = next_traffic_light->start_s - adc_front_edge_s;

        // If we're within approach distance, check if we need to stop
        if (distance_to_traffic_light <= scenario_config.start_traffic_light_scenario_distance()) {
            // Group nearby traffic lights (within 2m of the nearest one)
            std::vector<std::string> next_traffic_light_ids;
            static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
            bool has_non_green = false;
            for (const auto& overlap : signal_overlaps) {
                if (overlap.start_s <= adc_front_edge_s) {
                    continue;
                }
                const double dist = overlap.start_s - next_traffic_light->start_s;
                if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
                    next_traffic_light_ids.push_back(overlap.object_id);
                    const auto& signal_color = frame->GetSignal(overlap.object_id).color();
                    if (signal_color != TrafficLight::GREEN && signal_color != TrafficLight::BLACK) {
                        has_non_green = true;
                    }
                }
            }

            // If any light in the group is non-green, transition to Approach stage
            if (has_non_green) {
                ADEBUG << "Next traffic light group has non-green signals, "
                       << "transitioning to Approach. distance=" << distance_to_traffic_light;
                context->current_traffic_light_overlap_ids = next_traffic_light_ids;

                // Update planning_status so IntersectionCruise can find the overlap
                auto* traffic_light_status
                        = injector_->planning_context()->mutable_planning_status()->mutable_traffic_light();
                traffic_light_status->clear_current_traffic_light_overlap_id();
                for (const auto& id : next_traffic_light_ids) {
                    traffic_light_status->add_current_traffic_light_overlap_id(id);
                }
                AINFO << "RouteCruise: transitioning to Approach for " << next_traffic_light_ids.size()
                      << " traffic lights, distance=" << distance_to_traffic_light;
                return FinishStage();
            }
            // All lights are green — keep cruising through the intersection
        }
    }

    return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult TrafficLightProtectedStageRouteCruise::FinishStage() {
    next_stage_ = "TRAFFIC_LIGHT_PROTECTED_APPROACH";
    return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
