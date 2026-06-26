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

#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/planning/scenarios/valet_parking/proto/bus_bay_transfer.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/math/box2d.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

class Frame;

struct BusBayTransferContext : public ScenarioContext {
    ScenarioBusBayTransferConfig scenario_config;
    std::string target_parking_spot_id;
    bool pre_stop_rightaway_flag = false;
    hdmap::MapPathPoint pre_stop_rightaway_point;
    bool shuttle_mission_completed = false;
    std::vector<common::math::Box2d> latched_static_obstacle_boxes;

    void LatchStaticObstacles(const Frame& frame, const std::string& source);
    void InjectLatchedStaticObstacles(Frame* frame) const;
};

class BusBayTransferScenario : public Scenario {
public:
    bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) override;

    /**
     * @brief Get the scenario context.
     */
    BusBayTransferContext* GetContext() override {
        return &context_;
    }

    bool IsTransferable(const Scenario* const other_scenario, const Frame& frame) override;

    bool Enter(Frame* frame) override;

private:
    static bool SearchTargetParkingSpotOnPath(
            const hdmap::Path& nearby_path,
            const std::string& target_parking_id,
            hdmap::PathOverlap* parking_space_overlap);
    static bool CheckDistanceToParkingSpot(
            const Frame& frame,
            const common::VehicleState& vehicle_state,
            const hdmap::Path& nearby_path,
            const double parking_start_range,
            const hdmap::PathOverlap& parking_space_overlap);
    bool SearchForNearbyCandidate(
            const Frame& frame,
            const hdmap::Path& nearby_path,
            double parking_start_range,
            hdmap::PathOverlap* parking_space_overlap);

private:
    bool init_ = false;
    BusBayTransferContext context_;
    const hdmap::HDMap* hdmap_ = nullptr;
    std::unordered_set<std::string> forbiden;
    std::unordered_set<std::string> occupied_parking_spots_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::BusBayTransferScenario, Scenario)

}  // namespace planning
}  // namespace apollo
