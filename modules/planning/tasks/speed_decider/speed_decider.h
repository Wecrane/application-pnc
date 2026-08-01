/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/tasks/speed_decider/proto/speed_decider.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

class SpeedDecider : public Task {
public:
    bool Init(
            const std::string& config_dir,
            const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

    common::Status Execute(Frame* frame, ReferenceLineInfo* reference_line_info) override;

private:
    enum STLocation {
        ABOVE = 1,
        BELOW = 2,
        CROSS = 3,
    };

    STLocation GetSTLocation(
            const PathDecision* const path_decision,
            const SpeedData& speed_profile,
            const STBoundary& st_boundary) const;

    bool CheckKeepClearCrossable(
            const PathDecision* const path_decision,
            const SpeedData& speed_profile,
            const STBoundary& keep_clear_st_boundary) const;

    bool CheckKeepClearBlocked(const PathDecision* const path_decision, const Obstacle& keep_clear_obstacle) const;

    /**
     * @brief check if the ADC should follow an obstacle by examing the
     *StBoundary of the obstacle.
     * @param boundary The boundary of the obstacle.
     * @return true if the ADC believe it should follow the obstacle, and
     *         false otherwise.
     **/
    bool CheckIsFollow(const Obstacle& obstacle, const STBoundary& boundary) const;

    bool CheckStopForPedestrian(const Obstacle& obstacle) const;

    bool CreateStopDecision(const Obstacle& obstacle, ObjectDecisionType* const stop_decision, double stop_distance)
            const;

    /**
     * @brief create follow decision based on the boundary
     **/
    bool CreateFollowDecision(const Obstacle& obstacle, ObjectDecisionType* const follow_decision) const;

    /**
     * @brief create yield decision based on the boundary
     **/
    bool CreateYieldDecision(const Obstacle& obstacle, ObjectDecisionType* const yield_decision) const;

    /**
     * @brief create overtake decision based on the boundary
     **/
    bool CreateOvertakeDecision(const Obstacle& obstacle, ObjectDecisionType* const overtake_decision) const;

    common::Status MakeObjectDecision(const SpeedData& speed_profile, PathDecision* const path_decision) const;

    void AppendIgnoreDecision(Obstacle* obstacle) const;

    /**
     * @brief "too close" is determined by whether ego vehicle will hit the front
     * obstacle if the obstacle drive at current speed and ego vehicle use some
     * reasonable deceleration
     **/
    bool IsFollowTooClose(const Obstacle& obstacle) const;

    double EstimateProperOvertakingGap(const double target_obs_speed, const double adc_speed) const;

    double EstimateProperFollowGap(const double& adc_speed) const;

    // mayaochang add8: handle a PEDESTRIAN that is ahead in the path. The ego
    // must keep a FIXED stop fence (recorded on first stop) until the pedestrian
    // laterally clears the lane - so the ego does NOT creep behind a moving
    // pedestrian at <2m (competition "follow limit" check, scenario 6). The
    // stop distance differs by context: 1.75m on a crosswalk (keeps the
    // crosswalk rule's 1.5~2.0m stop intact), 6m otherwise (scenario 6 requires
    // >=2m stopping distance).
    void HandlePedestrianStop(Obstacle* obstacle) const;
    bool IsPedestrianOnCrosswalk(const Obstacle& obstacle) const;
    // mayaochang add13: look up a fixed stop fence by base id, falling back to
    // a position match (nearest recorded pedestrian within kMatchMeters) so a
    // completely new obstacle id (perception re-assignment after a dropout)
    // still reuses the fence instead of letting the ego drive through.
    double GetPedFenceS(const std::string& base_id, double ped_s) const;
    bool HasPedFence(const std::string& base_id, double ped_s) const;

private:
    SLBoundary adc_sl_boundary_;
    common::TrajectoryPoint init_point_;
    const ReferenceLine* reference_line_ = nullptr;
    SpeedDeciderConfig config_;
    std::vector<std::pair<double, double>> follow_distance_function_;
    // mayaochang add8: obstacle_id -> fixed stop fence s (reference-line s).
    // Populated when the ego first stops for the pedestrian; kept unchanged
    // until the pedestrian laterally clears the lane. Keyed by BASE id (add12)
    // so static (7673) and dynamic (7673_0) obstacles share the fence.
    mutable std::unordered_map<std::string, double> ped_fixed_fence_s_;
    // mayaochang add13: base_id -> pedestrian SL s AT RECORD TIME, used for
    // position fallback when the obstacle id changes completely.
    mutable std::unordered_map<std::string, double> ped_fixed_ped_s_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::SpeedDecider, Task)

}  // namespace planning
}  // namespace apollo
