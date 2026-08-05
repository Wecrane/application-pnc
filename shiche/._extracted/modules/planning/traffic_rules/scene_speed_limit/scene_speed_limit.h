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
 * @file scene_speed_limit.h
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"
#include "modules/planning/traffic_rules/scene_speed_limit/proto/scene_speed_limit.pb.h"

namespace apollo {
namespace planning {

class SceneSpeedLimit : public TrafficRule {
public:
    bool Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) override;
    virtual ~SceneSpeedLimit() = default;
    common::Status ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info);
    void Reset() override {}

private:
    SceneSpeedLimitConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::SceneSpeedLimit, TrafficRule)

}  // namespace planning
}  // namespace apollo