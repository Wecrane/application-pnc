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

#include <memory>
#include <string>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"
#include "modules/planning/scenarios/contest/context.h"

namespace apollo {
namespace planning {

class ContestScenarioBase : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;

  ContestScenarioContext* GetContext() override { return &context_; }

 protected:
  explicit ContestScenarioBase(ContestScenarioKind kind) : kind_(kind) {}

  bool IsReferenceLineReady(const Frame& frame) const;

 private:
  bool init_ = false;
  ContestScenarioKind kind_;
  ContestScenarioContext context_;
};

class ContestLaneChangeScenario : public ContestScenarioBase {
 public:
  ContestLaneChangeScenario()
      : ContestScenarioBase(ContestScenarioKind::LANE_CHANGE) {}

  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;
};

class ContestSCurveScenario : public ContestScenarioBase {
 public:
  ContestSCurveScenario() : ContestScenarioBase(ContestScenarioKind::S_CURVE) {}

  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;
};

class ContestUTurnScenario : public ContestScenarioBase {
 public:
  ContestUTurnScenario() : ContestScenarioBase(ContestScenarioKind::U_TURN) {}

  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;
};

class ContestConstructionZoneScenario : public ContestScenarioBase {
 public:
  ContestConstructionZoneScenario()
      : ContestScenarioBase(ContestScenarioKind::CONSTRUCTION_ZONE) {}

  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ContestLaneChangeScenario,
                                     apollo::planning::Scenario)
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ContestSCurveScenario,
                                     apollo::planning::Scenario)
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ContestUTurnScenario,
                                     apollo::planning::Scenario)
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::ContestConstructionZoneScenario,
    apollo::planning::Scenario)

}  // namespace planning
}  // namespace apollo
