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
#include "modules/perception/msg_adapter/msg_converter.h"

#include "cyber/time/clock.h"

namespace apollo {
namespace perception {

bool MsgConverter::EnableScenarioInjection(const std::string &scenario_topic,
                                           double timeout_sec, int id_offset) {
  scenario_timeout_sec_ = timeout_sec;
  scenario_id_offset_ = id_offset;
  scenario_reader_ = node_->CreateReader<PerceptionObstacles>(
      scenario_topic,
      [this](const std::shared_ptr<PerceptionObstacles> &scenario) {
        auto cached = std::make_shared<PerceptionObstacles>(*scenario);
        std::lock_guard<std::mutex> lock(scenario_mutex_);
        latest_scenario_ = cached;
        latest_scenario_time_ = cyber::Clock::NowInSeconds();
      });
  scenario_injection_enabled_ = true;
  AINFO << "Scenario injection enabled: merging " << scenario_topic
        << " into perception obstacles (timeout=" << timeout_sec
        << "s, id_offset=" << id_offset << ")";
  return true;
}

void MsgConverter::MergeScenario(PerceptionObstacles *obstacles) {
  if (!scenario_injection_enabled_) {
    return;
  }

  std::shared_ptr<PerceptionObstacles> scenario;
  {
    std::lock_guard<std::mutex> lock(scenario_mutex_);
    if (latest_scenario_ == nullptr) {
      return;
    }
    if (cyber::Clock::NowInSeconds() - latest_scenario_time_ >
        scenario_timeout_sec_) {
      return;
    }
    scenario = latest_scenario_;
  }

  const double now = cyber::Clock::NowInSeconds();
  for (const auto &injected : scenario->perception_obstacle()) {
    PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
    obstacle->CopyFrom(injected);
    if (scenario_id_offset_ >= 0) {
      obstacle->set_id(obstacle->id() + scenario_id_offset_);
    }
    if (!obstacle->has_timestamp() || obstacle->timestamp() == 0) {
      obstacle->set_timestamp(now);
    }
  }
}

}  // namespace perception
}  // namespace apollo
