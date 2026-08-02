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

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "modules/planning/traffic_rules/crosswalk/proto/crosswalk.pb.h"

#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"

namespace apollo {
namespace planning {

class Crosswalk : public TrafficRule {
 public:
  bool Init(const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  virtual ~Crosswalk() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);
  void Reset() override {
    crosswalk_overlaps_.clear();
    extra_crosswalk_overlaps_.clear();
  }

 private:
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool FindCrosswalks(ReferenceLineInfo* const reference_line_info);
  bool CheckStopForObstacle(ReferenceLineInfo* const reference_line_info,
                            const hdmap::CrosswalkInfoConstPtr crosswalk_ptr,
                            const Obstacle& obstacle,
                            const double stop_deceleration);

 private:
  CrosswalkConfig config_;
  static constexpr char const* CROSSWALK_VO_ID_PREFIX = "CW_";
  std::vector<const hdmap::PathOverlap*> crosswalk_overlaps_;
  // 全局规划补充: HDMap 直接查询参考线全程的人行道(map_path overlap 晚出现),
  // 这里持久存储补充的 PathOverlap, crosswalk_overlaps_ 存其指针。
  // 用 deque: push_back 不使已有元素指针失效(vector 会 realloc 导致指针失效)。
  std::deque<hdmap::PathOverlap> extra_crosswalk_overlaps_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::Crosswalk, TrafficRule)

}  // namespace planning
}  // namespace apollo
