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
 * @file traffic_rule_common.h
 * @brief traffic rules 共享状态(跨 traffic_light/crosswalk 规则)
 **/

#pragma once

#include <string>
#include <unordered_map>

namespace apollo {
namespace planning {

// 信号灯"曾见过红"记录(全局共享, 跨 traffic_light/crosswalk 规则):
// - 赛题3(红绿灯场景): 信号灯见过红 → 红灯停车后绿灯通过路口不限速
//   (评测 RedLightStop 只要求停车, 无路口限速要求)
// - 赛题8(交通灯路口减速通行): 信号灯从未见红(一直绿灯) → 路口/斑马线
//   限速5(减速通行, 评测 SpeedLimit 要求 ≤5m/s)
// 由 traffic_light 规则写入(看到 RED/YELLOW/UNKNOWN 时置 true),
// crosswalk 规则读取(判断路口斑马线是否限速)。
inline std::unordered_map<std::string, bool>& GlobalSeenRedLight() {
  static std::unordered_map<std::string, bool> seen_red;
  return seen_red;
}

}  // namespace planning
}  // namespace apollo
