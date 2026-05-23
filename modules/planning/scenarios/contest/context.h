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

#include "modules/planning/planning_interface_base/scenario_base/scenario.h"
#include "modules/planning/scenarios/contest/proto/contest.pb.h"

namespace apollo {
namespace planning {

enum class ContestScenarioKind {
    LANE_CHANGE = 0,
    S_CURVE = 1,
    U_TURN = 2,
    CONSTRUCTION_ZONE = 3,
    STATION_SHUTTLE = 4,
    ROUNDABOUT = 5,
};

enum class RoundaboutPhase {
    NONE = 0,
    ENTERING_OUTER = 1,   // 正在进入环岛外侧车道（忽略内侧来车）
    ON_OUTER = 2,         // 已在外侧车道行驶，准备变道进入内侧
    MERGING_INNER = 3,    // 正在从外侧变道到内侧
    DONE = 4,             // 已成功进入内侧，等待退出
};

struct ContestScenarioContext : public ScenarioContext {
    ScenarioContestConfig scenario_config;
    ContestScenarioKind kind = ContestScenarioKind::LANE_CHANGE;
    // 站点接驳状态
    bool shuttle_arrived_at_station = false;
    double shuttle_dwell_start_time = 0.0;
    bool shuttle_departed = false;
    bool u_turn_active = false;
    bool u_turn_completed = false;
    double u_turn_entry_heading = 0.0;
    int u_turn_exit_hold_frames = 0;  // 退出保持计数器
    // 环岛状态
    RoundaboutPhase roundabout_phase = RoundaboutPhase::NONE;
    bool roundabout_committed = false;   // 是否已提交进入（不再因为后车而刹车）
    double roundabout_entry_s = 0.0;     // 入口 s 坐标
    double roundabout_entry_x = 0.0;     // 入口 X 坐标
    double roundabout_entry_y = 0.0;     // 入口 Y 坐标
    int roundabout_commit_hold_frames = 0;  // 提交保持计数器
    bool roundabout_completed = false;   // 是否已完成本次环岛（防重入，需远离出口后重置）
    double roundabout_exit_x = 0.0;      // 上次退出 X 坐标
    double roundabout_exit_y = 0.0;      // 上次退出 Y 坐标
};

}  // namespace planning
}  // namespace apollo
