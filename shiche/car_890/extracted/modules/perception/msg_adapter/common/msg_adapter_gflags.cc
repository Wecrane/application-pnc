/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/msg_adapter/common/msg_adapter_gflags.h"

DEFINE_string(cameraframe_to_obstacles_in, "/fake_topic",
              "cameraframe subscribe topic");

DEFINE_string(cameraframe_to_obstacles_out, "/apollo/perception/obstacles",
              "obstacles publish topic");

DEFINE_string(sensorframe_message_to_obstacles_in, "/fake_topic",
              "cameraframe subscribe topic");

DEFINE_string(sensorframe_message_to_obstacles_out,
              "/apollo/perception/obstacles", "obstacles publish topic");

DEFINE_string(lidarframe_to_obstacles_in, "/fake_topic",
              "lidarframe subscribe topic");

DEFINE_string(lidarframe_to_obstacles_out, "/apollo/perception/obstacles",
              "obstacles publish topic");

// Scenario injection: merge externally injected obstacles (published by
// sim_obstacle / scenario_injector) into the perception obstacles output so
// the whole downstream stack (prediction/planning) sees them as real.
DEFINE_bool(enable_scenario_injection, false,
            "merge scenario injected obstacles into perception output");

DEFINE_string(scenario_obstacle_topic, "/apollo/perception/scenario_obstacles",
              "topic carrying scenario injected PerceptionObstacles");

DEFINE_double(scenario_injection_timeout_sec, 0.5,
              "drop scenario obstacles if no scenario frame arrives within "
              "this many seconds");

DEFINE_int32(scenario_injection_id_offset, 100000,
             "offset added to injected obstacle ids to avoid collisions with "
             "real perception ids; set <0 to keep ids unchanged");
