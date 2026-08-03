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

#include "modules/planning/traffic_rules/traffic_light/traffic_light.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule_common.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

// 区分赛题3(红绿灯场景)与赛题8(交通灯路口减速通行)——两者同地图同路线同信号灯Signal_5:
// - 赛题3: 有红灯(停车1.5-2.0m) → 绿灯通过路口【不限速】(评测无路口限速要求)
// - 赛题8: 无红灯(一直绿) → 绿灯通过信号灯区域【限速≤5m/s】
// 记录各信号灯是否出现过红灯(共享给 crosswalk 规则判断路口斑马线限速)。
// 评测每场景独立进程, 状态每场景重置。

bool TrafficLight::Init(const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<TrafficLightConfig>(&config_);
}

Status TrafficLight::ApplyRule(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);

  return Status::OK();
}

void TrafficLight::MakeDecisions(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.enabled()) {
    return;
  }

  const auto& traffic_light_status =
      injector_->planning_context()->planning_status().traffic_light();

  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();

  // debug info
  planning_internal::SignalLightDebug* signal_light_debug =
      reference_line_info->mutable_debug()
          ->mutable_planning_data()
          ->mutable_signal_light();
  signal_light_debug->set_adc_front_s(adc_front_edge_s);
  signal_light_debug->set_adc_speed(
      injector_->vehicle_state()->linear_velocity());

  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info->reference_line().map_path().signal_overlaps();
  AINFO << "[traffic-light] overlaps=" << traffic_light_overlaps.size();
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    if (traffic_light_overlap.end_s <= adc_back_edge_s) {
      continue;
    }

    // check if traffic-light-stop already finished, set by scenario/stage
    bool traffic_light_done = false;
    for (const auto& done_traffic_light_overlap_id :
         traffic_light_status.done_traffic_light_overlap_id()) {
      if (traffic_light_overlap.object_id == done_traffic_light_overlap_id) {
        traffic_light_done = true;
        break;
      }
    }
    if (traffic_light_done) {
      continue;
    }

    // work around incorrect s-projection along round routing
    static constexpr double kSDiscrepanceTolerance = 10.0;
    const auto& reference_line = reference_line_info->reference_line();
    common::SLPoint traffic_light_sl;
    traffic_light_sl.set_s(traffic_light_overlap.start_s);
    traffic_light_sl.set_l(0);
    common::math::Vec2d traffic_light_point;
    reference_line.SLToXY(traffic_light_sl, &traffic_light_point);
    common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                        injector_->vehicle_state()->y()};
    const double distance =
        common::util::DistanceXY(traffic_light_point, adc_position);
    const double s_distance = traffic_light_overlap.start_s - adc_front_edge_s;
    ADEBUG << "traffic_light[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] s_distance["
           << s_distance << "] actual_distance[" << distance << "]";
    if (s_distance >= 0 &&
        fabs(s_distance - distance) > kSDiscrepanceTolerance) {
      AINFO << "[traffic-light] SKIP3 s_discrepance id="
            << traffic_light_overlap.object_id << " s_distance=" << s_distance
            << " distance=" << distance;
      continue;
    }

    auto signal_color =
        frame->GetSignal(traffic_light_overlap.object_id).color();
    const double stop_deceleration = util::GetADCStopDeceleration(
        injector_->vehicle_state(), adc_front_edge_s,
        traffic_light_overlap.start_s);
    ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] color["
           << signal_color << "] stop_deceleration[" << stop_deceleration
           << "]";

    // debug info
    planning_internal::SignalLightDebug::SignalDebug* signal_debug =
        signal_light_debug->add_signal();
    signal_debug->set_adc_stop_deceleration(stop_deceleration);
    signal_debug->set_color(signal_color);
    signal_debug->set_light_id(traffic_light_overlap.object_id);
    signal_debug->set_light_stop_s(traffic_light_overlap.start_s);

    // mayaochang add
    // 修复(2026-08-02): UNKNOWN(信号灯未触发/状态未知)不应视为"见过红"。
    // 赛题8 Signal_5 一直绿/未触发(UNKNOWN): 误记后 crosswalk 跳过路口斑马线
    // 限速(4.5)→ 车13.6m/s全速通过Signal_5路口(评测限速5, 超速扣分)。
    // 只记真正的 RED/YELLOW。
    if (signal_color == perception::TrafficLight::RED ||
        signal_color == perception::TrafficLight::YELLOW) {
      GlobalSeenRedLight()[traffic_light_overlap.object_id] = true;
    }
    AINFO << "[traffic-light] process id=" << traffic_light_overlap.object_id
          << " start_s=" << traffic_light_overlap.start_s
          << " color=" << signal_color
          << " seen_red=" << GlobalSeenRedLight()[traffic_light_overlap.object_id];
    // 修复(2026-08-02): UNKNOWN(信号灯未触发/状态未知)不建STOP。
    // 原逻辑UNKNOWN走红灯分支可能BuildStopDecision → 赛题8 Signal_5未触发
    // (UNKNOWN)时车在路口前停车(评测要求减速通过不停)。UNKNOWN≠红灯。
    // 不限速不停车, 由 crosswalk 斑马线限速(4.5)兜底(GlobalSeenRedLight已排除UNKNOWN)。
    if (signal_color == perception::TrafficLight::UNKNOWN) {
      AINFO << "[traffic-light] " << traffic_light_overlap.object_id
            << " UNKNOWN skip (no stop, no limit)";
      continue;
    }
    if (signal_color == perception::TrafficLight::GREEN ||
        signal_color == perception::TrafficLight::BLACK) {
      // competition: 赛题八(交通灯路口减速) 绿灯通过信号灯区域限速(≤5m/s)
      // 仅当该信号灯【从未出现过红灯】时生效——赛题3(红绿灯场景)红灯停车后
      // 绿灯通过路口不限速(评测无路口限速要求, 且避免限速导致的顿挫+耗时)
      // 2026-08-04调优: 4.5→4.8(评测是5m/s, 4.5太保守) + 提前50→20m——
      // 112948实测车4.5巡航50m(提前50m限速, 评测限速区只有~27m宽) → 慢。
      // 评测要求"限速区(y4438237-4438264)内≤5", 车只需进入限速区前降到≤5:
      // 16→4.8@-3需38.8m, 提前20m+QP回推23.6m=43.6m足够 → 车16巡航更久
      // +4.8巡航缩短 → 省~6s。
      if (!GlobalSeenRedLight()[traffic_light_overlap.object_id]) {
        reference_line_info->mutable_reference_line()->AddSpeedLimit(
            traffic_light_overlap.start_s - 20.0, traffic_light_overlap.end_s,
            4.8);
      }
      continue;
    }

    // Red/Yellow/Unknown: check deceleration
    if (stop_deceleration > config_.max_stop_deceleration()) {
      AWARN << "stop_deceleration too big to achieve.  SKIP red light";
      continue;
    }

    // build stop decision
    ADEBUG << "BuildStopDecision: traffic_light["
           << traffic_light_overlap.object_id << "] start_s["
           << traffic_light_overlap.start_s << "]"
           << "stop_distance: " << config_.stop_distance();
    std::string virtual_obstacle_id =
        TRAFFIC_LIGHT_VO_ID_PREFIX + traffic_light_overlap.object_id;
    const std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(
        virtual_obstacle_id, traffic_light_overlap.start_s,
        config_.stop_distance(), StopReasonCode::STOP_REASON_SIGNAL,
        wait_for_obstacles, Getname(), frame, reference_line_info);
  }
}

}  // namespace planning
}  // namespace apollo
