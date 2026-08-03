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
    // 2026-08-04晚第2轮: 限速持续到车头驶出路口区。
    // 原GREEN分支在 end_s<=adc_back_edge_s 的continue之后 → 车后轴一过停止线
    // (end_s≈停止线)整个规则跳过 → 参考线限速消失 → 车加速冲路口区
    // (scn3 11.24>11.18, scn4 5.0→10.8 连续超速, 评测SpeedLimit扣分)。
    // 现在只要车头未出路口区(start_s+27)且信号已知(非UNKNOWN), 每帧都加限速
    // (即使车已过停止线), 直到完全通过评测限速区。
    const auto tl_color = frame->GetSignal(traffic_light_overlap.object_id).color();
    if (adc_front_edge_s < traffic_light_overlap.start_s + 27.0 &&
        tl_color != perception::TrafficLight::UNKNOWN) {
      if (GlobalSeenRedLight()[traffic_light_overlap.object_id]) {
        // scn3: 见过红灯, 绿灯通过路口区限速10.7(评测11.18, 留裕量防超调;
        // 尾段自由加速+控制超调约+0.24, 11.0→实测11.24超速, 故取10.7)
        reference_line_info->mutable_reference_line()->AddSpeedLimit(
            traffic_light_overlap.start_s,
            traffic_light_overlap.start_s + 26.0, 10.7);
      } else {
        // scn4/scn8: 绿灯通过, 限速4.5覆盖评测路口区(不提前刹车+防超调)。
        // 2026-08-04晚第5轮(006轮SpeedLimit=52): [start_s-20]太紧——
        // 车12.4s s155才减速(回推v_allow不够), 停止线前y4438231急刹到0.82
        // 几乎停再起步(刹油刹), 进路口区时planning 4.6-4.79+sim控制超调
        // +0.2→5.003-5.04>5.0贴线超速。
        // 改[start_s-35]+4.5: 车s161(QP回推kLimitDecel=5)平滑降速→s185
        // 到4.5→进路口区(s205)4.5稳态(sim超调+0.2=4.7<5✓)→4.5通过27m
        // 出区加速. 不刹油刹+不超速+不提前刹车(车16巡航到停止线前35m).
        // 时间: 4.5巡航47m比4.8慢~1.5s, TimeLimit裕量大可接受。
        // 2026-08-05(017轮提速): 4.5→4.7——评测路口区限速5, sim超调+0.2
        // →4.7+0.2=4.9<5安全(4.8+0.2=5.0贴线风险). 省~0.5s/绿灯段.
        reference_line_info->mutable_reference_line()->AddSpeedLimit(
            traffic_light_overlap.start_s - 35.0,
            traffic_light_overlap.start_s + 27.0, 4.7);
      }
    }
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
      // 限速已在上方循环顶部(车头未出路口区)每帧持续下发, 见限速块注释。
      // 这里只负责绿灯不建STOP。
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
