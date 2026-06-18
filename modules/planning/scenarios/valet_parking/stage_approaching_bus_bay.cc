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

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_bus_bay.h"

namespace apollo {
namespace planning {
namespace {

// ---------- straight reference line building ----------

constexpr double kPreviewLength = 200.0;
constexpr double kPreviewStep = 0.5;
constexpr double kHeadingSampleDist = 0.5;
constexpr double kHalfPi = 1.5707963267948966;
constexpr double kPi = 3.1415926535897932;

double ComputeLaneHeading(const hdmap::LaneInfoConstPtr& lane, double lane_s, double fallback) {
    double fwd_s = std::min(lane_s + kHeadingSampleDist, lane->total_length());
    double bwd_s = std::max(lane_s - kHeadingSampleDist, 0.0);
    if (fwd_s <= bwd_s)
        return fallback;
    auto fwd_pt = lane->GetSmoothPoint(fwd_s);
    auto bwd_pt = lane->GetSmoothPoint(bwd_s);
    return std::atan2(fwd_pt.y() - bwd_pt.y(), fwd_pt.x() - bwd_pt.x());
}

std::vector<ReferencePoint> BuildStraightReferencePoints(
        const hdmap::LaneInfoConstPtr& lane,
        double proj_s,
        double anchor_x,
        double anchor_y,
        double anchor_heading,
        double vehicle_anchor_s) {
    double cos_h = std::cos(anchor_heading);
    double sin_h = std::sin(anchor_heading);
    std::vector<ReferencePoint> points;
    points.reserve(static_cast<size_t>(kPreviewLength / kPreviewStep + 1.0));
    for (double d = 0.0; d <= kPreviewLength; d += kPreviewStep) {
        double ref_s = vehicle_anchor_s + d;
        hdmap::MapPathPoint mp({anchor_x + ref_s * cos_h, anchor_y + ref_s * sin_h}, anchor_heading);
        hdmap::LaneWaypoint wp;
        wp.lane = lane;
        wp.s = proj_s + d;
        mp.add_lane_waypoint(wp);
        points.emplace_back(mp, 0.0, 0.0);
    }
    return points;
}

}  // namespace

// ========== StageApproachingBusBay ==========

bool StageApproachingBusBay::Init(
        const StagePipeline& config,
        const std::shared_ptr<DependencyInjector>& injector,
        const std::string& config_dir,
        void* context) {
    if (!Stage::Init(config, injector, config_dir, context))
        return false;
    scenario_config_.CopyFrom(GetContextAs<BusBayTransferContext>()->scenario_config);
    straight_ref_anchor_locked_ = false;
    return true;
}

StageResult StageApproachingBusBay::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: StageApproachingBusBay";
    CHECK_NOTNULL(frame);
    StageResult result;
    auto* sc = GetContextAs<BusBayTransferContext>();
    sc->LatchStaticObstacles(*frame, "approach");

    if (sc->target_parking_spot_id.empty()) {
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) = sc->target_parking_spot_id;
    frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(sc->pre_stop_rightaway_flag);
    *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point()) = sc->pre_stop_rightaway_point;

    // ---- rebuild straight reference line ----
    {
        const auto& vs = frame->vehicle_state();
        double vx = vs.x(), vy = vs.y();
        auto* refs = frame->mutable_reference_line_info();

        hdmap::LaneInfoConstPtr lane;
        double lane_s = 0.0, lane_l = 0.0;
        auto adc_pt = common::util::PointFactory::ToPointENU(vs);
        hdmap::HDMapUtil::BaseMap().GetNearestLaneWithDistance(adc_pt, 5.0, &lane, &lane_s, &lane_l);

        if (refs->empty()) {
            AINFO << "Bus-bay approach: no ref line, skip straight rebuild";
        } else if (lane == nullptr) {
            AINFO << "Bus-bay approach: no lane, skip straight rebuild";
        } else {
            double proj_s = 0.0, proj_l = 0.0;
            if (!lane->GetProjection({vx, vy}, &proj_s, &proj_l)) {
                AINFO << "Bus-bay approach: projection failed, ns=" << lane_s << ", nl=" << lane_l;
            } else {
                // lock anchor on first call
                if (!straight_ref_anchor_locked_) {
                    auto anchor_pt = lane->GetSmoothPoint(proj_s);
                    double lane_head = ComputeLaneHeading(lane, proj_s, vs.heading());
                    if (std::fabs(common::math::NormalizeAngle(lane_head - vs.heading())) > kHalfPi) {
                        lane_head = common::math::NormalizeAngle(lane_head + kPi);
                    }
                    straight_ref_anchor_x_ = anchor_pt.x();
                    straight_ref_anchor_y_ = anchor_pt.y();
                    straight_ref_anchor_heading_ = lane_head;
                    straight_ref_anchor_locked_ = true;
                    AINFO << "Bus-bay approach: anchor locked, ax=" << straight_ref_anchor_x_
                          << ", ay=" << straight_ref_anchor_y_ << ", ah=" << straight_ref_anchor_heading_
                          << ", vh=" << vs.heading() << ", proj_s=" << proj_s << ", proj_l=" << proj_l;
                }

                double ah = straight_ref_anchor_heading_;
                double cos_ah = std::cos(ah), sin_ah = std::sin(ah);
                double adc_anchor_s = (vx - straight_ref_anchor_x_) * cos_ah + (vy - straight_ref_anchor_y_) * sin_ah;
                double adc_anchor_l = -(vx - straight_ref_anchor_x_) * sin_ah + (vy - straight_ref_anchor_y_) * cos_ah;

                auto ref_pts = BuildStraightReferencePoints(
                        lane, proj_s, straight_ref_anchor_x_, straight_ref_anchor_y_, ah, adc_anchor_s);

                if (ref_pts.size() < 2) {
                    AINFO << "Bus-bay approach: too few ref points";
                } else {
                    ReferenceLine straight_rl(ref_pts);
                    auto old_it = refs->begin();
                    auto old_idx = old_it->index();
                    auto old_speed = old_it->GetBaseCruiseSpeed();
                    auto new_it = refs->emplace(
                            old_it, frame->vehicle_state(), frame->PlanningStartPoint(), straight_rl, old_it->Lanes());
                    new_it->set_index(old_idx);
                    if (new_it->Init(frame->obstacles(), old_speed)) {
                        refs->erase(old_it);
                        AINFO << "Bus-bay approach: straight ref built, pts=" << ref_pts.size() << ", heading=" << ah
                              << ", heading_delta=" << common::math::NormalizeAngle(vs.heading() - ah)
                              << ", as=" << adc_anchor_s << ", al=" << adc_anchor_l << ", proj_s=" << proj_s
                              << ", proj_l=" << proj_l << ", obs=" << frame->obstacles().size();
                    } else {
                        refs->erase(new_it);
                        AINFO << "Bus-bay approach: straight ref build FAILED, pts=" << ref_pts.size()
                              << ", heading=" << ah
                              << ", heading_delta=" << common::math::NormalizeAngle(vs.heading() - ah);
                    }
                }
            }
        }
    }

    // ---- ignore destination obstacle ----
    for (auto& rl : *frame->mutable_reference_line_info()) {
        auto* pd = rl.path_decision();
        if (pd == nullptr)
            continue;
        auto* dest = pd->Find(FLAGS_destination_obstacle_id);
        if (dest == nullptr)
            continue;
        ObjectDecisionType decision;
        decision.mutable_ignore();
        dest->EraseDecision();
        dest->AddLongitudinalDecision("ignore-dest-in-bus-bay", decision);
    }

    result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

    sc->pre_stop_rightaway_flag = frame->open_space_info().pre_stop_rightaway_flag();
    sc->pre_stop_rightaway_point = frame->open_space_info().pre_stop_rightaway_point();

    if (CheckADCStop(*frame)) {
        next_stage_ = "BUS_BAY_TRANSFER_DWELLING";
        return StageResult(StageStatusType::FINISHED);
    }
    if (result.HasError()) {
        AERROR << "Bus-bay approach: planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }
    return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StageApproachingBusBay::CheckADCStop(const Frame& frame) {
    const auto& rl = frame.reference_line_info().front();
    double adc_speed = injector_->vehicle_state()->linear_velocity();
    double max_stop_speed
            = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped();

    double adc_front_s = rl.AdcSlBoundary().end_s();
    double fence_s = frame.open_space_info().open_space_pre_stop_fence_s();
    double dist = fence_s - adc_front_s;
    constexpr double kRollMaxSpeed = 0.8;
    constexpr double kRollDist = 2.0;
    constexpr double kRollPastBuf = 1.0;
    constexpr double kLogNearDist = 5.0;

    if (fence_s <= 1.0e-6) {
        ADEBUG << "Bus-bay approach: no pre-stop fence, speed=" << adc_speed;
        return false;
    }

    bool stopped = adc_speed <= max_stop_speed && dist <= scenario_config_.max_valid_stop_distance();
    bool rolling = adc_speed <= kRollMaxSpeed && dist <= kRollDist && dist >= -kRollPastBuf;

    if (dist < kLogNearDist || rolling || stopped) {
        AINFO << "Bus-bay approach: stop check, speed=" << adc_speed << ", max_stop=" << max_stop_speed
              << ", fence_s=" << fence_s << ", adc_front=" << adc_front_s << ", dist=" << dist
              << ", valid_dist=" << scenario_config_.max_valid_stop_distance() << ", roll_max_spd=" << kRollMaxSpeed
              << ", roll_dist=" << kRollDist << ", buffer=" << kRollPastBuf << ", stopped=" << stopped
              << ", rolling=" << rolling;
    }

    if (!stopped && !rolling)
        return false;
    AINFO << "Bus-bay approach: handoff to dwelling, stopped=" << stopped << ", rolling=" << rolling
          << ", speed=" << adc_speed << ", dist=" << dist;
    return true;
}

}  // namespace planning
}  // namespace apollo
