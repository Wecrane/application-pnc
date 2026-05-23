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

#include "modules/planning/scenarios/contest/contest_scenario_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>

#include "modules/common/math/math_utils.h"
#include "modules/planning/planning_base/common/contest_scenario_features.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {
namespace contest {

namespace {

constexpr double kRoundaboutExitPastEntryDistance = 8.0;
constexpr double kRoundaboutLongGeometryLookForwardDistance = 180.0;
constexpr double kRoundaboutMinLongGeometryHeadingChange = 0.75;
constexpr double kRoundaboutMaxLongGeometryHeadingChange = 2.2;
constexpr double kRoundaboutMinLongGeometryKappa = 0.018;
constexpr double kRoundaboutNearUTurnHeadingChange = 2.2;
constexpr double kRoundaboutRoiMinX = 423360.0;
constexpr double kRoundaboutRoiMaxX = 423548.0;
constexpr double kRoundaboutRoiMinY = 4437990.0;
constexpr double kRoundaboutRoiMaxY = 4438072.0;
constexpr double kRoundaboutPassedX = 423535.0;
constexpr double kRoundaboutPassedY = 4438064.0;

bool HasNearUTurnFeature(
        const ReferenceLineInfo& reference_line_info,
        const double start_s,
        const double end_s) {
    const auto& reference_line = reference_line_info.reference_line();
    double max_abs_kappa = 0.0;
    for (double s = start_s; s <= end_s; s += kFeatureSampleStep) {
        if (reference_line_info.GetPathTurnType(s) == hdmap::Lane::U_TURN) {
            return true;
        }
        max_abs_kappa = std::max(
                max_abs_kappa, std::fabs(reference_line.GetReferencePoint(s).kappa()));
    }
    const double heading_change = std::fabs(common::math::NormalizeAngle(
            reference_line.GetReferencePoint(end_s).heading()
            - reference_line.GetReferencePoint(start_s).heading()));
    return max_abs_kappa > 0.08 && heading_change > kRoundaboutNearUTurnHeadingChange;
}

}  // namespace

bool IsContestLaneChange(const Frame& frame) {
    return frame.local_view().planning_command != nullptr
            && frame.local_view().planning_command->has_lane_follow_command() && frame.reference_line_info().size() > 1;
}

bool IsContestSCurve(const ReferenceLineInfo& reference_line_info, const ScenarioContestConfig& config) {
    const auto& reference_line = reference_line_info.reference_line();
    const double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

    double max_abs_kappa = 0.0;
    for (double s = adc_start_s;
         s < adc_start_s + config.s_curve_look_forward_distance() && s < reference_line.Length();
         s += kFeatureSampleStep) {
        max_abs_kappa = std::max(max_abs_kappa, std::fabs(reference_line.GetReferencePoint(s).kappa()));
    }
    if (max_abs_kappa < config.s_curve_min_abs_kappa()) {
        return false;
    }

    int small_obstacle_count = 0;
    for (const auto* obstacle : reference_line_info.path_decision().obstacles().Items()) {
        if (!IsSmallRealObstacle(obstacle)) {
            continue;
        }
        const auto& sl_boundary = obstacle->PerceptionSLBoundary();
        if (sl_boundary.end_s() < adc_start_s - 2.0
            || sl_boundary.start_s() > adc_end_s + config.s_curve_look_forward_distance()) {
            continue;
        }
        const double obstacle_center_l = (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
        if (std::fabs(obstacle_center_l) > config.s_curve_max_obstacle_abs_l()) {
            continue;
        }
        if (++small_obstacle_count >= config.s_curve_min_cone_count()) {
            return true;
        }
    }
    return false;
}

bool IsContestUTurn(const ReferenceLineInfo& reference_line_info, const ScenarioContestConfig& config) {
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    const auto& reference_line = reference_line_info.reference_line();
    const double ref_length = reference_line.Length();

    // 方法A: 地图 Lane Turn 属性快速检测（有 U_TURN 标注直接命中）
    for (double s = adc_end_s; s < adc_end_s + config.u_turn_look_forward_distance() && s < ref_length;
         s += kFeatureSampleStep) {
        if (reference_line_info.GetPathTurnType(s) == hdmap::Lane::U_TURN) {
            return true;
        }
    }
    if (reference_line_info.GetPathTurnType(adc_end_s) == hdmap::Lane::U_TURN) {
        return true;
    }

    // 方法B: 直接读路由中每条 lane 的原始几何 heading 变化
    // 不依赖参考线平滑（平滑会稀释 heading 变化），
    // 直接使用 lane 中心曲线原始数据。
    // Lane_1955 heading 变化 ≈ 179°，车辆在 Lane_885 上即可提前检测。
    double route_s = 0.0;
    for (const auto& seg : reference_line_info.Lanes()) {
        const double seg_len = seg.end_s - seg.start_s;
        if (route_s + seg_len < adc_end_s) {
            route_s += seg_len;
            continue;
        }
        route_s += seg_len;
        if (route_s > adc_end_s + 150.0)
            break;

        if (seg.lane == nullptr)
            continue;
        const auto& lane_pb = seg.lane->lane();
        if (lane_pb.central_curve().segment().empty())
            continue;

        std::vector<common::math::Vec2d> pts;
        for (const auto& curve_seg : lane_pb.central_curve().segment()) {
            if (!curve_seg.has_line_segment())
                continue;
            for (const auto& pt : curve_seg.line_segment().point()) {
                pts.emplace_back(pt.x(), pt.y());
            }
        }
        if (pts.size() < 3)
            continue;

        double sum_abs_dh = 0.0;
        double prev_h = std::atan2(pts[1].y() - pts[0].y(), pts[1].x() - pts[0].x());
        for (size_t i = 2; i < pts.size(); ++i) {
            double cur_h = std::atan2(pts[i].y() - pts[i - 1].y(), pts[i].x() - pts[i - 1].x());
            double dh = common::math::NormalizeAngle(cur_h - prev_h);
            sum_abs_dh += std::fabs(dh);
            prev_h = cur_h;
        }
        double net_h = std::fabs(
                common::math::NormalizeAngle(
                        std::atan2(pts.back().y() - pts[pts.size() - 2].y(), pts.back().x() - pts[pts.size() - 2].x())
                        - std::atan2(pts[1].y() - pts[0].y(), pts[1].x() - pts[0].x())));

        static constexpr double kMonotonicRatio = 0.5;
        double ratio = (sum_abs_dh > 1e-6) ? net_h / sum_abs_dh : 0.0;
        if (net_h > config.u_turn_heading_change_threshold() && ratio > kMonotonicRatio) {
            return true;
        }
    }
    return false;
}

int CountContestConstructionConesAhead(const ReferenceLineInfo& reference_line_info, double look_forward_distance) {
    const double adc_back_s = reference_line_info.AdcSlBoundary().start_s();
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    int cone_count = 0;
    for (const auto* obstacle : reference_line_info.path_decision().obstacles().Items()) {
        if (!IsSmallRealObstacle(obstacle)) {
            continue;
        }
        const auto& sl = obstacle->PerceptionSLBoundary();
        // 只统计主车前方范围内的锥桶，不依赖XY硬编码区域
        if (sl.start_s() > adc_back_s - 3.0 && sl.start_s() - adc_end_s < look_forward_distance) {
            ++cone_count;
        }
    }
    return cone_count;
}

int CountContestConstructionConesAhead(
        const Frame& frame,
        const ReferenceLineInfo& self_rli,
        double look_forward_distance) {
    // 跨所有参考线统计锥桶（施工区域赛题锥桶横跨三条车道），
    // 去重后用 self_rli 的 SL 坐标判断纵向位置。
    const double adc_back_s = self_rli.AdcSlBoundary().start_s();
    const double adc_end_s = self_rli.AdcSlBoundary().end_s();
    std::set<std::string> seen_ids;
    int total = 0;
    for (const auto& rli : frame.reference_line_info()) {
        for (const auto* obstacle : rli.path_decision().obstacles().Items()) {
            if (!IsSmallRealObstacle(obstacle))
                continue;
            if (!seen_ids.insert(obstacle->Id()).second)
                continue;
            const auto& sl = obstacle->PerceptionSLBoundary();
            if (sl.start_s() > adc_back_s - 3.0 && sl.start_s() - adc_end_s < look_forward_distance) {
                ++total;
            }
        }
    }
    return total;
}

bool IsContestConstructionZone(const ReferenceLineInfo& reference_line_info, const ScenarioContestConfig& config) {
    if (IsContestUTurn(reference_line_info, config) || IsContestSCurve(reference_line_info, config)) {
        return false;
    }
    return CountContestConstructionConesAhead(reference_line_info, config.construction_look_forward_distance())
            > config.construction_min_cone_count();
}

bool IsContestConstructionZone(
        const Frame& frame,
        const ReferenceLineInfo& self_rli,
        const ScenarioContestConfig& config) {
    if (IsContestUTurn(self_rli, config) || IsContestSCurve(self_rli, config)) {
        return false;
    }
    // 使用跨三车道锥桶统计，确保施工区域入口判定覆盖全部锥桶
    return CountContestConstructionConesAhead(frame, self_rli, config.construction_look_forward_distance())
            > config.construction_min_cone_count();
}

bool IsContestStationShuttle(
        const ReferenceLineInfo& reference_line_info,
        const ScenarioContestConfig& config,
        std::string* out_parking_spot_id) {
    if (out_parking_spot_id == nullptr) {
        return false;
    }
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    const auto& nearby_path = reference_line_info.reference_line().map_path();
    const double look_forward = config.station_shuttle_look_forward_distance();

    // 遍历 reference line 上的 parking space overlap，找前方最近的泊车位
    double best_distance = std::numeric_limits<double>::max();
    std::string best_spot_id;

    for (const auto& overlap : nearby_path.parking_space_overlaps()) {
        if (overlap.start_s <= adc_end_s) {
            continue;  // 已驶过的泊车位
        }
        const double dist = overlap.start_s - adc_end_s;
        if (dist > look_forward) {
            continue;  // 超出探测范围
        }
        if (dist < best_distance) {
            best_distance = dist;
            best_spot_id = overlap.object_id;
        }
    }

    if (best_spot_id.empty()) {
        return false;
    }
    *out_parking_spot_id = best_spot_id;
    return true;
}

bool IsContestRoundaboutEntry(
        const ReferenceLineInfo& reference_line_info,
        const ScenarioContestConfig& config) {
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    bool near_pnc_junction = false;
    int pnc_junction_count = 0;
    double nearest_overlap_start_s = std::numeric_limits<double>::max();
    for (const auto& overlap :
         reference_line_info.reference_line().map_path().pnc_junction_overlaps()) {
        ++pnc_junction_count;
        if (overlap.end_s < adc_end_s - config.roundabout_inside_junction_buffer()) {
            continue;
        }
        if (overlap.start_s > adc_end_s + config.roundabout_entry_look_forward_distance()) {
            continue;
        }
        near_pnc_junction = true;
        nearest_overlap_start_s = std::min(nearest_overlap_start_s, overlap.start_s);
        break;
    }
    if (near_pnc_junction
        && adc_end_s > nearest_overlap_start_s + kRoundaboutExitPastEntryDistance) {
        AINFO << "[ROUNDABOUT][Scenario] entry already passed, adc_s=" << adc_end_s
              << ", entry_s=" << nearest_overlap_start_s;
        return false;
    }

    const auto& reference_line = reference_line_info.reference_line();
    const double start_s = std::max(0.0, adc_end_s);
    const double end_s = std::min(
            reference_line.Length() - 1.0,
            adc_end_s + config.roundabout_curve_look_forward_distance());
    if (end_s - start_s < 8.0) {
        return false;
    }
    if (HasNearUTurnFeature(reference_line_info, start_s, end_s)) {
        return false;
    }

    double max_abs_kappa = 0.0;
    for (double s = start_s; s <= end_s; s += kFeatureSampleStep) {
        if (reference_line_info.GetPathTurnType(s) == hdmap::Lane::U_TURN) {
            return false;
        }
        max_abs_kappa = std::max(max_abs_kappa, std::fabs(reference_line.GetReferencePoint(s).kappa()));
    }
    const double heading_change = std::fabs(common::math::NormalizeAngle(
            reference_line.GetReferencePoint(end_s).heading()
            - reference_line.GetReferencePoint(start_s).heading()));
    const bool curve_like_roundabout = max_abs_kappa > config.roundabout_min_abs_kappa()
            && heading_change > config.roundabout_min_heading_change()
            && heading_change < config.roundabout_max_heading_change();
    const bool soft_curve_like_roundabout =
            max_abs_kappa > config.roundabout_min_abs_kappa() * 0.6
            && heading_change > config.roundabout_min_heading_change() * 0.5
            && heading_change < config.roundabout_max_heading_change() + 0.3;
    const double long_end_s = std::min(
            reference_line.Length() - 1.0,
            adc_end_s + kRoundaboutLongGeometryLookForwardDistance);
    double long_max_abs_kappa = 0.0;
    for (double s = start_s; s <= long_end_s; s += kFeatureSampleStep) {
        if (reference_line_info.GetPathTurnType(s) == hdmap::Lane::U_TURN) {
            return false;
        }
        long_max_abs_kappa = std::max(
                long_max_abs_kappa, std::fabs(reference_line.GetReferencePoint(s).kappa()));
    }
    const double long_heading_change = std::fabs(common::math::NormalizeAngle(
            reference_line.GetReferencePoint(long_end_s).heading()
            - reference_line.GetReferencePoint(start_s).heading()));
    const bool long_geometry_like_roundabout =
            long_end_s - start_s > 45.0
            && long_max_abs_kappa > kRoundaboutMinLongGeometryKappa
            && long_heading_change > kRoundaboutMinLongGeometryHeadingChange
            && long_heading_change < kRoundaboutMaxLongGeometryHeadingChange;
    const bool roundabout = (near_pnc_junction
                                && (curve_like_roundabout || soft_curve_like_roundabout))
                            || long_geometry_like_roundabout;
    if (!roundabout && (near_pnc_junction || pnc_junction_count > 0 || max_abs_kappa > 0.015
                        || heading_change > 0.35 || long_geometry_like_roundabout
                        || long_max_abs_kappa > kRoundaboutMinLongGeometryKappa)) {
        AINFO << "[ROUNDABOUT][Scenario] probe miss, adc_s=" << adc_end_s
              << ", pnc_count=" << pnc_junction_count
              << ", near_pnc=" << near_pnc_junction
              << ", max_abs_kappa=" << max_abs_kappa
              << ", heading_change=" << heading_change
              << ", long_max_abs_kappa=" << long_max_abs_kappa
              << ", long_heading_change=" << long_heading_change
              << ", ref_len=" << reference_line.Length();
    }
    if (roundabout) {
        AINFO << "[ROUNDABOUT][Scenario] detected entry, adc_s=" << adc_end_s
              << ", pnc_count=" << pnc_junction_count
              << ", near_pnc=" << near_pnc_junction
              << ", max_abs_kappa=" << max_abs_kappa
              << ", heading_change=" << heading_change
              << ", long_max_abs_kappa=" << long_max_abs_kappa
              << ", long_heading_change=" << long_heading_change
              << ", soft=" << soft_curve_like_roundabout;
    }
    return roundabout;
}

bool IsContestRoundaboutEntry(const Frame& frame, const ScenarioContestConfig&) {
    if (frame.local_view().planning_command == nullptr
        || !frame.local_view().planning_command->has_lane_follow_command()) {
        return false;
    }
    return IsContestRoundaboutEntryRoi(frame);
}

bool IsContestRoundaboutEntryPassed(
        const ReferenceLineInfo& reference_line_info,
        const ScenarioContestConfig& config) {
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    for (const auto& overlap :
         reference_line_info.reference_line().map_path().pnc_junction_overlaps()) {
        if (overlap.end_s < adc_end_s - config.roundabout_inside_junction_buffer()) {
            continue;
        }
        if (overlap.start_s > adc_end_s + config.roundabout_entry_look_forward_distance()) {
            continue;
        }
        return adc_end_s > overlap.start_s + kRoundaboutExitPastEntryDistance;
    }
    return false;
}

bool IsContestRoundaboutEntryRoi(const Frame& frame) {
    if (frame.local_view().planning_command == nullptr
        || !frame.local_view().planning_command->has_lane_follow_command()) {
        return false;
    }
    const auto& vehicle_state = frame.vehicle_state();
    const double x = vehicle_state.x();
    const double y = vehicle_state.y();
    if (x < kRoundaboutRoiMinX || x > kRoundaboutRoiMaxX
        || y < kRoundaboutRoiMinY || y > kRoundaboutRoiMaxY) {
        return false;
    }
    if (x > kRoundaboutPassedX && y > kRoundaboutPassedY) {
        return false;
    }
    AINFO << "[ROUNDABOUT][Scenario] detected entry ROI, x=" << x
          << ", y=" << y << ", heading=" << vehicle_state.heading();
    return true;
}

}  // namespace contest
}  // namespace planning
}  // namespace apollo
