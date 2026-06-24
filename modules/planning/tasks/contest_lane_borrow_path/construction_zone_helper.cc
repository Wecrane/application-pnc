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

#include "modules/planning/tasks/contest_lane_borrow_path/construction_zone_helper.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <unordered_map>

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/contest_scenario_features.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/reference_line_info.h"

namespace apollo {
namespace planning {
namespace {

std::string FormatDouble(double value) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.2f", value);
    return std::string(buffer);
}

double InterpolateWallL(const std::vector<std::pair<double, double>>& wall, double s) {
    if (wall.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    auto it = std::lower_bound(
            wall.begin(), wall.end(), s, [](const auto& point, double value) { return point.first < value; });
    if (it == wall.end()) {
        if (s - wall.back().first > 20.0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return wall.back().second;
    }
    if (it == wall.begin()) {
        if (it->first - s > 20.0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return it->second;
    }
    const auto prev = std::prev(it);
    if (it->first - prev->first > 20.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double ratio = (s - prev->first) / (it->first - prev->first);
    return prev->second + ratio * (it->second - prev->second);
}

uint64_t HashXY(double x, double y) {
    const int64_t ix = static_cast<int64_t>(std::round(x));
    const int64_t iy = static_cast<int64_t>(std::round(y));
    return (static_cast<uint64_t>(ix) << 32) | static_cast<uint64_t>(static_cast<uint32_t>(iy));
}

std::pair<double, double> HashToXY(uint64_t hash) {
    const int32_t ix = static_cast<int32_t>(hash >> 32);
    const int32_t iy = static_cast<int32_t>(hash & 0xffffffffu);
    return {static_cast<double>(ix), static_cast<double>(iy)};
}

double NearestXYDistance(double x, double y, const std::vector<std::pair<double, double>>& points) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& point : points) {
        best = std::min(best, std::hypot(point.first - x, point.second - y));
    }
    return best;
}

double NearestXYDistanceExceptNear(
        double x,
        double y,
        const std::vector<std::pair<double, double>>& points,
        double min_dist) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& point : points) {
        const double dist = std::hypot(point.first - x, point.second - y);
        if (dist > min_dist) {
            best = std::min(best, dist);
        }
    }
    return best;
}

bool FindTwoRecentPointsByX(
        const std::vector<std::pair<double, double>>& points,
        double x,
        std::pair<double, double>* prev,
        std::pair<double, double>* last) {
    if (prev == nullptr || last == nullptr) {
        return false;
    }
    bool has_last = false;
    bool has_prev = false;
    std::pair<double, double> best_last;
    std::pair<double, double> best_prev;
    for (const auto& point : points) {
        if (point.first >= x - 1.0) {
            continue;
        }
        if (!has_last || point.first > best_last.first) {
            if (has_last) {
                best_prev = best_last;
                has_prev = true;
            }
            best_last = point;
            has_last = true;
        } else if (!has_prev || point.first > best_prev.first) {
            best_prev = point;
            has_prev = true;
        }
    }
    if (!has_prev || !has_last) {
        return false;
    }
    *prev = best_prev;
    *last = best_last;
    return true;
}

bool ShouldPreferLeftChainContinuation(
        double x,
        double y,
        double l,
        double road_right,
        const ConstructionZoneState& construction_zone,
        std::string* why) {
    std::pair<double, double> prev_left;
    std::pair<double, double> last_left;
    if (!FindTwoRecentPointsByX(construction_zone.classified_left_xy, x, &prev_left, &last_left)) {
        return false;
    }

    const double gap_x = x - last_left.first;
    if (gap_x < 2.0 || gap_x > 14.0) {
        return false;
    }
    if (l - road_right < 3.2) {
        return false;
    }

    const double last_dist = std::hypot(x - last_left.first, y - last_left.second);
    const double right_dist = NearestXYDistanceExceptNear(x, y, construction_zone.classified_right_xy, 1.0);
    const double prev_gap_x = last_left.first - prev_left.first;
    if (std::fabs(prev_gap_x) < 1.0) {
        return false;
    }

    const double slope = (last_left.second - prev_left.second) / prev_gap_x;
    const double predicted_y = last_left.second + slope * gap_x;
    const double y_error = std::fabs(y - predicted_y);
    const bool straight_extension = last_dist < 11.5 && y_error < 1.6
            && (!std::isfinite(right_dist) || last_dist + 0.8 < right_dist);

    const bool left_tail_turn = construction_zone.classified_left_xy.size() >= 4 && last_dist < 8.5
            && y + 0.5 >= last_left.second
            && (!std::isfinite(right_dist) || last_dist + 0.8 < right_dist);

    if (!straight_extension && !left_tail_turn) {
        return false;
    }
    if (why != nullptr) {
        *why = std::string(straight_extension ? "leftChain" : "leftTurn") + " dL=" + FormatDouble(last_dist)
                + " dR=" + FormatDouble(right_dist) + " yErr=" + FormatDouble(y_error);
    }
    return true;
}

bool IsGlobalTailRightCone(double x, double y, double l, double road_right, const ConstructionZoneState& construction_zone) {
    if (construction_zone.farthest_cone_x <= 0.0) {
        return false;
    }
    std::pair<double, double> prev_left;
    std::pair<double, double> last_left;
    if (construction_zone.classified_left_xy.size() < 8
        || !FindTwoRecentPointsByX(construction_zone.classified_left_xy, x, &prev_left, &last_left)) {
        return false;
    }
    constexpr double kGlobalTailDist = 10.0;
    constexpr double kRightRoadDist = 7.0;
    constexpr double kLeftTailDrop = 2.8;
    return std::hypot(x - construction_zone.farthest_cone_x, y - construction_zone.farthest_cone_y) < kGlobalTailDist
            && std::fabs(l - road_right) < kRightRoadDist
            && x > last_left.first + 1.0
            && last_left.second - y > kLeftTailDrop;
}

void AddUniqueXY(std::vector<std::pair<double, double>>* points, double x, double y) {
    if (points == nullptr) {
        return;
    }
    for (const auto& point : *points) {
        if (std::hypot(point.first - x, point.second - y) < 0.75) {
            return;
        }
    }
    points->emplace_back(x, y);
}

void RemoveNearXY(std::vector<std::pair<double, double>>* points, double x, double y) {
    if (points == nullptr) {
        return;
    }
    points->erase(
            std::remove_if(
                    points->begin(),
                    points->end(),
                    [x, y](const auto& point) { return std::hypot(point.first - x, point.second - y) < 0.75; }),
            points->end());
}

void PruneWallBehind(double adc_back_s, double cleanup_dist, std::vector<std::pair<double, double>>* wall) {
    if (wall == nullptr) {
        return;
    }
    wall->erase(
            std::remove_if(
                    wall->begin(),
                    wall->end(),
                    [adc_back_s, cleanup_dist](const auto& point) { return point.first < adc_back_s - cleanup_dist; }),
            wall->end());
}

void PruneConstructionZoneWallState(
        const ConstructionConeXYMap& cone_xy,
        double adc_back_s,
        ConstructionZoneState* construction_zone) {
    constexpr double kCleanupBehindDist = 30.0;
    PruneWallBehind(adc_back_s, kCleanupBehindDist, &construction_zone->left_wall);
    PruneWallBehind(adc_back_s, kCleanupBehindDist, &construction_zone->right_wall);

    if (cone_xy.empty()) {
        return;
    }

    constexpr double kMemoryKeepXYPadding = 160.0;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto& kv : cone_xy) {
        min_x = std::min(min_x, kv.second.first);
        max_x = std::max(max_x, kv.second.first);
        min_y = std::min(min_y, kv.second.second);
        max_y = std::max(max_y, kv.second.second);
    }
    min_x -= kMemoryKeepXYPadding;
    max_x += kMemoryKeepXYPadding;
    min_y -= kMemoryKeepXYPadding;
    max_y += kMemoryKeepXYPadding;

    auto in_current_zone_xy = [min_x, max_x, min_y, max_y](double x, double y) {
        return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
    };

    const size_t old_mem = construction_zone->cone_wall_memory.size();
    for (auto it = construction_zone->cone_wall_memory.begin(); it != construction_zone->cone_wall_memory.end();) {
        const auto xy = HashToXY(it->first);
        if (!in_current_zone_xy(xy.first, xy.second)) {
            it = construction_zone->cone_wall_memory.erase(it);
        } else {
            ++it;
        }
    }

    auto prune_xy_points = [&in_current_zone_xy](std::vector<std::pair<double, double>>* points) {
        points->erase(
                std::remove_if(
                        points->begin(),
                        points->end(),
                        [&in_current_zone_xy](const auto& point) {
                            return !in_current_zone_xy(point.first, point.second);
                        }),
                points->end());
    };
    const size_t old_left_xy = construction_zone->classified_left_xy.size();
    const size_t old_right_xy = construction_zone->classified_right_xy.size();
    prune_xy_points(&construction_zone->classified_left_xy);
    prune_xy_points(&construction_zone->classified_right_xy);

    if (old_mem != construction_zone->cone_wall_memory.size()
        || old_left_xy != construction_zone->classified_left_xy.size()
        || old_right_xy != construction_zone->classified_right_xy.size()) {
        ADEBUG << "[WALL] PRUNE|mem=" << old_mem << "->" << construction_zone->cone_wall_memory.size()
               << "|lxy=" << old_left_xy << "->" << construction_zone->classified_left_xy.size()
               << "|rxy=" << old_right_xy << "->" << construction_zone->classified_right_xy.size();
    }
}

void SeedConstructionZoneWalls(
        const std::vector<SLPolygon>& cones,
        double road_left,
        double road_right,
        ConstructionZoneState* construction_zone) {
    if (!construction_zone->left_wall.empty() || !construction_zone->right_wall.empty() || cones.empty()) {
        return;
    }

    std::vector<std::pair<double, double>> sorted_cones;
    for (const auto& cone : cones) {
        sorted_cones.emplace_back((cone.MinS() + cone.MaxS()) * 0.5, (cone.MinL() + cone.MaxL()) * 0.5);
    }
    std::sort(sorted_cones.begin(), sorted_cones.end());

    int left_count = 0;
    int right_count = 0;
    for (const auto& cone_sl : sorted_cones) {
        const double s = cone_sl.first;
        const double l = cone_sl.second;
        const double left_wall_l = InterpolateWallL(construction_zone->left_wall, s);
        const double right_wall_l = InterpolateWallL(construction_zone->right_wall, s);

        bool to_left = false;
        if (std::isnan(left_wall_l) && std::isnan(right_wall_l)) {
            to_left = std::fabs(l - road_left) < std::fabs(l - road_right);
        } else if (std::isnan(left_wall_l)) {
            const double dist_to_right_wall = std::fabs(l - right_wall_l);
            const double dist_to_road_left = std::fabs(l - road_left);
            to_left = dist_to_right_wall > 5.0 && dist_to_right_wall > dist_to_road_left;
        } else if (std::isnan(right_wall_l)) {
            const double dist_to_left_wall = std::fabs(l - left_wall_l);
            const double dist_to_road_right = std::fabs(l - road_right);
            to_left = !(dist_to_left_wall > 5.0 && dist_to_left_wall > dist_to_road_right);
        } else {
            to_left = std::fabs(l - left_wall_l) < std::fabs(l - right_wall_l);
        }

        if (to_left) {
            construction_zone->left_wall.emplace_back(s, l);
            ++left_count;
        } else {
            construction_zone->right_wall.emplace_back(s, l);
            ++right_count;
        }
    }
    ADEBUG << "[WALL] INIT|sliding|nL=" << left_count << "|nR=" << right_count;
}

bool MergeConstructionZoneWallPoint(bool to_left, double s, double l, ConstructionZoneState* construction_zone) {
    constexpr double kWallMergeDist = 0.8;
    auto& wall = to_left ? construction_zone->left_wall : construction_zone->right_wall;
    for (auto& point : wall) {
        if (std::fabs(point.first - s) < kWallMergeDist) {
            point.second = point.second * 0.7 + l * 0.3;
            return true;
        }
    }
    wall.emplace_back(s, l);
    std::sort(wall.begin(), wall.end());
    return false;
}

// 墙壁趋势外推：对墙最后 num_pts 个点做线性回归，预测 s 处的 l 值。
// 返回 NaN 表示点数不足或回归失败。
double ExtrapolateWallL(const std::vector<std::pair<double, double>>& wall, double s, int num_pts = 5) {
    const int n = std::min(num_pts, static_cast<int>(wall.size()));
    if (n < 2) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    // 取最后 n 个点
    double sum_s = 0.0, sum_l = 0.0, sum_ss = 0.0, sum_sl = 0.0;
    for (int i = static_cast<int>(wall.size()) - n; i < static_cast<int>(wall.size()); ++i) {
        sum_s += wall[i].first;
        sum_l += wall[i].second;
        sum_ss += wall[i].first * wall[i].first;
        sum_sl += wall[i].first * wall[i].second;
    }
    const double denom = n * sum_ss - sum_s * sum_s;
    if (std::fabs(denom) < 1e-12) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double slope = (n * sum_sl - sum_s * sum_l) / denom;
    const double intercept = (sum_l - slope * sum_s) / n;
    return intercept + slope * s;
}

void ApplyConstructionZoneNudgeChoices(
        const ConstructionZoneState& construction_zone,
        const std::unordered_map<std::string, bool>& frame_wall_choice,
        std::vector<SLPolygon>* cones) {
    if (cones == nullptr) {
        return;
    }
    for (auto& cone : *cones) {
        const auto frame_choice = frame_wall_choice.find(cone.id());
        if (frame_choice != frame_wall_choice.end()) {
            cone.SetNudgeInfo(frame_choice->second ? SLPolygon::RIGHT_NUDGE : SLPolygon::LEFT_NUDGE);
            continue;
        }

        const double s = (cone.MinS() + cone.MaxS()) * 0.5;
        const double l = (cone.MinL() + cone.MaxL()) * 0.5;
        const double left_wall_l = InterpolateWallL(construction_zone.left_wall, s);
        const double right_wall_l = InterpolateWallL(construction_zone.right_wall, s);
        if (std::isnan(left_wall_l) && std::isnan(right_wall_l)) {
            // 墙壁不可用：fallback 到道路边界
            cone.SetNudgeInfo(
                    std::fabs(l - construction_zone.max_left_bound) < std::fabs(l - construction_zone.max_right_bound)
                            ? SLPolygon::RIGHT_NUDGE
                            : SLPolygon::LEFT_NUDGE);
            continue;
        }
        if (std::isnan(left_wall_l)) {
            cone.SetNudgeInfo(std::fabs(l - right_wall_l) > 5.0 ? SLPolygon::RIGHT_NUDGE : SLPolygon::LEFT_NUDGE);
        } else if (std::isnan(right_wall_l)) {
            cone.SetNudgeInfo(std::fabs(l - left_wall_l) > 5.0 ? SLPolygon::LEFT_NUDGE : SLPolygon::RIGHT_NUDGE);
        } else {
            cone.SetNudgeInfo(
                    std::fabs(l - left_wall_l) < std::fabs(l - right_wall_l) ? SLPolygon::RIGHT_NUDGE
                                                                             : SLPolygon::LEFT_NUDGE);
        }
    }
}

}  // namespace

void ComputeConstructionZoneBoundary(
        std::vector<SLPolygon>* cones,
        const ConstructionConeXYMap& cone_xy,
        double adc_back_s,
        ConstructionZoneState* construction_zone) {
    if (cones == nullptr || cones->empty() || construction_zone == nullptr) {
        return;
    }

    const double road_left = construction_zone->max_left_bound;
    const double road_right = construction_zone->max_right_bound;

    auto dbg = [](const std::string& msg) { AINFO << "[WALL] " << msg; };
    dbg("FRAME|cones=" + std::to_string(cones->size()) + "|lw=" + std::to_string(construction_zone->left_wall.size())
        + "|rw=" + std::to_string(construction_zone->right_wall.size()) + "|road=[" + FormatDouble(road_right) + ","
        + FormatDouble(road_left) + "]");

    PruneConstructionZoneWallState(cone_xy, adc_back_s, construction_zone);
    SeedConstructionZoneWalls(*cones, road_left, road_right, construction_zone);

    bool has_tail_right_seed = false;
    double tail_right_seed_x = 0.0;
    double tail_right_seed_y = 0.0;
    if (construction_zone->left_wall.size() >= 8 && construction_zone->right_wall.size() >= 8 && cones->size() <= 6) {
        double best_seed_x = -std::numeric_limits<double>::infinity();
        for (const auto& cone : *cones) {
            auto xy_it = cone_xy.find(cone.id());
            if (xy_it == cone_xy.end()) {
                continue;
            }
            const double l = (cone.MinL() + cone.MaxL()) * 0.5;
            if (l - road_right < 3.5 && xy_it->second.first > best_seed_x) {
                best_seed_x = xy_it->second.first;
                tail_right_seed_x = xy_it->second.first;
                tail_right_seed_y = xy_it->second.second;
                has_tail_right_seed = true;
            }
        }
    }
    auto in_tail_right_cluster = [has_tail_right_seed, tail_right_seed_x, tail_right_seed_y](double x, double y) {
        constexpr double kTailRightClusterDist = 18.0;
        return has_tail_right_seed && x + 1.0 >= tail_right_seed_x
                && std::hypot(x - tail_right_seed_x, y - tail_right_seed_y) < kTailRightClusterDist;
    };

    std::unordered_map<std::string, bool> frame_wall_choice;
    for (const auto& cone : *cones) {
        const double s = (cone.MinS() + cone.MaxS()) * 0.5;
        const double l = (cone.MinL() + cone.MaxL()) * 0.5;

        bool to_left = false;
        std::string why;

        auto xy_it = cone_xy.find(cone.id());
        bool from_memory = false;
        double pl = std::numeric_limits<double>::quiet_NaN();
        double pr = std::numeric_limits<double>::quiet_NaN();
        if (xy_it != cone_xy.end()) {
            const double cx = xy_it->second.first;
            const double cy = xy_it->second.second;
            const uint64_t h = HashXY(cx, cy);
            auto mem_it = construction_zone->cone_wall_memory.find(h);
            if (in_tail_right_cluster(cx, cy)) {
                to_left = false;
                why = "tailR";
                from_memory = true;
            } else if (mem_it != construction_zone->cone_wall_memory.end()) {
                to_left = mem_it->second;
                why = "mem";
                from_memory = true;
                if (!construction_zone->classified_left_xy.empty() && !construction_zone->classified_right_xy.empty()) {
                    constexpr double kMemoryOverrideDist = 25.0;
                    constexpr double kMemoryOverrideMargin = 2.0;
                    const double nl = NearestXYDistance(cx, cy, construction_zone->classified_left_xy);
                    const double nr = NearestXYDistance(cx, cy, construction_zone->classified_right_xy);
                    if (to_left && nr < kMemoryOverrideDist && nr + kMemoryOverrideMargin < nl) {
                        to_left = false;
                        why = "memFixR";
                    } else if (!to_left && nl < kMemoryOverrideDist && nl + kMemoryOverrideMargin < nr) {
                        to_left = true;
                        why = "memFixL";
                    }
                }
            }
        }

        // 全局尾部锚点：靠近全局最远锥桶且贴近右侧道路边界的锥桶，强制归类为右侧。
        // 这解决了墙壁插值在尾部不可靠时，最后几个右侧锥桶被错误归类为左侧的问题。
        // 仅当 cone 确实靠近全局最远锥桶（非雷达范围中途截断）时生效。
        if (!from_memory && xy_it != cone_xy.end()) {
            const double cx = xy_it->second.first;
            const double cy = xy_it->second.second;
            if (IsGlobalTailRightCone(cx, cy, l, road_right, *construction_zone)) {
                to_left = false;
                why = "globalTail";
                from_memory = true;
            }
        }

        if (!from_memory) {
            pl = InterpolateWallL(construction_zone->left_wall, s);
            pr = InterpolateWallL(construction_zone->right_wall, s);

            if (std::isnan(pl) && std::isnan(pr)) {
                // 两面墙都无法覆盖此 s：用道路边界做 fallback 分类，
                // 避免 cone 被跳过导致后续 frame_wall_choice 缺失
                to_left = std::fabs(l - road_left) < std::fabs(l - road_right);
                why = "fallback";
            } else if (std::isnan(pl)) {
                const double d_to_rw = std::fabs(l - pr);
                const double d_to_road_l = std::fabs(l - road_left);
                if (d_to_rw > 5.0 && d_to_rw > d_to_road_l) {
                    to_left = true;
                    why = "farL";
                } else if (d_to_rw > 5.0) {
                    to_left = false;
                    why = "newL";
                } else if (xy_it != cone_xy.end() && !construction_zone->classified_left_xy.empty()) {
                    const double cx = xy_it->second.first;
                    const double cy = xy_it->second.second;
                    const double nl = NearestXYDistance(cx, cy, construction_zone->classified_left_xy);
                    const double nr = NearestXYDistance(cx, cy, construction_zone->classified_right_xy);
                    to_left = nl < nr && nl < 15.0;
                    why = "nnL=" + FormatDouble(nl) + " nnR=" + FormatDouble(nr);
                } else {
                    to_left = false;
                    why = "noL";
                }
            } else if (std::isnan(pr)) {
                const double d_to_lw = std::fabs(l - pl);
                const double d_to_road_r = std::fabs(l - road_right);
                if (d_to_lw > 5.0 && d_to_lw > d_to_road_r) {
                    to_left = false;
                    why = "farR";
                } else if (d_to_lw > 5.0) {
                    to_left = true;
                    why = "newR";
                } else if (xy_it != cone_xy.end() && !construction_zone->classified_right_xy.empty()) {
                    const double cx = xy_it->second.first;
                    const double cy = xy_it->second.second;
                    const double nl = NearestXYDistance(cx, cy, construction_zone->classified_left_xy);
                    const double nr = NearestXYDistance(cx, cy, construction_zone->classified_right_xy);
                    to_left = !(nr < nl && nr < 15.0);
                    why = "nnL=" + FormatDouble(nl) + " nnR=" + FormatDouble(nr);
                } else {
                    to_left = true;
                    why = "noR";
                }
            } else {
                const double dl = std::fabs(l - pl);
                const double dr = std::fabs(l - pr);

                // 先走标准 XY/距离判断
                {
                    if ((std::fabs(dl - dr) < 1.5 || std::min(dl, dr) > 3.0) && xy_it != cone_xy.end()) {
                        const double cx = xy_it->second.first;
                        const double cy = xy_it->second.second;
                        const double nl = NearestXYDistance(cx, cy, construction_zone->classified_left_xy);
                        const double nr = NearestXYDistance(cx, cy, construction_zone->classified_right_xy);
                        to_left = nl < nr;
                        why = "nnL=" + FormatDouble(nl) + " nnR=" + FormatDouble(nr);
                    } else {
                        to_left = dl < dr;
                        why = "dL=" + FormatDouble(dl) + " dR=" + FormatDouble(dr);
                    }
                }

                // 趋势/lGap 修正：仅当标准判断为 R 且左墙稀疏时，尝试翻转为 L
                if (!to_left && construction_zone->left_wall.size() >= 2) {
                    // 方法1: 墙壁趋势外推
                    const double pl_trend = ExtrapolateWallL(
                            construction_zone->left_wall, s,
                            std::min(5, static_cast<int>(construction_zone->left_wall.size())));
                    if (!std::isnan(pl_trend)) {
                        const double d_trend = std::fabs(l - pl_trend);
                        if (d_trend < dr - 0.5 && d_trend < dl) {
                            to_left = true;
                            why = "trend";
                        }
                    }
                    // 方法2: lGap 兜底 — l 高于右墙且（显著高于 或 左墙插值滞后）
                    if (!to_left && l > pr + 0.5
                        && (l > pr + 0.8 || pl > l + 2.0)
                        && s - construction_zone->left_wall.back().first < 15.0) {
                        to_left = true;
                        why = "lGap";
                        // XY 复核：若锥桶明显更靠近右墙 XY 参考集，撤销 lGap 翻转
                        if (xy_it != cone_xy.end()
                            && !construction_zone->classified_left_xy.empty()
                            && !construction_zone->classified_right_xy.empty()) {
                            const double cx = xy_it->second.first;
                            const double cy = xy_it->second.second;
                            const double nl = NearestXYDistance(cx, cy, construction_zone->classified_left_xy);
                            const double nr = NearestXYDistance(cx, cy, construction_zone->classified_right_xy);
                            if (nr < nl && nr < 12.0) {
                                to_left = false;
                                why = "lGapXY";
                            }
                        }
                    }
                }
            }
        }

        if (!to_left && why != "tailR" && why != "globalTail" && xy_it != cone_xy.end()) {
            std::string chain_why;
            if (ShouldPreferLeftChainContinuation(
                        xy_it->second.first, xy_it->second.second, l, road_right, *construction_zone, &chain_why)) {
                to_left = true;
                why = chain_why;
            }
        }

        if (xy_it != cone_xy.end()
            && IsGlobalTailRightCone(xy_it->second.first, xy_it->second.second, l, road_right, *construction_zone)) {
            to_left = false;
            why = "globalTail";
        }

        frame_wall_choice[cone.id()] = to_left;
        if (xy_it != cone_xy.end()) {
            const uint64_t h = HashXY(xy_it->second.first, xy_it->second.second);
            construction_zone->cone_wall_memory[h] = to_left;
            if (to_left) {
                RemoveNearXY(&construction_zone->classified_right_xy, xy_it->second.first, xy_it->second.second);
                AddUniqueXY(&construction_zone->classified_left_xy, xy_it->second.first, xy_it->second.second);
            } else {
                RemoveNearXY(&construction_zone->classified_left_xy, xy_it->second.first, xy_it->second.second);
                AddUniqueXY(&construction_zone->classified_right_xy, xy_it->second.first, xy_it->second.second);
            }
        }

        const bool merged = MergeConstructionZoneWallPoint(to_left, s, l, construction_zone);
        dbg(std::string("CONE|id=") + cone.id() + "|xy="
            + (xy_it != cone_xy.end() ? FormatDouble(xy_it->second.first) + "," + FormatDouble(xy_it->second.second)
                                      : "nan,nan")
            + "|s=" + FormatDouble(s) + "|l=" + FormatDouble(l) + "|pL=" + (std::isnan(pl) ? "nan" : FormatDouble(pl))
            + "|pR=" + (std::isnan(pr) ? "nan" : FormatDouble(pr)) + "|->" + (to_left ? "L" : "R") + "|" + why
            + (merged ? "|m" : "|n"));
    }
    ApplyConstructionZoneNudgeChoices(*construction_zone, frame_wall_choice, cones);
}

void UpdateConstructionZoneTrackingState(
        Frame* frame,
        ReferenceLineInfo* reference_line_info,
        int low_cone_exit_threshold,
        ConstructionZoneState* construction_zone) {
    if (frame == nullptr || reference_line_info == nullptr || construction_zone == nullptr) {
        return;
    }

    constexpr double kConeHistoryCleanupDist = 50.0;
    constexpr double kExitPastLastConeDist = 15.0;
    constexpr int kEmptyFramesThreshold = 50;

    const double adc_start_s = reference_line_info->AdcSlBoundary().start_s();
    const double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
    const double adc_x = frame->vehicle_state().x();
    const double adc_y = frame->vehicle_state().y();
    const double adc_heading = frame->vehicle_state().heading();
    (void)low_cone_exit_threshold;

    auto reset_state = [construction_zone](const std::string& reason) {
        ADEBUG << "[WALL] EXIT construct_zone by " << reason;
        construction_zone->Reset();
    };

    construction_zone->cone_history.erase(
            std::remove_if(
                    construction_zone->cone_history.begin(),
                    construction_zone->cone_history.end(),
                    [adc_start_s](const std::pair<double, double>& point) {
                        return point.first < adc_start_s - kConeHistoryCleanupDist;
                    }),
            construction_zone->cone_history.end());

    int small_obs_count = 0;
    double farthest_cone_s_this_frame = -1.0;
    for (const auto* obstacle : reference_line_info->path_decision()->obstacles().Items()) {
        if (!contest::IsSmallRealObstacle(obstacle)) {
            continue;
        }
        const auto& sl = obstacle->PerceptionSLBoundary();
        const double obs_s = sl.end_s();
        if (obs_s <= adc_start_s || sl.start_s() - adc_end_s >= contest::kDefaultConstructionLookForwardDistance) {
            continue;
        }

        ++small_obs_count;
        farthest_cone_s_this_frame = std::max(farthest_cone_s_this_frame, obs_s);

        double cx = 0.0;
        double cy = 0.0;
        if (contest::GetObstacleCenterXY(obstacle, &cx, &cy)) {
            const double cone_rel_s = (cx - adc_x) * std::cos(adc_heading) + (cy - adc_y) * std::sin(adc_heading);
            const double saved_cone_rel_s = (construction_zone->farthest_cone_x - adc_x) * std::cos(adc_heading)
                    + (construction_zone->farthest_cone_y - adc_y) * std::sin(adc_heading);
            if (construction_zone->farthest_cone_x < 0.0 || cone_rel_s > saved_cone_rel_s) {
                construction_zone->farthest_cone_x = cx;
                construction_zone->farthest_cone_y = cy;
            }
        }

        const double obs_l = (sl.start_l() + sl.end_l()) * 0.5;
        bool already_recorded = false;
        for (const auto& history_cone : construction_zone->cone_history) {
            if (std::fabs(history_cone.first - obs_s) < 1.0 && std::fabs(history_cone.second - obs_l) < 1.0) {
                already_recorded = true;
                break;
            }
        }
        if (!already_recorded) {
            construction_zone->cone_history.emplace_back(obs_s, obs_l);
        }
    }

    int history_cone_ahead = 0;
    for (const auto& history_cone : construction_zone->cone_history) {
        if (history_cone.first > adc_end_s && history_cone.first - adc_end_s < 100.0) {
            ++history_cone_ahead;
            farthest_cone_s_this_frame = std::max(farthest_cone_s_this_frame, history_cone.first);
        }
    }
    if (farthest_cone_s_this_frame > 0.0) {
        construction_zone->farthest_cone_s = std::max(construction_zone->farthest_cone_s, farthest_cone_s_this_frame);
    }

    const int total_cone_estimate = small_obs_count + history_cone_ahead;
    const bool hold_by_s = construction_zone->active && construction_zone->farthest_cone_s > 0.0
            && adc_end_s < construction_zone->farthest_cone_s + kExitPastLastConeDist;
    const double last_cone_rel_s = (construction_zone->farthest_cone_x - adc_x) * std::cos(adc_heading)
            + (construction_zone->farthest_cone_y - adc_y) * std::sin(adc_heading);
    const double last_cone_xy_dist
            = std::hypot(adc_x - construction_zone->farthest_cone_x, adc_y - construction_zone->farthest_cone_y);
    const bool hold_by_xy = construction_zone->active && construction_zone->farthest_cone_x > 0.0
            && last_cone_rel_s > -kExitPastLastConeDist && last_cone_xy_dist < 80.0;
    const bool should_hold = hold_by_s || hold_by_xy;

    if (construction_zone->active && total_cone_estimate < 3 && should_hold) {
        ADEBUG << "[WALL] HOLD construct_zone tail|total=" << total_cone_estimate << "|by_s=" << hold_by_s
              << "|by_xy=" << hold_by_xy << "|last_xy=(" << construction_zone->farthest_cone_x << ","
              << construction_zone->farthest_cone_y << ")|adc_xy=(" << adc_x << "," << adc_y
              << ")|rel_s=" << last_cone_rel_s;
    }

    if (total_cone_estimate > 0) {
        construction_zone->active = true;
        construction_zone->low_cone_counter = 0;
    } else {
        construction_zone->low_cone_counter = 0;
    }

    if (construction_zone->active && small_obs_count == 0) {
        if (should_hold || total_cone_estimate > 0) {
            construction_zone->no_cone_counter = 0;
        } else {
            ++construction_zone->no_cone_counter;
        }
        if (construction_zone->no_cone_counter > kEmptyFramesThreshold) {
            reset_state("empty frames");
        }
    } else if (small_obs_count > 0) {
        construction_zone->no_cone_counter = 0;
    }
}

}  // namespace planning
}  // namespace apollo
