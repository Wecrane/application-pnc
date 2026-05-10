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

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include "modules/planning/tasks/lane_borrow_path/proto/lane_borrow_path.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

namespace apollo {
namespace planning {

enum SidePassDirection { LEFT_BORROW = 1, RIGHT_BORROW = 2 };
class LaneBorrowPath : public PathGeneration {
public:
    bool Init(
            const std::string& config_dir,
            const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

private:
    apollo::common::Status Process(Frame* frame, ReferenceLineInfo* reference_line_info) override;
    /**
     * @brief Calculate all path boundaries
     * @param boundary is calculated path boundaries
     */
    bool DecidePathBounds(std::vector<PathBoundary>* boundary);
    /**
     * @brief Optimize paths for each path boundary
     * @param path_boundaries is input path boundaries
     * @param candidate_path_data is output paths
     */
    bool OptimizePath(const std::vector<PathBoundary>& path_boundaries, std::vector<PathData>* candidate_path_data);
    /**
     * @brief Assess the feasibility of each path and select the best one
     * @param candidate_path_data is input paths
     * @param final_path is output the best path
     */
    bool AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path);
    bool HasDynamicVehicleConflictOnBorrowPath(const std::vector<PathData>& candidate_path_data) const;
    bool GenerateBorrowHoldPath(PathData* final_path);
    /**
     * @brief Generate path boundary by left or right neightbor lane and self lane
     * @param pass_direction is side pass direction (left or right)
     * @param final_path is output the best path
     */
    bool GetBoundaryFromNeighborLane(
            const SidePassDirection pass_direction,
            PathBoundary* const path_bound,
            std::string* borrow_lane_type);
    /**
     * @brief Determine whether to borrow neighbor lane
     * @return if need to borrow lane return true
     */
    bool IsNecessaryToBorrowLane();

    bool HasSingleReferenceLine(const Frame& frame);

    bool IsWithinSidePassingSpeedADC(const Frame& frame);

    bool IsLongTermBlockingObstacle();

    bool IsBlockingObstacleWithinDestination(const ReferenceLineInfo& reference_line_info);

    bool IsBlockingObstacleFarFromIntersection(const ReferenceLineInfo& reference_line_info);

    bool IsSidePassableObstacle(const ReferenceLineInfo& reference_line_info);

    void UpdateSelfPathInfo();
    /**
     * @brief Check whether neighbor lane is borrowable
     * @param reference_line_info is input reference line info
     * @param left_neighbor_lane_borrowable is output that left neighbor lane can
     * be borrowable will be true
     * @param right_neighbor_lane_borrowable is output  that right neighbor lane
     * can be borrowable will be true
     */
    void CheckLaneBorrow(
            const ReferenceLineInfo& reference_line_info,
            bool* left_neighbor_lane_borrowable,
            bool* right_neighbor_lane_borrowable);
    /**
     * @brief Check whether neighbor lane is borrowable according to boudary type,
     * if solid line can not be borrowed
     * @param reference_line_info is input reference line info
     * @param check_s is check s position boundary type
     * @param lane_borrow_info is borrow side.
     */
    bool CheckLaneBoundaryType(
            const ReferenceLineInfo& reference_line_info,
            const double check_s,
            const SidePassDirection& lane_borrow_info);
    /**
     * @brief Set path point decision guide info which describe
     * @param reference_line_info is input reference line info
     * @param check_s is check s position boundary type
     * @param lane_borrow_info is borrow side.
     */
    void SetPathInfo(PathData* const path_data);

    void ConstructDecision(
            const ReferenceLineInfo& reference_line_info,
            std::vector<SLPolygon>* const sl_polygon,
            PathBoundary* const path_boundary);
    /**
     * @brief Generate a path boundary spanning all available lanes
     *        (both left and right neighbors) for construction zone passage.
     */
    void GetConstructZoneBoundary(PathBoundary* const path_bound);
    /**
     * @brief Decide path bounds for construction zone mode,
     *        generating a single bidirectional boundary.
     */
    bool DecideConstructZoneBoundary(std::vector<PathBoundary>* boundary);
    /**
     * @brief Compute the navigable corridor for construction zone by finding
     *        the widest gap between cones at each S point, replacing the
     *        per-cone nudge approach for scenarios where cones form S-curves.
     */
    void ComputeConstructZoneBoundary(
            std::vector<SLPolygon>& cones,
            const std::unordered_map<std::string, std::pair<double, double>>& cone_xy,
            PathBoundary* const path_bound);
    LaneBorrowPathConfig config_;
    std::vector<SidePassDirection> decided_side_pass_direction_;
    int use_self_lane_;
    std::string blocking_obstacle_id_;
    bool construct_zone;
    // 锥桶位置记忆：记录所有出现过的锥桶 (s, l) 坐标，用于闪烁容错
    std::vector<std::pair<double, double>> cone_history_;
    // 锥桶墙追踪：S 弯两侧锥桶分别形成"左墙"和"右墙"，
    // 边界收缩到墙线而非单个锥桶，容忍闪烁
    std::vector<std::pair<double, double>> left_wall_;   // (s, l) 按 s 排序
    std::vector<std::pair<double, double>> right_wall_;  // (s, l) 按 s 排序
    double zone_left_base;
    double zone_right_base;
    std::unordered_map<std::string, bool> construct_decision;
    bool U_turn_construct;
    std::unique_ptr<PathData> lastframe_;
    double mx_left_bound = 0.0;
    double mx_right_bound = 0.0;

    // XY位置→墙分类记忆：用1m网格Hash，消除SL投影变化导致的分类闪烁
    // key = ((int64_t)round(x) << 32) | (uint32_t)round(y)
    // value: true=左墙, false=右墙
    std::unordered_map<uint64_t, bool> cone_wall_memory_;
};

/////////////////////////////////////////////////////////////////////////////
// Below are helper functions.

int ContainsOutOnReverseLane(
        const std::vector<std::tuple<double, PathData::PathPointType, double>>& path_point_decision);

int GetBackToInLaneIndex(const std::vector<std::tuple<double, PathData::PathPointType, double>>& path_point_decision);

bool ComparePathData(const PathData& lhs, const PathData& rhs, const Obstacle* blocking_obstacle);

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneBorrowPath, Task)
}  // namespace planning
}  // namespace apollo
