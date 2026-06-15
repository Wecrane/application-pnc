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
#include <unordered_set>
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
    bool IsConstructionStraightRoad() const;
    void ResetConstructZoneState(const std::string& reason);
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
    bool construct_zone = false;
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

    // 已分类锥桶的XY坐标（用于模糊分类时的近邻投票）
    std::vector<std::pair<double, double>> classified_left_xy_;
    std::vector<std::pair<double, double>> classified_right_xy_;

    // 施工区退出：连续无锥桶帧计数器，超过阈值强制退出
    int no_cone_counter_ = 0;
    // 施工区退出滞回：total_cone_estimate < 1 的连续帧计数，防止锥桶闪烁导致中途退出
    int low_cone_counter_ = 0;
    // 施工区最后已知锥桶 s，用于尾段保活，避免只剩最后几个锥桶时退出后重进
    double construct_zone_farthest_cone_s_ = -1.0;
    // 施工区最后已知锥桶 XY。尾段 SL 投影会跳变，用真实 XY 再做一层保活。
    double construct_zone_farthest_cone_x_ = -1.0;
    double construct_zone_farthest_cone_y_ = 0.0;
    // 施工区内已被 path boundary 纳入横向绕行的大障碍物，后续只忽略纵向 stop。
    std::unordered_set<std::string> construct_zone_nudge_obstacle_ids_;
    static constexpr int kLowConeExitThreshold = 30;  // 3秒@10Hz，锥桶持续消失才退出

    // ── 倒车恢复：车辆起始位置被锥桶卡死无法前进时，先倒车拉开距离 ──
    bool in_reverse_ = false;                            // 当前处于倒车模式
    int reverse_frame_count_ = 0;                        // 倒车已执行帧数 / 卡死帧计数
    double reverse_start_s_ = 0.0;                       // 倒车起始 s 坐标
    double reverse_start_x_ = 0.0;                       // 倒车起始真实 x 坐标
    double reverse_start_y_ = 0.0;                       // 倒车起始真实 y 坐标
    bool reverse_path_initialized_ = false;              // 倒车路径是否已固定
    double reverse_fixed_start_s_ = 0.0;                 // 固定倒车路径起点 s
    double reverse_fixed_end_s_ = 0.0;                   // 固定倒车路径终点 s
    double reverse_fixed_l_ = 0.0;                       // 固定倒车路径横向位置
    double reverse_fixed_start_x_ = 0.0;                 // 固定倒车路径起点真实 x
    double reverse_fixed_start_y_ = 0.0;                 // 固定倒车路径起点真实 y
    double reverse_fixed_heading_ = 0.0;                 // 固定倒车直参考线 heading
    PathBoundary reverse_boundary_cache_;                // 固定倒车边界缓存
    std::unique_ptr<ReferenceLine> reverse_reference_line_cache_;  // 固定倒车直参考线缓存
    double last_adc_s_for_stuck_ = 0.0;                  // 上一帧 ADC s 坐标，用于检测位移
    double last_adc_x_for_stuck_ = 0.0;                  // 上一帧 ADC x 坐标，用于检测真实位移
    double last_adc_y_for_stuck_ = 0.0;                  // 上一帧 ADC y 坐标，用于检测真实位移
    static constexpr double kMinReverseDistance = 0.5;   // 倒车目标距离配置下限 (m)
    static constexpr double kMinReverseTargetRemainThreshold = 0.05;  // 终点剩余距离阈值配置下限
    static constexpr int kReverseMinFrames = 10;         // 防止定位/启动瞬间抖动导致倒车刚开始就结束
    static constexpr int kReverseMaxFrames = 250;        // 倒车最大帧数 (25s @ 10Hz)
    static constexpr double kStuckSpeedThreshold = 0.3;  // 判定"卡死"的速度阈值 (m/s)
    static constexpr int kStuckFrameThreshold = 15;      // 连续低速帧数阈值

    // 倒车路径生成：生成一条向后直线倒车路径
    bool GenerateReversePathBoundary(PathBoundary* boundary);
    bool BuildReverseStraightReferenceLine();
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
