/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).

// 定义x和y方向上的平移增加量，然后搜索整个空间，从小到大进行排序，得到低分辨率
class SlidingWindowMaximum {
 public:
  void AddValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  void RemoveValue(const float value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.
  std::deque<float> non_ascending_maxima_;
};

}  // namespace
// 分枝定界
proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions2D options;
  // 定义搜索窗口的长度和角度以及分枝定界算法的深度
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  return options;
}

PrecomputationGrid2D::PrecomputationGrid2D(
    const Grid2D& grid, const CellLimits& limits, const int width,
    std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,  // x向宽度限制
                   limits.num_y_cells + width - 1), // y向宽度限制
      min_score_(1.f - grid.GetMaxCorrespondenceCost()),  // 最小分数
      max_score_(1.f - grid.GetMinCorrespondenceCost()),  // 最大分数
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells)  // 格子的面积
       {
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);
  const int stride = wide_limits_.num_x_cells;


  // 首先计算一个分支下去，遍历所有叶子节点，得到最底层叶子节点的最大值
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  std::vector<float>& intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  for (int y = 0; y != limits.num_y_cells; ++y) 
  {
    SlidingWindowMaximum current_values;
    current_values.AddValue(
        1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(0, y))));
    for (int x = -width + 1; x != 0; ++x) 
    {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      if (x + width < limits.num_x_cells) 
      {
        current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                          Eigen::Array2i(x + width, y))));
      }
    }
    for (int x = 0; x < limits.num_x_cells - width; ++x) 
    {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
      current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                        Eigen::Array2i(x + width, y))));
    }
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) 
    {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
    }
    current_values.CheckIsEmpty();
  }
  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
  // 计算与x同级的y的分数，和刚才的叶子节点的分数作比较
  for (int x = 0; x != wide_limits_.num_x_cells; ++x) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);
    for (int y = -width + 1; y != 0; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
}

uint8 PrecomputationGrid2D::ComputeCellValue(const float probability) const {
  const int cell_value = common::RoundToInt(
      (probability - min_score_) * (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}
// 构造概率栅格金字塔模型，每个金字塔都代表着不同的分辨率
PrecomputationGridStack2D::PrecomputationGridStack2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options) {
  CHECK_GE(options.branch_and_bound_depth(), 1);
  const int max_width = 1 << (options.branch_and_bound_depth() - 1);
  precomputation_grids_.reserve(options.branch_and_bound_depth());
  std::vector<float> reusable_intermediate_grid;
  const CellLimits limits = grid.limits().cell_limits();
  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);
  for (int i = 0; i != options.branch_and_bound_depth(); ++i) {
    const int width = 1 << i;
    precomputation_grids_.emplace_back(grid, limits, width,
                                       &reusable_intermediate_grid);
  }
}

FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options),
      limits_(grid.limits()),
      precomputation_grid_stack_(
          common::make_unique<PrecomputationGridStack2D>(grid, options)) {}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}
// match函数
bool FastCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // 设置搜索参数
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());
  // 对每个参数进行匹配
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

bool FastCorrelativeScanMatcher2D::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // Compute a search window around the center of the submap that includes it
  // fully.
  // 定义搜索参数，使它包含全部的子地图范围
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());
  // 检测搜索窗口的起点，把这个中点作为搜索中心
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));
  // 返回匹配成功的搜索函数
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}

bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  CHECK_NOTNULL(score);
  CHECK_NOTNULL(pose_estimate);
  // 把激光数据进行旋转，转到世界坐标系0的位置
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  // 生成一系列的旋转scan
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  // 把旋转激光数据转到地图坐标系中
  // 这里的离散激光点是在最低分辨率上的
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  // 计算最低分辨率内的所有候选解
  
  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  // 分枝定界
  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      // 栅格金字塔最大的深度，对应找到最低的分数
      precomputation_grid_stack_->max_depth(), min_score);
  
  // 这里就是整个对应论文上的分枝定界算法，如果最好的分支节点分数大于开始设置的最小值
  // 就把这个叶子节点分数赋予给分数这个节点，相应的位姿估计也同样调换
  // 如果计算出来的解大于最小的阈值，就认为匹配成功，返回对应的位姿
  if (best_candidate.score > min_score) {
    *score = best_candidate.score;
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

std::vector<Candidate2D>
// 这个函数就是在计算低分辨率的节点分数
FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const {
    // 计算最低分辨率的候选解
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  // 对每个候选解进行打分
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  // 得到最低分辨率的解
  return lowest_resolution_candidates;
}

std::vector<Candidate2D>

// 产生低分辨率节点
FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}
// 节点的得分函数
void FastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    int sum = 0;
    // 枚举每个候选解，枚举所有激光点，计算累计占用栅格概率
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      // 激光点在地图坐标系的坐标
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }
    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }
  // 进行排序
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());
}
// 分枝定界算法
Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float min_score) const {
  // 如果只有一层，那么刚才最低分辨率的解就是最好的，返回
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }
 // 定义高分辨率节点
  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;
  // 从高到低枚举所有的解
  for (const Candidate2D& candidate : candidates) 
  {
    // 如果某个候选解的分数小于最低的分数，直接剪枝掉
    // 在遍历的过程中，这个最低分数是会不断被更新的
    // 但是只会更新叶子节点的分数，即不会更新到刚刚的候选解分数
    
    if (candidate.score <= min_score)
     {
      break;
    }
    // 开始进行分支
    std::vector<Candidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    // 该节点分解为4个子节点，这里就是分支
    for (int x_offset : {0, half_width})
     // 如果超出搜索范围就不会进行操作了
     {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) 
      // 超出范围就不进行操作
      {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        // 把这个可能存在最优解的候选解放入队列中
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    // 对细分后的解进行打分
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    
    
    // 分支定界，BranchAndBound，下界min_score ，小于该得分的候选粒子直接就pass了。
    // 计算高一层分辨率地图所需要的粒子位置偏移量，然后在高一层分辨率地图中，
    // 再次计算每个粒子的平均得分并排序，最后递归
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  // 返回最好的高分辨率的解
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
