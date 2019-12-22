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

#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

std::vector<Candidate2D>
// 生成所有的解
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  
  // num scan 代表搜索次数
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    // 计算x向的候选解
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    // 计算y向的候选解
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);

    // 累加候选解的个数
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  
  // 用三层for循环，得到所有的候选解
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}
// 匹配，没有多分辨率，单纯枚举
double RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud,
    const ProbabilityGrid& probability_grid,
    transform::Rigid2d* pose_estimate) const {
  CHECK_NOTNULL(pose_estimate);
  // 初始位姿
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  const SearchParameters search_parameters(
      options_.linear_search_window(), options_.angular_search_window(),
      rotated_point_cloud, probability_grid.limits().resolution());
  // 把图像进行离散化
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      probability_grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  
  std::vector<Candidate2D> candidates =
     // 生成所有的解
      GenerateExhaustiveSearchCandidates(search_parameters);
  // 对所有的解进行打分，得到最优的解
  ScoreCandidates(probability_grid, discrete_scans, search_parameters,
                  &candidates);
  // 找到最大的候选解
  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  // 候选解的位姿就是优化之后的位姿
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  // 返回最优得解
  return best_candidate.score;
}
// 给所有的解进行评分
void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const ProbabilityGrid& probability_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    candidate.score = 0.f;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
     // 单纯相加概率栅格内的所有的点
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
    // 计算概率栅格的累加和
      const float probability =
          probability_grid.GetProbability(proposed_xy_index);
      candidate.score += probability;
    }
    // 进行平均
    candidate.score /=
        static_cast<float>(discrete_scans[candidate.scan_index].size());
    // 进行惩罚，离初始值越远，惩罚越严重
    candidate.score *=
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight() +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight()));
    CHECK_GT(candidate.score, 0.f);
  }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
