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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

typedef std::vector<Eigen::Array2i> DiscreteScan2D;

// Describes the search space.
// 定义搜索空间
struct SearchParameters {
  // Linear search window in pixel offsets; bounds are inclusive.
  // 线性搜索窗口中的像素偏移； 边界是包容的
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  SearchParameters(double linear_search_window, double angular_search_window,
                   const sensor::PointCloud& point_cloud, double resolution);

  // For testing.
  SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                   double angular_perturbation_step_size, double resolution);

  // Tightens the search window as much as possible.
  // 尽可能的缩小搜索窗口
  void ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                   const CellLimits& cell_limits);

  int num_angular_perturbations;
  double angular_perturbation_step_size;
  double resolution;
  int num_scans;
  std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
};

// Generates a collection of rotated scans.
// 生成旋转扫描的集合
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
// 将旋转的扫描转换并离散化为整数索引的向量。
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
struct Candidate2D {
  Candidate2D(const int init_scan_index, const int init_x_index_offset,
              const int init_y_index_offset,
              const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  int scan_index = 0;

  // Linear offset from the initial pose.
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this Candidate2D relative to the initial pose.
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  // 设置分数，越高越好
  float score = 0.f;

  bool operator<(const Candidate2D& other) const { return score < other.score; }
  bool operator>(const Candidate2D& other) const { return score > other.score; }
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_2D_H_
