/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
#define CARTOGRAPHER_MAPPING_2D_GRID_2D_H_

#include <vector>

#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/grid_interface.h"
#include "cartographer/mapping/proto/2d/grid_2d.pb.h"
#include "cartographer/mapping/proto/2d/submaps_options_2d.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"

// 2D的概率栅格
namespace cartographer {
namespace mapping {

proto::GridOptions2D CreateGridOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary);

class Grid2D : public GridInterface {
 public:
 // MapLimits定义在/mapping/2d/map_limits.h
 // MapLimits包含三个成员变量：double resolution_;分辨率 程序中设置是0.05m.也就是5cm。
 //                         Eigen::Vector2d max_;这是一个浮点型二维向量，max_.x()和.y()分别表示x、y方向的最大值
 //                         CellLimits cell_limits_;栅格化后的x和y方向的最大范围
 // 划分栅格
  explicit Grid2D(const MapLimits& limits, float min_correspondence_cost,
                  float max_correspondence_cost);
  explicit Grid2D(const proto::Grid2D& proto);

  // Returns the limits of this Grid2D.
  // 返回该栅格地图的范围、分辨率等
  const MapLimits& limits() const { return limits_; }

  // Finishes the update sequence.
  // 停止更新
  void FinishUpdate();
  // Returns the correspondence cost of the cell with 'cell_index'.
  // 返回一个坐标的CorrespondenceCost值
  float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const;

  // Returns the minimum possible correspondence cost.
  // 获取最小值

  float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

  // Returns the maximum possible correspondence cost.
  
  float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

  // Returns true if the probability at the specified index is known.
  // 判断一个pixel是否已经有相应的概率值
  bool IsKnown(const Eigen::Array2i& cell_index) const;

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  // 进行一下裁剪。裁剪一个subregion，使得该subregion包含了所有的已有概率值的cells
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const;

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  // 必要时grow我们的submap
  virtual void GrowLimits(const Eigen::Vector2f& point);
  // 得到一个裁剪后的栅格图
  virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const = 0;
  
  virtual proto::Grid2D ToProto() const;

  virtual bool DrawToSubmapTexture(
      proto::SubmapQuery::Response::SubmapTexture* const texture,
      transform::Rigid3d local_pose) const = 0;

 protected:
 // 返回记录栅格地图概率值的向量
  const std::vector<uint16>& correspondence_cost_cells() const {
    return correspondence_cost_cells_;
  }
  // 更新索引
  const std::vector<int>& update_indices() const { return update_indices_; }
  // 返回一个已知概率值的区域
  const Eigen::AlignedBox2i& known_cells_box() const {
    return known_cells_box_;
  }

  std::vector<uint16>* mutable_correspondence_cost_cells() {
    return &correspondence_cost_cells_;
  }
  std::vector<int>* mutable_update_indices() { return &update_indices_; }
  Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }

  // Converts a 'cell_index' into an index into 'cells_'.
  // 将pixel坐标再转化为局部坐标系中的点的坐标
  int ToFlatIndex(const Eigen::Array2i& cell_index) const;

 private:
  MapLimits limits_;
  std::vector<uint16> correspondence_cost_cells_;
  float min_correspondence_cost_;
  float max_correspondence_cost_;
  std::vector<int> update_indices_;

  // Bounding box of known cells to efficiently compute cropping limits.
  Eigen::AlignedBox2i known_cells_box_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
