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

#ifndef CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_
#define CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_

#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/3d/hybrid_grid.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Converts an 'index' with each dimension from 0 to 2^'bits' - 1 to a flat
// z-major index.
// 将每个维度从0到2^'bits' - 1的“索引”转换为平面z主索引
inline int ToFlatIndex(const Eigen::Array3i& index, const int bits) {
  DCHECK((index >= 0).all() && (index < (1 << bits)).all()) << index;
  return (((index.z() << bits) + index.y()) << bits) + index.x();
}

// Converts a flat z-major 'index' to a 3-dimensional index with each dimension
// from 0 to 2^'bits' - 1.
// 把平面z为主索引转换为以每个维度的索引
inline Eigen::Array3i To3DIndex(const int index, const int bits) {
  DCHECK_LT(index, 1 << (3 * bits));
  const int mask = (1 << bits) - 1;
  return Eigen::Array3i(index & mask, (index >> bits) & mask,
                        (index >> bits) >> bits);
}

// A function to compare value to the default value. (Allows specializations).
// 默认值和现有值的比较函数
template <typename TValueType>
bool IsDefaultValue(const TValueType& v) {
  return v == TValueType();
}

// Specialization to compare a std::vector to the default value.
// 特殊的去比较默认值
template <typename TElementType>
bool IsDefaultValue(const std::vector<TElementType>& v) {
  return v.empty();
}

// A flat grid of '2^kBits' x '2^kBits' x '2^kBits' voxels storing values of
// type 'ValueType' in contiguous memory. Indices in each dimension are 0-based.
// 2^kBits' x '2^kBits' x '2^kBits'体素的平面网格，在连续存储器中存储“ValueType”类型的值。每个维度中的索引都是基于0的
template <typename TValueType, int kBits>
class FlatGrid {
 public:
  using ValueType = TValueType;

  // Creates a new flat grid with all values being default constructed.
  // 用默认值产生一个新的栅格图
  FlatGrid() {
    for (ValueType& value : cells_) {
      value = ValueType();
    }
  }

  FlatGrid(const FlatGrid&) = delete;
  FlatGrid& operator=(const FlatGrid&) = delete;

  // Returns the number of voxels per dimension.
  // 返回每一维度的体素格数量
  static int grid_size() { return 1 << kBits; }

  // Returns the value stored at 'index', each dimension of 'index' being
  // between 0 and grid_size() - 1.
  // 返回每个索引的储存值
  ValueType value(const Eigen::Array3i& index) const {
    return cells_[ToFlatIndex(index, kBits)];
  }

  // Returns a pointer to a value to allow changing it.
  // 定义一个值的指针
  ValueType* mutable_value(const Eigen::Array3i& index) {
    return &cells_[ToFlatIndex(index, kBits)];
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  // 一个递推器，地推所有和默认值不相等的值
  class Iterator {
   public:
    Iterator() : current_(nullptr), end_(nullptr) {}
    // 递推器构造函数
    explicit Iterator(const FlatGrid& flat_grid)
        : current_(flat_grid.cells_.data()),
          end_(flat_grid.cells_.data() + flat_grid.cells_.size()) {
      while (!Done() && IsDefaultValue(*current_)) {
        ++current_;
      }
    }
    // 产生循环
    void Next() {
      DCHECK(!Done());
      do {
        ++current_;
      } while (!Done() && IsDefaultValue(*current_));
    }
    // 完成设定
    bool Done() const { return current_ == end_; }
    // 得到每一个cell的索引
    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int index = (1 << (3 * kBits)) - (end_ - current_);
      return To3DIndex(index, kBits);
    }
    // 得到每一个储存值
    const ValueType& GetValue() const {
      DCHECK(!Done());
      return *current_;
    }

   private:
    const ValueType* current_;
    const ValueType* end_;
  };

 private:
  std::array<ValueType, 1 << (3 * kBits)> cells_;
};

// A grid consisting of '2^kBits' x '2^kBits' x '2^kBits' grids of type
// 'WrappedGrid'. Wrapped grids are constructed on first access via
// 'mutable_value()'.
// 由“WrappedGrid”类型的'2^kBits' x '2^kBits' x '2^kBits'网格组成的网格。
// 包装网格是通过“可变值()”在第一次访问时构造的。
template <typename WrappedGrid, int kBits>
class NestedGrid {
 public:
  using ValueType = typename WrappedGrid::ValueType;

  // Returns the number of voxels per dimension.
  // 返回每一个维度体素的数量
  static int grid_size() { return WrappedGrid::grid_size() << kBits; }

  // Returns the value stored at 'index', each dimension of 'index' being
  // between 0 and grid_size() - 1.
  // 返回每个索引的储存值
  ValueType value(const Eigen::Array3i& index) const {
    const Eigen::Array3i meta_index = GetMetaIndex(index);
    const WrappedGrid* const meta_cell =
        meta_cells_[ToFlatIndex(meta_index, kBits)].get();
    if (meta_cell == nullptr) {
      return ValueType();
    }
    const Eigen::Array3i inner_index =
        index - meta_index * WrappedGrid::grid_size();
    return meta_cell->value(inner_index);
  }

  // Returns a pointer to the value at 'index' to allow changing it. If
  // necessary a new wrapped grid is constructed to contain that value.
  // 构造一个指向索引的值的指针
  ValueType* mutable_value(const Eigen::Array3i& index) {
    const Eigen::Array3i meta_index = GetMetaIndex(index);
    std::unique_ptr<WrappedGrid>& meta_cell =
        meta_cells_[ToFlatIndex(meta_index, kBits)];
    if (meta_cell == nullptr) {
      meta_cell = common::make_unique<WrappedGrid>();
    }
    const Eigen::Array3i inner_index =
        index - meta_index * WrappedGrid::grid_size();
    return meta_cell->mutable_value(inner_index);
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  // 一个递推器，地推所有和默认值不相等的值
  // 和上面相同
  class Iterator {
   public:
    Iterator() : current_(nullptr), end_(nullptr), nested_iterator_() {}
    
    explicit Iterator(const NestedGrid& nested_grid)
        : current_(nested_grid.meta_cells_.data()),
          end_(nested_grid.meta_cells_.data() + nested_grid.meta_cells_.size()),
          nested_iterator_() {
      AdvanceToValidNestedIterator();
    }

    void Next() {
      DCHECK(!Done());
      nested_iterator_.Next();
      if (!nested_iterator_.Done()) {
        return;
      }
      ++current_;
      AdvanceToValidNestedIterator();
    }

    bool Done() const { return current_ == end_; }

    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int index = (1 << (3 * kBits)) - (end_ - current_);
      return To3DIndex(index, kBits) * WrappedGrid::grid_size() +
             nested_iterator_.GetCellIndex();
    }

    const ValueType& GetValue() const {
      DCHECK(!Done());
      return nested_iterator_.GetValue();
    }

   private:
    void AdvanceToValidNestedIterator() {
      for (; !Done(); ++current_) {
        if (*current_ != nullptr) {
          nested_iterator_ = typename WrappedGrid::Iterator(**current_);
          if (!nested_iterator_.Done()) {
            break;
          }
        }
      }
    }

    const std::unique_ptr<WrappedGrid>* current_;
    const std::unique_ptr<WrappedGrid>* end_;
    typename WrappedGrid::Iterator nested_iterator_;
  };

 private:
  // Returns the Eigen::Array3i (meta) index of the meta cell containing
  // 'index'.
  // 返回包含索引值的向量
  Eigen::Array3i GetMetaIndex(const Eigen::Array3i& index) const {
    DCHECK((index >= 0).all()) << index;
    const Eigen::Array3i meta_index = index / WrappedGrid::grid_size();
    DCHECK((meta_index < (1 << kBits)).all()) << index;
    return meta_index;
  }

  std::array<std::unique_ptr<WrappedGrid>, 1 << (3 * kBits)> meta_cells_;
};

// A grid consisting of 2x2x2 grids of type 'WrappedGrid' initially. Wrapped
// grids are constructed on first access via 'mutable_value()'. If necessary,
// the grid grows to twice the size in each dimension. The range of indices is
// (almost) symmetric around the origin, i.e. negative indices are allowed.
// 最初由2x2x2“包裹网格”类型的网格组成的网格。包装网格是通过“可变值()”在第一次访问时构造的。
// 如有必要，网格会在每个维度中增长到两倍大小。指数的范围(几乎)围绕原点对称，即允许负指数。
template <typename WrappedGrid>
// 定义一个动态栅格类，同上
class DynamicGrid {
 public:
  using ValueType = typename WrappedGrid::ValueType;

  DynamicGrid() : bits_(1), meta_cells_(8) {}
  DynamicGrid(DynamicGrid&&) = default;
  DynamicGrid& operator=(DynamicGrid&&) = default;

  // Returns the current number of voxels per dimension.
  int grid_size() const { return WrappedGrid::grid_size() << bits_; }

  // Returns the value stored at 'index'.
  ValueType value(const Eigen::Array3i& index) const {
    const Eigen::Array3i shifted_index = index + (grid_size() >> 1);
    // The cast to unsigned is for performance to check with 3 comparisons
    // shifted_index.[xyz] >= 0 and shifted_index.[xyz] < grid_size.
    if ((shifted_index.cast<unsigned int>() >= grid_size()).any()) {
      return ValueType();
    }
    const Eigen::Array3i meta_index = GetMetaIndex(shifted_index);
    const WrappedGrid* const meta_cell =
        meta_cells_[ToFlatIndex(meta_index, bits_)].get();
    if (meta_cell == nullptr) {
      return ValueType();
    }
    const Eigen::Array3i inner_index =
        shifted_index - meta_index * WrappedGrid::grid_size();
    return meta_cell->value(inner_index);
  }

  // Returns a pointer to the value at 'index' to allow changing it, dynamically
  // growing the DynamicGrid and constructing new WrappedGrids as needed.
  ValueType* mutable_value(const Eigen::Array3i& index) {
    const Eigen::Array3i shifted_index = index + (grid_size() >> 1);
    // The cast to unsigned is for performance to check with 3 comparisons
    // shifted_index.[xyz] >= 0 and shifted_index.[xyz] < grid_size.
    if ((shifted_index.cast<unsigned int>() >= grid_size()).any()) {
      Grow();
      return mutable_value(index);
    }
    const Eigen::Array3i meta_index = GetMetaIndex(shifted_index);
    std::unique_ptr<WrappedGrid>& meta_cell =
        meta_cells_[ToFlatIndex(meta_index, bits_)];
    if (meta_cell == nullptr) {
      meta_cell = common::make_unique<WrappedGrid>();
    }
    const Eigen::Array3i inner_index =
        shifted_index - meta_index * WrappedGrid::grid_size();
    return meta_cell->mutable_value(inner_index);
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  class Iterator {
   public:
    explicit Iterator(const DynamicGrid& dynamic_grid)
        : bits_(dynamic_grid.bits_),
          current_(dynamic_grid.meta_cells_.data()),
          end_(dynamic_grid.meta_cells_.data() +
               dynamic_grid.meta_cells_.size()),
          nested_iterator_() {
      AdvanceToValidNestedIterator();
    }

    void Next() {
      DCHECK(!Done());
      nested_iterator_.Next();
      if (!nested_iterator_.Done()) {
        return;
      }
      ++current_;
      AdvanceToValidNestedIterator();
    }

    bool Done() const { return current_ == end_; }

    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int outer_index = (1 << (3 * bits_)) - (end_ - current_);
      const Eigen::Array3i shifted_index =
          To3DIndex(outer_index, bits_) * WrappedGrid::grid_size() +
          nested_iterator_.GetCellIndex();
      return shifted_index - ((1 << (bits_ - 1)) * WrappedGrid::grid_size());
    }

    const ValueType& GetValue() const {
      DCHECK(!Done());
      return nested_iterator_.GetValue();
    }

    void AdvanceToEnd() { current_ = end_; }

    const std::pair<Eigen::Array3i, ValueType> operator*() const {
      return std::pair<Eigen::Array3i, ValueType>(GetCellIndex(), GetValue());
    }

    Iterator& operator++() {
      Next();
      return *this;
    }

    bool operator!=(const Iterator& it) const {
      return it.current_ != current_;
    }

   private:
    void AdvanceToValidNestedIterator() {
      for (; !Done(); ++current_) {
        if (*current_ != nullptr) {
          nested_iterator_ = typename WrappedGrid::Iterator(**current_);
          if (!nested_iterator_.Done()) {
            break;
          }
        }
      }
    }

    int bits_;
    const std::unique_ptr<WrappedGrid>* current_;
    const std::unique_ptr<WrappedGrid>* const end_;
    typename WrappedGrid::Iterator nested_iterator_;
  };

 private:
  // Returns the Eigen::Array3i (meta) index of the meta cell containing
  // 'index'.
  Eigen::Array3i GetMetaIndex(const Eigen::Array3i& index) const {
    DCHECK((index >= 0).all()) << index;
    const Eigen::Array3i meta_index = index / WrappedGrid::grid_size();
    DCHECK((meta_index < (1 << bits_)).all()) << index;
    return meta_index;
  }

  // Grows this grid by a factor of 2 in each of the 3 dimensions.
  // 对三维成长两倍
  void Grow() {
    const int new_bits = bits_ + 1;
    CHECK_LE(new_bits, 8);
    std::vector<std::unique_ptr<WrappedGrid>> new_meta_cells_(
        8 * meta_cells_.size());
    // 遍历每个栅格
    for (int z = 0; z != (1 << bits_); ++z) {
      for (int y = 0; y != (1 << bits_); ++y) {
        for (int x = 0; x != (1 << bits_); ++x) {
          const Eigen::Array3i original_meta_index(x, y, z);
          const Eigen::Array3i new_meta_index =
              original_meta_index + (1 << (bits_ - 1));
          new_meta_cells_[ToFlatIndex(new_meta_index, new_bits)] =
              std::move(meta_cells_[ToFlatIndex(original_meta_index, bits_)]);
        }
      }
    }
    meta_cells_ = std::move(new_meta_cells_);
    bits_ = new_bits;
  }

  int bits_;
  std::vector<std::unique_ptr<WrappedGrid>> meta_cells_;
};

template <typename ValueType>
using GridBase = DynamicGrid<NestedGrid<FlatGrid<ValueType, 3>, 3>>;

// Represents a 3D grid as a wide, shallow tree.
// 代表一个3D栅格地图
template <typename ValueType>
class HybridGridBase : public GridBase<ValueType> {
 public:
  using Iterator = typename GridBase<ValueType>::Iterator;

  // Creates a new tree-based probability grid with voxels having edge length
  // 'resolution' around the origin which becomes the center of the cell at
  // index (0, 0, 0).
  // 创建一个新的基于树的概率网格，体素在原点周围具有边缘长度“分辨率”，
  // 该原点在索引(0，0，0)处成为单元格的中心。
  explicit HybridGridBase(const float resolution) : resolution_(resolution) {}
  // 分辨率
  float resolution() const { return resolution_; }

  // Returns the index of the cell containing the 'point'. Indices are integer
  // vectors identifying cells, for this the coordinates are rounded to the next
  // multiple of the resolution.
  // 返回包含“点”的单元格的索引。索引是标识单元格的整数向量，为此，坐标被舍入到分辨率的下一个倍数。
  Eigen::Array3i GetCellIndex(const Eigen::Vector3f& point) const {
    Eigen::Array3f index = point.array() / resolution_;
    return Eigen::Array3i(common::RoundToInt(index.x()),
                          common::RoundToInt(index.y()),
                          common::RoundToInt(index.z()));
  }

  // Returns one of the octants, (0, 0, 0), (1, 0, 0), ..., (1, 1, 1).
  static Eigen::Array3i GetOctant(const int i) {
    DCHECK_GE(i, 0);
    DCHECK_LT(i, 8);
    return Eigen::Array3i(static_cast<bool>(i & 1), static_cast<bool>(i & 2),
                          static_cast<bool>(i & 4));
  }

  // Returns the center of the cell at 'index'.
  // 返回cell在索引的中心
  Eigen::Vector3f GetCenterOfCell(const Eigen::Array3i& index) const {
    return index.matrix().cast<float>() * resolution_;
  }

  // Iterator functions for range-based for loops.
  // 进行递推
  Iterator begin() const { return Iterator(*this); }

  Iterator end() const {
    Iterator it(*this);
    it.AdvanceToEnd();
    return it;
  }

 private:
  // Edge length of each voxel.
  // 每个体素的边缘
  const float resolution_;
};

// A grid containing probability values stored using 15 bits, and an update
// marker per voxel.
// Points are expected to be close to the origin. Points far from the origin
// require the grid to grow dynamically. For centimeter resolution, points
// can only be tens of meters from the origin.
// The hard limit of cell indexes is +/- 8192 around the origin.
// 包含使用15位存储的概率值的网格，以及每个体素的更新标记。点应该靠近原点。
// 远离原点的点要求网格动态增长。对于厘米分辨率，点只能离原点几十米。单元格索引的硬限制在原点周围+/- 8192
class HybridGrid : public HybridGridBase<uint16> {
 public:
  explicit HybridGrid(const float resolution)
      : HybridGridBase<uint16>(resolution) {}

  explicit HybridGrid(const proto::HybridGrid& proto)
      : HybridGrid(proto.resolution()) {
        // 检查x，y，z的范围大小
    CHECK_EQ(proto.values_size(), proto.x_indices_size());
    CHECK_EQ(proto.values_size(), proto.y_indices_size());
    CHECK_EQ(proto.values_size(), proto.z_indices_size());
    for (int i = 0; i < proto.values_size(); ++i) {
      // SetProbability does some error checking for us.
      // 设置可操作性为我们做一些错误检查
      SetProbability(Eigen::Vector3i(proto.x_indices(i), proto.y_indices(i),
                                     proto.z_indices(i)),
                     ValueToProbability(proto.values(i)));
    }
  }

  // Sets the probability of the cell at 'index' to the given 'probability'.
  // 将“索引”处单元格的概率设置为给定的“概率”
  void SetProbability(const Eigen::Array3i& index, const float probability) {
    *mutable_value(index) = ProbabilityToValue(probability);
  }

  // Finishes the update sequence.
  // 完成更新
  void FinishUpdate() {
    while (!update_indices_.empty()) {
      DCHECK_GE(*update_indices_.back(), kUpdateMarker);
      *update_indices_.back() -= kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'index' if the cell has not already been
  // updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  // 如果单元格尚未更新，则将调用ComputeLookupTableToApplyOdds()时指定的“优势”应用于单元格在“索引”处的概率。
  // 在调用完成日期()之前，将忽略同一单元格的多次更新。如果单元格已更新，则返回true。
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  // 检查是否为第一次呼叫
  bool ApplyLookupTable(const Eigen::Array3i& index,
                        const std::vector<uint16>& table) {
    DCHECK_EQ(table.size(), kUpdateMarker);
    uint16* const cell = mutable_value(index);
    if (*cell >= kUpdateMarker) {
      return false;
    }
    update_indices_.push_back(cell);
    *cell = table[*cell];
    DCHECK_GE(*cell, kUpdateMarker);
    return true;
  }

  // Returns the probability of the cell with 'index'.
  // 返回cell的可能值带有索引
  float GetProbability(const Eigen::Array3i& index) const {
    return ValueToProbability(value(index));
  }

  // Returns true if the probability at the specified 'index' is known.
  //  如果是已知的返回结果
  bool IsKnown(const Eigen::Array3i& index) const { return value(index) != 0; }

  proto::HybridGrid ToProto() const {
    CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                      "not supported. Finish the update first.";
    proto::HybridGrid result;
    result.set_resolution(resolution());
    for (const auto it : *this) {
      result.add_x_indices(it.first.x());
      result.add_y_indices(it.first.y());
      result.add_z_indices(it.first.z());
      result.add_values(it.second);
    }
    return result;
  }

 private:
  // Markers at changed cells.
  // 标记已经改变的cell
  std::vector<ValueType*> update_indices_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_