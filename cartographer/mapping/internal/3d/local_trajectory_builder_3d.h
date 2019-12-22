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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H_

#include <chrono>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/real_time_correlative_scan_matcher_3d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/3d/local_trajectory_builder_options_3d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// 构建local SLAM的方法，这里是没有闭环检测的
class LocalTrajectoryBuilder3D {

 public:
 // 创建结构体激光插入结果
  struct InsertionResult {
      // 一个是轨迹节点数据
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data;
    // 另外一个就是产生的子地图向量
    std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps;
  };
  // 定义匹配结果的结构体
  struct MatchingResult {
    common::Time time;// 时间
    transform::Rigid3d local_pose;// 坐标转换，从世界坐标系下转换到地图坐标系中
    sensor::RangeData range_data_in_local;// 在局部范围内的范围数据
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;// 插入结果的智能指针
  };
  // 构造函数
  explicit LocalTrajectoryBuilder3D(
      // 局部轨迹参数配置
      const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
      // 插入的激光数据id向量
      const std::vector<std::string>& expected_range_sensor_ids);
  ~LocalTrajectoryBuilder3D();

  LocalTrajectoryBuilder3D(const LocalTrajectoryBuilder3D&) = delete;
  LocalTrajectoryBuilder3D& operator=(const LocalTrajectoryBuilder3D&) = delete;
  // 添加imu数据，这是必须要用到的
  void AddImuData(const sensor::ImuData& imu_data);
  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'.  `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  // 添加范围数据，通俗来讲也就是激光数据吧
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  // 添加里程计数据
  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
 // 指向聚集激光数据的智能指针
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time,
      const sensor::RangeData& filtered_range_data_in_tracking);
 // 指向正在形成的智能指针
  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& filtered_range_data_in_local,
      const sensor::RangeData& filtered_range_data_in_tracking,
      const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
      const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  const mapping::proto::LocalTrajectoryBuilderOptions3D options_;
  // 这里是和2d是一样的，定义一个acive submap
  mapping::ActiveSubmaps3D active_submaps_;
  // 运动滤波
  mapping::MotionFilter motion_filter_;
  // real time CSM，计算初值
  std::unique_ptr<scan_matching::RealTimeCorrelativeScanMatcher3D>
      real_time_correlative_scan_matcher_;
  // ceres scan来计算优化
  std::unique_ptr<scan_matching::CeresScanMatcher3D> ceres_scan_matcher_;
  // 指向外推差值器的指针
  std::unique_ptr<mapping::PoseExtrapolator> extrapolator_;
  // 初始化激光数据为0
  int num_accumulated_ = 0;
  sensor::RangeData accumulated_range_data_;
  std::chrono::steady_clock::time_point accumulation_started_;

  RangeDataCollator range_data_collator_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H_
