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

#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/proto/3d/local_trajectory_builder_options_3d.pb.h"
#include "cartographer/mapping/proto/3d/submaps_options_3d.pb.h"
#include "cartographer/mapping/proto/scan_matching//ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/mapping/proto/scan_matching//real_time_correlative_scan_matcher_options.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();
// 构造函数
LocalTrajectoryBuilder3D::LocalTrajectoryBuilder3D(
    const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),// 正在形成的submap
      motion_filter_(options.motion_filter_options()),// 运动滤波
      real_time_correlative_scan_matcher_(// real time CSM
          common::make_unique<scan_matching::RealTimeCorrelativeScanMatcher3D>(
              options_.real_time_correlative_scan_matcher_options())),
      ceres_scan_matcher_( // ceres scan match
          common::make_unique<scan_matching::CeresScanMatcher3D>(
              options_.ceres_scan_matcher_options())),
      accumulated_range_data_{Eigen::Vector3f::Zero(), {}, {}},// 插入的激光数据向量
      range_data_collator_(expected_range_sensor_ids) {}// 机关数据整理

LocalTrajectoryBuilder3D::~LocalTrajectoryBuilder3D() {}
// 添加imu数据的构造函数
void LocalTrajectoryBuilder3D::AddImuData(const sensor::ImuData& imu_data) {
  // 首先检验外推插值器中的imu数据
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  // 定义测量速度的时间
  // 用imu数据计算得到速度信息
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant(), imu_data);
}
// 定义指向match结果的智能指针
std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
// 添加范围数据的构造函数
LocalTrajectoryBuilder3D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  // 定义一个指针
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  // 如果范围collator里面的数据为空，则返回空指针
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }
   // 取到时间
  const common::Time& time = synchronized_data.time;
  // 如果外推差值器为空，说明imu还没初始化，返回空指针
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return nullptr;
  }
  // 检查范围数据是否为空，以及限制的时间
  CHECK(!synchronized_data.ranges.empty());
  CHECK_LE(synchronized_data.ranges.back().point_time[3], 0.f);
  // 计算相隔时间
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time[3]);
      // 如果第一帧时间小于插值器获得的最新的位姿时间，返回空指针
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  if (num_accumulated_ == 0) {
    accumulation_started_ = std::chrono::steady_clock::now();
  }
  // 对于击中的激光数据进行体素滤波
  std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> hits =
      sensor::VoxelFilter(0.5f * options_.voxel_filter_size())
          .Filter(synchronized_data.ranges);

  std::vector<transform::Rigid3f> hits_poses;
  hits_poses.reserve(hits.size());
  bool warned = false;
  // 检查每个hit的激光数据
  for (const auto& hit : hits) {
    common::Time time_point = time + common::FromSeconds(hit.point_time[3]);
    // 判断时间点
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      // 改变位姿插值器的时间
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      // 再把插值器时间给到时间点数据
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    // 得到hit激光数据的位姿
    hits_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }
  // 判断是佛初始化
  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is not used.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }
  // 遍历每一个hit的激光数据
  for (size_t i = 0; i < hits.size(); ++i) {
    // hit激光数据在local里面的向量
    const Eigen::Vector3f hit_in_local =
        hits_poses[i] * hits[i].point_time.head<3>();
    // 初始激光数据在local里面的向量
    const Eigen::Vector3f origin_in_local =
        hits_poses[i] * synchronized_data.origins.at(hits[i].origin_index);
    // 计算向量差
    const Eigen::Vector3f delta = hit_in_local - origin_in_local;
    const float range = delta.norm();
    // 如果范围数据在给定的范围之内
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
          // 则计算在内，即为hit
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        // We insert a ray cropped to 'max_range' as a miss for hits beyond the
        // maximum range. This way the free space up to the maximum range will
        // be updated.
        // 否则则为miss
        accumulated_range_data_.misses.push_back(
            origin_in_local + options_.max_range() / range * delta);
      }
    }
  }
  // hit数量++
  ++num_accumulated_;
  // 如果数量大于规定的范围数据
  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    // 初始为0
    num_accumulated_ = 0;
    // 位姿转换
    transform::Rigid3f current_pose =
        extrapolator_->ExtrapolatePose(time).cast<float>();
    // 进行滤波
    const sensor::RangeData filtered_range_data = {
        current_pose.translation(),
        sensor::VoxelFilter(options_.voxel_filter_size())
            .Filter(accumulated_range_data_.returns),
        sensor::VoxelFilter(options_.voxel_filter_size())
            .Filter(accumulated_range_data_.misses)};
    // 返回激光数据，转到AddAccumulatedRangeData
    return AddAccumulatedRangeData(
        time, sensor::TransformRangeData(filtered_range_data,
                                         current_pose.inverse()));
  }
  return nullptr;
}
// 指向match结果
std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
// ADD函数体
LocalTrajectoryBuilder3D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_tracking) {
  // 如果滤波数据为空，返回为空指针
  if (filtered_range_data_in_tracking.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }
 // 位姿转换，把插值器里的最新数据给到位姿预测
  const transform::Rigid3d pose_prediction =
      extrapolator_->ExtrapolatePose(time);
 // 定义正在匹配的子地图指针
  std::shared_ptr<const mapping::Submap3D> matching_submap =
      active_submaps_.submaps().front();
      // 位姿转换初始化ceres 里面的位姿
  transform::Rigid3d initial_ceres_pose =
      matching_submap->local_pose().inverse() * pose_prediction;
      // 高分辨的自适应体素滤波
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());
      // 滤波结果
  const sensor::PointCloud high_resolution_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(filtered_range_data_in_tracking.returns);
  // 如果高分辨率点云数据为空，返回空指针
  if (high_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty high resolution point cloud data.";
    return nullptr;
  }
  // 如果用到下面这种滤波方法（应该是没有用到）
  if (options_.use_online_correlative_scan_matching()) {
    // We take a copy since we use 'initial_ceres_pose' as an output argument.
    const transform::Rigid3d initial_pose = initial_ceres_pose;
    double score = real_time_correlative_scan_matcher_->Match(
        initial_pose, high_resolution_point_cloud_in_tracking,
        matching_submap->high_resolution_hybrid_grid(), &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }
  // 位姿转换
  transform::Rigid3d pose_observation_in_submap;
  ceres::Solver::Summary summary;
  // 低分辨率滤波
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      options_.low_resolution_adaptive_voxel_filter_options());
      // 低分辨率滤波结果
  const sensor::PointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(
          filtered_range_data_in_tracking.returns);
          // 如果数据为空，返回空指针
  if (low_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty low resolution point cloud data.";
    return nullptr;
  }
  // 进行匹配
  ceres_scan_matcher_->Match(
      (matching_submap->local_pose().inverse() * pose_prediction).translation(),
      initial_ceres_pose,
      // 这是不同的参数配置
      {{&high_resolution_point_cloud_in_tracking,
        &matching_submap->high_resolution_hybrid_grid()},
       {&low_resolution_point_cloud_in_tracking,
        &matching_submap->low_resolution_hybrid_grid()}},
      &pose_observation_in_submap, &summary);
  kCeresScanMatcherCostMetric->Observe(summary.final_cost);
  // 距离差
  double residual_distance = (pose_observation_in_submap.translation() -
                              initial_ceres_pose.translation())
                                 .norm();
  kScanMatcherResidualDistanceMetric->Observe(residual_distance);
  // 角度差
  double residual_angle = pose_observation_in_submap.rotation().angularDistance(
      initial_ceres_pose.rotation());
  kScanMatcherResidualAngleMetric->Observe(residual_angle);
  // 位姿转换，把预估位姿给到local位姿
  const transform::Rigid3d pose_estimate =
      matching_submap->local_pose() * pose_observation_in_submap;
  extrapolator_->AddPose(time, pose_estimate);
  // 查询重力数据
  const Eigen::Quaterniond gravity_alignment =
      extrapolator_->EstimateGravityOrientation(time);
  
  sensor::RangeData filtered_range_data_in_local = sensor::TransformRangeData(
      filtered_range_data_in_tracking, pose_estimate.cast<float>());
  // 插入结果指针定义
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, filtered_range_data_in_local, filtered_range_data_in_tracking,
      high_resolution_point_cloud_in_tracking,
      low_resolution_point_cloud_in_tracking, pose_estimate, gravity_alignment);
  // 时间差
  auto duration = std::chrono::steady_clock::now() - accumulation_started_;
  kLocalSlamLatencyMetric->Set(
      std::chrono::duration_cast<std::chrono::seconds>(duration).count());
  // 返回匹配结果
  return common::make_unique<MatchingResult>(MatchingResult{
      time, pose_estimate, std::move(filtered_range_data_in_local),
      std::move(insertion_result)});
}
// 添加里程计函数体
void LocalTrajectoryBuilder3D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

std::unique_ptr<LocalTrajectoryBuilder3D::InsertionResult>
// 子地图插入函数体定义
LocalTrajectoryBuilder3D::InsertIntoSubmap(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_local,
    const sensor::RangeData& filtered_range_data_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // Querying the active submaps must be done here before calling
  // InsertRangeData() since the queried values are valid for next insertion.
  std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps;
  for (const std::shared_ptr<mapping::Submap3D>& submap :
       active_submaps_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  active_submaps_.InsertRangeData(filtered_range_data_in_local,
                                  gravity_alignment);
  const Eigen::VectorXf rotational_scan_matcher_histogram =
      scan_matching::RotationalScanMatcher::ComputeHistogram(
          sensor::TransformPointCloud(
              filtered_range_data_in_tracking.returns,
              transform::Rigid3f::Rotation(gravity_alignment.cast<float>())),
          options_.rotational_histogram_size());
  return common::make_unique<InsertionResult>(
      InsertionResult{std::make_shared<const mapping::TrajectoryNode::Data>(
                          mapping::TrajectoryNode::Data{
                              time,
                              gravity_alignment,
                              {},  // 'filtered_point_cloud' is only used in 2D.
                              high_resolution_point_cloud_in_tracking,
                              low_resolution_point_cloud_in_tracking,
                              rotational_scan_matcher_histogram,
                              pose_estimate}),
                      std::move(insertion_submaps)});
}

void LocalTrajectoryBuilder3D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_internal_3d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_internal_3d_local_trajectory_builder_scores",
      "Local scan matcher scores", score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_internal_3d_local_trajectory_builder_costs",
      "Local scan matcher costs", cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_internal_3d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
