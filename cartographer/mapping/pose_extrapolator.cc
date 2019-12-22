/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "cartographer/common/make_unique.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
// 构造函数
PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}
// 初始化位姿
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) 
    {
       // 先生成一个PoseExtrapolator的变量
  auto extrapolator = common::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  /*
    下面的意思是把imu数据加入到外推器中来，调用了imu_tracker数据
  */
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      common::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);
  // 得到位姿
  // 添加一个Pose。从这儿来看，Pose队列并不是从ScanMatching而来的，而是从Imu测量得到的
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}
// 返回Pose队列的最新时间
common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}
// 得到最后一次位姿外推的时间
common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}

// 添加位姿，下面是一些判断语句，来判断imu数据是否为空
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  if (imu_tracker_ == nullptr) {// 如果ImuTracker还没有建立，则需要建立一个ImuTracker
    common::Time tracker_start = time;
    if (!imu_data_.empty()) {// 如果IMU数据队列不为空，则以当前时间和IMU数据中的最早时刻的较小值为初始时刻建立一个ImuTracker
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    //　生成ImuTracker
    imu_tracker_ =
        common::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
// 运行时间记录imu数据
  timed_pose_queue_.ush_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  // 对齐imu数据和里程计数据
  UpdateVelocitiesFromPoses();
  AdvanceImuTracker(time, imu_tracker_.get());
  TrimImuData();
  TrimOdometryData();
  odometry_imu_tracker_ = common::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = common::make_unique<ImuTracker>(*imu_tracker_);
}
// 添加imu数据
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();
}
// 添加里程计数据，判断里程计数据是否为空
void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);//里程计数据时间是否晚于Pose队列的最新时间
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();//删去过期数据
  if (odometry_data_.size() < 2) {//如果里程计数据小于2，则添加
    return;
  }


  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  // 如果里程计数据大于2了，则我们可以根据里程计数据也进行一下状态估计
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  // 计算差值时间
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  // 计算姿态变化量
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  // 姿态变化量除以时间得到角速度
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
   // 解算从里程计得到的线速度
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}
// 推算指定时刻的Pose
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  // 取出Pose队列中的最新值
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    const Eigen::Vector3d translation =
    // 最新translation等于最新Pose的translation再加上这段时间的位移增长量
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}
// 估计重力，运行时间得到imu数据
Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}
// 用位姿来更新速度
void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  // 判断时间，用来得到线速度和角速度
  CHECK(!timed_pose_queue_.empty());
  // 取出队列最末尾的一个Pose,也就是最新时间点的Pose,并记录相应的时间
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  // 取出队列最开头的一个Pose，也就是最旧时间点的Pose,并记录相应的时间
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  // 两者的时间差
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  // 如果时间差小于1ms，则估计不准，弹出警告信息
  if (queue_delta < 0.001) {  // 1 ms
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " ms";
    return;
  }
  // 获取两个时刻各自的Pose
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  // 计算线速度
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  // 计算角速度
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}
// 修剪imu数据
void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}
// 修建里程计数据
void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}
// 更新imu数据
// 从IMU数据队列中取出最新的数据，更新ImuTracker的状态到指定的时间time
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    // 如果IMU数据队列为空，或当前时间要比IMU数据队列中最早期的时间还要早。说明没有可用的IMU数据。
    // 这时候根据时间推算。对于角速度，是用从Pose中估计的角速度或从里程计获得的角速度更新ImuTracker的角速度
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}
// 结算姿态的变化量

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());//检查指定时间是否大于等于ImuTracker的时间
  AdvanceImuTracker(time, imu_tracker);//更新ImuTracker到指定的time
  // 上一时刻的姿态
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  // 求取姿态变化量：上一时刻姿态的逆乘以当前的姿态
  return last_orientation.inverse() * imu_tracker->orientation();
}
// 解算位移的增长量
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  // 判断里程计数据，用新的里程计数据来更新线速度
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  // 假如里程计不满足条件，通过位姿来得到线速度
  return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace mapping
}  // namespace cartographer
