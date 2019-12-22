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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

ImuTracker::ImuTracker(const double imu_gravity_time_constant,const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(Eigen::Vector3d::UnitZ()),// 重力加速度
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

// imu数据的随时间变化
void ImuTracker::Advance(const common::Time time)
 {
  CHECK_LE(time_, time);//检查ImuTracker维护的时间time_是否大于等于指定时间time
   // 求指定时间相对当前时间的时间差
  const double delta_t = common::ToSeconds(time - time_);
  // 角速度乘以时间，然后转化成RotationnQuaternion,这是这段时间的姿态变化量
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  // 以当前姿态orientation_为基准，再乘以姿态变化量。得到最新的姿态
  orientation_ = (orientation_ * rotation).normalized();
  // 更新加速度
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  // 更新时间
  time_ = time;
}

// 时间变化之后引起线加速度变化，添加进来
// 根据读数更新线加速度。这里的线加速度是经过重力校正的
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  // 用一定的时间来更新重力矢量，具体来说，应该是z方向上的加速度
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
      // 更新矢量方向
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  // 根据目前的重力方向来更新方向角
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}
// 添加角加速度
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
