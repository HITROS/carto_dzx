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

#include "cartographer/mapping/internal/optimization/optimization_problem_3d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/imu_integration.h"
#include "cartographer/mapping/internal/3d/rotation_parameterization.h"
#include "cartographer/mapping/internal/optimization/ceres_pose.h"
#include "cartographer/mapping/internal/optimization/cost_functions/acceleration_cost_function_3d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_3d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/rotation_cost_function_3d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_3d.h"
#include "cartographer/transform/timestamped_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"
#include "ceres/rotation.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using LandmarkNode = ::cartographer::mapping::PoseGraphInterface::LandmarkNode;
using TrajectoryData =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryData;

// For odometry.
// 对于里程计数据进行插值，产生轨迹
std::unique_ptr<transform::Rigid3d> Interpolate(
    const sensor::MapByTime<sensor::OdometryData>& map_by_time,
    const int trajectory_id, const common::Time time) {
  const auto it = map_by_time.lower_bound(trajectory_id, time);// 得到时间
  // 如果时间指向估计的终点，返回空指针
  if (it == map_by_time.EndOfTrajectory(trajectory_id)) {
    return nullptr;
  }
  // 如果时间是在轨迹开始的地方
  if (it == map_by_time.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return common::make_unique<transform::Rigid3d>(it->pose);
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  return common::make_unique<transform::Rigid3d>(
      // 按照时间进行插值
      Interpolate(transform::TimestampedTransform{prev_it->time, prev_it->pose},
                  transform::TimestampedTransform{it->time, it->pose}, time)
          .transform);
}

// For fixed frame pose.
// 对于固定结构位姿来进行插值
std::unique_ptr<transform::Rigid3d> Interpolate(
    const sensor::MapByTime<sensor::FixedFramePoseData>& map_by_time,
    const int trajectory_id, const common::Time time) {
  const auto it = map_by_time.lower_bound(trajectory_id, time);
  if (it == map_by_time.EndOfTrajectory(trajectory_id) ||
      !it->pose.has_value()) {
    return nullptr;
  }
  if (it == map_by_time.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return common::make_unique<transform::Rigid3d>(it->pose.value());
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  if (prev_it->pose.has_value()) {
    return common::make_unique<transform::Rigid3d>(
        Interpolate(transform::TimestampedTransform{prev_it->time,
                                                    prev_it->pose.value()},
                    transform::TimestampedTransform{it->time, it->pose.value()},
                    time)
            .transform);
  }
  return nullptr;
}

// Selects a trajectory node closest in time to the landmark observation and
// applies a relative transform from it.
// 选择一条轨迹节点最接近于观察到的地标，进行位姿转换
transform::Rigid3d GetInitialLandmarkPose(
    const LandmarkNode::LandmarkObservation& observation,
    const NodeSpec3D& prev_node, const NodeSpec3D& next_node,
    const CeresPose& prev_node_pose, const CeresPose& next_node_pose) {
  // 设置插值参数
  const double interpolation_parameter =
      common::ToSeconds(observation.time - prev_node.time) /
      common::ToSeconds(next_node.time - prev_node.time);

  const std::tuple<std::array<double, 4>, std::array<double, 3>>
      rotation_and_translation = InterpolateNodes3D(
          prev_node_pose.rotation(), prev_node_pose.translation(),
          next_node_pose.rotation(), next_node_pose.translation(),
          interpolation_parameter);
  // 返回转换后的地标坐标
  return transform::Rigid3d::FromArrays(std::get<0>(rotation_and_translation),
                                        std::get<1>(rotation_and_translation)) *
         observation.landmark_to_tracking_transform;
}
// 添加地标惩罚函数
void AddLandmarkCostFunctions(
    const std::map<std::string, LandmarkNode>& landmark_nodes,
    bool freeze_landmarks, const MapById<NodeId, NodeSpec3D>& node_data,
    MapById<NodeId, CeresPose>* C_nodes,
    std::map<std::string, CeresPose>* C_landmarks, ceres::Problem* problem) {
  for (const auto& landmark_node : landmark_nodes) {
    // Do not use landmarks that were not optimized for localization.
    // 不要用没有优化的地标
    if (!landmark_node.second.global_landmark_pose.has_value() &&
        freeze_landmarks) {
      continue;
    }
    for (const auto& observation : landmark_node.second.landmark_observations) {
      const std::string& landmark_id = landmark_node.first;
      const auto& begin_of_trajectory =
          node_data.BeginOfTrajectory(observation.trajectory_id);
      // The landmark observation was made before the trajectory was created.
      // 地标在轨迹没有产生之前就已经存在了
      if (observation.time < begin_of_trajectory->data.time) {
        continue;
      }
      // Find the trajectory nodes before and after the landmark observation.
      // 发现轨迹节点，再发现地标
      auto next =
          node_data.lower_bound(observation.trajectory_id, observation.time);
      // The landmark observation was made, but the next trajectory node has
      // not been added yet.
      // 发现的地标已经完成，但是轨迹节点还没添加进来
      if (next == node_data.EndOfTrajectory(observation.trajectory_id)) {
        continue;
      }
      if (next == begin_of_trajectory) {
        next = std::next(next);
      }
      auto prev = std::prev(next);
      // Add parameter blocks for the landmark ID if they were not added before.
      // 为新发现的地标分配参数
      CeresPose* prev_node_pose = &C_nodes->at(prev->id);
      CeresPose* next_node_pose = &C_nodes->at(next->id);
      if (!C_landmarks->count(landmark_id)) {
        const transform::Rigid3d starting_point =
            landmark_node.second.global_landmark_pose.has_value()
                ? landmark_node.second.global_landmark_pose.value()
                // 调用函数，得到地标位姿
                : GetInitialLandmarkPose(observation, prev->data, next->data,
                                         *prev_node_pose, *next_node_pose);
        C_landmarks->emplace(
            landmark_id,
            CeresPose(starting_point, nullptr /* translation_parametrization */,
                      common::make_unique<ceres::QuaternionParameterization>(),
                      problem));
        if (freeze_landmarks) {
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).translation());
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).rotation());
        }
      }
      problem->AddResidualBlock(
          LandmarkCostFunction3D::CreateAutoDiffCostFunction(
              observation, prev->data, next->data),
          nullptr /* loss function */, prev_node_pose->rotation(),
          prev_node_pose->translation(), next_node_pose->rotation(),
          next_node_pose->translation(),
          C_landmarks->at(landmark_id).rotation(),
          C_landmarks->at(landmark_id).translation());
    }
  }
}

}  // namespace
// 3D优化问题构造参数
OptimizationProblem3D::OptimizationProblem3D(
    const optimization::proto::OptimizationProblemOptions& options)
    : options_(options) {}

OptimizationProblem3D::~OptimizationProblem3D() {}
// 添加里程计数据
void OptimizationProblem3D::AddImuData(const int trajectory_id,
                                       const sensor::ImuData& imu_data) {
  imu_data_.Append(trajectory_id, imu_data);
}
// 添加里程计数据
void OptimizationProblem3D::AddOdometryData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  odometry_data_.Append(trajectory_id, odometry_data);
}
// 添加固定结构位姿数据
void OptimizationProblem3D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  fixed_frame_pose_data_.Append(trajectory_id, fixed_frame_pose_data);
}
// 添加轨迹节点
void OptimizationProblem3D::AddTrajectoryNode(const int trajectory_id,
                                              const NodeSpec3D& node_data) {
  node_data_.Append(trajectory_id, node_data);
  trajectory_data_[trajectory_id];
}
// 设置轨迹数据
void OptimizationProblem3D::SetTrajectoryData(
    int trajectory_id, const TrajectoryData& trajectory_data) {
  trajectory_data_[trajectory_id] = trajectory_data;
}
// 插入轨迹节点
void OptimizationProblem3D::InsertTrajectoryNode(const NodeId& node_id,
                                                 const NodeSpec3D& node_data) {
  node_data_.Insert(node_id, node_data);
  trajectory_data_[node_id.trajectory_id];
}
// 修剪轨迹节点
void OptimizationProblem3D::TrimTrajectoryNode(const NodeId& node_id) {
  imu_data_.Trim(node_data_, node_id);
  odometry_data_.Trim(node_data_, node_id);
  fixed_frame_pose_data_.Trim(node_data_, node_id);
  node_data_.Trim(node_id);
  // 如果轨迹中节点数目为0，就清除这条轨迹数据
  if (node_data_.SizeOfTrajectoryOrZero(node_id.trajectory_id) == 0) {
    trajectory_data_.erase(node_id.trajectory_id);
  }
}
// 添加submap
void OptimizationProblem3D::AddSubmap(
    const int trajectory_id, const transform::Rigid3d& global_submap_pose) {
  submap_data_.Append(trajectory_id, SubmapSpec3D{global_submap_pose});
}
// 插入子地图（和上面有什么不一样么？）
void OptimizationProblem3D::InsertSubmap(
    const SubmapId& submap_id, const transform::Rigid3d& global_submap_pose) {
  submap_data_.Insert(submap_id, SubmapSpec3D{global_submap_pose});
}
// 修剪submap
void OptimizationProblem3D::TrimSubmap(const SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}
// 设置最大的递归次数
void OptimizationProblem3D::SetMaxNumIterations(
    const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}
// 解决优化问题
void OptimizationProblem3D::Solve(
    const std::vector<Constraint>& constraints,
    const std::set<int>& frozen_trajectories,
    const std::map<std::string, LandmarkNode>& landmark_nodes) {
  // 如果节点数为0，直接返回
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }
  // 参数配置
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  const auto translation_parameterization =
      [this]() -> std::unique_ptr<ceres::LocalParameterization> {
    return options_.fix_z_in_3d()
               ? common::make_unique<ceres::SubsetParameterization>(
                     3, std::vector<int>{2})
               : nullptr;
  };

  // Set the starting point.
  // 设置开始的点
  CHECK(!submap_data_.empty());// 检查submap数据是否为空
  MapById<SubmapId, CeresPose> C_submaps;// submapid
  MapById<NodeId, CeresPose> C_nodes;// nodeid
  std::map<std::string, CeresPose> C_landmarks;
  bool first_submap = true;
  bool freeze_landmarks = !frozen_trajectories.empty();
  // submapid进行匹配
  for (const auto& submap_id_data : submap_data_) {
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    // 如果是第一帧submap，
    if (first_submap) {
      first_submap = false;
      // Fix the first submap of the first trajectory except for allowing
      // gravity alignment
      // 对submap进行重力同步对齐
            C_submaps.Insert(
          submap_id_data.id,
          CeresPose(submap_id_data.data.global_pose,
                    translation_parameterization(),
                    common::make_unique<ceres::AutoDiffLocalParameterization<
                        ConstantYawQuaternionPlus, 4, 2>>(),
                    &problem));
      problem.SetParameterBlockConstant(
          C_submaps.at(submap_id_data.id).translation());
    } 
    // 如果不是第一帧submap，插入cerespose
    else {
      C_submaps.Insert(
          submap_id_data.id,
          CeresPose(submap_id_data.data.global_pose,
                    translation_parameterization(),
                    common::make_unique<ceres::QuaternionParameterization>(),
                    &problem));
    }
    if (frozen) {
      problem.SetParameterBlockConstant(
          C_submaps.at(submap_id_data.id).rotation());
      problem.SetParameterBlockConstant(
          C_submaps.at(submap_id_data.id).translation());
    }
  }
  // 插入轨迹节点
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    C_nodes.Insert(
        node_id_data.id,
        CeresPose(node_id_data.data.global_pose, translation_parameterization(),
                  common::make_unique<ceres::QuaternionParameterization>(),
                  &problem));
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).rotation());
      problem.SetParameterBlockConstant(
          C_nodes.at(node_id_data.id).translation());
    }
  }
  // Add cost functions for intra- and inter-submap constraints.
  // 添加惩罚函数
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        SpaCostFunction3D::CreateAutoDiffCostFunction(constraint.pose),
        // Only loop closure constraints should have a loss function.
        // 只有回环约束，才会有惩罚函数
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr /* loss function */,
        C_submaps.at(constraint.submap_id).rotation(),// submap的旋转
        C_submaps.at(constraint.submap_id).translation(),// submap的平移
        C_nodes.at(constraint.node_id).rotation(),// 节点的旋转
        C_nodes.at(constraint.node_id).translation());// 节点的平移
  }
  // Add cost functions for landmarks.
  // 给landmark添加惩罚函数
  AddLandmarkCostFunctions(landmark_nodes, freeze_landmarks, node_data_,
                           &C_nodes, &C_landmarks, &problem);
  // Add constraints based on IMU observations of angular velocities and
  // linear acceleration.
  // 基于imu所测得角加速度和线加速度添加约束
  if (!options_.fix_z_in_3d()) {
    for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
      const int trajectory_id = node_it->id.trajectory_id;
      const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
      if (frozen_trajectories.count(trajectory_id) != 0) {
        // We skip frozen trajectories.
        // 跳过已经完成好的轨迹
        node_it = trajectory_end;
        continue;
      }
      TrajectoryData& trajectory_data = trajectory_data_.at(trajectory_id);
      // 添加参数
      problem.AddParameterBlock(trajectory_data.imu_calibration.data(), 4,
                                new ceres::QuaternionParameterization());
      // 检查imu数据是否有轨迹节点
      CHECK(imu_data_.HasTrajectory(trajectory_id));
      const auto imu_data = imu_data_.trajectory(trajectory_id);
      // 具有连续的imu数据
      CHECK(imu_data.begin() != imu_data.end());
      // 对imu数据进行遍历
      auto imu_it = imu_data.begin();
      auto prev_node_it = node_it;
      for (++node_it; node_it != trajectory_end; ++node_it) {
        const NodeId first_node_id = prev_node_it->id;
        const NodeSpec3D& first_node_data = prev_node_it->data;
        prev_node_it = node_it;
        const NodeId second_node_id = node_it->id;
        const NodeSpec3D& second_node_data = node_it->data;

        if (second_node_id.node_index != first_node_id.node_index + 1) {
          continue;
        }

        // Skip IMU data before the node.
        // 在节点之前跳过imu数据
        while (std::next(imu_it) != imu_data.end() &&
               std::next(imu_it)->time <= first_node_data.time) {
          ++imu_it;
        }

        auto imu_it2 = imu_it;
        const IntegrateImuResult<double> result = IntegrateImu(
            imu_data, first_node_data.time, second_node_data.time, &imu_it);
        const auto next_node_it = std::next(node_it);
        if (next_node_it != trajectory_end &&
            next_node_it->id.node_index == second_node_id.node_index + 1) {
          const NodeId third_node_id = next_node_it->id;
          const NodeSpec3D& third_node_data = next_node_it->data;
          const common::Time first_time = first_node_data.time;
          const common::Time second_time = second_node_data.time;
          const common::Time third_time = third_node_data.time;
          const common::Duration first_duration = second_time - first_time;
          const common::Duration second_duration = third_time - second_time;
          const common::Time first_center = first_time + first_duration / 2;
          const common::Time second_center = second_time + second_duration / 2;
          const IntegrateImuResult<double> result_to_first_center =
              IntegrateImu(imu_data, first_time, first_center, &imu_it2);
          const IntegrateImuResult<double> result_center_to_center =
              IntegrateImu(imu_data, first_center, second_center, &imu_it2);
          // 'delta_velocity' is the change in velocity from the point in time
          // halfway between the first and second poses to halfway between
          // second and third pose. It is computed from IMU data and still
          // contains a delta due to gravity. The orientation of this vector is
          // in the IMU frame at the second pose.
          // “delta_velocity”是从第一和第二姿势之间的时间点到第二和第三姿势之间的时间点的速度变化。
          // 它是根据惯性测量单元的数据计算出来的，由于重力，它仍然包含一个增量。
          // 该矢量的方向在第二姿态的惯性测量单元帧中。
          const Eigen::Vector3d delta_velocity =
              (result.delta_rotation.inverse() *
               result_to_first_center.delta_rotation) *
              result_center_to_center.delta_velocity;
          problem.AddResidualBlock(
              AccelerationCostFunction3D::CreateAutoDiffCostFunction(
                  options_.acceleration_weight(), delta_velocity,
                  common::ToSeconds(first_duration),
                  common::ToSeconds(second_duration)),
              nullptr /* loss function */,
              C_nodes.at(second_node_id).rotation(),
              C_nodes.at(first_node_id).translation(),
              C_nodes.at(second_node_id).translation(),
              C_nodes.at(third_node_id).translation(),
              &trajectory_data.gravity_constant,
              trajectory_data.imu_calibration.data());
        }
        problem.AddResidualBlock(
            RotationCostFunction3D::CreateAutoDiffCostFunction(
                options_.rotation_weight(), result.delta_rotation),
            nullptr /* loss function */, C_nodes.at(first_node_id).rotation(),
            C_nodes.at(second_node_id).rotation(),
            trajectory_data.imu_calibration.data());
      }
    }
  }

  if (options_.fix_z_in_3d()) {
    // Add penalties for violating odometry (if available) and changes between
    // consecutive nodes.
    // 添加惩罚
    for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
      const int trajectory_id = node_it->id.trajectory_id;
      const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
      if (frozen_trajectories.count(trajectory_id) != 0) {
        node_it = trajectory_end;
        continue;
      }

      auto prev_node_it = node_it;
      for (++node_it; node_it != trajectory_end; ++node_it) {
        const NodeId first_node_id = prev_node_it->id;
        const NodeSpec3D& first_node_data = prev_node_it->data;
        prev_node_it = node_it;
        const NodeId second_node_id = node_it->id;
        const NodeSpec3D& second_node_data = node_it->data;

        if (second_node_id.node_index != first_node_id.node_index + 1) {
          continue;
        }

        // Add a relative pose constraint based on the odometry (if available).
        // 基于里程计信息添加两个相邻位姿之间的约束
        const std::unique_ptr<transform::Rigid3d> relative_odometry =
            CalculateOdometryBetweenNodes(trajectory_id, first_node_data,
                                          second_node_data);
        if (relative_odometry != nullptr) {
          problem.AddResidualBlock(
              SpaCostFunction3D::CreateAutoDiffCostFunction(Constraint::Pose{
                  *relative_odometry, options_.odometry_translation_weight(),
                  options_.odometry_rotation_weight()}),
              nullptr /* loss function */, C_nodes.at(first_node_id).rotation(),
              C_nodes.at(first_node_id).translation(),
              C_nodes.at(second_node_id).rotation(),
              C_nodes.at(second_node_id).translation());
        }

        // Add a relative pose constraint based on consecutive local SLAM poses.
        // 基于local slam位姿添加相邻位姿之间的约束
        const transform::Rigid3d relative_local_slam_pose =
            first_node_data.local_pose.inverse() * second_node_data.local_pose;
        problem.AddResidualBlock(
            SpaCostFunction3D::CreateAutoDiffCostFunction(
                Constraint::Pose{relative_local_slam_pose,
                                 options_.local_slam_pose_translation_weight(),
                                 options_.local_slam_pose_rotation_weight()}),
            nullptr /* loss function */, C_nodes.at(first_node_id).rotation(),
            C_nodes.at(first_node_id).translation(),
            C_nodes.at(second_node_id).rotation(),
            C_nodes.at(second_node_id).translation());
      }
    }
  }

  // Add fixed frame pose constraints.
  // 添加固定结构位姿之间的约束
  std::map<int, CeresPose> C_fixed_frames;
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (!fixed_frame_pose_data_.HasTrajectory(trajectory_id)) {
      node_it = trajectory_end;
      continue;
    }

    const TrajectoryData& trajectory_data = trajectory_data_.at(trajectory_id);
    // 是否已经初始化
    bool fixed_frame_pose_initialized = false;
    for (; node_it != trajectory_end; ++node_it) {
      const NodeId node_id = node_it->id;
      const NodeSpec3D& node_data = node_it->data;
      // 找到固定结构的位姿
      const std::unique_ptr<transform::Rigid3d> fixed_frame_pose =
          Interpolate(fixed_frame_pose_data_, trajectory_id, node_data.time);
      if (fixed_frame_pose == nullptr) {
        continue;
      }

      const Constraint::Pose constraint_pose{
          *fixed_frame_pose, options_.fixed_frame_pose_translation_weight(),
          options_.fixed_frame_pose_rotation_weight()};

      if (!fixed_frame_pose_initialized) {
        transform::Rigid3d fixed_frame_pose_in_map;
        if (trajectory_data.fixed_frame_origin_in_map.has_value()) {
          fixed_frame_pose_in_map =
              trajectory_data.fixed_frame_origin_in_map.value();
        } else {
          fixed_frame_pose_in_map =
              node_data.global_pose * constraint_pose.zbar_ij.inverse();
        }
        C_fixed_frames.emplace(
            std::piecewise_construct, std::forward_as_tuple(trajectory_id),
            std::forward_as_tuple(
                transform::Rigid3d(
                    fixed_frame_pose_in_map.translation(),
                    Eigen::AngleAxisd(
                        transform::GetYaw(fixed_frame_pose_in_map.rotation()),
                        Eigen::Vector3d::UnitZ())),
                nullptr,
                common::make_unique<ceres::AutoDiffLocalParameterization<
                    YawOnlyQuaternionPlus, 4, 1>>(),
                &problem));
        fixed_frame_pose_initialized = true;
      }

      problem.AddResidualBlock(
          SpaCostFunction3D::CreateAutoDiffCostFunction(constraint_pose),
          nullptr /* loss function */,
          C_fixed_frames.at(trajectory_id).rotation(),
          C_fixed_frames.at(trajectory_id).translation(),
          C_nodes.at(node_id).rotation(), C_nodes.at(node_id).translation());
    }
  }
  // Solve.
  // 运用ceres库进行问题解决
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);
  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
    for (const auto& trajectory_id_and_data : trajectory_data_) {
      const int trajectory_id = trajectory_id_and_data.first;
      const TrajectoryData& trajectory_data = trajectory_id_and_data.second;
      if (trajectory_id != 0) {
        LOG(INFO) << "Trajectory " << trajectory_id << ":";
      }
      LOG(INFO) << "Gravity was: " << trajectory_data.gravity_constant;
      const auto& imu_calibration = trajectory_data.imu_calibration;
      LOG(INFO) << "IMU correction was: "
                << common::RadToDeg(2. * std::acos(imu_calibration[0]))
                << " deg (" << imu_calibration[0] << ", " << imu_calibration[1]
                << ", " << imu_calibration[2] << ", " << imu_calibration[3]
                << ")";
    }
  }

  // Store the result.
  // 存储结果
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        C_submap_id_data.data.ToRigid();
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose =
        C_node_id_data.data.ToRigid();
  }
  for (const auto& C_fixed_frame : C_fixed_frames) {
    trajectory_data_.at(C_fixed_frame.first).fixed_frame_origin_in_map =
        C_fixed_frame.second.ToRigid();
  }
  for (const auto& C_landmark : C_landmarks) {
    landmark_data_[C_landmark.first] = C_landmark.second.ToRigid();
  }
}

std::unique_ptr<transform::Rigid3d>
// 计算两个节点之间的里程计信息
OptimizationProblem3D::CalculateOdometryBetweenNodes(
    const int trajectory_id, const NodeSpec3D& first_node_data,
    const NodeSpec3D& second_node_data) const {
  // 如果里程计信息已经被添加到节点
  if (odometry_data_.HasTrajectory(trajectory_id)) {
    const std::unique_ptr<transform::Rigid3d> first_node_odometry =
        Interpolate(odometry_data_, trajectory_id, first_node_data.time);
    const std::unique_ptr<transform::Rigid3d> second_node_odometry =
        Interpolate(odometry_data_, trajectory_id, second_node_data.time);
    // 检查第一帧和第二帧里程计数据是否为空，进行计算1
    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      const transform::Rigid3d relative_odometry =
          first_node_odometry->inverse() * (*second_node_odometry);
      return common::make_unique<transform::Rigid3d>(relative_odometry);
    }
  }
  return nullptr;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
