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

#include "cartographer/mapping/internal/constraints/constraint_builder_3d.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching//ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/mapping/proto/scan_matching//fast_correlative_scan_matcher_options_3d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kConstraintRotationalScoresMetric = metrics::Histogram::Null();
static auto* kConstraintLowResolutionScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintRotationalScoresMetric =
    metrics::Histogram::Null();
static auto* kGlobalConstraintLowResolutionScoresMetric =
    metrics::Histogram::Null();
// 约束构造函数体
ConstraintBuilder3D::ConstraintBuilder3D(
    const proto::ConstraintBuilderOptions& options,
    common::ThreadPoolInterface* const thread_pool)
    : options_(options),// 参数配置
      thread_pool_(thread_pool),// 线程池
      finish_node_task_(common::make_unique<common::Task>()),// 完成节点任务
      when_done_task_(common::make_unique<common::Task>()),// 任务完成
      sampler_(options.sampling_ratio()),// 采样
      ceres_scan_matcher_(options.ceres_scan_matcher_options_3d()) {}// scan match
// 析构函数
ConstraintBuilder3D::~ConstraintBuilder3D() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}
// MaybeAddConstraint函数体
void ConstraintBuilder3D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap3D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const std::vector<TrajectoryNode>& submap_nodes,
    const transform::Rigid3d& global_node_pose,
    const transform::Rigid3d& global_submap_pose) {
  // 如果节点和子地图距离大于约束规定的最大距离，直接返回，匹配不上
  if ((global_node_pose.translation() - global_submap_pose.translation())
          .norm() > options_.max_constraint_distance()) {
    return;
  }
  if (!sampler_.Pulse()) return;

  common::MutexLocker locker(&mutex_);// 互斥锁
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }
  // 回调约束
  constraints_.emplace_back();
  // 设置约束尺寸
  kQueueLengthMetric->Set(constraints_.size());
  // 返回约束尺寸
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      // 调用函数DispatchScanMatcherConstruction，定义在下面
      DispatchScanMatcherConstruction(submap_id, submap_nodes, submap);
  auto constraint_task = common::make_unique<common::Task>();
  // 把约束任务放回到工作序列里面
  constraint_task->SetWorkItem([=]() EXCLUDES(mutex_) {
    // 进行约束的计算，调用函数
    ComputeConstraint(submap_id, node_id, false, /* match_full_submap */
                      constant_data, global_node_pose, global_submap_pose,
                      *scan_matcher, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}
// MaybeAddGlobalConstraint函数体
void ConstraintBuilder3D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap3D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const std::vector<TrajectoryNode>& submap_nodes,
    const Eigen::Quaterniond& global_node_rotation,
    const Eigen::Quaterniond& global_submap_rotation) {
  common::MutexLocker locker(&mutex_);// 互斥锁
  // 如果已经完成，输出下面语句
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  // 回调约束
  constraints_.emplace_back();
  // 设定约束尺寸
  kQueueLengthMetric->Set(constraints_.size());
  // 返回约束尺寸
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      // 调用函数，返回匹配好的submap
      DispatchScanMatcherConstruction(submap_id, submap_nodes, submap);
  auto constraint_task = common::make_unique<common::Task>();
  // 把约束任务放到工作序列中
  constraint_task->SetWorkItem([=]() EXCLUDES(mutex_) {
     // 计算约束
    ComputeConstraint(submap_id, node_id, true, /* match_full_submap */
                      constant_data,
                      transform::Rigid3d::Rotation(global_node_rotation),
                      transform::Rigid3d::Rotation(global_submap_rotation),
                      *scan_matcher, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}
// 最后节点的添加
void ConstraintBuilder3D::NotifyEndOfNode() {
  common::MutexLocker locker(&mutex_);// 互斥锁
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->SetWorkItem([this] {
    common::MutexLocker locker(&mutex_);
    ++num_finished_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  finish_node_task_ = common::make_unique<common::Task>();
  when_done_task_->AddDependency(finish_node_task_handle);
  ++num_started_nodes_;
}
// 当约束已经计算完成时，回调函数
void ConstraintBuilder3D::WhenDone(
    const std::function<void(const ConstraintBuilder3D::Result&)>& callback) {
  common::MutexLocker locker(&mutex_);
  // 检查是否为空指针
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  when_done_ =
      common::make_unique<std::function<void(const Result&)>>(callback);
  // 如果不是空指针，就进行工作序列中函数体的计算
  CHECK(when_done_task_ != nullptr);
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = common::make_unique<common::Task>();
}
// DispatchScanMatcherConstruction函数体
// 
const ConstraintBuilder3D::SubmapScanMatcher*
ConstraintBuilder3D::DispatchScanMatcherConstruction(
    const SubmapId& submap_id, const std::vector<TrajectoryNode>& submap_nodes,
    const Submap3D* submap) {
  // 如果submapid不为0，返回submapid
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  // 高分辨率栅格
  submap_scan_matcher.high_resolution_hybrid_grid =
      &submap->high_resolution_hybrid_grid();
      // 低分辨率栅格
  submap_scan_matcher.low_resolution_hybrid_grid =
      &submap->low_resolution_hybrid_grid();
      // scan match参数选择
  auto& scan_matcher_options =
      options_.fast_correlative_scan_matcher_options_3d();
  auto scan_matcher_task = common::make_unique<common::Task>();
  // 把scan match 放到工作序列里面
  scan_matcher_task->SetWorkItem(
      [&submap_scan_matcher, &scan_matcher_options, submap_nodes]() {
        submap_scan_matcher.fast_correlative_scan_matcher =
            common::make_unique<scan_matching::FastCorrelativeScanMatcher3D>(
                *submap_scan_matcher.high_resolution_hybrid_grid,
                submap_scan_matcher.low_resolution_hybrid_grid, submap_nodes,
                scan_matcher_options);
      });
      // 产生creation_task_handle
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));
      // 返回匹配好的submapid

  return &submap_scan_matchers_.at(submap_id);
}
// 计算约束函数体
void ConstraintBuilder3D::ComputeConstraint(
    const SubmapId& submap_id, const NodeId& node_id, bool match_full_submap,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid3d& global_node_pose,
    const transform::Rigid3d& global_submap_pose,
    const SubmapScanMatcher& submap_scan_matcher,
    std::unique_ptr<Constraint>* constraint) {
  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'high_resolution_point_cloud' in node j and
  // - the initial guess 'initial_pose' (submap i <- node j).
  // 调用FAST CSM进行匹配，得到匹配结果
  std::unique_ptr<scan_matching::FastCorrelativeScanMatcher3D::Result>
      match_result;

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  // 计算位姿估计
  // 如果进行全匹配
  if (match_full_submap) 

  {
    kGlobalConstraintsSearchedMetric->Increment();
    // 得到匹配结果
    match_result =
        submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
            global_node_pose.rotation(), global_submap_pose.rotation(),
            *constant_data, options_.global_localization_min_score());
    if (match_result != nullptr) {
      CHECK_GT(match_result->score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(match_result->score);
      kGlobalConstraintRotationalScoresMetric->Observe(
          match_result->rotational_score);
      kGlobalConstraintLowResolutionScoresMetric->Observe(
          match_result->low_resolution_score);
    }
    // 如果没有匹配上直接返回 
    else {
      return;
    }
  } 
  // 如果不是全匹配
  else {
    kConstraintsSearchedMetric->Increment();
    // 得到匹配结果
    match_result = submap_scan_matcher.fast_correlative_scan_matcher->Match(
        global_node_pose, global_submap_pose, *constant_data,
        options_.min_score());
    if (match_result != nullptr) {
      // We've reported a successful local match.
      CHECK_GT(match_result->score, options_.min_score());
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(match_result->score);
      kConstraintRotationalScoresMetric->Observe(
          match_result->rotational_score);
      kConstraintLowResolutionScoresMetric->Observe(
          match_result->low_resolution_score);
    }
    // 如果没有匹配上直接返回 
    else {
      return;
    }
  }
  {
    // 向分数直方图里添加匹配结果
    common::MutexLocker locker(&mutex_);
    score_histogram_.Add(match_result->score);
    rotational_score_histogram_.Add(match_result->rotational_score);
    low_resolution_score_histogram_.Add(match_result->low_resolution_score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  // 用CSM 估计预估初始位姿和先前的位姿，为ceres scan提供初值
  ceres::Solver::Summary unused_summary;
  transform::Rigid3d constraint_transform;
  // 调用ceres scan进行匹配
  ceres_scan_matcher_.Match(match_result->pose_estimate.translation(),
                            match_result->pose_estimate,
                            {{&constant_data->high_resolution_point_cloud,
                              submap_scan_matcher.high_resolution_hybrid_grid},
                             {&constant_data->low_resolution_point_cloud,
                              submap_scan_matcher.low_resolution_hybrid_grid}},
                            &constraint_transform, &unused_summary);
  // 设置新的约束
  constraint->reset(new Constraint{
      submap_id,// submapid

      node_id,// 激光节点id
      {constraint_transform, options_.loop_closure_translation_weight(),
       options_.loop_closure_rotation_weight()},// 权重值
      Constraint::INTER_SUBMAP});

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->high_resolution_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      // Compute the difference between (submap i <- node j) according to loop
      // closure ('constraint_transform') and according to global SLAM state.
      // 计算回环和全局slam位子的不同
      const transform::Rigid3d difference = global_node_pose.inverse() *
                                            global_submap_pose *
                                            constraint_transform;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << transform::GetAngle(difference);
    }
    info << " with score " << std::setprecision(1) << 100. * match_result->score
         << "%.";
    LOG(INFO) << info.str();
  }
}
// 当回调whendone函数时，调用此函数，清掉已经计算完成的约束
void ConstraintBuilder3D::RunWhenDoneCallback() {
  // 约束结果已经得到
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    common::MutexLocker locker(&mutex_);
    CHECK(when_done_ != nullptr);
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }
    if (options_.log_matches()) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.\n"
                << "Score histogram:\n"
                << score_histogram_.ToString(10) << "\n"
                << "Rotational score histogram:\n"
                << rotational_score_histogram_.ToString(10) << "\n"
                << "Low resolution score histogram:\n"
                << low_resolution_score_histogram_.ToString(10);
    }
    // 清掉约束
    constraints_.clear();
    callback = std::move(when_done_);
    // 重新设置whendone函数
    when_done_.reset();
    // 重新设置约束尺寸
    kQueueLengthMetric->Set(constraints_.size());
  }
  (*callback)(result);
}
// 计算参加约束的节点
int ConstraintBuilder3D::GetNumFinishedNodes() {
  common::MutexLocker locker(&mutex_);
  return num_finished_nodes_;
}
// 消除scanmatch结果（已经匹配完成）
void ConstraintBuilder3D::DeleteScanMatcher(const SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
}

void ConstraintBuilder3D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "mapping_internal_constraints_constraint_builder_3d_constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  auto* queue_length = factory->NewGaugeFamily(
      "mapping_internal_constraints_constraint_builder_3d_queue_length",
      "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "mapping_internal_constraints_constraint_builder_3d_scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric =
      scores->Add({{"search_region", "local"}, {"kind", "score"}});
  kConstraintRotationalScoresMetric =
      scores->Add({{"search_region", "local"}, {"kind", "rotational_score"}});
  kConstraintLowResolutionScoresMetric = scores->Add(
      {{"search_region", "local"}, {"kind", "low_resolution_score"}});
  kGlobalConstraintScoresMetric =
      scores->Add({{"search_region", "global"}, {"kind", "score"}});
  kGlobalConstraintRotationalScoresMetric =
      scores->Add({{"search_region", "global"}, {"kind", "rotational_score"}});
  kGlobalConstraintLowResolutionScoresMetric = scores->Add(
      {{"search_region", "global"}, {"kind", "low_resolution_score"}});
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
