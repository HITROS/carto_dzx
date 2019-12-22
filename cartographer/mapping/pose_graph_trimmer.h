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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_TRIMMER_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_TRIMMER_H_

#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"

namespace cartographer {
namespace mapping {

// Implemented by the pose graph to provide thread-safe access to functions for
// trimming the graph.
class Trimmable {
 public:
  virtual ~Trimmable() {}

  virtual int num_submaps(int trajectory_id) const = 0;

  virtual std::vector<SubmapId> GetSubmapIds(int trajectory_id) const = 0;
  // Returns finished submaps with optimized poses only.
  // 只返回有优化位姿的已经完成的子地图
  virtual MapById<SubmapId, PoseGraphInterface::SubmapData>
  GetOptimizedSubmapData() const = 0;
  virtual const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const = 0;
  virtual const std::vector<PoseGraphInterface::Constraint>& GetConstraints()
      const = 0;

  // Marks 'submap_id' and corresponding intra-submap nodes as trimmed. They
  // will no longer take part in scan matching, loop closure, visualization.
  // Submaps and nodes are only marked, the numbering remains unchanged.
  // 没有加入全局地图的子地图将不会被进行接下来的步骤
  virtual void MarkSubmapAsTrimmed(const SubmapId& submap_id) = 0;

  // Checks if the given trajectory is finished or not.
  // 检查轨迹是否已经完成还是没有被完成
  virtual bool IsFinished(int trajectory_id) const = 0;
};

// An interface to implement algorithms that choose how to trim the pose graph.
// 添加一个修建位姿图算法的借口
class PoseGraphTrimmer {
 public:
  virtual ~PoseGraphTrimmer() {}

  // Called once after each pose graph optimization.
  // 在每一个优化位姿之后都会被调用
  virtual void Trim(Trimmable* pose_graph) = 0;

  // Checks if this trimmer is in a terminatable state.
  // 检查修剪是否在一个最终的状态
  virtual bool IsFinished() = 0;
};

// Keeps the last 'num_submaps_to_keep' of the trajectory with 'trajectory_id'
// to implement localization without mapping.
class PureLocalizationTrimmer : public PoseGraphTrimmer {
 public:
  PureLocalizationTrimmer(int trajectory_id, int num_submaps_to_keep);
  ~PureLocalizationTrimmer() override {}

  void Trim(Trimmable* pose_graph) override;
  bool IsFinished() override;

 private:
  const int trajectory_id_;
  int num_submaps_to_keep_;
  bool finished_ = false;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_TRIMMER_H_
