/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <hydra_utils/dsg_types.h>
#include "hydra_topology/voxblox_types.h"

#include <memory>
#include <unordered_set>
#include <vector>

namespace hydra {
namespace topology {

// TODO(nathan) this probably belongs in spark_dsg
class NearestNodeFinder {
 public:
  using Callback = std::function<void(NodeId, size_t, double)>;

  NearestNodeFinder(const SceneGraphLayer& layer, const std::vector<NodeId>& nodes);

  NearestNodeFinder(const SceneGraphLayer& layer,
                    const std::unordered_set<NodeId>& nodes);

  virtual ~NearestNodeFinder();

  void find(const Eigen::Vector3d& position,
            size_t num_to_find,
            bool skip_first,
            const Callback& callback);

 private:
  struct Detail;

  std::unique_ptr<Detail> internals_;
};

class NearestVoxelFinder {
 public:
  using Callback = std::function<void(const GlobalIndex&, size_t, int64_t)>;

  explicit NearestVoxelFinder(const voxblox::AlignedVector<GlobalIndex>& indices);

  virtual ~NearestVoxelFinder();

  void find(const GlobalIndex& index, size_t num_to_find, const Callback& callback);

 private:
  struct Detail;

  std::unique_ptr<Detail> internals_;
};

struct FurthestIndexResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  int64_t distance = 0;
  bool from_source = true;
  GlobalIndex index;
};

FurthestIndexResult findFurthestIndexFromLine(
    const voxblox::AlignedVector<GlobalIndex>& indices,
    const GlobalIndex& start,
    const GlobalIndex& end,
    size_t number_source_edges);

inline FurthestIndexResult findFurthestIndexFromLine(
    const voxblox::AlignedVector<GlobalIndex>& indices,
    const GlobalIndex& start,
    const GlobalIndex& end) {
  return findFurthestIndexFromLine(indices, start, end, indices.size());
}

}  // namespace topology
}  // namespace hydra
