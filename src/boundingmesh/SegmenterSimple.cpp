//	Copyright (c) 2015, Andre Gaschler, Quirin Fischer
//	All rights reserved.
//
//	Redistribution and use in source and binary forms, with or without
// modification,
//	are permitted provided that the following conditions are met:
//
//	* Redistributions of source code must retain the above copyright notice,
// this
//	  list of conditions and the following disclaimer.
//
//	* Redistributions in binary form must reproduce the above copyright
// notice, this
//	  list of conditions and the following disclaimer in the documentation
// and/or
//	  other materials provided with the distribution.
//
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND
//	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED
//	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR
//	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES
//	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES;
//	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON
//	ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS
//	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "boundingmesh/SegmenterSimple.h"

#include <math.h>
#include <iostream>

namespace boundingmesh {
SegmenterSimple::SegmenterSimple() : passes_(32), min_gain_(0.001) {}

void SegmenterSimple::setMaxPasses(int passes) { passes_ = passes; }

void SegmenterSimple::setMinVolumeGain(Real min_gain) { min_gain_ = min_gain; }

void SegmenterSimple::setMeshWithVoxelSize(std::shared_ptr<Mesh> mesh,
                                           Real voxel_size) {
  voxels_ = std::make_shared<VoxelSet>(mesh, voxel_size);
}

void SegmenterSimple::setMesh(std::shared_ptr<Mesh> mesh, int min_voxel_count) {
  int count = 1;
  double length = 100;
  VoxelSet set;
  while (count < min_voxel_count * 0.9 || count > min_voxel_count * 1.1) {
    set = VoxelSet(mesh, length, true);
    count = set.nVoxels();
    if (count < min_voxel_count * 0.9 || count > min_voxel_count * 1.1) {
      double a = pow((double)(min_voxel_count) / count, 1.0 / 3.0);
      length = length / a;
    }
  }

  voxels_ = std::make_shared<VoxelSet>(mesh, length, true);
}

void SegmenterSimple::compute() {
  std::vector<VoxelSubset> subsets;
  VoxelSubset whole_mesh(voxels_);
  for (int i = 0; i < voxels_->nVoxels(); ++i) {
    whole_mesh.addVoxel(i);
  }
  whole_mesh.calculateConvexHull();
  subsets.push_back(whole_mesh);

  // Segmentation algorithm:
  // Keep a list with all current parts, starting with whole modedl
  // In every pass, try to split each part along a plane
  // Brute-Force search: evaluate all split planes, take best
  for (unsigned int pass = 0; pass < passes_; ++pass) {
    std::cout << "Pass " << pass << std::endl;
    bool changed = false;
    // evaluation_blend_ = (pass+1) / (passes_+1);
    std::vector<VoxelSubset> new_subsets;
    for (unsigned int subset_index = 0; subset_index < subsets.size();
         ++subset_index) {
      if (subsets[subset_index].isFinal()) {
        new_subsets.push_back(subsets[subset_index]);
        continue;
      }

      std::vector<VoxelSubset> best_partition;
      best_partition.push_back(subsets[subset_index]);
      Real best_value =
          evaluateSubset(subsets[subset_index]) -
          min_gain_;  //<-Bonus for no split to avoide overly-precise splitting
      if (subsets[subset_index].isFinal()) {
        new_subsets.push_back(subsets[subset_index]);
        continue;
      }

      std::vector<Split> possible_splits =
          generateSplits(subsets[subset_index]);

      for (unsigned int split_index = 0; split_index < possible_splits.size();
           ++split_index) {
        Split& split = possible_splits[split_index];
        std::vector<VoxelSubset> new_partition =
            subsets[subset_index].partition(split);
        Real new_value =
            evaluatePartition(subsets[subset_index], new_partition);

        if (new_value < best_value) {
          best_partition = new_partition;
          best_value = new_value;
        }
      }
      // std::cout<<"Best split value: "<<best_value<<std::endl;
      if (best_partition.size() == 1) {
        best_partition[0].setFinal();
      } else {
        changed = true;
      }
      new_subsets.insert(new_subsets.end(), best_partition.begin(),
                         best_partition.end());
    }
    if (!changed) break;
    subsets = new_subsets;
  }

  subsets_ = subsets;
  for (unsigned int i = 0; i < subsets_.size(); ++i) {
    segmentation_.push_back(subsets_[i].getConvexHull());
  }
}

std::vector<std::shared_ptr<Mesh> > SegmenterSimple::getSegmentation() {
  return segmentation_;
}

std::vector<Split> SegmenterSimple::generateSplits(const VoxelSubset& subset) {
  std::vector<Split> possible_splits;
  for (int dimension = 0; dimension < 3; ++dimension)
    for (int index = 0; index < voxels_->resolution(dimension); ++index) {
      possible_splits.push_back(Split(dimension, index));
    }

  return possible_splits;
}

Real SegmenterSimple::evaluatePartition(
    const VoxelSubset& previous_set,
    const std::vector<VoxelSubset>& partition) {
  Real volume_added_by_convex = std::max(
      0.0, previous_set.convexVolume() - previous_set.volume());  // [0, inf)
  Real balance = (partition[0].volume() * partition[0].volume() +
                  partition[1].volume() * partition[1].volume()) /
                 (previous_set.volume() * previous_set.volume());  //[0.5, 1]
  Real necessity_to_split = 1 / (volume_added_by_convex + 1);      //[0,1]
  Real exp = 1 / balance * 2;
  Real split_fitness = std::pow(necessity_to_split, exp);  //[0,1]

  Real volume_factor =
      (partition[0].convexVolume() + partition[1].convexVolume()) /
      previous_set.convexVolume();  // [0, 1]

  return (volume_factor + split_fitness) / 2;
}

Real SegmenterSimple::evaluateSubset(const VoxelSubset& subset) {
  // Real val = subset.evaluate();
  return 1;
}
}
