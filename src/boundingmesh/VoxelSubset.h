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

#ifndef BOUNDINGMESH_VOXELSUBSET_H
#define BOUNDINGMESH_VOXELSUBSET_H

#include "Mesh.h"
#include "Split.h"
#include "VoxelSet.h"

namespace boundingmesh {
class VoxelSubset {
 public:
  VoxelSubset(std::shared_ptr<VoxelSet> voxels);
  VoxelSubset(const VoxelSubset& subset, bool empty = false);
  ~VoxelSubset();

  VoxelSubset& operator=(VoxelSubset other);
  friend void swap(VoxelSubset& first, VoxelSubset& second);

  void addVoxel(Index index);
  void addSplit(AppliedSplit split);
  void mergeSplit(AppliedSplit split);

  VoxelSubset merge(VoxelSubset other);
  std::vector<VoxelSubset> partition(Split split);
  std::vector<VoxelSubset> partitionVoxel(Split split);
  std::vector<VoxelSubset> partitionVoxel(Split split, bool calculateHulls);
  Real convexVolume() const;
  Real volume() const;
  int voxelCount() const;
  int surfaceVoxelCount() const;
  VoxelSubset getSurfaceSubset() const;

  void ComputePrincipalAxes();
  void ComputeBB();
  Vector3 GetBoundingBoxMin() const;
  Vector3 GetBoundingBoxMax() const;
  Real GetBoundingBoxSize() const;

  Real GetEigenValue(int axis) const;

  void calculateConvexHull();
  void calculateConvexHullVoxel();
  std::pair<Real, Real> calculate2DConvexHullVolume(int direction);
  std::shared_ptr<Mesh> getConvexHull();

  void writeWRL(std::string filename, bool debugTriangles = false);

  bool isFinal();
  void setFinal();

  std::vector<Index> const& getIndices() const;
  std::shared_ptr<VoxelSet> const& getVoxels() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  std::vector<Index> indices_;
  std::vector<AppliedSplit> splits_;
  std::shared_ptr<VoxelSet> voxels_;

  Vector3 bounding_box_min_;
  Vector3 bounding_box_max_;
  Vector3 barycenter_;

  // Real q_[3][3];
  // Real d_[3][3];
  Matrix33 D_;

  Vector3 barycenterPCA_;

  Convex convex_hull_;
  bool final_;
};
}

#endif  // BOUNDINGMESH_VOXELSUBSET_H
