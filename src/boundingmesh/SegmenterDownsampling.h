//	Copyright (c) 2015, Andre Gaschler, Quirin Fischer, Philip Gergen
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

#ifndef BOUNDINGMESH_SEGMENTER_DOWNSAMPLING_H
#define BOUNDINGMESH_SEGMENTER_DOWNSAMPLING_H

#include "boundingmesh/Mesh.h"
#include "boundingmesh/Segmenter.h"
#include "boundingmesh/VoxelSet.h"
#include "boundingmesh/VoxelSubset.h"

namespace boundingmesh {

class SegmenterDownsampling {
 public:
  SegmenterDownsampling();

  void setMaxPasses(int passes);
  void setMaximumConcavity(Real maximum_concavity);

  void setMesh(std::shared_ptr<Mesh> mesh, int min_voxel_count);
  void compute();

  std::vector<std::shared_ptr<Mesh> > getSegmentation();

  std::shared_ptr<VoxelSet> getVoxels();

  void setHeuristic(int heuristic) { heuristic_ = heuristic; }

  void setDebug(bool debug) { debug_ = debug; }

  void setAlpha(Real alpha) { alpha_ = alpha; }

  void setBeta(Real beta) { beta_ = beta; }

  void setConvexhullDownsampling(int convexhullDownsampling) {
    convexhull_downsampling_ = convexhullDownsampling;
  }

  void setDelta(Real delta) { delta_ = delta; }

  void setGamma(Real gamma) { gamma_ = gamma; }

  void setPlaneDownsampling(int planeDownsampling) {
    plane_downsampling_ = planeDownsampling;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  int passes_ = 32;
  Real maximum_concavity_ = 0.001;
  int heuristic_ = 1;
  bool debug_ = false;
  bool superDebug_ = false;
  Real voxel_size_ = 1;
  Real alpha_ = 0.05;
  Real beta_ = 0.05;
  Real gamma_ = 0.0005;
  Real delta_ = 0.05;
  int convexhull_downsampling_ = 1;
  int plane_downsampling_ = 4;
  Real initial_volume_ = 1;
  Real initial_size_ = 1;

  Real initial_surface_voxels_;
  Vector3 initial_area_;

  std::shared_ptr<VoxelSet> voxels_;
  std::vector<VoxelSubset> subsets_;
  std::vector<std::shared_ptr<Mesh> > segmentation_;

  bool arePartsAdjacent(VoxelSubset& p1, VoxelSubset& p2);
  std::vector<Split> generateSplits(const VoxelSubset& subset,
                                    const int downsampling);
  std::vector<Split> refineSplits(const VoxelSubset& subset,
                                  const Split& bestSplit,
                                  const int downsampling);
  std::vector<VoxelSubset> mergeConvexHulls(std::vector<VoxelSubset> parts);
  Real computePreferredCuttingDirection(VoxelSubset& tset, Vector3& dir);

  Real rateSplit(VoxelSubset& inputSet, Split split,
                 const Vector3& preferredCuttingDirection, const Real w,
                 const Real alpha, const Real beta, const Real delta);
  Real rateSplit2(VoxelSubset& inputSet, Split split,
                  const Vector3& preferredCuttingDirection, const Real w,
                  const Real alpha, const Real beta, const Real delta);
  Real rateSplit3(VoxelSubset& inputSet, Split split,
                  const Vector3& preferredCuttingDirection, const Real w,
                  const Real alpha, const Real beta, const Real delta);
  void computeBestClippingPlane(const VoxelSubset& inputPSet, const Real volume,
                                const std::vector<Split>& splits,
                                const Vector3& preferredCuttingDirection,
                                const Real w, const Real alpha, const Real beta,
                                const Real delta,
                                const int convexhullDownsampling,
                                Split& bestSplit);
  void printDebugSlices(VoxelSubset whole_mesh);
};
}  // namespace boundingmesh

#endif  // BOUNDINGMESH_SEGMENTER_DOWNSAMPLING_H
