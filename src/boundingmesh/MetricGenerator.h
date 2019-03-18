//	Copyright (c) 2013, Andre Gaschler, Quirin Fischer
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

#ifndef BOUNDINGMESH_METRICGENERATOR_H
#define BOUNDINGMESH_METRICGENERATOR_H

#include <iostream>
#include <map>
#include <memory>

#include "Mesh.h"

namespace boundingmesh {
enum Metric {
  ClassicQEM,
  ModifiedQEM,
  MinimizedConstant,
  Diagonalization,
  Average
};

enum Initialization { DistancePrimitives, Midpoint };

struct HoleEdge {
  Index vertex_1;
  Index vertex_2;
  Matrix44 old_triangle_qem;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

const Real epsilon = 0.00001;
const Real distance_factor_ = 0.5;

typedef std::deque<Matrix44, Eigen::aligned_allocator<Matrix44> > MatrixArray;

class MetricGenerator {
 public:
  MetricGenerator();
  ~MetricGenerator();

  void setMesh(::std::shared_ptr<Mesh> mesh);

  void setMetric(Metric metric);
  void setInitialization(Initialization initialization);

  Matrix44 getErrorMetric(Index edge_index);
  void contractEdge(Index edge_index);

  void cleanAndRenumber();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  void initialize();
  Metric metric_;
  Initialization initialization_;
  ::std::shared_ptr<Mesh> mesh_;

  // General
  void shrinkIndexedArray(MatrixArray* array,
                          std::stack<Index> deleted_indices);

  // ClassicQEM
  MatrixArray qems_;
  Matrix44 computeQEM(Index vertex_index);

  // ModifiedQEM
  MatrixArray vertices_qem_;
  MatrixArray edges_qem_;
  MatrixArray triangles_qem_;

  Matrix44 computeModifiedVertexQEM(Index vertex_index);
  Matrix44 computeModifiedEdgeQEM(Index edge_index);

  void contractEdgeModifiedQEM(Index edge_index);
  void collectRemovalData(
      Index vertex_index, Index other_index,
      std::vector<HoleEdge, Eigen::aligned_allocator<HoleEdge> >& hole_border,
      std::map<Index, Matrix44, std::less<Index>,
               Eigen::aligned_allocator<std::pair<const Index, Matrix44> > >&
          new_edges_qem);

  // RepeatedMerge
  MatrixArray qems_merge_;

  Matrix44 computeInitialMergeMetric(Index vertex_index);
  Matrix44 mergeMax(const Matrix44& a, const Matrix44& b);
  Matrix44 mergeMatrices(const Matrix44& a, const Matrix44& b);

  Matrix44 mergeMinConstant(const Matrix44& a, const Matrix44& b);
  Matrix44 mergeDiagonalization(const Matrix44& a, const Matrix44& b);
  Matrix44 mergeAverage(const Matrix44& a, const Matrix44& b);
};
}  // namespace boundingmesh
#endif  // BOUNDINGMESH_METRICGENERATOR_H
