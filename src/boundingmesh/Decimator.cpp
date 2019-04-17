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

#include "boundingmesh/Decimator.h"

#include <cmath>
#include <limits>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "boundingmesh/SimpleOptimizer.h"
// for debugging
#include <Eigen/Eigenvalues>

namespace boundingmesh {
Decimator::Decimator(DecimationDirection direction)
    : direction_(direction), optimizer_(new SimpleOptimizer) {}

Decimator::Decimator(std::unique_ptr<OptimizerInterface> optimizer,
                     DecimationDirection direction)
    : target_vertices_(default_target_vertices),
      maximum_error_(default_maximum_error),
      direction_(direction),
      optimizer_(std::move(optimizer)) {}

Decimator::~Decimator() {}

Real Decimator::currentError() { return current_error_; }

Real Decimator::nextError() {
  if (queue_.size() == 0) return 0;
  const EdgeContraction& contraction = queue_.first();
  return contraction.cost();
}

void Decimator::setTargetVertices(int target_vertices) {
  target_vertices_ = target_vertices;
  target_vertices_used_ = true;
}

void Decimator::unsetTargetVertices() { target_vertices_used_ = false; }

void Decimator::setMaximumError(Real maximum_error) {
  maximum_error_ = maximum_error;
  maximum_error_used_ = true;
}

void Decimator::unsetMaximumError() { maximum_error_used_ = false; }

void Decimator::setDirection(DecimationDirection direction) {
  direction_ = direction;

  if (result_mesh_ != NULL) recomputeQueue();
}

void Decimator::setMetric(Metric metric) {
  metric_generator_.setMetric(metric);

  if (result_mesh_ != NULL) recomputeQueue();
}

void Decimator::setInitialization(Initialization initialization) {
  metric_generator_.setInitialization(initialization);

  if (result_mesh_ != NULL) recomputeQueue();
}

void Decimator::setMesh(const Mesh& mesh) {
  result_mesh_ = std::make_shared<Mesh>(mesh);
  metric_generator_.setMesh(result_mesh_);

  recomputeQueue();
}

::std::shared_ptr<Mesh> Decimator::getMesh() { return result_mesh_; }

void Decimator::recomputeQueue() {
  queue_ = ContractionQueue();
  for (unsigned int i = 0; i < result_mesh_->nEdges(); ++i)
    queue_.insert(computeEdgeContraction(i));
}

::std::shared_ptr<Mesh> Decimator::compute(ComputeCallback callback) {
  if (queue_.size() == 0) {
    std::cout << "Bad mesh, no edges in list" << std::endl;
    result_mesh_.reset(new Mesh());
    return result_mesh_;
  }

  if (!target_vertices_used_ && !maximum_error_used_) return result_mesh_;

  // Greedy decimation, repeatedly contract best edge
  while ((queue_.size() > 0) &&
         (!target_vertices_used_ ||
          result_mesh_->nVertices() > target_vertices_) &&
         (!maximum_error_used_ || queue_.first().cost() < maximum_error_)) {
    if (callback != NULL)
      callback(result_mesh_->nVertices(), queue_.first().cost());
    const EdgeContraction contraction = queue_.first();
    executeEdgeContraction(contraction);
  }
  cleanAndRenumber();
  return result_mesh_;
}

::std::shared_ptr<Mesh> Decimator::doContractions(unsigned int n) {
  if (queue_.size() <= n) {
    std::cerr << "Warning: Cannot perform this many contractions" << std::endl;
    return ::std::make_shared<Mesh>();
  }
  for (unsigned int i = 0; i < n; ++i) {
    const EdgeContraction& contraction = queue_.first();
    if (maximum_error_used_ && contraction.cost() > maximum_error_) break;
    executeEdgeContraction(contraction);
  }
  cleanAndRenumber();
  return result_mesh_;
}

void Decimator::executeEdgeContraction(const EdgeContraction& contraction) {
  current_error_ = contraction.cost();

  Edge edge = result_mesh_->edge(contraction.edge());
  metric_generator_.contractEdge(contraction.edge());

  // First, remove edge, vertices and surrounding triangles
  std::set<Index> edges_to_remove;
  std::vector<Index*> hole_border;
  collectRemovalData(edge.vertex(0), edge.vertex(1), edges_to_remove,
                     hole_border);
  collectRemovalData(edge.vertex(1), edge.vertex(0), edges_to_remove,
                     hole_border);

  for (std::set<Index>::iterator it = edges_to_remove.begin();
       it != edges_to_remove.end(); ++it) {
    queue_.remove(*it);
  }

  result_mesh_->removeVertex(edge.vertex(0));
  result_mesh_->removeVertex(edge.vertex(1));

  // Then insert new vertex and connect it correctly
  Index new_vertex_index = result_mesh_->addVertex(contraction.new_point());
  for (unsigned int i = 0; i < hole_border.size(); ++i) {
    result_mesh_->addTriangle(new_vertex_index, hole_border[i][0],
                              hole_border[i][1]);
    delete[] hole_border[i];
  }

  const Vertex& new_vertex = result_mesh_->vertex(new_vertex_index);
  std::set<Index> edges_to_add;
  for (unsigned int i = 0; i < new_vertex.nTriangles(); ++i) {
    const Triangle& triangle = result_mesh_->triangle(new_vertex.triangle(i));
    for (unsigned int j = 0; j < 3; ++j) edges_to_add.insert(triangle.edge(j));
  }
  for (std::set<Index>::iterator it = edges_to_add.begin();
       it != edges_to_add.end(); ++it) {
    queue_.insert(computeEdgeContraction(*it));
  }
}

void Decimator::collectRemovalData(Index vertex_index, Index other_index,
                                   std::set<Index>& edges_to_remove,
                                   std::vector<Index*>& hole_border) {
  const Vertex& vertex = result_mesh_->vertex(vertex_index);
  // Collects data to remove a patch of triangles around a vertex.
  // Is called for both vertices of an edge, with vertex_index and other_index
  // swapped
  for (unsigned int i = 0; i < vertex.nTriangles(); ++i) {
    const Triangle& triangle = result_mesh_->triangle(vertex.triangle(i));
    bool is_shared_triangle = false;
    for (unsigned int j = 0; j < 3; ++j) {
      edges_to_remove.insert(triangle.edge(j));

      if (triangle.vertex(j) == other_index) is_shared_triangle = true;
    }
    if (!is_shared_triangle) {
      Index* border_edge = new Index[2];
      if (triangle.vertex(0) == vertex_index) {
        border_edge[0] = triangle.vertex(1);
        border_edge[1] = triangle.vertex(2);
      } else if (triangle.vertex(1) == vertex_index) {
        border_edge[0] = triangle.vertex(2);
        border_edge[1] = triangle.vertex(0);
      } else if (triangle.vertex(2) == vertex_index) {
        border_edge[0] = triangle.vertex(0);
        border_edge[1] = triangle.vertex(1);
      }
      hole_border.push_back(border_edge);
    }
  }
}

EdgeContraction Decimator::computeEdgeContraction(Index edge_index) {
  const Edge& edge = result_mesh_->edge(edge_index);
  const Vertex& vertex_1 = result_mesh_->vertex(edge.vertex(0));
  const Vertex& vertex_2 = result_mesh_->vertex(edge.vertex(1));

  Matrix44 qem = metric_generator_.getErrorMetric(edge_index);
  Vector3 new_point = Vector3::Zero();

#ifndef NDEBUG
  // Debugging: error matrices shoud be positive semidefinite
  Eigen::SelfAdjointEigenSolver<Matrix44> es;
  es.compute(qem);
  if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
      es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
    std::cerr << "Warning: Bad metric eigenvalues: "
              << es.eigenvalues().transpose() << std::endl
              << "Metric:" << std::endl
              << qem << std::endl;
  }
#endif

  // Detect edges with one vertex on a border edge
  int border_vertices = 0;
  for (unsigned int i = 0; i < vertex_1.nEdges(); ++i)
    if (result_mesh_->edge(vertex_1.edge(i)).border()) {
      new_point = vertex_1.position();
      border_vertices++;
      break;
    }
  for (unsigned int i = 0; i < vertex_2.nEdges(); ++i)
    if (result_mesh_->edge(vertex_2.edge(i)).border()) {
      new_point = vertex_2.position();
      border_vertices++;
      break;
    }
  if (border_vertices == 2 && !edge.border()) {
    /* Edge connects two borders, collapsing this would result in bad topology
    Current situation:
            ___________(Border)
            ...../|....
            ..../.|....
            .../..|<--This edge
            ...\..|....
            ....\.|....
            _____\|____(Border)
    Would result in:
            ___      __(Border)
            ...\    /..
            ....\  /...
            .....\/<--New Point
            ...../\....
            ..../  \...
            ___/    \__(Border)
    */
    return EdgeContraction(edge_index, Vector3::Zero(),
                           std::numeric_limits<Real>::max(), Matrix44::Zero());
  }

  if (direction_ == Any) {
    if (border_vertices == 0 || border_vertices == 2) {
#ifndef NDEBUG
      Real unused_cost;
#endif
      assert(optimizer_->optimize(qem, {}, false, &new_point, &unused_cost));
     }
    // For border_vertices == 1 the new point is already found

    Vector4 new_point_homogeneous;
    new_point_homogeneous << new_point, 1;
    Real cost = new_point_homogeneous.transpose() * qem * new_point_homogeneous;
    return EdgeContraction(edge_index, new_point, cost, qem);
  }

  // collect constraint planes
  std::vector<Plane> constraints;
  for (unsigned int i = 0; i < vertex_1.nTriangles(); ++i)
    constraints.push_back(result_mesh_->triangle(vertex_1.triangle(i)).plane());

  for (unsigned int i = 0; i < vertex_2.nTriangles(); ++i) {
    const Triangle& triangle = result_mesh_->triangle(vertex_2.triangle(i));
    bool shared_triangle = false;
    for (unsigned int j = 0; j < 3; ++j)
      if (triangle.vertex(j) == edge.vertex(0)) {
        shared_triangle = true;
        break;
      }
    if (!shared_triangle) {
      constraints.push_back(triangle.plane());
    }
  }

  // if one point is on the border, check if decimation is allowed
  if (border_vertices == 1) {
    bool valid_solution = true;
    Vector4 new_point_homogeneous;
    new_point_homogeneous << new_point, 1;
    for (unsigned int i = 0; i < constraints.size(); ++i) {
      if (direction_ == Outward &&
          constraints[i].vector().transpose() * new_point_homogeneous +
                  epsilon <
              0) {
        valid_solution = false;
        break;
      } else if (direction_ == Inward &&
                 constraints[i].vector().transpose() * new_point_homogeneous -
                         epsilon >
                     0) {
        valid_solution = false;
        break;
      }
    }
    if (valid_solution) {
      Real cost =
          new_point_homogeneous.transpose() * qem * new_point_homogeneous;
      return EdgeContraction(edge_index, new_point, cost, qem);
    } else
      return EdgeContraction(edge_index, Vector3::Zero(),
                             std::numeric_limits<Real>::max(),
                             Matrix44::Zero());
  }

  Real minimal_cost;
  bool found = optimizer_->optimize(
      qem, constraints, (direction_ == DecimationDirection::Outward),
      &new_point, &minimal_cost);

  if (found)
    return EdgeContraction(edge_index, new_point, minimal_cost, qem);
  else
    return EdgeContraction(edge_index, Vector3::Zero(),
                           std::numeric_limits<Real>::max(), Matrix44::Zero());
}

void Decimator::cleanAndRenumber() {
  ContractionQueue new_queue;
  Index next_index = 0;
  for (std::set<ContractionIndex>::iterator it = queue_.indices_.begin();
       it != queue_.indices_.end(); ++it) {
    /*
    The iteration over the queue_.indices_ set is ordered by index.
    Since the indices of deleted edges are missing, we can just put the position
    in the list as new index.
    This changes the edge indices just like Mesh::cleanAndRenumber
    Example:
    Set of indices: {1, 3, 5, 6, 7}
    Renumbered: 	{1, 2, 3, 4, 5}
    */
    ContractionIterator contraction_it = it->contraction();
    new_queue.insert(EdgeContraction(next_index, contraction_it->new_point(),
                                     contraction_it->cost(),
                                     contraction_it->qem()));
    next_index++;
  }
  swap(queue_, new_queue);
  metric_generator_.cleanAndRenumber();
  result_mesh_->cleanAndRenumber();
}

}  // namespace boundingmesh
