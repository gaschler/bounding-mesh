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

#include "Decimator.h"
#include "../../thirdparty/EigenQP.h"

#include <cmath>
#include <limits>
// for debugging
#include <Eigen/Eigenvalues>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace boundingmesh {
Decimator::Decimator(DecimationDirection direction)
    : target_vertices_(default_target_vertices),
      target_vertices_used_(false),
      maximum_error_(default_maximum_error),
      maximum_error_used_(false),
      direction_(direction),
      current_error_(-1),
      queue_(),
      result_mesh_(NULL),
      metric_generator_() {}

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
      assert(solveConstrainedMinimization(qem, std::vector<Plane>(),
                                          std::vector<unsigned int>(),
                                          direction_, new_point));
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
  bool found = solveConstrainedMinimizationInequalities(
      qem, constraints, direction_, new_point, minimal_cost);

  if (found)
    return EdgeContraction(edge_index, new_point, minimal_cost, qem);
  else
    return EdgeContraction(edge_index, Vector3::Zero(),
                           std::numeric_limits<Real>::max(), Matrix44::Zero());
}

unsigned int Decimator::nSubsets(unsigned int subset_size,
                                 unsigned int total_size) {
  unsigned int result = 1;
  for (unsigned int i = 0; i < subset_size; ++i)
    result *= (total_size - i) / (i + 1);
  return result;
}

void Decimator::nextSubset(std::vector<unsigned int>& indices_subset,
                           unsigned int total_size) {
  for (int i = (int)indices_subset.size() - 1; i >= 0; --i) {
    unsigned int max_next_index =
        indices_subset[i] + 1 + (indices_subset.size() - 1 - i);
    if (max_next_index < total_size) {
      indices_subset[i] = indices_subset[i] + 1;
      for (unsigned int j = i + 1; j < indices_subset.size(); ++j) {
        indices_subset[j] = indices_subset[j - 1] + 1;
      }
      break;
    }
  }
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

bool Decimator::solveConstrainedMinimization(
    const Matrix44& qem, const std::vector<Plane>& constraints,
    const std::vector<unsigned int>& subset, DecimationDirection direction,
    Vector3& result) {
  // Guess solution for the minimization problem by solving only for a subset of
  // constraints.
  // Return a bool indicating if the guess is allowed by all constraints.(even
  // then it might be non-optimal)
  Vector3 result_position = Vector3::Zero();
  switch (subset.size()) {
    case 0:
      result_position = minimizeSubspace(qem);
      break;
    case 1:
      result_position = minimizeSubspace(qem, constraints[subset[0]]);
      break;
    case 2:
      result_position =
          minimizeSubspace(qem, constraints[subset[0]], constraints[subset[1]]);
      break;
    case 3:
      result_position =
          minimizeSubspace(qem, constraints[subset[0]], constraints[subset[1]],
                           constraints[subset[2]]);
      break;
    default:
      assert(false);
  }

#define CHECK_LAGRANGE_SOLUTION 0
#if CHECK_LAGRANGE_SOLUTION
  // Compare to Lagrange multiplier solution
  Vector3 result_lagrange = Vector3::Zero();
  switch (subset.size()) {
    case 0:
      result_lagrange = minimizeLagrange(qem);
      break;
    case 1:
      result_lagrange = minimizeLagrange(qem, constraints[subset[0]]);
      break;
    case 2:
      result_lagrange =
          minimizeLagrange(qem, constraints[subset[0]], constraints[subset[1]]);
      break;
    case 3:
      result_lagrange =
          minimizeLagrange(qem, constraints[subset[0]], constraints[subset[1]],
                           constraints[subset[2]]);
      break;
    default:
      assert(false);
  }

  if ((result_lagrange - result_position).norm() > 1e-3) {
    std::cout << "Debug: Direct QEM minimizer differs from Lagrange solution: "
              << "Subspace result: " << result_position.transpose() << std::endl
              << "Lagrange result: " << result_lagrange.transpose()
              << std::endl;
  } else {
    std::cout << "Debug: m=0: direct QEM minimizer equals Lagrange solution"
              << std::endl;
  }
#endif

  // Check if minimizer fulfills all constraints
  Vector4 result_homogeneous;
  result_homogeneous << result_position, 1;

  bool valid_solution = true;
  for (unsigned int i = 0; i < constraints.size(); ++i) {
    if (direction == Outward &&
        constraints[i].vector().transpose() * result_homogeneous + epsilon <
            0) {
      valid_solution = false;
      break;
    } else if (direction == Inward &&
               constraints[i].vector().transpose() * result_homogeneous -
                       epsilon >
                   0) {
      valid_solution = false;
      break;
    }
  }
  result = result_position;

  return valid_solution;
}
Vector3 Decimator::minimizeSubspace(const Matrix44& quadratic_cost) {
  // Direct solution
  Eigen::Matrix<Real, 4, 3> E = Eigen::Matrix<Real, 4, 3>::Identity();
  Eigen::Matrix<Real, 4, 1> f(0, 0, 0, 1);
  Eigen::Matrix<Real, 3, 3> A = E.transpose() * quadratic_cost * E;
  Eigen::Matrix<Real, 3, 1> b_ = -(E.transpose() * quadratic_cost * f);
  Eigen::Matrix<Real, 3, 1> result = A.ldlt().solve(b_);
  return result;
}

Vector3 Decimator::minimizeSubspace(const Matrix44& quadratic_cost,
                                    Plane plane) {
  // Numerically stable orthonormals
  Vector3 some_unit(1, 0, 0);
  Vector3 some_other_unit(0, 1, 0);
  Vector3 plane_direction_1, plane_direction_2;

  if ((plane.normal.transpose() * some_unit).norm() <
      (plane.normal.transpose() * some_other_unit).norm())
    plane_direction_1 = plane.normal.cross(some_unit);
  else
    plane_direction_1 = plane.normal.cross(some_other_unit);

  plane_direction_1.normalize();
  plane_direction_2 = plane.normal.cross(plane_direction_1);
  plane_direction_2.normalize();

  Eigen::Matrix<Real, 4, 2> E;
  E << plane_direction_1, plane_direction_2, 0, 0;
  Eigen::Matrix<Real, 4, 1> f;
  f << (-plane.d) * plane.normal, 1;
  // Direct solution
  Eigen::Matrix<Real, 2, 1> m = (E.transpose() * quadratic_cost * E).inverse() *
                                (-(E.transpose() * quadratic_cost * f));
  Eigen::Matrix<Real, 4, 1> minimizer = E * m + f;
  Vector3 result = minimizer.topRows(3);
  return result;
}

bool Decimator::solveConstrainedMinimizationInequalities(
    const Matrix44& qem, const std::vector<Plane>& constraints,
    DecimationDirection direction, Vector3& result, Real& result_cost) {
#if USE_EIGENQUADPROG
  Eigen::MatrixXd G = qem.topLeftCorner<3, 3>();
  Eigen::VectorXd g0 = qem.topRightCorner<3, 1>();
  Eigen::MatrixXd CE = Eigen::MatrixXd::Zero(3, 0);
  Eigen::VectorXd ce0 = Eigen::VectorXd::Zero(0);
  Eigen::MatrixXd CI(3, constraints.size());
  Eigen::VectorXd ci0(constraints.size());
  for (int i = 0; i < constraints.size(); ++i) {
    CI.col(i) = constraints[i].normal;
    ci0(i) = constraints[i].d;
  }
  Eigen::VectorXd x(3);
  try {
    // TODO: check for inequalities and return false sometimes
    QP::solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
  } catch (const ::std::logic_error& e) {
    return false;
  }

  // result_cost = (x.transpose() * G * x)(0, 0) + 2 * (g0.transpose() * x)(0,
  // 0) + qem(3, 3);
  Vector4 result_homogeneous;
  result_homogeneous << x, 1;
  result_cost = result_homogeneous.transpose() * qem * result_homogeneous;
  result = x;

  return true;
#else
  // Search for valid minimizer that complies with 0 - 3 constraints
  bool found = false;
  Real minimal_cost = std::numeric_limits<Real>::max();
  int used_m = -1;
  Vector3 new_point;
  for (int m = 3; m >= 0; --m) {
    unsigned int n_subsets = nSubsets(m, constraints.size());
    std::vector<unsigned int> subset;
    subset.reserve(m);
    for (unsigned int i = 0; i < m; ++i) subset.push_back(i);
    for (unsigned int i = 0; i < n_subsets; ++i) {
      Vector3 result;
      bool found_valid = solveConstrainedMinimization(qem, constraints, subset,
                                                      direction, result);
      Vector4 result_homogeneous;
      result_homogeneous << result, 1;
      if (found_valid) {
        found = true;
        Real new_cost =
            result_homogeneous.transpose() * qem * result_homogeneous;
        if (new_cost < minimal_cost)  // + 1e-16)
        {
          minimal_cost = new_cost;
          new_point = result;
          used_m = m;
        }
      }
      nextSubset(subset, constraints.size());
    }
    if (found) break;
  }

  if (minimal_cost < -epsilon)  // Catch really wrong cases
  {
    std::cerr << "Warning: Bad contraction, negative cost: " << minimal_cost
              << std::endl
              << "qem: " << qem << std::endl
              << "point" << new_point << std::endl;
    minimal_cost = std::numeric_limits<Real>::max();
  }

  result_cost = minimal_cost;
  result = new_point;

  return found;
#endif
}

Vector3 Decimator::minimizeSubspace(const Matrix44& quadratic_cost,
                                    Plane plane1, Plane plane2) {
  Vector3 point_on_p = plane1.normal * (-plane1.d);
  Vector3 along_p_to_edge =
      plane1.normal.cross(plane1.normal.cross(plane2.normal));
  Real mu = ((-plane2.d) - plane2.normal.transpose() * point_on_p) /
            (plane2.normal.transpose() * along_p_to_edge);
  Vector3 point_on_edge = point_on_p + mu * along_p_to_edge;
  if (std::abs(point_on_edge.transpose() * plane1.normal + plane1.d) >
          epsilon ||
      std::abs(point_on_edge.transpose() * plane2.normal + plane2.d) >
          epsilon) {
    // std::cout << "Debug: m=2: point_on_edge is not on edge (p,q)." <<
    // std::endl;
    // That means the planes are close to parallel and a bad choice anyways
    return Vector3::Zero();
  } else {
    // std::cout << "Debug: point_on_edge is on edge (p,q)." << std::endl;
  }

  // Constraint subspace { E m + f | x }
  Eigen::Matrix<Real, 4, 1> E;
  E << plane1.normal.cross(plane2.normal), 0;
  Eigen::Matrix<Real, 4, 1> f;
  f << point_on_edge, 1;

  // Direct solution
  Real m = -static_cast<Real>(E.transpose() * quadratic_cost * f) /
           static_cast<Real>(E.transpose() * quadratic_cost * E);
  Eigen::Matrix<Real, 4, 1> minimizer = E * m + f;
  Vector3 result = minimizer.topRows(3);
  return result;
}

Vector3 Decimator::minimizeSubspace(const Matrix44& quadratic_cost,
                                    Plane plane1, Plane plane2, Plane plane3) {
  Eigen::Matrix<Real, 3, 3> m, m_inverse;
  m << plane1.normal.transpose(), plane2.normal.transpose(),
      plane3.normal.transpose();
  m_inverse = m.inverse();

  Vector3 b;
  b << (-plane1.d), (-plane2.d), (-plane3.d);
  if (m_inverse.norm() > 1e3) {
    // std::cout << "Warning, intersection is ill-conditioned." <<
    // m_inverse.norm() << std::endl;
    return Vector3::Zero();
  }
  Vector3 result = m_inverse * b;  // m.colPivHouseholderQr().solve(b);
  return result;
}

Vector3 Decimator::minimizeLagrange(const Matrix44& quadratic_cost) {
  Eigen::Matrix<Real, 4, 1> linear_constraints;
  linear_constraints << 0, 0, 0, 1;
  Eigen::Matrix<Real, 4 + 1, 1> b;
  b << 0, 0, 0, 0, 1;
  Eigen::Matrix<Real, 4 + 1, 4 + 1> A;
  A << quadratic_cost, linear_constraints, linear_constraints.transpose(), 0;

  Eigen::Matrix<Real, 4 + 1, 1> x;
  x = A.fullPivLu().solve(b);

  // check if solution is valid
  if ((A * x - b).topRows(3).norm() > 1e-4) {
    // std::cout << "Debug verbose: QEM minimum without constraints: Lagrange
    // seems not invertible" << std::endl;
    return Vector3::Zero();
  }
  if (std::abs(x(3) - 1) > 1e-4) {
    // std::cout << "Debug verbose: QEM minimum without constraints: violates
    // Lagrange constraints, skipping" << std::endl;
    return Vector3::Zero();
  }

  Vector3 result = x.topRows(3);
  return result;
}

Vector3 Decimator::minimizeLagrange(const Matrix44& quadratic_cost,
                                    Plane plane) {
  Eigen::Matrix<Real, 6, 6> A = Eigen::Matrix<Real, 6, 6>::Zero();
  A.topLeftCorner(4, 4) = quadratic_cost;
  A.col(4).topRows(3) = plane.normal;
  A.row(4).leftCols(3) = plane.normal.transpose();
  A(3, 5) = A(5, 3) = 1;
  Eigen::Matrix<Real, 6, 1> b, x;
  b << 0, 0, 0, 0, -plane.d, 1;
  x = A.fullPivLu().solve(b);

  if (std::abs(x(3) - 1) > 1e-4) {
    // std::cout << "Debug verbose: m=1: violates Lagrange constraints,
    // skipping" << std::endl;
    return Vector3::Zero();
  }

  if (std::abs(x.topRows(3).transpose() * plane.normal + plane.d) > epsilon) {
    // std::cout << "Debug: m=1: Minimizer is not on plane" << std::endl;
    return Vector3::Zero();
  }
  Vector3 result = x.topRows(3);
  return result;
}

Vector3 Decimator::minimizeLagrange(const Matrix44& quadratic_cost,
                                    Plane plane1, Plane plane2) {
  Eigen::Matrix<Real, 4, 3> linear_constraints =
      Eigen::Matrix<Real, 4, 3>::Zero();
  linear_constraints.col(0).topRows(3) = plane1.normal;
  linear_constraints.col(1).topRows(3) = plane2.normal;
  linear_constraints(3, 2) = 1;
  Eigen::Matrix<Real, 4 + 3, 1> b;
  b << 0, 0, 0, 0, -plane1.d, -plane2.d, 1;
  Eigen::Matrix<Real, 4 + 3, 4 + 3> A;
  A << quadratic_cost, linear_constraints, linear_constraints.transpose(),
      Eigen::Matrix<Real, 3, 3>::Zero();
  Eigen::Matrix<Real, 4 + 3, 1> x;
  x = A.fullPivLu().solve(b);

  // check if solution is valid
  if ((A * x - b).topRows(3).norm() > 1e-4) {
    // std::cout << "Debug verbose: m=2: Lagrange seems not invertible" <<
    // std::endl;
    return Vector3::Zero();
  }
  if (std::abs(x(3) - 1) > 1e-4) {
    // std::cout << "Debug verbose: m=2: violates Lagrange constraints,
    // skipping" << std::endl;
    return Vector3::Zero();
  }

  if (std::abs(x.topRows(3).transpose() * plane1.normal + plane1.d) > epsilon ||
      std::abs(x.topRows(3).transpose() * plane2.normal + plane2.d) > epsilon) {
    // std::cout << "Debug: m=2: Lagrange minimizer is not on edge (p,q)." <<
    // std::endl;
    return Vector3::Zero();
  }

  Vector3 result = x.topRows(3);
  return result;
}

Vector3 Decimator::minimizeLagrange(const Matrix44& quadratic_cost,
                                    Plane plane1, Plane plane2, Plane plane3) {
  return minimizeSubspace(quadratic_cost, plane1, plane2, plane3);
}
}
