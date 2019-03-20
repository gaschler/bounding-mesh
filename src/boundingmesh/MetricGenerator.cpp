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

#include "boundingmesh/MetricGenerator.h"

namespace boundingmesh {
MetricGenerator::MetricGenerator()
    : mesh_(NULL), metric_(ClassicQEM), initialization_(Midpoint) {}

MetricGenerator::~MetricGenerator() {}

void MetricGenerator::setMesh(::std::shared_ptr<Mesh> mesh) {
  mesh_ = mesh;
  initialize();
}

void MetricGenerator::setMetric(Metric metric) {
  metric_ = metric;
  initialize();
}

void MetricGenerator::setInitialization(Initialization initialization) {
  initialization_ = initialization;
  initialize();
}

Matrix44 MetricGenerator::getErrorMetric(Index edge_index) {
  Matrix44 qem = Matrix44::Zero();
  const Edge& edge = mesh_->edge(edge_index);
  switch (metric_) {
    case ClassicQEM:
      qem = qems_[edge.vertex(0)] + qems_[edge.vertex(1)];
      break;
    case ModifiedQEM:
      qem = vertices_qem_[edge.vertex(0)] + vertices_qem_[edge.vertex(1)];
      // qem -= edges_qem_[edge_index];
      break;
    case MinimizedConstant:
    case Diagonalization:
    case Average:
      qem = mergeMatrices(qems_merge_[edge.vertex(0)],
                          qems_merge_[edge.vertex(1)]);
      break;
    default:
      break;
  }

  // Debugging: QEM matrices should be positive-semidefinite
  Eigen::SelfAdjointEigenSolver<Matrix44> es;
  es.compute(qem);
  if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
      es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
    std::cout << "Edge " << edge_index << std::endl;
    std::cout << "Bad metric eigenvalues: " << es.eigenvalues().transpose()
              << std::endl;
    // std::cout << edge.vertex(0) << ": " << std::endl <<
    // vertices_qem_[edge.vertex(0)] << std::endl << edge.vertex(1) << ": " <<
    // vertices_qem_[edge.vertex(1)] << std::endl << "edge " << edge_index << ":
    // " << edges_qem_[edge_index] << std::endl;
  }
  return qem;
}

void MetricGenerator::contractEdge(Index edge_index) {
  const Edge& edge = mesh_->edge(edge_index);
  switch (metric_) {
    case ClassicQEM:
      // The contraction removes two vertices from the mesh and inserts the new
      // one.
      // Since the freed indices are managed in a stack, the last one to be
      // deleted will be the new index.
      // The removal is performed in ascending order -> first remove(v0), then
      // remove(v1)
      qems_[edge.vertex(1)] = getErrorMetric(edge_index);
      break;
    case ModifiedQEM:
      contractEdgeModifiedQEM(edge_index);
      break;
    case MinimizedConstant:
    case Diagonalization:
    case Average:
      // See ClassicQEM for indexing method
      qems_merge_[edge.vertex(1)] = getErrorMetric(edge_index);
      break;
    default:
      break;
  }
}

void MetricGenerator::contractEdgeModifiedQEM(Index edge_index) {
  /*
          This method depends on the implementation of
     Decimator::executeEdgeContraction
          and the Mesh add/remove methods. It essentially simulates the mesh
     changes
          to get the correct indices for new vertices, edges and triangles.
  */

  const Edge& edge = mesh_->edge(edge_index);

  Matrix44 new_vertex_qem = getErrorMetric(edge_index);

  // An explanation of these data structures can be found in the method that
  // uses them, collectRemovalData.
  std::vector<HoleEdge, Eigen::aligned_allocator<HoleEdge> > hole_border;
  std::map<Index, Matrix44, std::less<Index>,
           Eigen::aligned_allocator<std::pair<const Index, Matrix44> > >
      new_edges_qem;

  collectRemovalData(edge.vertex(0), edge.vertex(1), hole_border,
                     new_edges_qem);
  collectRemovalData(edge.vertex(1), edge.vertex(0), hole_border,
                     new_edges_qem);

  for (unsigned int i = 0; i < hole_border.size(); ++i) {
    Eigen::SelfAdjointEigenSolver<Matrix44> es;
    es.compute(hole_border[i].old_triangle_qem);
    if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
        es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
      std::cout << "Hole border " << i << std::endl;
      std::cout << "Bad metric eigenvalues: " << es.eigenvalues().transpose()
                << std::endl;
      // std::cout << edge.vertex(0) << ": " << std::endl <<
      // vertices_qem_[edge.vertex(0)] << std::endl << edge.vertex(1) << ": " <<
      // vertices_qem_[edge.vertex(1)] << std::endl << "edge " << edge_index <<
      // ": " << edges_qem_[edge_index] << std::endl;
    }
  }

  for (std::map<Index, Matrix44, std::less<Index>,
                Eigen::aligned_allocator<std::pair<const Index, Matrix44> > >::
           iterator it = new_edges_qem.begin();
       it != new_edges_qem.end(); ++it) {
    Eigen::SelfAdjointEigenSolver<Matrix44> es;
    es.compute(it->second);
    if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
        es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
      std::cout << "Hole vertex " << it->first << std::endl;
      std::cout << "Bad metric eigenvalues: " << es.eigenvalues().transpose()
                << std::endl;
      // std::cout << edge.vertex(0) << ": " << std::endl <<
      // vertices_qem_[edge.vertex(0)] << std::endl << edge.vertex(1) << ": " <<
      // vertices_qem_[edge.vertex(1)] << std::endl << "edge " << edge_index <<
      // ": " << edges_qem_[edge_index] << std::endl;
    }
  }

  std::stack<Index> deleted_triangles;
  std::set<Index> triangles_already_deleted;
  std::stack<Index> deleted_edges;
  std::map<Index, unsigned int> edges_with_deleted_tris;
  std::set<Index> vertices_of_inserted_border_edges;

  // Removal of triangles and their edges
  for (unsigned int i = 0; i < mesh_->vertex(edge.vertex(0)).nTriangles();
       ++i) {
    Index triangle_index = mesh_->vertex(edge.vertex(0)).triangle(i);
    const Triangle& tri = mesh_->triangle(triangle_index);
    if (triangles_already_deleted.find(triangle_index) ==
        triangles_already_deleted.end()) {
      deleted_triangles.push(triangle_index);
      triangles_already_deleted.insert(triangle_index);

      for (unsigned int j = 0; j < 3; ++j) {
        Index next_edge = tri.edge(j);
        if (mesh_->edge(next_edge).border()) {
          deleted_edges.push(next_edge);
        } else {
          if (edges_with_deleted_tris.find(next_edge) !=
                  edges_with_deleted_tris.end() &&
              edges_with_deleted_tris[next_edge] == 1)
            deleted_edges.push(next_edge);
          else if (edges_with_deleted_tris.find(next_edge) !=
                   edges_with_deleted_tris.end())
            edges_with_deleted_tris[next_edge] =
                edges_with_deleted_tris[next_edge] - 1;
          else
            edges_with_deleted_tris[next_edge] =
                mesh_->edge(next_edge).nTriangles() - 1;
        }
      }
    }
  }

  for (unsigned int i = 0; i < mesh_->vertex(edge.vertex(1)).nTriangles();
       ++i) {
    Index triangle_index = mesh_->vertex(edge.vertex(1)).triangle(i);
    const Triangle& tri = mesh_->triangle(triangle_index);
    if (triangles_already_deleted.find(triangle_index) ==
        triangles_already_deleted.end()) {
      deleted_triangles.push(triangle_index);
      triangles_already_deleted.insert(triangle_index);
      for (unsigned int j = 0; j < 3; ++j) {
        Index next_edge = tri.edge(j);
        if (mesh_->edge(next_edge).border()) {
          deleted_edges.push(next_edge);
        } else {
          if (edges_with_deleted_tris.find(next_edge) !=
                  edges_with_deleted_tris.end() &&
              edges_with_deleted_tris[next_edge] == 1)
            deleted_edges.push(next_edge);
          else if (edges_with_deleted_tris.find(next_edge) !=
                   edges_with_deleted_tris.end())
            edges_with_deleted_tris[next_edge] =
                edges_with_deleted_tris[next_edge] - 1;
          else
            edges_with_deleted_tris[next_edge] =
                mesh_->edge(next_edge).nTriangles() - 1;
        }
      }
    }
  }

  Index new_vertex_index = edge.vertex(1);
  vertices_qem_[new_vertex_index] = new_vertex_qem;

  // Updating all edge/triangle QEMs
  for (unsigned int i = 0; i < hole_border.size(); ++i) {
    Index new_triangle_index = deleted_triangles.top();
    deleted_triangles.pop();
    triangles_qem_[new_triangle_index] = hole_border[i].old_triangle_qem;

    if (vertices_of_inserted_border_edges.find(hole_border[i].vertex_1) ==
        vertices_of_inserted_border_edges.end()) {
      assert(deleted_edges.size() > 0);

      Index new_edge_index = deleted_edges.top();
      deleted_edges.pop();

      vertices_of_inserted_border_edges.insert(hole_border[i].vertex_1);
      edges_qem_[new_edge_index] = new_edges_qem[hole_border[i].vertex_1];
    }
    if (vertices_of_inserted_border_edges.find(hole_border[i].vertex_2) ==
        vertices_of_inserted_border_edges.end()) {
      assert(deleted_edges.size() > 0);

      Index new_edge_index = deleted_edges.top();
      deleted_edges.pop();

      vertices_of_inserted_border_edges.insert(hole_border[i].vertex_2);
      edges_qem_[new_edge_index] = new_edges_qem[hole_border[i].vertex_2];
    }
  }
}

void MetricGenerator::collectRemovalData(
    Index vertex_index, Index other_index,
    std::vector<HoleEdge, Eigen::aligned_allocator<HoleEdge> >& hole_border,
    std::map<Index, Matrix44, std::less<Index>,
             Eigen::aligned_allocator<std::pair<const Index, Matrix44> > >&
        new_edges_qem) {
  /*
          Hole border:
                  Edges that will become the mesh border when the vertex is
     deleted.
                  For every neighbouring triangle of the removed vertex,
                  this is the one edge not containing the vertex.
                  In the following picture, the edges 1, 2, 3 are border edges.
                          3
                       ___v___
                      /\     /\
                  1->/  \   /  \<-2
                    /____\./____\
                          ^
                    The vertex to be removed

          Shared triangles:
                  Triangles that contain both vertices of the removed edge.
                  These do not produce a border edge (since 2 vertices get
     removed).
                  Also, the QEM matrix for the new edge that will be inserted is
     computed
                  differently than for non-shared triangles.
                  Example: The edge that will be removed connects vertices 1 and
     2.
                          The two adjacent triangles are shared (marked with
     "sh").
                       ______________
                      /\     /\     /\
                     /  \   /sh\   /  \
                    /____\1/____\2/____\
                    \    / \    / \    /
                     \  /   \sh/   \  /
                      \/_____\/_____\/

          Stored matrices:
                  Every border edge stores the QEM matrix of the triangle that
     generated it.
                  This allows us to restore the QEM for the new triangle that
     will be formed
                  with the border edge and the new vertex. Also, for every point
     on the
                  hole border the QEM of the edge that connected it to the
     removed
                  vertex is stored(this is the map new_edges_qem). This data is
     needed for
                  the new edge that will connect it to the inserted vertex.
                  For the third vertex of shared triangles, both the edges to
     removed
                  vertices as well as the shared triangle are incorporated to
     compute the QEM.

                  Summarizing, the matrices on the hole border edges are used to
     generate new triangles, while
                  the matrices in the map new_edges_qem are used for new edges
     (but the map identifies them
                  by the vertex that will define the edge).


  */
  const Vertex& vertex = mesh_->vertex(vertex_index);
  for (unsigned int i = 0; i < vertex.nTriangles(); ++i) {
    const Triangle& triangle = mesh_->triangle(vertex.triangle(i));
    HoleEdge border_edge;
    bool is_shared_triangle = false;

    for (unsigned int j = 0; j < 3; ++j)
      if (triangle.vertex(j) == other_index) is_shared_triangle = true;

    if (!is_shared_triangle) {
      if (triangle.vertex(0) == vertex_index) {
        border_edge.vertex_1 = triangle.vertex(1);
        border_edge.vertex_2 = triangle.vertex(2);
      } else if (triangle.vertex(1) == vertex_index) {
        border_edge.vertex_1 = triangle.vertex(2);
        border_edge.vertex_2 = triangle.vertex(0);
      } else if (triangle.vertex(2) == vertex_index) {
        border_edge.vertex_1 = triangle.vertex(0);
        border_edge.vertex_2 = triangle.vertex(1);
      }
      border_edge.old_triangle_qem = triangles_qem_[vertex.triangle(i)];

      hole_border.push_back(border_edge);
      // Save qem for edges to be created
      for (unsigned int j = 0; j < 3; ++j) {
        const Edge& edge_triangle = mesh_->edge(triangle.edge(j));
        Index hole_vertex = 0;

        if (edge_triangle.vertex(0) == vertex_index)
          hole_vertex = edge_triangle.vertex(1);
        else if (edge_triangle.vertex(1) == vertex_index)
          hole_vertex = edge_triangle.vertex(0);
        else
          continue;

        Index edge_index = triangle.edge(j);
        if (new_edges_qem.count(hole_vertex) == 0)
          new_edges_qem[hole_vertex] = edges_qem_[edge_index];
        else
          new_edges_qem[hole_vertex] += edges_qem_[edge_index];
      }
    } else {
      Index hole_vertex;
      for (unsigned int j = 0; j < 3; ++j)
        if (triangle.vertex(j) != vertex_index &&
            triangle.vertex(j) != other_index) {
          hole_vertex = triangle.vertex(j);
          break;
        }
      if (new_edges_qem.count(hole_vertex) == 0)
        new_edges_qem[hole_vertex] = -triangles_qem_[vertex.triangle(i)];
      else
        new_edges_qem[hole_vertex] -= triangles_qem_[vertex.triangle(i)];
    }
  }
}

void MetricGenerator::initialize() {
  if (mesh_ == NULL) return;

  switch (metric_) {
    case ClassicQEM:
      qems_.clear();
      for (unsigned int i = 0; i < mesh_->nVertices(); ++i)
        qems_.push_back(computeQEM(i));
      break;
    case ModifiedQEM:
      vertices_qem_.clear();
      for (unsigned int i = 0; i < mesh_->nVertices(); ++i)
        vertices_qem_.push_back(computeModifiedVertexQEM(i));

      edges_qem_.clear();
      for (unsigned int i = 0; i < mesh_->nEdges(); ++i)
        edges_qem_.push_back(computeModifiedEdgeQEM(i));

      triangles_qem_.clear();
      for (unsigned int i = 0; i < mesh_->nTriangles(); ++i)
        triangles_qem_.push_back(mesh_->triangle(i).plane().distanceMatrix());
      break;
    case MinimizedConstant:
    case Diagonalization:
    case Average:
      qems_merge_.clear();
      for (unsigned int i = 0; i < mesh_->nVertices(); ++i)
        qems_merge_.push_back(computeInitialMergeMetric(i));
      break;
    default:
      break;
  }
}

Matrix44 MetricGenerator::computeQEM(Index vertex_index) {
  Matrix44 qem = Matrix44::Zero();

  const Vertex& vertex = mesh_->vertex(vertex_index);
  for (unsigned int i = 0; i < vertex.nTriangles(); ++i)
    qem += mesh_->triangle(vertex.triangle(i)).plane().distanceMatrix();

  return qem;
}

Matrix44 MetricGenerator::computeModifiedVertexQEM(Index vertex_index) {
  const Vertex& vertex = mesh_->vertex(vertex_index);
  Matrix44 qem = Matrix44::Zero();
  std::vector<Vector3> normals;
  // Distance to triangles
  for (unsigned int i = 0; i < vertex.nTriangles(); ++i) {
    const Triangle& triangle = mesh_->triangle(vertex.triangle(i));
    normals.push_back(triangle.plane().normal);
    qem += triangle.plane().distanceMatrix();
  }

  // Correction of distances to edges

  for (unsigned int i = 0; i < vertex.nEdges(); ++i) {
    const Edge& edge = mesh_->edge(vertex.edge(i));
    if (edge.border()) {
      Vector3 third_point = mesh_->vertex(edge.vertex(0)).position() +
                            mesh_->triangle(edge.triangle(0)).plane().normal;
      Plane border_plane =
          Plane(mesh_->vertex(edge.vertex(0)).position(),
                mesh_->vertex(edge.vertex(1)).position(), third_point);
      normals.push_back(border_plane.normal);
      qem += border_plane.distanceMatrix();
    } else {
      Vector3 normal1 = mesh_->triangle(edge.triangle(0)).plane().normal;
      Vector3 normal2 = mesh_->triangle(edge.triangle(1)).plane().normal;
      Real angle = std::acos(normal1.dot(normal2));  // in radians
      Real alpha = std::cos(3.141592653589793238462 - angle);
      if (alpha > epsilon) {
        Vector3 normal_interpolated = (normal1 + normal2).normalized();
        Real d =
            normal_interpolated.dot(mesh_->vertex(edge.vertex(0)).position());
        Plane new_plane(normal_interpolated, d);
        normals.push_back(normal_interpolated);
        qem += alpha * new_plane.distanceMatrix();
      }
    }
  }

  // Correction of distance to vertex

  unsigned int n_normals = normals.size();
  Vector3 n_star = Vector3::Zero();
  Eigen::Matrix<Real, 3, 3> sum_Ni = Eigen::Matrix<Real, 3, 3>::Zero();
  Eigen::Matrix<Real, 3, Eigen::Dynamic> T;
  T.resize(3, n_normals);
  for (unsigned int i = 0; i < n_normals; ++i) {
    n_star += normals[i];
    sum_Ni += normals[i] * normals[i].transpose();
    T.col(i) << normals[i];
  }

  n_star.normalize();
  Eigen::Matrix<Real, 3, 3> N_star = n_star * n_star.transpose();

  Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> M_1;
  M_1.resize(n_normals, n_normals);
  M_1 = T.transpose() * T;
  Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> M_2;
  M_2.resize(n_normals, n_normals);
  M_2 = T.transpose() * sum_Ni * T;
  Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> M_3;
  M_3.resize(n_normals, n_normals);
  M_3 = T.transpose() * N_star * T;
  bool possible = true;
  Real lambda_min = -std::numeric_limits<Real>::max();
  Real lambda_max = std::numeric_limits<Real>::max();
  for (unsigned int i = 0; i < n_normals; ++i) {
    for (unsigned int j = 0; j < n_normals; ++j) {
      if (M_3(i, j) == 0) {
        if (M_1(i, j) - M_2(i, j) < 0) {
          // do nothing, is always ok
        } else {
          // std::cout<<"Impossible to correct "<<vertex_index<<":
          // "<<i<<","<<j<<std::endl;
          possible = false;
          break;
        }
      } else if (M_3(i, j) > 0) {
        Real new_lower_bound = (M_1(i, j) - M_2(i, j)) / M_3(i, j);
        if (new_lower_bound > lambda_min) {
          lambda_min = new_lower_bound;
        }
      } else if (M_3(i, j) < 0) {
        Real new_upper_bound = (M_1(i, j) - M_2(i, j)) / M_3(i, j);
        if (new_upper_bound < lambda_max) {
          lambda_max = new_upper_bound;
        }
      }
    }
    if (!possible) break;
  }

  if (lambda_min > lambda_max) {
    possible = false;
    // std::cout<<"bad interval: min "<<lambda_min<<" max
    // "<<lambda_max<<std::endl;
  }
  if (!possible) {
    // std::cout<<"Impossible to get valid lamdba"<<std::endl;
  }

  Plane correction_plane(n_star, vertex.position().dot(n_star));
  if (lambda_min < 0) lambda_min = 0;

  qem += lambda_min * correction_plane.distanceMatrix();

  // Hack: add distance to vertex
  Matrix44 distance;
  distance << Matrix44::Identity();
  distance.col(3) << -vertex.position(), 0;
  distance = distance.transpose() * distance;
  qem += distance_factor_ * distance;

  return qem;
}

Matrix44 MetricGenerator::computeModifiedEdgeQEM(Index edge_index) {
  Matrix44 qem = Matrix44::Zero();
  const Edge& edge = mesh_->edge(edge_index);
  for (unsigned int i = 0; i < edge.nTriangles(); ++i) {
    Plane plane = mesh_->triangle(edge.triangle(i)).plane();
    qem += plane.distanceMatrix();
  }
  if (edge.nTriangles() == 1) {
    Vector3 third_point = mesh_->vertex(edge.vertex(0)).position() +
                          mesh_->triangle(edge.triangle(0)).plane().normal;
    Plane border_plane =
        Plane(mesh_->vertex(edge.vertex(0)).position(),
              mesh_->vertex(edge.vertex(1)).position(), third_point);
    qem += border_plane.distanceMatrix();
  } else if (edge.nTriangles() == 2) {
    Vector3 normal1 = mesh_->triangle(edge.triangle(0)).plane().normal;
    Vector3 normal2 = mesh_->triangle(edge.triangle(1)).plane().normal;
    Real angle = std::acos(normal1.dot(normal2));  // in radians
    Real alpha = std::cos(3.141592653589793238462 - angle);
    if (alpha > epsilon) {
      Vector3 normal_interpolated = (normal1 + normal2).normalized();
      Real d =
          normal_interpolated.dot(mesh_->vertex(edge.vertex(0)).position());
      Plane new_plane(normal_interpolated, d);
      qem += alpha * new_plane.distanceMatrix();
    }
  }
  return qem;
}

Matrix44 MetricGenerator::computeInitialMergeMetric(Index vertex_index) {
  Matrix44 qem = Matrix44::Zero();
  if (initialization_ == Midpoint) {
    const Vertex& vertex = mesh_->vertex(vertex_index);
    for (unsigned int i = 0; i < vertex.nTriangles(); ++i) {
      const Triangle& triangle = mesh_->triangle(vertex.triangle(i));
      Vector3 midpoint = Vector3::Zero();
      for (unsigned int j = 0; j < 3; ++j) {
        const Vertex& triangle_vertex = mesh_->vertex(triangle.vertex(j));
        midpoint += triangle_vertex.position();
      }
      midpoint /= 3;
      Matrix44 distance_point = Matrix44::Identity();
      distance_point.col(3) << -midpoint, 0;
      distance_point = distance_point.transpose() * distance_point;
      qem = mergeMatrices(qem, distance_point);
    }
    Matrix44 distance_point = Matrix44::Identity();
    distance_point.col(3) << -vertex.position(), 0;
    distance_point = distance_point.transpose() * distance_point;
    qem = mergeMatrices(qem, distance_point);
  } else {
    const Vertex& vertex = mesh_->vertex(vertex_index);

    for (unsigned int i = 0; i < vertex.nTriangles(); ++i)
      qem = mergeMax(
          qem, mesh_->triangle(vertex.triangle(i)).plane().distanceMatrix());

    for (unsigned int i = 0; i < vertex.nEdges(); ++i) {
      const Edge& edge = mesh_->edge(vertex.edge(i));
      Vector3 position = mesh_->vertex(edge.vertex(0)).position();
      Vector3 direction =
          (mesh_->vertex(edge.vertex(1)).position() - position).normalized();
      Matrix44 distance_line = Matrix44::Zero();
      // Compute distance to line
      Matrix44 subtracted = Matrix44::Identity();
      subtracted.col(3) << -position, 0;
      distance_line += subtracted;
      Matrix44 dot_prod = Matrix44::Zero();
      dot_prod(0, 0) = direction(0);
      dot_prod(1, 1) = direction(1);
      dot_prod(2, 2) = direction(2);
      Matrix44 scalar_prod = Matrix44::Zero();
      scalar_prod.block(0, 0, 3, 3) << direction, direction, direction;
      distance_line -= scalar_prod * dot_prod * subtracted;
      distance_line = distance_line.transpose() * distance_line;
      qem = mergeMax(qem, distance_line);
    }

    Matrix44 distance_point = Matrix44::Identity();
    distance_point.col(3) << -vertex.position(), 0;
    distance_point = distance_point.transpose() * distance_point;
    qem = mergeMax(qem, distance_point);
  }
  return qem;
}

Matrix44 MetricGenerator::mergeMatrices(const Matrix44& a, const Matrix44& b) {
  Matrix44 result = Matrix44::Zero();
  switch (metric_) {
    case MinimizedConstant:
      result = mergeMinConstant(a, b);
      break;
    case Diagonalization:
      result = mergeDiagonalization(a, b);
      break;
    case Average:
      result = mergeAverage(a, b);
      break;
    default:
      assert(false);
  }
  return result;
}

Matrix44 MetricGenerator::mergeMax(const Matrix44& a, const Matrix44& b) {
  Matrix44 result = a + b;
  Eigen::Matrix<Real, 4, 3> E = Eigen::Matrix<Real, 4, 3>::Identity();
  Eigen::Matrix<Real, 4, 1> f(0, 0, 0, 1);

  Eigen::Matrix<Real, 3, 3> A_a = E.transpose() * a * E;
  Eigen::Matrix<Real, 3, 1> b_a = -(E.transpose() * a * f);
  Eigen::Matrix<Real, 3, 1> minimizer_a = A_a.ldlt().solve(b_a);
  Eigen::Matrix<Real, 4, 1> minimizer_4_a;
  minimizer_4_a << minimizer_a, 1;
  Real minimum_a = minimizer_4_a.transpose() * a * minimizer_4_a;

  Eigen::Matrix<Real, 3, 1> b_b = -(E.transpose() * b * f);
  Eigen::Matrix<Real, 3, 1> minimizer_b = A_a.ldlt().solve(b_b);
  Eigen::Matrix<Real, 4, 1> minimizer_4_b;
  minimizer_4_b << minimizer_b, 1;
  Real minimum_b = minimizer_4_b.transpose() * b * minimizer_4_b;

  Real constant_correction = std::min(minimum_a, minimum_b) - epsilon;
  if (constant_correction > 0) result(3, 3) -= constant_correction;
  return result;
}

Matrix44 MetricGenerator::mergeMinConstant(const Matrix44& a,
                                           const Matrix44& b) {
  Matrix44 result = a + b;
  Eigen::Matrix<Real, 4, 3> E = Eigen::Matrix<Real, 4, 3>::Identity();
  Eigen::Matrix<Real, 4, 1> f(0, 0, 0, 1);
  Eigen::Matrix<Real, 3, 3> A = E.transpose() * result * E;
  Eigen::Matrix<Real, 3, 1> b_ = -(E.transpose() * result * f);
  Eigen::Matrix<Real, 3, 1> minimizer = A.ldlt().solve(b_);

  Eigen::Matrix<Real, 4, 1> minimizer_4;
  minimizer_4 << minimizer, 1;
  Real minimum_r = minimizer_4.transpose() * result * minimizer_4;
  Real minimum_a = minimizer_4.transpose() * a * minimizer_4;
  Real minimum_b = minimizer_4.transpose() * b * minimizer_4;

  Real constant_correction =
      minimum_r - std::min(minimum_a, minimum_b) - epsilon;
  if (constant_correction > 0) result(3, 3) -= constant_correction;

  // Debugging
  Eigen::SelfAdjointEigenSolver<Matrix44> es;
  es.compute(result);
  if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
      es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
    std::cout << "mergeMinConstant eigenvalues: "
              << es.eigenvalues().transpose() << std::endl;
    std::cout << "correction: " << constant_correction << std::endl
              << result << std::endl;
    es.compute(a + b);
    if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
        es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon)
      std::cout << "eigenvalues without subtract: "
                << es.eigenvalues().transpose() << std::endl;
    else
      std::cout << "Fail introduced by correction" << std::endl;
  }
  return result;
}

Matrix44 MetricGenerator::mergeDiagonalization(const Matrix44& a,
                                               const Matrix44& b) {
  // std::cout<<"merging "<<std::endl<<a<<std::endl<<b<<std::endl;
  // std::cout<<"a"<<std::endl<<a<<std::endl;
  // std::cout<<"b"<<std::endl<<b<<std::endl;

  Matrix44 result = a + b;
  // std::cout<<"result"<<std::endl<<result<<std::endl;

  Eigen::Matrix<Real, 4, 3> E = Eigen::Matrix<Real, 4, 3>::Identity();
  Eigen::Matrix<Real, 4, 1> f(0, 0, 0, 1);
  Eigen::Matrix<Real, 3, 3> A = E.transpose() * result * E;
  Eigen::Matrix<Real, 3, 1> b_ = -(E.transpose() * result * f);
  Eigen::Matrix<Real, 3, 1> minimizer = A.ldlt().solve(b_);

#if 0
		std::cout<<"A:"<<std::endl<<A<<std::endl;
		std::cout<<"b: "<<b_.transpose()<<std::endl;
		std::cout<<"min for transl "<<minimizer.transpose()<<std::endl;
		std::cout<<"A*min "<<(A*minimizer).transpose()<<std::endl;
#endif

  Matrix44 translation = Matrix44::Identity();
  translation.col(3) << -minimizer, 1;
  Matrix44 translation_inv = Matrix44::Identity();
  translation_inv.col(3) << minimizer, 1;

  Matrix44 modified = translation_inv.transpose() * result * translation_inv;
  Eigen::Matrix<Real, 3, 3> block = modified.block(0, 0, 3, 3);
  Eigen::JacobiSVD<Eigen::Matrix<Real, 3, 3> > svd(
      block, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Matrix44 rotation_inv = Matrix44();
  rotation_inv << svd.matrixV(), Vector3::Zero(), Vector3::Zero().transpose(),
      1;

  Matrix44 transform = rotation_inv.transpose() * translation;
  Matrix44 transform_inv = translation_inv * rotation_inv;

  // std::cout<<"transform"<<std::endl<<transform<<std::endl;

  Matrix44 a_transformed = transform_inv.transpose() * a * transform_inv;
  Matrix44 b_transformed = transform_inv.transpose() * b * transform_inv;

  // std::cout<<"a_t"<<std::endl<<a_transformed<<std::endl;
  // std::cout<<"b_t"<<std::endl<<b_transformed<<std::endl;

  // Diagonalisation
  Vector3 a_diagonalised = Vector3::Zero();
  a_diagonalised(0) = std::sqrt(a_transformed(0, 0) * a_transformed(0, 0) +
                                a_transformed(1, 0) * a_transformed(1, 0) +
                                a_transformed(2, 0) * a_transformed(2, 0));
  a_diagonalised(1) = std::sqrt(a_transformed(0, 1) * a_transformed(0, 1) +
                                a_transformed(1, 1) * a_transformed(1, 1) +
                                a_transformed(2, 1) * a_transformed(2, 1));
  a_diagonalised(2) = std::sqrt(a_transformed(0, 2) * a_transformed(0, 2) +
                                a_transformed(1, 2) * a_transformed(1, 2) +
                                a_transformed(2, 2) * a_transformed(2, 2));
  a_diagonalised(3) = a_transformed(3, 3);

  Vector3 b_diagonalised = Vector3::Zero();
  b_diagonalised(0) = std::sqrt(b_transformed(0, 0) * b_transformed(0, 0) +
                                b_transformed(1, 0) * b_transformed(1, 0) +
                                b_transformed(2, 0) * b_transformed(2, 0));
  b_diagonalised(1) = std::sqrt(b_transformed(0, 1) * b_transformed(0, 1) +
                                b_transformed(1, 1) * b_transformed(1, 1) +
                                b_transformed(2, 1) * b_transformed(2, 1));
  b_diagonalised(2) = std::sqrt(b_transformed(0, 2) * b_transformed(0, 2) +
                                b_transformed(1, 2) * b_transformed(1, 2) +
                                b_transformed(2, 2) * b_transformed(2, 2));
  b_diagonalised(3) = b_transformed(3, 3);

  Vector3 linear_terms = Vector3::Zero();
  linear_terms(0) =
      std::max(std::abs(a_transformed(0, 3)), std::abs(b_transformed(0, 3)));
  linear_terms(1) =
      std::max(std::abs(a_transformed(1, 3)), std::abs(b_transformed(1, 3)));
  linear_terms(2) =
      std::max(std::abs(a_transformed(2, 3)), std::abs(b_transformed(2, 3)));
  // std::cout<<"a diag "<<a_diagonalised.transpose()<<std::endl;
  // std::cout<<"b diag "<<b_diagonalised.transpose()<<std::endl;
  // std::cout<<"linear "<<linear_terms.transpose()<<std::endl;
  Real l = 10;  // free parameter

  Matrix44 final_diagonalized = Matrix44::Zero();
  final_diagonalized(0, 0) = std::max(a_diagonalised(0), b_diagonalised(0)) +
                             linear_terms(0) / (2 * l);
  final_diagonalized(1, 1) = std::max(a_diagonalised(1), b_diagonalised(1)) +
                             linear_terms(1) / (2 * l);
  final_diagonalized(2, 2) = std::max(a_diagonalised(2), b_diagonalised(2)) +
                             linear_terms(2) / (2 * l);
  final_diagonalized(3, 3) = std::max(a_diagonalised(3), b_diagonalised(3)) +
                             linear_terms(0) * l / 2 + linear_terms(1) * l / 2 +
                             linear_terms(2) * l / 2;
  // std::cout<<"final_d"<<std::endl<<final_diagonalized<<std::endl;

  Matrix44 final_retransformed =
      transform.transpose() * final_diagonalized * transform;
  Matrix44 ret_uncorr = final_retransformed;
  // std::cout<<"final_d*t"<<std::endl<<final_diagonalized*
  // transform<<std::endl;
  // std::cout<<"final "<<std::endl<<final_retransformed<<std::endl;

  // Find correct constant
  /*Matrix44 m = final_retransformed - a;
  Matrix44 copy_m1 = m;
  A = E.transpose() * m * E;
  b_ = -(E.transpose() * m * f);
  minimizer = A.colPivHouseholderQr().solve(b_);

  Eigen::Matrix<Real, 4, 1> minimizer_4;
  minimizer_4<<minimizer, 1;
  //std::cout<<"min (q-a) "<<std::endl<<minimizer_4<<std::endl;
//	std::cout<<"q-a"<<std::endl<<m<<std::endl;
  Real m1 = minimizer_4.transpose() * m * minimizer_4;
//	std::cout<<"m*min"<<std::endl<<m * minimizer_4<<std::endl;
//	std::cout<<"m1 "<<m1<<std::endl;
  m = final_retransformed - b;
  Matrix44 copy_m2 = m;
  A = E.transpose() * m * E;
  b_ = -(E.transpose() * m * f);
  minimizer = A.colPivHouseholderQr().solve(b_);
  minimizer_4<<minimizer, 1;
//	std::cout<<"min (q-b) "<<minimizer_4.transpose()<<std::endl;
//	std::cout<<"q-a"<<std::endl<<m<<std::endl;
  Real m2 = minimizer_4.transpose() * m * minimizer_4;
//	std::cout<<"m2 "<<m2<<std::endl;
//	std::cout<<final_diagonalized(3,3)<<std::endl;
  final_retransformed(3,3) -= std::min(m1, m2);
//	std::cout<<final_diagonalized(3,3)<<std::endl;
  */
  // std::cout<<"corrected final "<<std::endl<<final_retransformed<<std::endl;

  // Debugging
  Eigen::SelfAdjointEigenSolver<Matrix44> es;
  es.compute(final_retransformed);
  Real epsilon = 0.001;
  if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
      es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
    std::cout << "Bad matrix eigenvalues: " << es.eigenvalues().transpose()
              << std::endl
              << "Metric:" << std::endl
              << final_retransformed << std::endl
              << "transform: " << transform
              << std::endl;  //<<"Const correct "<<std::min(m1, m2)<<std::endl;
    es.compute(ret_uncorr);
    if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
        es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
      std::cout << "final retransformed is bad" << std::endl;
    } else
      std::cout << "const correct broke matrix" << std::endl;
    // std::cout<<copy_m1<<std::endl<<copy_m2<<std::endl;
  }
  return final_retransformed;
}

Matrix44 MetricGenerator::mergeAverage(const Matrix44& a, const Matrix44& b) {
  Matrix44 result = (a + b) * 0.5;
  return result;
}

void MetricGenerator::cleanAndRenumber() {
  switch (metric_) {
    case ClassicQEM:
      shrinkIndexedArray(&qems_, mesh_->deleted_vertices_);
      break;
    case ModifiedQEM:
      shrinkIndexedArray(&vertices_qem_, mesh_->deleted_vertices_);
      shrinkIndexedArray(&edges_qem_, mesh_->deleted_edges_);
      shrinkIndexedArray(&triangles_qem_, mesh_->deleted_triangles_);
      break;
    case MinimizedConstant:
    case Diagonalization:
    case Average:
      shrinkIndexedArray(&qems_merge_, mesh_->deleted_vertices_);
      break;
    default:
      break;
  }
}

void MetricGenerator::shrinkIndexedArray(MatrixArray* array,
                                         std::stack<Index> deleted_indices) {
  MatrixArray new_array;
  std::vector<Index> deleted_indices_sorted;
  deleted_indices_sorted.reserve(deleted_indices.size());
  while (deleted_indices.size() > 0) {
    deleted_indices_sorted.push_back(deleted_indices.top());
    deleted_indices.pop();
  }
  std::sort(deleted_indices_sorted.begin(), deleted_indices_sorted.end());

  Index next_index = 0;
  unsigned int deleted_i = 0;
  if (array->size() <= deleted_indices_sorted.size()) {
    // TODO find cause for too many deleted indices
    std::cout << "What" << std::endl;
    array->swap(new_array);
    return;
  }
  unsigned int number_valid = array->size() - deleted_indices_sorted.size();
  for (unsigned int i = 0; i < number_valid; ++i) {
    while (deleted_i < deleted_indices_sorted.size() &&
           next_index == deleted_indices_sorted[deleted_i]) {
      next_index++;
      deleted_i++;
    }
    new_array.push_back((*array)[next_index]);
    next_index++;
  }
  array->swap(new_array);
}
}
