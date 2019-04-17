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

#ifndef BOUNDINGMESH_MESH_H
#define BOUNDINGMESH_MESH_H

#include "boundingmesh/Primitives.h"

#include <Eigen/StdVector>
#include <deque>
#include <memory>
#include <set>
#include <stack>
#include <string>

#define DEBUG_COLOR 0
#define DEBUG_VRML 0

namespace boundingmesh {
// Forward declaration
class Mesh;

class VertexPosition {
 public:
  VertexPosition(const Vector3& position);
  VertexPosition(Index index, const Vector3& position);

  Index index() const;

  friend bool operator<(const VertexPosition& lhs, const VertexPosition& rhs);
  friend bool operator>(const VertexPosition& lhs, const VertexPosition& rhs);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  Vector3 position_;
  Index index_;
  bool searching_;
};

const Real vertex_epsilon = 0.000001;

class VertexPositionSet {
 public:
  VertexPositionSet(Mesh* mesh);

  Index addVertex(const Vector3& position);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  Mesh* mesh_;
  std::set<VertexPosition, std::less<VertexPosition>,
           Eigen::aligned_allocator<VertexPosition> >
      set_;
};

const Real normal_epsilon = 0.00001;

class Mesh {
 public:
  Mesh();
  Mesh(const Mesh& mesh);
  Mesh(const std::vector<Vector3>& vertices,
       const std::vector<Index*>& triangles);
  ~Mesh();

  Mesh& operator=(Mesh other);
  friend void swap(Mesh& first, Mesh& second);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Loading/Saving files
  void loadOff(const std::string& filename);
  void loadObj(const std::string& filename);
  void loadWrl(const std::string& filename, int faceset = -1,
               bool debugOutput = true);
  void loadStl(const std::string& filename);
  void writeOff(const std::string& filename);
  void writeObj(const std::string& filename);
  void writeWrl(const std::string& filename, bool colored = false);
  void writeStl(const std::string& filename, bool binary = false);

  static void writeMultimeshWrl(std::vector<std::shared_ptr<Mesh> > submeshes,
                                const std::string& filename,
                                bool colored = false);

  unsigned int nVertices() const;
  unsigned int nEdges() const;
  unsigned int nTriangles() const;

  const Vertex& vertex(Index i) const;
  const Edge& edge(Index i) const;
  const Triangle& triangle(Index i) const;

  Index addVertex(const Vector3& vertex);
  Index addTriangle(Index vertex1, Index vertex2, Index vertex3);

  Real getBoundingBoxDiagonal();

  void removeVertex(Index vertex);
  void removeTriangle(Index triangle);

  void closeHoles();
  Real calculateConvexVolume();

  void cleanAndRenumber();
  bool isDirty();

  void setDebugData(std::string debug_string);

 private:
  std::deque<Vertex> vertices_;
  std::deque<Edge> edges_;
  std::deque<Triangle> triangles_;

  void clean();

  // Helper for edge insertion
  Index registerEdge(Index vertex1, Index vertex2, Index triangle);

  // Lazy removal of vertices/triangles
  bool dirty_ = false;
  unsigned int n_valid_vertices_ = 0;
  unsigned int n_valid_edges_ = 0;
  unsigned int n_valid_triangles_ = 0;
  std::stack<Index> deleted_vertices_;
  std::stack<Index> deleted_edges_;
  std::stack<Index> deleted_triangles_;

  friend class MetricGenerator;

  // Debug data
  unsigned int n_original = 0;
  std::string debug_vrml;

  // For multipart-output
  std::vector<std::shared_ptr<Mesh> > computeSubmeshes();

  static Vector3 HSVtoRGB(Vector3 color);
  static std::vector<Vector3> generateColors(int n);
};

class Convex {
 public:
  Convex();
  Convex(const Convex& other);
  Convex(const std::vector<Vector3>& points);
  ~Convex();

  Convex& operator=(Convex other);
  friend void swap(Convex& first, Convex& second);

  double ComputeVolume();

  std::shared_ptr<Mesh> mesh;
  Real volume;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace boundingmesh

#endif  // BOUNDINGMESH_MESH_H
