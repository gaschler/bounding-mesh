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

#include "Primitives.h"

namespace boundingmesh {
// Implementation of class Plane
Plane::Plane() : normal(Vector3::Zero()), d(0) {}

Plane::Plane(const Vector3& normal, Real d) : normal(normal), d(d) {
  // Used plane equation: normal*x + d = 0
}

Plane::Plane(const Vector3& normal, const Vector3& point) : normal(normal) {
  this->normal.normalize();
  d = -this->normal.dot(point);
}

Plane::Plane(const Vector3& point1, const Vector3& point2,
             const Vector3& point3) {
  // Center point for cross product: Point1
  // Used plane equation: normal*x + d = 0
  normal = (point2 - point1).cross(point3 - point1);
  normal.normalize();
  d = -normal.dot(point1);
}

Real Plane::distance(const Vector3& point) const {
  return normal.dot(point) + d;
}

Eigen::Matrix<Real, 4, 1> Plane::vector() const {
  Eigen::Matrix<Real, 4, 1> result;
  result << normal, d;
  return result;
}

Eigen::Matrix<Real, 4, 4> Plane::distanceMatrix() const {
  return vector() * vector().transpose();
}

// Implementation of class Vertex
Vertex::Vertex() {}

Vertex::Vertex(Vector3 position)
    : position_(position), triangles_(), edges_() {}

Vertex::Vertex(Real x, Real y, Real z)
    : position_(Vector3(x, y, z)), triangles_(), edges_() {}

Vector3 Vertex::position() const { return position_; }

unsigned int Vertex::nTriangles() const { return triangles_.size(); }

unsigned int Vertex::nEdges() const { return edges_.size(); }

Index Vertex::triangle(unsigned int i) const { return triangles_[i]; }

Index Vertex::edge(unsigned int i) const { return edges_[i]; }

// Implementation of class Edge
Edge::Edge() {
  vertices_[0] = 0;
  vertices_[1] = 0;
  triangles_ = std::vector<Index>();
}

Edge::Edge(Index vertex_1, Index vertex_2) {
  vertices_[0] = vertex_1;
  vertices_[1] = vertex_2;
  triangles_ = std::vector<Index>();
}

unsigned int Edge::nTriangles() const { return triangles_.size(); }

bool Edge::border() const { return triangles_.size() == 1; }

Index Edge::vertex(unsigned int i) const { return vertices_[i]; }

Index Edge::triangle(unsigned int i) const { return triangles_[i]; }

// Implementation of class Triangle
Triangle::Triangle() : plane_() {
  vertices_[0] = 0;
  vertices_[1] = 0;
  vertices_[2] = 0;

  edges_[0] = 0;
  edges_[1] = 0;
  edges_[2] = 0;
}

Triangle::Triangle(Index vertex_1, Index vertex_2, Index vertex_3) : plane_() {
  vertices_[0] = vertex_1;
  vertices_[1] = vertex_2;
  vertices_[2] = vertex_3;

  edges_[0] = 0;
  edges_[1] = 0;
  edges_[2] = 0;
}

Index Triangle::vertex(unsigned int i) const { return vertices_[i]; }

Index Triangle::edge(unsigned int i) const { return edges_[i]; }

Plane Triangle::plane() const { return plane_; }
}
