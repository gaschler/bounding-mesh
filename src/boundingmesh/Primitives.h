//	Copyright (c) 2013, Andre Gaschler, Quirin Fischer
//	All rights reserved.
//	
//	Redistribution and use in source and binary forms, with or without modification,
//	are permitted provided that the following conditions are met:
//	
//	* Redistributions of source code must retain the above copyright notice, this
//	  list of conditions and the following disclaimer.
//	
//	* Redistributions in binary form must reproduce the above copyright notice, this
//	  list of conditions and the following disclaimer in the documentation and/or
//	  other materials provided with the distribution.
//	
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
//	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//	ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef BOUNDINGMESH_PRIMITIVES_H
#define BOUNDINGMESH_PRIMITIVES_H

#include <Eigen/Dense>
#include <vector>

namespace boundingmesh
{
	typedef double Real;
	typedef Eigen::Matrix<Real, 3, 1> Vector3;
	typedef Eigen::Matrix<Real, 4, 1> Vector4;
	typedef Eigen::Matrix<Real, 4, 4> Matrix44;
	typedef unsigned int Index;

	class Plane
	{
	public:
		Plane();
		Plane(const Vector3& normal, Real d);
		Plane(const Vector3& normal, const Vector3& point);
		Plane(const Vector3& point1, const Vector3& point2, const Vector3& point3);		

		Vector3 normal;
		Real d;

		Real distance(const Vector3& point) const;
		Vector4 vector() const;
		Matrix44 distanceMatrix() const;
		
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	class Vertex
	{
		friend class Mesh;
	public:
		Vertex();
		Vertex(Vector3 position);
		Vertex(Real x, Real y, Real z);

		Vector3 position() const;
		unsigned int nTriangles() const;
		unsigned int nEdges() const;
		Index triangle(unsigned int i) const;
		Index edge(unsigned int i) const;
		
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		Vector3 position_;
		std::vector<Index> triangles_;
		std::vector<Index> edges_;
	};

	class Edge
	{
		friend class Mesh;
	public:
		Edge();
		Edge(Index vertex_1, Index vertex_2);
		
		unsigned int nTriangles() const;
		bool border() const;

		Index vertex(unsigned int i) const;
		Index triangle(unsigned int i) const;
	
	private:
		Index vertices_[2];
		std::vector<Index> triangles_;	
	};

	class Triangle
	{
		friend class Mesh;
	public:
		Triangle();
		Triangle(Index vertex_1, Index vertex_2, Index vertex_3);
		
		Index vertex(unsigned int i) const;
		Index edge(unsigned int i) const;
		Plane plane() const;
	private:
		Index vertices_[3];
		Index edges_[3];
		Plane plane_;
	};
}

#endif //BOUNDINGMESH_PRIMITIVES_H
