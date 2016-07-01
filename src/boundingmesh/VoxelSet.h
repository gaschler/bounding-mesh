//	Copyright (c) 2015, Andre Gaschler, Quirin Fischer
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

#ifndef BOUNDINGMESH_VOXELSET_H
#define BOUNDINGMESH_VOXELSET_H

#include "Primitives.h"
#include "Mesh.h"

#include <string>
#include <memory>

namespace boundingmesh
{
	enum VoxelType
	{
		INNER,
		SURFACE
	};
	
	class Voxel
	{
	public:
		Voxel();
		Voxel(Index x, Index y, Index z, VoxelType type);
		Voxel(const Voxel& voxel);
		~Voxel();

		Voxel& operator=(Voxel other);
		friend void swap(Voxel& first, Voxel& second);

		Index x() const;
		Index y() const;
		Index z() const;
		Index coordinate(int dimension) const;
		VoxelType type() const;
		unsigned int nTriangles() const;
		Index triangle(unsigned int i) const;

		void addTriangle(Index triangle);

	private:
		Index x_, y_, z_;
		VoxelType type_;
		std::vector<Index> triangles_;
	};


	class VoxelSet
	{
	public:
		VoxelSet();
		VoxelSet(const VoxelSet& voxel_set);
		VoxelSet(std::shared_ptr<Mesh> triangle_mesh, Real voxel_size, bool newVersion = true);
		~VoxelSet();
		
		VoxelSet& operator=(VoxelSet other);
		friend void swap(VoxelSet& first, VoxelSet& second);

		unsigned int nVoxels() const;
		const Voxel& voxel(unsigned int i) const; 
		const Vector3& origin() const;
		Real voxelSize() const;
		unsigned int resolution(int dimension) const;
		std::shared_ptr<Mesh> mesh() const;
		int voxelAt(unsigned int x, unsigned int y, unsigned int z) const;
		Vector3 computePosition(const Voxel& voxel) const;

		void addVoxel(const Voxel& voxel);

		Real volume();

		void writeWRL(std::string filename);

	private:
		Vector3 origin_;
		Index resolution_[3];
		Real voxel_size_;
		std::vector<Voxel> voxels_;
		std::vector<int> grid_;
		std::shared_ptr<Mesh> mesh_;
	};

}

#endif //BOUNDINGMESH_VOXELMESH_H
